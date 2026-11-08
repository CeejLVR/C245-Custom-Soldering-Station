#include <math.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#include "hardware/watchdog.h"
#include "pico/time.h"

#include "pid.h"
#include "storage.h"
#include "config.h"
#include "station.h"
#include "heater.h"
#include "sensors.h"
#include "storage.h"

// The control loop, every 100ms:
//   heater off -> let the sense lines settle -> read the TC, read the cold junction (once a second)
//   -> run the PID -> heater on for whatever's left of the period
//
// On the C245 the thermocouple and heater share wires, so we can only measure
// with the heater off. If the order here ever gets messed up you get an
// INTERLOCK fault instead of a bad reading going into the PID.

static QueueHandle_t status_q;  // 1 slot, always the newest status
static QueueHandle_t cmd_q;

static volatile TickType_t last_beat;
static TimerHandle_t sleep_timer, off_timer;

static pid_ctrl_t pid;
static bool pid_running;

static fault_t fault = FAULT_NONE;
static bool enabled, standby, tip_present;
static int cj_fails, adc_fails;
static uint32_t cycle;


// ---------- messaging ----------

void station_init(void) {
    status_q = xQueueCreate(1, sizeof(station_status_t));
    cmd_q = xQueueCreate(8, sizeof(station_cmd_t));
    configASSERT(status_q && cmd_q);
}

void status_publish(const station_status_t *st) {
    xQueueOverwrite(status_q, st);
}

bool status_get(station_status_t *out) {
    return xQueuePeek(status_q, out, 0) == pdTRUE;
}

bool cmd_send(station_cmd_type_t type, int32_t value) {
    station_cmd_t c = { type, value };
    return xQueueSend(cmd_q, &c, 0) == pdTRUE;
}

const char *state_name(station_state_t s) {
    switch (s) {
    case ST_OFF:     return "OFF";
    case ST_HEATING: return "HEATING";
    case ST_READY:   return "READY";
    case ST_STANDBY: return "STANDBY";
    case ST_NO_TIP:  return "NO TIP";
    case ST_FAULT:   return "FAULT";
    }
    return "?";
}

const char *fault_name(fault_t f) {
    switch (f) {
    case FAULT_NONE:          return "none";
    case FAULT_ADC:           return "ADC";
    case FAULT_CJ_SENSOR:     return "CJ SENSOR";
    case FAULT_TC_RANGE:      return "TC RANGE";
    case FAULT_OVERTEMP:      return "OVERTEMP";
    case FAULT_RUNAWAY:       return "NO TEMP RISE";
    case FAULT_CONTROL_STALL: return "CTRL STALL";
    case FAULT_EXT_WDT:       return "HW WATCHDOG";
    case FAULT_INTERLOCK:     return "INTERLOCK";
    case FAULT_WDT_RESET:     return "WDT RESET";
    case FAULT_COUNT:         break;
    }
    return "?";
}


// ---------- helpers ----------

bool control_alive(uint32_t timeout_ms) {
    return (xTaskGetTickCount() - last_beat) <= pdMS_TO_TICKS(timeout_ms);
}

// vTaskDelay only does whole 1ms ticks, so sleep for most of it and spin
// out the rest
static void wait_us(uint32_t us) {
    uint64_t end = time_us_64() + us;
    if (us >= 2000)
        vTaskDelay(pdMS_TO_TICKS(us / 1000 - 1));
    while (time_us_64() < end)
        tight_loop_contents();
}

// only the first fault sticks, so the screen shows what actually went wrong
// first and not whatever it cascaded into
static void set_fault(fault_t f) {
    if (fault == FAULT_NONE)
        fault = f;
}

static void sleep_expired(TimerHandle_t t) { (void)t; cmd_send(CMD_STANDBY, 0); }
static void off_expired(TimerHandle_t t)   { (void)t; cmd_send(CMD_SET_ENABLE, 0); }

static void handle_commands(void) {
    station_cmd_t c;
    while (xQueueReceive(cmd_q, &c, 0) == pdTRUE) {
        switch (c.type) {
        case CMD_TOGGLE_ENABLE:
        case CMD_SET_ENABLE:
            enabled = (c.type == CMD_TOGGLE_ENABLE) ? !enabled : (c.value != 0);
            standby = false;
            xTimerStop(off_timer, 0);
            if (enabled) xTimerReset(sleep_timer, 0);
            else xTimerStop(sleep_timer, 0);
            pid_running = false;
            break;

        case CMD_WAKE:
            if (standby) {
                standby = false;
                xTimerStop(off_timer, 0);
            }
            if (enabled) xTimerReset(sleep_timer, 0);
            break;

        case CMD_STANDBY:
            if (enabled && !standby) {
                standby = true;
                xTimerReset(off_timer, 0);
            }
            break;

        case CMD_CLEAR_FAULT:
            fault = FAULT_NONE;
            cj_fails = adc_fails = 0;
            enabled = false;        // make the user switch it back on themselves
            safety_clear();
            pid_running = false;
            break;
        }
    }
}

// Once a second, push 10uA into AIN1 and see where it goes. With a tip in it
// flows to ground through R8 and the TC. With no tip AIN1 floats. The
// reading has to land in the expected band, so a dead source reads as no tip.
static void check_tip(void) {
    int32_t raw;
    if (ads_tip_source(true)) {
        vTaskDelay(pdMS_TO_TICKS(TIP_CHECK_SETTLE_MS));
        if (ads_convert(&raw, ADC_TIMEOUT_MS)) {
            float drop_uv = -ads_to_uv(raw);
            tip_present = drop_uv > TIP_CHECK_MIN_UV && drop_uv < TIP_CHECK_MAX_UV;
        } else {
            tip_present = false;
        }
    } else {
        tip_present = false;
    }
    ads_tip_source(false);
    wait_us(TIP_CHECK_DRAIN_US);      // let C8 drain before the real reading
}

// Returns true if we got a usable thermocouple reading.
static bool measure(int32_t *code) {
    if (!heater_settled()) {
        set_fault(FAULT_INTERLOCK);
        return false;
    }

    if (cycle % TIP_CHECK_EVERY_N == 0)
        check_tip();

    bool ok = false;
    if (ads_convert(code, ADC_TIMEOUT_MS)) {
        ok = true;
        adc_fails = 0;
    } else if (++adc_fails >= ADC_MAX_FAILS) {
        set_fault(FAULT_ADC);
    }

    return ok;
}


// ---------- the task ----------

static void control_task(void *arg) {
    if (watchdog_caused_reboot()) set_fault(FAULT_WDT_RESET);
    if (!ads_init()) set_fault(FAULT_ADC);
    if (!cj_init()) set_fault(FAULT_CJ_SENSOR);

    sleep_timer = xTimerCreate("sleep", pdMS_TO_TICKS(SLEEP_TIMEOUT_S * 1000u), pdFALSE, NULL, sleep_expired);
    off_timer = xTimerCreate("autooff", pdMS_TO_TICKS(AUTO_OFF_TIMEOUT_S * 1000u), pdFALSE, NULL, off_expired);
    configASSERT(sleep_timer && off_timer);

    const float dt = CONTROL_PERIOD_MS / 1000.0f;
    pid_init(&pid, PID_KP, PID_KI, PID_KD, dt, 0.0f, HEATER_MAX_DUTY);

    settings_t cfg;
    station_status_t st = {0};
    float cj = 25.0f, tip = 25.0f, uv = 0.0f;
    uint32_t seq = 0;

    const TickType_t period = pdMS_TO_TICKS(CONTROL_PERIOD_MS);
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        TickType_t start = xTaskGetTickCount();
        heater_off();
        storage_get(&cfg);
        handle_commands();

        wait_us(HEATER_OFF_SETTLE_US);

        int32_t code = 0;
        bool got_reading = measure(&code);

        // Cold junction, once a second, read AFTER the thermocouple. The first
        // conversion after leaving temperature mode reads about 20uV off, so
        // doing it here gives the ADC 100ms to recover before the next TC read.
        // This cycle uses the cold junction from a second ago, which is fine.
        if (cycle % CJ_READ_EVERY_N == CJ_READ_PHASE) {
            float t;
            if (cj_read(&t) && t > CJ_MIN_C && t < CJ_MAX_C) {
                cj = t;
                cj_fails = 0;
            } else if (++cj_fails >= CJ_MAX_FAILS) {
                set_fault(FAULT_CJ_SENSOR);
            }
        }
        if (got_reading) {
            uv = ads_to_uv(code);
            tc_cal_t cal = { cfg.tc_sens_uv_per_c, cfg.tc_offset_uv, cfg.tc_polarity };
            tip = tip_temp(uv, cj, &cal);

            // pulling the tip floats AIN1, which can happen between tip checks
            if (fabsf(uv) > TC_MAX_PLAUSIBLE_UV)
                tip_present = false;

            if (tip_present) {
                if (tip > TEMP_ABS_MAX_C) set_fault(FAULT_OVERTEMP);
                else if (tip < TEMP_MIN_PLAUSIBLE_C) set_fault(FAULT_TC_RANGE);
            }
        }

        // Show the safety task's fault without copying it into ours. If we
        // copied it, clearing it over there would never show up here.
        fault_t shown = (fault != FAULT_NONE) ? fault : safety_fault();

        int target = cfg.setpoint_c;
        if (standby && cfg.standby_c < target)
            target = cfg.standby_c;

        station_state_t state;
        if (shown != FAULT_NONE)
            state = ST_FAULT;
        else if (!enabled)
            state = ST_OFF;
        else if (!tip_present)
            state = ST_NO_TIP;
        else if (standby)
            state = ST_STANDBY;
        else if (fabsf(tip - (float)target) <= READY_BAND_C)
            state = ST_READY;
        else
            state = ST_HEATING;

        float demand = 0.0f;
        bool regulating = got_reading && (state == ST_HEATING || state == ST_READY || state == ST_STANDBY);
        if (regulating) {
            if (!pid_running) {
                pid_initialize(&pid, tip, 0.0f);    // start from 0 output, no D kick
                pid_running = true;
            }
            demand = pid_compute(&pid, (float)target, tip);
        } else {
            pid_running = false;
        }

        // How long can the heater stay on without running into the next
        // measurement? Normally period minus the reserve, but if this loop
        // ran long, shrink it to whatever time is actually left.
        TickType_t elapsed = xTaskGetTickCount() - start;
        TickType_t slot = period - pdMS_TO_TICKS(MEAS_WINDOW_RESERVE_MS);
        if (elapsed + slot + 1 > period)
            slot = (elapsed + 1 < period) ? period - elapsed - 1 : 0;

        TickType_t on = (TickType_t)lroundf(demand * (float)period);
        if (on > slot)
            on = slot;

        bool heating = on > 0 && heater_on();
        // it should have been allowed and wasn't, so the sequence is off somewhere
        if (on > 0 && !heating && CFG_HEATER_ARMED && !heater_locked())
            set_fault(FAULT_INTERLOCK);

        st.state = state;
        st.fault = shown;
        st.tip_c = tip;
        st.cj_c = cj;
        st.tc_uv = uv;
        st.duty_demand = demand;
        st.duty_applied = heating ? (float)on / (float)period : 0.0f;
        st.setpoint_c = (int16_t)target;
        st.preset = cfg.active_preset;
        st.tip_present = tip_present;
        st.enabled = enabled;
        st.armed = CFG_HEATER_ARMED && !heater_locked();
        st.time_ms = to_ms_since_boot(get_absolute_time());
        st.seq = ++seq;
        status_publish(&st);
        last_beat = xTaskGetTickCount();

        if (heating) {
            vTaskDelay(on);
            heater_off();
        }

        cycle++;
        vTaskDelayUntil(&last_wake, period);
    }
}

void control_start(void) {
    BaseType_t ok = xTaskCreate(control_task, "control", STACK_CONTROL, NULL, PRIO_CONTROL, NULL);
    configASSERT(ok == pdPASS);
}
