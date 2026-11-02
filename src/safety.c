#include "FreeRTOS.h"
#include "task.h"

#include "hardware/watchdog.h"
#include "pico/time.h"

#include "config.h"
#include "station.h"
#include "heater.h"

// Highest priority task, every 25ms. Checks that:
//   - the control task is still checking in
//   - the tip isn't over temp
//   - the heater isn't pushing hard with no temperature rise (runaway)
//   - the 74HC123 output is actually high while we're kicking it
// The 74HC123 only gets kicked while all of that is fine. If we stop kicking,
// the hardware cuts the heater by itself ~100ms later. This task also feeds
// the Pico's own watchdog, so if it hangs the whole chip resets.

static volatile fault_t latched = FAULT_NONE;
static volatile bool clear_requested;

fault_t safety_fault(void) {
    return latched;
}

void safety_clear(void) {
    clear_requested = true;
}

static inline uint32_t millis(void) {
    return to_ms_since_boot(get_absolute_time());
}

static void sleep_and_feed(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
    watchdog_update();
}

// Kick once, the output should go high. Wait, and it should drop again on
// its own. The heater stays locked out until this passes.
static bool hwdog_selftest(void) {
    // the 74HC123 can fire a pulse on power-up (datasheet 11.2), let that finish
    if (hwdog_running()) sleep_and_feed(EXT_WDT_SELFTEST_MS);
    if (hwdog_running()) return false;      // stuck high

    hwdog_kick();
    busy_wait_us_32(50);
    if (!hwdog_running()) return false;     // never triggered

    sleep_and_feed(EXT_WDT_SELFTEST_MS);
    return !hwdog_running();                // should have timed out by now
}

static void safety_task(void *arg) {
    const uint32_t boot_ms = millis();

    hwdog_init();
    watchdog_enable(INTERNAL_WDT_MS, true);     // true = pauses while debugging

    bool selftest_ok = hwdog_selftest();
    if (!selftest_ok)
        latched = FAULT_EXT_WDT;

    uint32_t kicks = 0;
    bool watching_rise = false;
    uint32_t rise_start_ms = 0;
    float rise_start_c = 0.0f;
    station_status_t st;

    TickType_t last_wake = xTaskGetTickCount();
    while (1) {
        uint32_t now = millis();

        if (clear_requested) {
            clear_requested = false;
            if (latched == FAULT_EXT_WDT || !selftest_ok)
                selftest_ok = hwdog_selftest();
            latched = selftest_ok ? FAULT_NONE : FAULT_EXT_WDT;
            kicks = 0;
            watching_rise = false;
        }

        fault_t f = FAULT_NONE;

        // give the control task a second to get going after boot
        if (now - boot_ms > 1000 && !control_alive(CONTROL_HEARTBEAT_TIMEOUT_MS))
            f = FAULT_CONTROL_STALL;

        if (status_get(&st) && (now - st.time_ms) < STATUS_STALE_MS) {
            if (st.tip_present && st.tip_c > TEMP_ABS_MAX_C)
                f = FAULT_OVERTEMP;

            // Runaway: while the heater is above RUNAWAY_DUTY and still well
            // below setpoint, the temp has to climb RUNAWAY_MIN_RISE_C every 5s.
            bool pushing = st.duty_applied >= RUNAWAY_DUTY
                        && st.tip_c < (float)st.setpoint_c - RUNAWAY_NEAR_SETPOINT_C;
            if (!pushing) {
                watching_rise = false;
            } else if (!watching_rise) {
                watching_rise = true;
                rise_start_ms = now;
                rise_start_c = st.tip_c;
            } else if (now - rise_start_ms >= RUNAWAY_WINDOW_MS) {
                if (st.tip_c - rise_start_c < RUNAWAY_MIN_RISE_C)
                    f = FAULT_RUNAWAY;
                rise_start_ms = now;
                rise_start_c = st.tip_c;
            }
        }

        if (selftest_ok && kicks > 2 && !hwdog_running())
            f = FAULT_EXT_WDT;

        if (f != FAULT_NONE && latched == FAULT_NONE) {
            latched = f;
            heater_kill();
        }

        if (latched != FAULT_NONE || !selftest_ok) {
            // stop kicking so the hardware cuts the heater too
            heater_lockout(true);
            kicks = 0;
        } else {
            hwdog_kick();
            if (++kicks > 2)
                heater_lockout(false);
        }

        watchdog_update();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SAFETY_PERIOD_MS));
    }
}

void safety_start(void) {
    BaseType_t ok = xTaskCreate(safety_task, "safety", STACK_SAFETY, NULL, PRIO_SAFETY, NULL);
    configASSERT(ok == pdPASS);
}
