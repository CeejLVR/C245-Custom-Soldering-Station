#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "ssd1306.h"    // pico-ssd1306, see lib/ssd1306

#include "config.h"
#include "station.h"
#include "board.h"
#include "storage.h"

// Controls:
//   encoder    setpoint up/down 5C
//   PSH        heater on/off, or clear the fault if there is one
//   CON        next preset
//   hold CON   save the current setpoint into this preset
//   BAK        flip between the big number and the graph

#define OLED_ADDR       0x3C    // 0.96" boards are almost always SSD1306. if the image is shifted it's probably an SH1106
#define OLED_W          128
#define OLED_H          64
#define CHAR_W          6       // 5px font + 1px gap

#define GRAPH_T_MAX_C   500
#define GRAPH_TOP       11

static ssd1306_t oled;

static enum { VIEW_NUMBER, VIEW_GRAPH } view = VIEW_NUMBER;

static int16_t history[OLED_W];     // one temp sample per pixel column
static int hist_len, hist_head;

static char popup[22];
static TickType_t popup_until;


// ---------- display glue ----------
// The library talks to the I2C port directly, so the bus lock is handled here.

static bool oled_init(void) {
    if (!bus_lock(pdMS_TO_TICKS(200)))
        return false;
    bool ok = ssd1306_init(&oled, OLED_W, OLED_H, OLED_ADDR, bus_port());
    bus_unlock();
    return ok;
}

// Push the frame one page (8 rows) at a time and let go of the bus in
// between. A full frame is ~23ms, and this keeps the bus free for anything
// else that ends up on it.
static bool oled_flush(void) {
    for (uint8_t page = 0; page < OLED_H / 8; page++) {
        if (!bus_lock(pdMS_TO_TICKS(50)))
            return false;
        bool ok = ssd1306_show_page(&oled, page);
        bus_unlock();
        if (!ok)
            return false;
    }
    return true;
}

static int text(int x, int y, const char *s, int scale) {
    ssd1306_draw_string(&oled, x, y, scale, s);
    return x + (int)strlen(s) * CHAR_W * scale;
}

static int text_width(const char *s, int scale) {
    return (int)strlen(s) * CHAR_W * scale;
}

static void text_right(int y, const char *s) {
    text(OLED_W - text_width(s, 1), y, s, 1);
}


// ---------- buttons ----------

static void show_popup(const char *msg) {
    snprintf(popup, sizeof popup, "%s", msg);
    popup_until = xTaskGetTickCount() + pdMS_TO_TICKS(1200);
}

static void handle_event(input_event_t ev, const station_status_t *st) {
    settings_t s;
    char buf[22];
    storage_get(&s);

    switch (ev) {
    case EV_ENC_CW:
    case EV_ENC_CCW: {
        int sp = s.setpoint_c + (ev == EV_ENC_CW ? SETPOINT_STEP_C : -SETPOINT_STEP_C);
        if (sp < SETPOINT_MIN_C) sp = SETPOINT_MIN_C;
        if (sp > SETPOINT_MAX_C) sp = SETPOINT_MAX_C;
        s.setpoint_c = (int16_t)sp;
        storage_update(&s);
        break;
    }
    case EV_PSH:
        cmd_send(st->state == ST_FAULT ? CMD_CLEAR_FAULT : CMD_TOGGLE_ENABLE, 0);
        break;
    case EV_CON:
        s.active_preset = (uint8_t)((s.active_preset + 1) % NUM_PRESETS);
        s.setpoint_c = s.preset_c[s.active_preset];
        storage_update(&s);
        snprintf(buf, sizeof buf, "Preset %d: %dC", s.active_preset + 1, s.setpoint_c);
        show_popup(buf);
        break;
    case EV_CON_LONG:
        s.preset_c[s.active_preset] = s.setpoint_c;
        storage_update(&s);
        snprintf(buf, sizeof buf, "Saved P%d = %dC", s.active_preset + 1, s.setpoint_c);
        show_popup(buf);
        break;
    case EV_BAK:
        view = (view == VIEW_NUMBER) ? VIEW_GRAPH : VIEW_NUMBER;
        break;
    case EV_PSH_LONG:
    case EV_BAK_LONG:
        break;  // TODO: calibration menu
    }

    cmd_send(CMD_WAKE, 0);  // any input resets the sleep timer
}


// ---------- screens ----------

// top row: state on the left, preset on the right ("DRY" = heater not armed)
static void draw_header(const station_status_t *st) {
    char buf[22];
    if (st->state == ST_FAULT)
        snprintf(buf, sizeof buf, "FAULT %s", fault_name(st->fault));
    else
        snprintf(buf, sizeof buf, "%s", state_name(st->state));
    text(0, 0, buf, 1);

    snprintf(buf, sizeof buf, "%sP%d", st->armed ? "" : "DRY ", st->preset + 1);
    text_right(0, buf);
}

static void draw_number_view(const station_status_t *st) {
    char buf[22];

    if (st->tip_present)
        snprintf(buf, sizeof buf, "%3d", (int)(st->tip_c + 0.5f));
    else
        snprintf(buf, sizeof buf, "---");
    int x = text(8, 14, buf, 3);
    text(x + 2, 14, "C", 2);

    snprintf(buf, sizeof buf, "SET %dC", st->setpoint_c);
    text(0, 42, buf, 1);
    snprintf(buf, sizeof buf, "CJ %dC", (int)(st->cj_c + 0.5f));
    text_right(42, buf);

    // duty bar along the bottom
    int pct = (int)(st->duty_demand * 100.0f + 0.5f);
    ssd1306_draw_empty_square(&oled, 0, 54, 99, 8);     // the library's outline is w+1 x h+1
    ssd1306_draw_square(&oled, 1, 55, pct * 98 / 100, 7);
    snprintf(buf, sizeof buf, "%d%%", pct);
    text_right(55, buf);
}

static int temp_to_y(int t) {
    if (t < 0) t = 0;
    if (t > GRAPH_T_MAX_C) t = GRAPH_T_MAX_C;
    return (OLED_H - 1) - t * (OLED_H - 1 - GRAPH_TOP) / GRAPH_T_MAX_C;
}

static void draw_graph_view(const station_status_t *st) {
    // dotted setpoint line
    int ysp = temp_to_y(st->setpoint_c);
    for (int x = 0; x < OLED_W; x += 4)
        ssd1306_draw_pixel(&oled, x, ysp);

    // oldest sample on the left, newest on the right. Each column gets a
    // vertical run from the previous sample so steep heat-ups don't come out
    // as scattered dots.
    int first = (hist_head - hist_len + OLED_W) % OLED_W;
    int x0 = OLED_W - hist_len;
    int prev_y = -1;
    for (int i = 0; i < hist_len; i++) {
        int y = temp_to_y(history[(first + i) % OLED_W]);
        if (prev_y >= 0)
            ssd1306_draw_line(&oled, x0 + i, prev_y, x0 + i, y);
        else
            ssd1306_draw_pixel(&oled, x0 + i, y);
        prev_y = y;
    }

    char buf[22];
    snprintf(buf, sizeof buf, "%dC", (int)(st->tip_c + 0.5f));
    text(0, GRAPH_TOP + 1, buf, 1);
}

static void draw_popup(void) {
    ssd1306_draw_square(&oled, 0, 30, OLED_W, 12);
    ssd1306_clear_square(&oled, 1, 31, OLED_W - 2, 10);
    text(3, 32, popup, 1);
}


// ---------- the task ----------

static void ui_task(void *arg) {
    bool oled_ok = oled_init();
    if (oled_ok) {
        ssd1306_clear(&oled);
        oled_ok = oled_flush();
    }
    TickType_t next_sample = xTaskGetTickCount();
    station_status_t st;

    while (1) {
        st = (station_status_t){0};
        status_get(&st);

        // wait up to one frame for input, then drain whatever else came in
        input_event_t ev;
        TickType_t wait = pdMS_TO_TICKS(UI_FRAME_MS);
        while (input_get(&ev, wait)) {
            handle_event(ev, &st);
            wait = 0;
        }
        status_get(&st);

        if ((int32_t)(xTaskGetTickCount() - next_sample) >= 0) {
            next_sample += pdMS_TO_TICKS(GRAPH_SAMPLE_MS);
            history[hist_head] = (int16_t)st.tip_c;
            hist_head = (hist_head + 1) % OLED_W;
            if (hist_len < OLED_W)
                hist_len++;
        }

        // if the screen wasn't found, keep trying every frame
        if (!oled_ok) {
            oled_ok = oled_init();
            if (!oled_ok)
                continue;
        }

        ssd1306_clear(&oled);
        draw_header(&st);
        if (view == VIEW_NUMBER)
            draw_number_view(&st);
        else
            draw_graph_view(&st);
        if ((int32_t)(popup_until - xTaskGetTickCount()) > 0)
            draw_popup();
        oled_ok = oled_flush();
    }
}

void ui_start(void) {
    BaseType_t ok = xTaskCreate(ui_task, "ui", STACK_UI, NULL, PRIO_UI, NULL);
    configASSERT(ok == pdPASS);
}
