#include <stdio.h>

#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "station.h"
#include "heater.h"
#include "board.h"
#include "storage.h"

int main(void) {
    heater_init();      // before anything else so GP15 is low from the start
    stdio_init_all();

    bus_init();
    station_init();
    storage_init();
    input_init();

    safety_start();
    control_start();
    ui_start();
    log_start();

    vTaskStartScheduler();

    // only lands here if the scheduler couldn't start
    while (1) heater_kill();
}


// Kernel hooks. Whatever goes wrong, kill the heater first and then hang -
// the watchdog will reset the chip.

void vAssertCalled(const char *file, int line) {
    (void)file;
    (void)line;
    taskDISABLE_INTERRUPTS();
    heater_kill();
    while (1) {}
}

void vApplicationMallocFailedHook(void) {
    heater_kill();
    configASSERT(0);
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *name) {
    (void)task;
    (void)name;
    heater_kill();
    configASSERT(0);
}


// USB serial log for PID tuning. Prints one CSV line per new status, paste
// it from the serial monitor into a spreadsheet and graph it.
static void log_task(void *arg) {
    uint32_t lines = 0;
    uint32_t last_seq = 0;
    station_status_t st;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(LOG_PERIOD_MS));
        if (!status_get(&st) || st.seq == last_seq)
            continue;
        last_seq = st.seq;

        // reprint the header every so often so it's there if you connect late
        if (lines++ % 50 == 0)
            printf("# ms,state,tip_c,cj_c,tc_uv,set_c,demand,applied,tip,armed,fault\n");

        printf("%lu,%s,%.1f,%.2f,%.1f,%d,%.3f,%.3f,%d,%d,%s\n",
               (unsigned long)st.time_ms, state_name(st.state),
               st.tip_c, st.cj_c, st.tc_uv, st.setpoint_c,
               st.duty_demand, st.duty_applied, st.tip_present, st.armed,
               fault_name(st.fault));
    }
}

void log_start(void) {
    BaseType_t ok = xTaskCreate(log_task, "log", STACK_LOG, NULL, PRIO_LOG, NULL);
    configASSERT(ok == pdPASS);
}
