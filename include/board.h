#ifndef BOARD_H
#define BOARD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "hardware/i2c.h"
#include "FreeRTOS.h"

// I2C0 carries the OLED. Take the lock first.
// bus_write/bus_read
void bus_init(void);
bool bus_lock(TickType_t wait);
void bus_unlock(void);
i2c_inst_t *bus_port(void);     // for the OLED library, which talks to the port directly
bool bus_write(uint8_t addr, const uint8_t *data, size_t len, bool nostop);
bool bus_read(uint8_t addr, uint8_t *data, size_t len);

// encoder + the three buttons
typedef enum {
    EV_ENC_CW,
    EV_ENC_CCW,
    EV_PSH, EV_PSH_LONG,    // encoder push
    EV_CON, EV_CON_LONG,
    EV_BAK, EV_BAK_LONG,
} input_event_t;

void input_init(void);      // before the scheduler starts
bool input_get(input_event_t *ev, TickType_t wait);

#endif
