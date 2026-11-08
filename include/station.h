#ifndef STATION_H
#define STATION_H

#include <stdbool.h>
#include <stdint.h>

// Shared between the tasks.
//
// The control task builds a status struct every loop and overwrites a 1-slot
// queue with it, so there's always exactly one "latest" status. Everyone else
// just peeks at it. Commands from the UI and the sleep timers go the other
// direction through a normal queue.

typedef enum {
    ST_OFF = 0,
    ST_HEATING,
    ST_READY,
    ST_STANDBY,
    ST_NO_TIP,
    ST_FAULT,
} station_state_t;

typedef enum {
    FAULT_NONE = 0,
    FAULT_ADC,              // ADS1220 isn't answering
    FAULT_CJ_SENSOR,        // ADS1220 temp sensor not reading or out of range
    FAULT_TC_RANGE,         // thermocouple pegged or impossibly cold
    FAULT_OVERTEMP,
    FAULT_RUNAWAY,          // heating hard, temp not going up
    FAULT_CONTROL_STALL,    // control task stopped checking in
    FAULT_EXT_WDT,          // 74HC123 failed its self test
    FAULT_INTERLOCK,        // tried to measure before the heater was off and settled
    FAULT_WDT_RESET,        // the Pico's own watchdog reset us
    FAULT_COUNT
} fault_t;

typedef struct {
    station_state_t state;
    fault_t fault;
    float tip_c;
    float cj_c;             // cold junction
    float tc_uv;            // raw thermocouple voltage
    float duty_demand;      // what the PID asked for
    float duty_applied;     // what the heater actually got (0 when not armed)
    int16_t setpoint_c;
    uint8_t preset;
    bool tip_present;
    bool enabled;
    bool armed;
    uint32_t time_ms;
    uint32_t seq;
} station_status_t;

typedef enum {
    CMD_TOGGLE_ENABLE,
    CMD_SET_ENABLE,         // value is 0 or 1
    CMD_WAKE,               // user touched something
    CMD_STANDBY,            // sleep timer ran out
    CMD_CLEAR_FAULT,
} station_cmd_type_t;

typedef struct {
    station_cmd_type_t type;
    int32_t value;
} station_cmd_t;


// messaging (control.c)
void station_init(void);
void status_publish(const station_status_t *st);
bool status_get(station_status_t *out);
bool cmd_send(station_cmd_type_t type, int32_t value);
const char *state_name(station_state_t s);
const char *fault_name(fault_t f);

// tasks
void control_start(void);
void safety_start(void);
void ui_start(void);
void log_start(void);

// the safety task uses this to make sure the control loop is still running
bool control_alive(uint32_t timeout_ms);

// latched fault from the safety task. the control task reads it every loop
fault_t safety_fault(void);
void safety_clear(void);

#endif
