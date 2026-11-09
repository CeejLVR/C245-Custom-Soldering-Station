#ifndef CONFIG_H
#define CONFIG_H

// Every tunable number lives in here. If you're changing a timing or a limit,
// change it here and not in the .c files.

// Master heater switch. Leave this at 0 until the measurement timing has been
// checked on the scope - with it at 0 the heater pin physically can't go high,
// so the sensors, UI and watchdog can all be tested safely.
#define CFG_HEATER_ARMED            1

// 24V / 2.5 ohm is about 9.6A and the fuse is 8A, so cap the duty for now
#define HEATER_MAX_DUTY             0.50f


// task priorities (higher number wins)
#define PRIO_SAFETY                 5
#define PRIO_CONTROL                4
#define PRIO_UI                     1
#define PRIO_LOG                    1

// stack sizes are in words, not bytes
#define STACK_SAFETY                512
#define STACK_CONTROL               768
#define STACK_UI                    768
#define STACK_LOG                   768


// control loop timing
#define CONTROL_PERIOD_MS           100
#define MEAS_WINDOW_RESERVE_MS      12      // heater must be off at least this long each period
// Wait after heater off before reading the TC. 1ms for Q2 to turn off (guess -
// check on scope) + 1.3ms for R7/R8 (4.7k each) and C8 (10nF) to settle from
// the return wire drop down to 1uV. Rounded up.
#define HEATER_OFF_SETTLE_US        2500
#define ADC_TIMEOUT_MS              10      // one conversion at 330 SPS is ~3.1ms

// Tip detection uses ADS1220 IDAC1, 10uA into AIN1. With a tip in, the current
// goes through R8 and the TC to ground, so the reading is about -47mV. With no
// tip, AIN1 is floating and the reading lands somewhere else. The reading has
// to land between the two limits, so a dead current source reads as no tip too.
#define TIP_CHECK_EVERY_N           10      // every 10 loops = once a second
#define TIP_CHECK_SETTLE_MS         5       // C8 charges fast, but let the conversion from the register write finish
#define TIP_CHECK_DRAIN_US          1500    // let C8 drain after the check
#define TIP_CHECK_MIN_UV            35000.0f    // measured about 46.7mV with a tip in
#define TIP_CHECK_MAX_UV            65000.0f    // leaves room for IDAC tolerance and a hot tip

// A real TC reading can't get near this (about 700C above the cold junction).
// Anything past it means the input is floating, so treat it as no tip.
#define TC_MAX_PLAUSIBLE_UV         15000.0f


// safety limits
#define SAFETY_PERIOD_MS            25      // 74HC123 wants >10.5ms between kicks, times out at ~110ms
#define EXT_WDT_SELFTEST_MS         250
#define INTERNAL_WDT_MS             500
#define CONTROL_HEARTBEAT_TIMEOUT_MS 300
#define STATUS_STALE_MS             500
#define TEMP_ABS_MAX_C              480.0f
#define TEMP_MIN_PLAUSIBLE_C        (-20.0f) // reading this cold probably means the TC is backwards
#define CJ_MIN_C                    (-10.0f)
#define CJ_MAX_C                    85.0f
#define CJ_MAX_FAILS                3       // reads happen once a second now
#define CJ_READ_EVERY_N             10      // every 10 loops = once a second
#define CJ_READ_PHASE               5       // half a second away from the tip check
#define ADC_MAX_FAILS               3
#define ADC_SATURATION_CODE         0x7F0000

// Runaway check: heater pushing hard but the temp isn't climbing. Usually a
// broken thermocouple or the tip not actually being in the handle.
#define RUNAWAY_DUTY                0.40f
#define RUNAWAY_WINDOW_MS           5000
#define RUNAWAY_MIN_RISE_C          10.0f
#define RUNAWAY_NEAR_SETPOINT_C     15.0f   // skip the check near setpoint, big joints can stall it there


// temperatures in C
#define SETPOINT_MIN_C              100
#define SETPOINT_MAX_C              450
#define SETPOINT_DEFAULT_C          320
#define SETPOINT_STEP_C             5
#define READY_BAND_C                5.0f
#define STANDBY_TEMP_C              150
#define SLEEP_TIMEOUT_S             600     // 10 min with no input -> standby
#define AUTO_OFF_TIMEOUT_S          1800    // 30 min in standby -> off

// Stand detection. Read every loop while the heater is off, because the tip
// rises a little during heater pulses (drop along the return wire). The input
// has to read the same this many loops in a row (100ms each) before it counts.
#define STAND_DETECT_ENABLED        1
#define STAND_DEBOUNCE_N            3

// Thermocouple defaults. Saved settings override these once they exist.
// ~19-22 uV/K according to the eevblog thread, still needs a real calibration.
#define TC_SENS_UV_PER_C            21.0f
#define TC_OFFSET_UV                0.0f
#define TC_POLARITY                 (-1)    // green is TC+ on this cartridge, red is grounded

// starting PID gains, nowhere near tuned yet
#define PID_KP                      0.030f
#define PID_KI                      0.004f
#define PID_KD                      0.000f


#define I2C_BAUD_HZ                 400000
#define ADC_SPI_BAUD_HZ             2000000

// encoder / buttons
#define ENCODER_TRANSITIONS_PER_DETENT 4    // try 2 if it skips clicks
#define ENCODER_REVERSE             0
#define BUTTON_POLL_MS              5
#define BUTTON_DEBOUNCE_SAMPLES     4
#define BUTTON_LONG_PRESS_MS        800

// screen + logging
#define UI_FRAME_MS                 100
#define GRAPH_SAMPLE_MS             500     // 128 points across the screen ~= 1 minute of history
#define LOG_PERIOD_MS               200
#define SETTINGS_SAVE_DELAY_MS      2000

#endif
