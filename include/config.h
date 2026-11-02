#ifndef CONFIG_H
#define CONFIG_H

// Every tunable number lives in here. If you're changing a timing or a limit,
// change it here and not in the .c files.

// Master heater switch. Leave this at 0 until the measurement timing has been
// checked on the scope - with it at 0 the heater pin physically can't go high,
// so the sensors, UI and watchdog can all be tested safely.
#define CFG_HEATER_ARMED            0

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
#define HEATER_OFF_SETTLE_US        1000    // wait after heater off before closing relays (guess - check on scope)
#define RELAY_ON_US                 2000    // AQV210E turn-on, 2ms max
#define RELAY_OFF_US                1000    // AQV210E turn-off, 1ms max
#define INPUT_RC_SETTLE_US          300     // R7/R8 + C8 input filter
#define ADC_TIMEOUT_MS              10      // one conversion at 330 SPS is ~3.1ms

// Tip detection uses the ADS1220 burnout current sources. With no tip in, the
// reading should shoot up to full scale. Not tested on real hardware yet.
#define TIP_CHECK_EVERY_N           10      // every 10 loops = once a second
#define TIP_CHECK_SETTLE_MS         8       // C8 charging from 10uA takes a bit
#define TIP_CHECK_OPEN_UV           400000.0f


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
#define CJ_MAX_FAILS                20
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

// Thermocouple defaults. Saved settings override these once they exist.
// ~19-22 uV/K according to the eevblog thread, still needs a real calibration.
#define TC_SENS_UV_PER_C            21.0f
#define TC_OFFSET_UV                0.0f
#define TC_POLARITY                 (+1)    // flip to -1 if C1 and C3 end up swapped

// starting PID gains, nowhere near tuned yet
#define PID_KP                      0.030f
#define PID_KI                      0.004f
#define PID_KD                      0.000f


#define I2C_BAUD_HZ                 400000
#define ADC_SPI_BAUD_HZ             2000000

// encoder / buttons
#define ENCODER_TRANSITIONS_PER_DETENT 4    // EC11 is normally 4, try 2 if it skips clicks
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
