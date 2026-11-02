/*
 * C port of the Arduino PID Library v1.2.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 * https://github.com/br3ttb/Arduino-PID-Library
 * Licensed under the MIT License (see LICENSE in this folder)
 *
 * Changes from the original:
 *  - plain C struct + functions instead of a C++ class, floats instead of doubles
 *  - no millis()/SetSampleTime - the caller runs pid_compute() at a fixed rate
 *  - no manual/auto mode or P_ON_M, only direct acting with P on error
 *  - anti-windup uses conditional integration (see pid.c)
 */
#ifndef PID_H
#define PID_H

typedef struct {
    float kp, ki, kd;       // ki and kd are pre-scaled by the sample time, like the original
    float out_min, out_max;
    float output_sum;
    float last_input;
} pid_ctrl_t;

// SetTunings + SetOutputLimits rolled into one
void pid_init(pid_ctrl_t *p, float kp, float ki, float kd, float sample_s,
              float out_min, float out_max);

// Initialize() - call when switching the controller on so it picks up from
// the current input/output without a bump
void pid_initialize(pid_ctrl_t *p, float input, float output);

// Compute()
float pid_compute(pid_ctrl_t *p, float setpoint, float input);

#endif
