/*
 * C port of the Arduino PID Library v1.2.1 by Brett Beauregard (MIT License).
 * See pid.h for what changed.
 */
#include "pid.h"

static float clamp(float v, float lo, float hi) {
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
}

void pid_init(pid_ctrl_t *p, float kp, float ki, float kd, float sample_s,
              float out_min, float out_max) {
    p->kp = kp;
    p->ki = ki * sample_s;
    p->kd = kd / sample_s;
    p->out_min = out_min;
    p->out_max = out_max;
    p->output_sum = 0.0f;
    p->last_input = 0.0f;
}

void pid_initialize(pid_ctrl_t *p, float input, float output) {
    p->output_sum = clamp(output, p->out_min, p->out_max);
    p->last_input = input;
}

float pid_compute(pid_ctrl_t *p, float setpoint, float input) {
    float error = setpoint - input;
    float d_input = input - p->last_input;   // derivative on measurement, no kick on setpoint change

    float sum = clamp(p->output_sum + p->ki * error, p->out_min, p->out_max);
    float output = p->kp * error + sum - p->kd * d_input;

    // Changed from the original: it always keeps the new sum (clamped to the
    // output limits). For a soldering iron heating from cold that leaves the
    // integral parked at max the whole way up and it overshoots. Here the sum
    // only grows while the output isn't saturated, or when it would pull the
    // output back off the limit.
    if (output > p->out_max) {
        output = p->out_max;
        if (error < 0.0f) p->output_sum = sum;
    } else if (output < p->out_min) {
        output = p->out_min;
        if (error > 0.0f) p->output_sum = sum;
    } else {
        p->output_sum = sum;
    }

    p->last_input = input;
    return output;
}
