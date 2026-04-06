/*
 * PID Controller — Fixed-point (Q8) implementation for M2003
 *
 * - Derivative on measurement (not error) to avoid derivative kick
 *   on setpoint changes
 * - Conditional integration anti-windup: integral only updates when
 *   output is unsaturated, or when error would help desaturate
 */

#include "pid.h"

void pid_init(pid_t *pid, int32_t kp, int32_t ki, int32_t kd,
              int32_t out_min, int32_t out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0;
    pid->prev_meas = 0;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->int_max = out_max * PID_SCALE;
}

int32_t pid_update(pid_t *pid, int32_t error, int32_t measurement)
{
    /* Proportional */
    int32_t p_term = pid->kp * error;

    /* Tentative integral update */
    int32_t new_integral = pid->integral + pid->ki * error;
    if (new_integral > pid->int_max)
        new_integral = pid->int_max;
    else if (new_integral < -pid->int_max)
        new_integral = -pid->int_max;

    /* Derivative on measurement (negated: rising measurement = falling error) */
    int32_t d_term = -pid->kd * (measurement - pid->prev_meas);
    pid->prev_meas = measurement;

    /* Sum and scale back from Q8 */
    int32_t output = (p_term + new_integral + d_term) / PID_SCALE;

    /* Clamp output + conditional integration anti-windup */
    if (output >= pid->out_max) {
        output = pid->out_max;
        if (error <= 0)
            pid->integral = new_integral;
    } else if (output <= pid->out_min) {
        output = pid->out_min;
        if (error >= 0)
            pid->integral = new_integral;
    } else {
        pid->integral = new_integral;
    }

    return output;
}

void pid_reset(pid_t *pid)
{
    pid->integral = 0;
    pid->prev_meas = 0;
}
