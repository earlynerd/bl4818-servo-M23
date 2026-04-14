/*
 * PID Controller — Fixed-point (Q8) implementation for M2003
 *
 * - Derivative on measurement (not error) to avoid derivative kick
 *   on setpoint changes
 * - Conditional integration anti-windup: integral only updates when
 *   output is unsaturated, or when error would help desaturate
 */

#include "pid.h"
#include "m2003_config.h"

void pid_init(pid_t *pid, int32_t kp, int32_t ki, int32_t kd, int32_t kf,
              int32_t out_min, int32_t out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->kf = kf;
    pid->integral = 0;
    pid->prev_meas = 0;
    pid->prev_d_term = 0;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->int_max = out_max * PID_SCALE;
}

void pid_set_gains(pid_t *pid, int32_t kp, int32_t ki, int32_t kd, int32_t kf,
                   int32_t out_min, int32_t out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->kf = kf;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->int_max = out_max * PID_SCALE;

    /* Clamp existing integral to new limits */
    if (pid->integral > pid->int_max)
        pid->integral = pid->int_max;
    else if (pid->integral < -pid->int_max)
        pid->integral = -pid->int_max;
}

int32_t pid_update(pid_t *pid, int32_t setpoint, int32_t measurement)
{
    int32_t error = setpoint - measurement;

    /* Proportional */
    int32_t p_term = pid->kp * error;

    /* Tentative integral update */
    int64_t new_integral = pid->integral + pid->ki * error;
    if (new_integral > pid->int_max)
        new_integral = pid->int_max;
    else if (new_integral < -pid->int_max)
        new_integral = -pid->int_max;

    /* Derivative on measurement (negated: rising measurement = falling error)
     * Filtered with a simple first-order IIR.
     * The internal state is kept in Q8 (* PID_SCALE) to eliminate the integer
     * truncation deadband that causes the D-term to get stuck. */
    int32_t raw_d_term = -pid->kd * (measurement - pid->prev_meas);
    int32_t raw_d_q8 = raw_d_term * PID_SCALE;
    pid->prev_d_term += (raw_d_q8 - pid->prev_d_term) / (1 << PID_D_FILTER_SHIFT);
    pid->prev_meas = measurement;
    
    int32_t d_term = pid->prev_d_term / PID_SCALE;

    /* Feedforward */
    int32_t ff_term = pid->kf * setpoint;

    /* Sum and scale back from Q8 */
    int32_t output = (p_term + new_integral + d_term + ff_term) / PID_SCALE;

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
    pid->prev_d_term = 0;
}

void pid_set_int_max(pid_t *pid, int64_t int_max)
{
    pid->int_max = int_max;

    if (pid->integral > int_max)
        pid->integral = int_max;
    else if (pid->integral < -int_max)
        pid->integral = -int_max;
}

void pid_set_prev_meas(pid_t *pid, int32_t meas)
{
    pid->prev_meas = meas;
}

void pid_preload(pid_t *pid, int32_t current_output,
                 int32_t setpoint, int32_t current_meas)
{
    int32_t error = setpoint - current_meas;
    int64_t integral = (int64_t)current_output * PID_SCALE;

    /* Seed the integrator so the next update reproduces current_output
     * with the live proportional/feedforward terms already accounted for.
     * D is intentionally reset by aligning prev_meas with current_meas. */
    integral -= (int64_t)pid->kp * error;
    integral -= (int64_t)pid->kf * setpoint;

    if (integral > pid->int_max)
        integral = pid->int_max;
    else if (integral < -pid->int_max)
        integral = -pid->int_max;

    pid->integral = integral;
    pid->prev_meas = current_meas;
    pid->prev_d_term = 0;
}
