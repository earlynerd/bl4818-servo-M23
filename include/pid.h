/*
 * PID Controller — Fixed-point (Q8) implementation
 *
 * All gains are in Q8 fixed-point: actual_gain = value / 256.
 */

#ifndef PID_H
#define PID_H

#include <stdint.h>

#define PID_SCALE 256      /* Q8 */
#define PID_SCALE_SHIFT 8

typedef struct {
    int32_t kp;
    int32_t ki;
    int32_t kd;
    int32_t kf;
    int64_t integral;
    int64_t int_max;
    int32_t prev_meas;
    int32_t prev_d_term;
    int32_t out_min;
    int32_t out_max;
} pid_t;

void pid_init(pid_t *pid, int32_t kp, int32_t ki, int32_t kd, int32_t kf,
              int32_t out_min, int32_t out_max);
void pid_set_gains(pid_t *pid, int32_t kp, int32_t ki, int32_t kd, int32_t kf,
                   int32_t out_min, int32_t out_max);
int32_t pid_update(pid_t *pid, int32_t setpoint, int32_t measurement);
void pid_reset(pid_t *pid);
void pid_preload(pid_t *pid, int32_t current_output,
                 int32_t setpoint, int32_t current_meas);

#endif /* PID_H */
