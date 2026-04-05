/*
 * PID Controller — Fixed-point (Q8) implementation
 *
 * All gains are in Q8 fixed-point: actual_gain = value / 256.
 */

#ifndef PID_H
#define PID_H

#include <stdint.h>

#define PID_SCALE 256  /* Q8 */

typedef struct {
    int16_t kp;
    int16_t ki;
    int16_t kd;
    int32_t integral;
    int32_t int_max;
    int16_t prev_error;
    int16_t out_min;
    int16_t out_max;
} pid_t;

void pid_init(pid_t *pid, int16_t kp, int16_t ki, int16_t kd,
              int16_t out_min, int16_t out_max);
int16_t pid_update(pid_t *pid, int16_t setpoint, int16_t measurement);
void pid_reset(pid_t *pid);

#endif /* PID_H */
