/*
 * Motor Control — Duty-cycle drive with overcurrent protection
 */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

typedef enum {
    MOTOR_IDLE,         /* Outputs disabled, coasting */
    MOTOR_RUN,          /* Commutating */
    MOTOR_FAULT         /* Overcurrent or hall fault */
} motor_state_t;

typedef enum {
    FAULT_NONE          = 0,
    FAULT_OVERCURRENT   = 1,
    FAULT_HALL_INVALID  = 2
} fault_code_t;

void motor_init(void);

void motor_set_duty(int16_t duty);       /* -PWM_MAX_DUTY to +PWM_MAX_DUTY */
void motor_set_torque_limit(uint16_t ma);

void motor_start(void);
void motor_stop(void);          /* Coast — all outputs off */
void motor_clear_fault(void);

void motor_poll_fast(void);     /* Call between 1 kHz ticks for commutation */
void motor_update(void);        /* 1 kHz control tick */

motor_state_t motor_get_state(void);
fault_code_t  motor_get_fault(void);
uint16_t motor_get_current(void);

#endif /* MOTOR_H */
