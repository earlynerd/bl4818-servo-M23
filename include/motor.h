#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

typedef enum {
    MOTOR_IDLE,
    MOTOR_RUN,
    MOTOR_FAULT
} motor_state_t;

typedef enum {
    FAULT_NONE          = 0,
    FAULT_OVERCURRENT   = 1,
    FAULT_HALL_INVALID  = 2
} fault_code_t;

void motor_init(void);

void motor_set_duty(int16_t duty);
void motor_set_torque_limit(uint16_t ma);

void motor_start(void);
void motor_stop(void);
void motor_clear_fault(void);

void motor_poll_fast(void);     /* Main loop: commutation on hall transitions */
void motor_tick_2khz(void);     /* 2kHz tick: current limit, duty update */

motor_state_t motor_get_state(void);
fault_code_t  motor_get_fault(void);
uint16_t motor_get_current(void);

#endif /* MOTOR_H */
