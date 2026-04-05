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

typedef enum {
    CTRL_DUTY     = 0,
    CTRL_VELOCITY = 1
} ctrl_mode_t;

void motor_init(void);

/* Mode selection */
void motor_set_mode(ctrl_mode_t mode);
ctrl_mode_t motor_get_mode(void);

/* Duty mode target (signed, ±PWM_MAX_DUTY) */
void motor_set_duty(int16_t duty);

/* Velocity mode target (signed, RPM) */
void motor_set_velocity(int16_t rpm);

/* Torque envelope (applies in all modes) */
void motor_set_torque_limit(uint16_t ma);

/* PID tuning (Q8 gains) */
void motor_set_vel_pid(int16_t kp, int16_t ki, int16_t kd);
void motor_set_vel_ff(int16_t gain);  /* Q8 feedforward: duty per RPM */

void motor_start(void);
void motor_stop(void);
void motor_clear_fault(void);

void motor_poll_fast(void);     /* Main loop: commutation on hall transitions */
void motor_tick_2khz(void);     /* 2kHz tick: control loop */

motor_state_t motor_get_state(void);
fault_code_t  motor_get_fault(void);
uint16_t motor_get_current(void);
int16_t  motor_get_velocity(void);  /* Measured RPM (signed) */
int16_t  motor_get_target(void);    /* Active target (duty or RPM depending on mode) */

#endif /* MOTOR_H */
