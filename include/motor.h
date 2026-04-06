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
    CTRL_VELOCITY = 1,
    CTRL_POSITION = 2
} ctrl_mode_t;

void motor_init(void);

/* Mode selection */
void motor_set_mode(ctrl_mode_t mode);
ctrl_mode_t motor_get_mode(void);

/* Duty mode target (signed, ±PWM_MAX_DUTY) */
void motor_set_duty(int32_t duty);

/* Velocity mode target (signed, RPM) */
void motor_set_velocity(int32_t rpm);

/* Position mode target (encoder counts, continuous) */
void motor_set_position(int32_t counts);

/* Torque envelope (applies in all modes) */
void motor_set_torque_limit(uint32_t ma);

/* PID tuning (Q8 gains) */
void motor_set_vel_pid(int32_t kp, int32_t ki, int32_t kd);
void motor_set_vel_ff(int32_t gain);  /* Q8 feedforward: duty per RPM */
void motor_set_pos_pid(int32_t kp, int32_t ki, int32_t kd);

void motor_start(void);
void motor_stop(void);
void motor_coast(void);        /* Float phases, stay MOTOR_RUN (for strike coast) */
void motor_arm_coast(int32_t threshold, int8_t direction);  /* Auto-coast at position */
void motor_disarm_coast(void);
uint8_t motor_is_coasting(void);
void motor_clear_fault(void);

void motor_poll_fast(void);     /* Main loop: commutation on hall transitions */
void motor_tick_2khz(void);     /* 2kHz tick: control loop */

motor_state_t motor_get_state(void);
fault_code_t  motor_get_fault(void);
uint32_t motor_get_current(void);
int32_t  motor_get_velocity(void);  /* Measured RPM (signed) */
int32_t  motor_get_target(void);    /* Active target (duty or RPM depending on mode) */

#endif /* MOTOR_H */
