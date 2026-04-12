/*
 * Motor Control for Nuvoton M2003
 *
 * Supports three control modes:
 *   CTRL_DUTY     — open-loop signed duty cycle (torque-clamped)
 *   CTRL_VELOCITY — closed-loop RPM: velocity PID → current PI → duty
 *   CTRL_POSITION — cascaded: position PID → velocity PID → current PI → duty
 *
 * Loop rates are configured in m2003_config.h (_LOOP_HZ defines).
 * Control tick runs from SysTick at CONTROL_LOOP_HZ; inner loops use dividers.
 * ADC ISR oversamples at PWM rate and hands the control loop an averaged window.
 */

#include <stdint.h>
#include "m2003_config.h"
#include "M2003.h"
#include "motor.h"
#include "pid.h"
#include "app_pwm.h"
#include "app_adc.h"
#include "commutation.h"
#include "hall.h"
#include "encoder.h"

/* ── State ────────────────────────────────────────────────────────────── */
static motor_state_t state;
static fault_code_t  fault;
static ctrl_mode_t   ctrl_mode;

static int32_t       target_duty;
static int32_t       target_velocity;   /* signed RPM (set directly or by pos PID) */
static int32_t       target_position;   /* encoder counts, continuous */
static int32_t       target_current;    /* signed mA (CTRL_TORQUE mode) */
static int32_t       measured_velocity;  /* signed RPM */
static int8_t        drive_dir;          /* +1 / -1, commutation direction */
static uint32_t      torque_limit_ma;
static uint32_t      current_ma;
static int32_t       current_filt_q8;    /* filtered current estimate in Q8 mA */
static uint8_t       current_filt_initialized;
static uint16_t      overcurrent_ticks;

static int32_t       prev_enc_position;
static int32_t       vel_filt_q8;       /* IIR-filtered velocity in Q8 */
static uint8_t       vel_initialized;

static int32_t       last_applied_duty;  /* for bumpless transfer on mode switch */
static uint8_t       coasting;           /* 1 = phases floating, skip duty dispatch */
static int32_t       coast_trip_pos;     /* auto-coast position threshold */
static int8_t        coast_trip_dir;     /* +1: coast when pos >= trip, -1: when <= */
static uint8_t       coast_armed;        /* 1 = watching for coast trigger at 5 kHz */
static pid_t         vel_pid;
static pid_t         pos_pid;
static pid_t         cur_pid;
static int32_t       current_setpoint;   /* mA, output of velocity PID for current loop */
static uint8_t       cur_div;            /* divider counter for current loop */
static uint8_t       vel_div;            /* divider counter for velocity loop */
static uint8_t       pos_div;            /* divider counter for position loop */

/*
 * Encoder-based velocity:
 *   RPM = delta_counts * 60 * CONTROL_LOOP_HZ / ENCODER_COUNTS_PER_REV
 */
#define VEL_CONV_NUM  (60 * CONTROL_LOOP_HZ)

/* ── Helpers ─────────────────────────────────────────────────────────── */

static int32_t estimate_velocity(void)
{
    int32_t pos = encoder_get_position();
    int32_t delta = pos - prev_enc_position;
    prev_enc_position = pos;

    if (!vel_initialized) {
        vel_initialized = 1;
        return 0;
    }

    /* Negate: encoder counts positive in opposite direction to commutation forward */
    int32_t raw_rpm = -(delta * VEL_CONV_NUM) / ENCODER_COUNTS_PER_REV;

    /* First-order IIR in Q8 for sub-RPM precision */
    int32_t raw_q8 = raw_rpm << PID_SCALE_SHIFT;
    /* Use division to avoid asymmetric deadband and -1 RPM stuck state with signed shift */
    vel_filt_q8 += (raw_q8 - vel_filt_q8) / (1 << VEL_FILTER_SHIFT);

    return vel_filt_q8 / PID_SCALE;
}

static int32_t run_velocity_loop(void)
{
    return pid_update(&vel_pid, target_velocity, measured_velocity);
}

static void apply_duty(int32_t duty)
{
    uint32_t abs_duty;
    int8_t new_dir;
    last_applied_duty = duty;

    if (duty >= 0) {
        new_dir = 1;
        abs_duty = (uint32_t)duty;
    } else {
        new_dir = -1;
        abs_duty = (uint32_t)(-duty);
    }

    if (abs_duty > PWM_MAX_DUTY)
        abs_duty = PWM_MAX_DUTY;

    /* Update commutation immediately on direction change — otherwise
     * the motor runs on a stale phase pattern until the next hall
     * transition, which may never come if the motor is near standstill. */
    if (new_dir != drive_dir) {
        drive_dir = new_dir;
        commutation_update(drive_dir);
    }

    pwm_set_duty((uint16_t)abs_duty);
}

static int32_t torque_clamp(int32_t duty)
{
    if (current_ma <= torque_limit_ma)
        return duty;

    return (duty * (int32_t)torque_limit_ma) / (int32_t)current_ma;
}

static int32_t run_current_loop(int32_t setpoint_ma)
{
    int32_t magnitude;
    int32_t duty;

    /* Clamp to torque envelope */
    if (setpoint_ma > (int32_t)torque_limit_ma)
        setpoint_ma = (int32_t)torque_limit_ma;
    else if (setpoint_ma < -(int32_t)torque_limit_ma)
        setpoint_ma = -(int32_t)torque_limit_ma;

    /* Current PI operates on magnitude — shunt measures unsigned current */
    magnitude = (setpoint_ma >= 0) ? setpoint_ma : -setpoint_ma;
    duty = pid_update(&cur_pid, magnitude, (int32_t)current_ma);

    return (setpoint_ma >= 0) ? duty : -duty;
}

static void enter_fault(fault_code_t code)
{
    target_duty = 0;
    target_velocity = 0;
    target_current = 0;
    pid_reset(&vel_pid);
    pid_reset(&pos_pid);
    pid_reset(&cur_pid);
    current_setpoint = 0;
    coasting = 0;
    coast_armed = 0;
    overcurrent_ticks = 0;
    state = MOTOR_FAULT;
    fault = code;
}

/* ── Public API ──────────────────────────────────────────────────────── */

void motor_init(void)
{
    state = MOTOR_IDLE;
    fault = FAULT_NONE;
    ctrl_mode = CTRL_DUTY;
    target_duty = 0;
    target_velocity = 0;
    target_position = 0;
    target_current = 0;
    measured_velocity = 0;
    drive_dir = 1;
    torque_limit_ma = DEFAULT_TORQUE_LIMIT_MA;
    current_ma = 0;
    current_filt_q8 = 0;
    current_filt_initialized = 0;
    overcurrent_ticks = 0;
    prev_enc_position = 0;
    vel_filt_q8 = 0;
    vel_initialized = 0;
    last_applied_duty = 0;
    coasting = 0;
    coast_armed = 0;
    current_setpoint = 0;
    cur_div = 0;
    vel_div = 0;
    pos_div = 0;

    pid_init(&vel_pid,
             VEL_PID_KP_DEFAULT, VEL_PID_KI_DEFAULT, VEL_PID_KD_DEFAULT, VEL_FF_DEFAULT,
             -(int32_t)CURRENT_LIMIT_MA, (int32_t)CURRENT_LIMIT_MA);

    pid_init(&cur_pid,
             CUR_PID_KP_DEFAULT, CUR_PID_KI_DEFAULT, 0, 0,
             0, (int32_t)PWM_MAX_DUTY);

    pid_init(&pos_pid,
             POS_PID_KP_DEFAULT, POS_PID_KI_DEFAULT, POS_PID_KD_DEFAULT, 0,
             -(int32_t)(POS_MAX_VEL_RPM * POS_ERROR_PRESCALE),
             (int32_t)(POS_MAX_VEL_RPM * POS_ERROR_PRESCALE));
    pos_pid.int_max = (int64_t)POS_INT_MAX_RPM * POS_ERROR_PRESCALE * PID_SCALE;
}

void motor_set_mode(ctrl_mode_t mode)
{
    if (mode == ctrl_mode)
        return;

    coasting = 0;

    if (state == MOTOR_RUN) {
        /* Bumpless transfer: preload PIDs to match current outputs
         * so the motor doesn't jerk on mode switch */
        switch (mode) {
        case CTRL_VELOCITY:
            if (ctrl_mode == CTRL_DUTY || ctrl_mode == CTRL_TORQUE) {
                /* vel_pid wasn't running — seed with current draw */
                int32_t signed_ma = (int32_t)current_ma * drive_dir;
                pid_preload(&vel_pid, signed_ma,
                            target_velocity, measured_velocity);
                if (ctrl_mode == CTRL_DUTY) {
                    int32_t abs_duty = last_applied_duty >= 0 ? last_applied_duty : -last_applied_duty;
                    pid_preload(&cur_pid, abs_duty,
                                (int32_t)current_ma, (int32_t)current_ma);
                }
                /* CTRL_TORQUE: cur_pid already running, keep state */
            }
            /* else: vel_pid and cur_pid are running, keep their state */
            pid_reset(&pos_pid);
            break;
        case CTRL_POSITION:
            if (ctrl_mode == CTRL_DUTY || ctrl_mode == CTRL_TORQUE) {
                int32_t signed_ma = (int32_t)current_ma * drive_dir;
                target_velocity = measured_velocity;
                pid_preload(&vel_pid, signed_ma,
                            target_velocity, measured_velocity);
                if (ctrl_mode == CTRL_DUTY) {
                    int32_t abs_duty = last_applied_duty >= 0 ? last_applied_duty : -last_applied_duty;
                    pid_preload(&cur_pid, abs_duty,
                                (int32_t)current_ma, (int32_t)current_ma);
                }
            }
            /* Do not carry the incoming velocity-mode bias into the outer
             * position loop. Near-home capture behaves better when the
             * position controller starts from a clean state instead of
             * inheriting a large homeward integrator/preload. */
            pid_reset(&pos_pid);
            pos_pid.prev_meas = -encoder_get_position();
            target_velocity = 0;
            break;
        case CTRL_TORQUE:
            if (ctrl_mode == CTRL_DUTY) {
                int32_t abs_duty = last_applied_duty >= 0 ? last_applied_duty : -last_applied_duty;
                pid_preload(&cur_pid, abs_duty,
                            (int32_t)current_ma, (int32_t)current_ma);
            }
            /* else: cur_pid already running, keep state */
            pid_reset(&vel_pid);
            pid_reset(&pos_pid);
            break;
        default:
            pid_reset(&vel_pid);
            pid_reset(&pos_pid);
            pid_reset(&cur_pid);
            break;
        }
    } else {
        pid_reset(&vel_pid);
        pid_reset(&pos_pid);
        pid_reset(&cur_pid);
    }

    ctrl_mode = mode;
    current_setpoint = 0;
    cur_div = 0;
    vel_div = 0;
    pos_div = 0;
}

ctrl_mode_t motor_get_mode(void) { return ctrl_mode; }

void motor_set_duty(int32_t duty)        { target_duty = duty; }
void motor_set_velocity(int32_t rpm)     { target_velocity = rpm; }
void motor_set_position(int32_t counts)  { target_position = counts; }
void motor_set_current(int32_t ma)       { target_current = ma; }
void motor_shift_position_reference(int32_t delta)
{
    target_position -= delta;
    pos_pid.prev_meas -= delta;
    coast_trip_pos -= delta;

    /* encoder_position moves to encoder_position + delta when re-zeroed */
    prev_enc_position += delta;
}

void motor_set_torque_limit(uint32_t ma)
{
    if (ma > CURRENT_LIMIT_MA)
        ma = CURRENT_LIMIT_MA;
    torque_limit_ma = ma;
}

uint32_t motor_get_torque_limit(void)
{
    return torque_limit_ma;
}

void motor_set_vel_pid(int32_t kp, int32_t ki, int32_t kd)
{
    pid_set_gains(&vel_pid, kp, ki, kd, vel_pid.kf,
                  -(int32_t)CURRENT_LIMIT_MA, (int32_t)CURRENT_LIMIT_MA);
}

void motor_set_vel_ff(int32_t gain)
{
    vel_pid.kf = gain;
}

void motor_get_vel_pid(int32_t *kp, int32_t *ki, int32_t *kd)
{
    if (kp != 0)
        *kp = vel_pid.kp;
    if (ki != 0)
        *ki = vel_pid.ki;
    if (kd != 0)
        *kd = vel_pid.kd;
}

int32_t motor_get_vel_ff(void)
{
    return vel_pid.kf;
}

void motor_set_pos_pid(int32_t kp, int32_t ki, int32_t kd)
{
    pid_set_gains(&pos_pid, kp, ki, kd, 0,
                  -(int32_t)(POS_MAX_VEL_RPM * POS_ERROR_PRESCALE),
                  (int32_t)(POS_MAX_VEL_RPM * POS_ERROR_PRESCALE));
    pos_pid.int_max = (int64_t)POS_INT_MAX_RPM * POS_ERROR_PRESCALE * PID_SCALE;
}

void motor_get_pos_pid(int32_t *kp, int32_t *ki, int32_t *kd)
{
    if (kp != 0)
        *kp = pos_pid.kp;
    if (ki != 0)
        *ki = pos_pid.ki;
    if (kd != 0)
        *kd = pos_pid.kd;
}

void motor_set_cur_pid(int32_t kp, int32_t ki)
{
    pid_set_gains(&cur_pid, kp, ki, 0, 0,
                  0, (int32_t)PWM_MAX_DUTY);
}

void motor_get_cur_pid(int32_t *kp, int32_t *ki)
{
    if (kp != 0)
        *kp = cur_pid.kp;
    if (ki != 0)
        *ki = cur_pid.ki;
}

void motor_start(void)
{
    if (state == MOTOR_FAULT)
        return;

    if (ctrl_mode == CTRL_DUTY && target_duty == 0)
        return;

    if (state == MOTOR_RUN)
        return;

    pid_reset(&vel_pid);
    pid_reset(&pos_pid);
    pid_reset(&cur_pid);
    current_setpoint = 0;
    cur_div = 0;
    vel_div = 0;
    pos_div = 0;
    coasting = 0;
    current_filt_q8 = 0;
    current_filt_initialized = 0;
    overcurrent_ticks = 0;
    state = MOTOR_RUN;

    /* Set initial commutation direction */
    if (ctrl_mode == CTRL_DUTY)
        drive_dir = (target_duty >= 0) ? 1 : -1;
    else if (ctrl_mode == CTRL_VELOCITY)
        drive_dir = (target_velocity >= 0) ? 1 : -1;
    else if (ctrl_mode == CTRL_TORQUE)
        drive_dir = (target_current >= 0) ? 1 : -1;
    else
        drive_dir = 1;  /* position PID will sort it out */

    commutation_update(drive_dir);
    pwm_enable();
}

void motor_stop(void)
{
    target_duty = 0;
    target_velocity = 0;
    target_current = 0;
    pwm_set_duty(0);
    pwm_disable();
    pid_reset(&vel_pid);
    pid_reset(&pos_pid);
    pid_reset(&cur_pid);
    current_setpoint = 0;
    coasting = 0;
    coast_armed = 0;
    current_filt_q8 = 0;
    current_filt_initialized = 0;
    overcurrent_ticks = 0;
    state = MOTOR_IDLE;
}

void motor_coast(void)
{
    commutation_coast();
    pwm_set_duty(0);
    last_applied_duty = 0;
    coasting = 1;
}

void motor_arm_coast(int32_t threshold, int8_t direction)
{
    coast_trip_pos = threshold;
    coast_trip_dir = direction;
    coast_armed = 1;
}

void motor_disarm_coast(void)
{
    coast_armed = 0;
}

uint8_t motor_is_coasting(void)
{
    return coasting;
}

void motor_clear_fault(void)
{
    target_duty = 0;
    target_velocity = 0;
    target_current = 0;
    fault = FAULT_NONE;
    pid_reset(&vel_pid);
    pid_reset(&pos_pid);
    pid_reset(&cur_pid);
    current_setpoint = 0;
    pwm_disable();
    coasting = 0;
    coast_armed = 0;
    current_filt_q8 = 0;
    current_filt_initialized = 0;
    overcurrent_ticks = 0;
    state = MOTOR_IDLE;
}

motor_state_t motor_get_state(void)   { return state; }
fault_code_t  motor_get_fault(void)   { return fault; }
uint32_t      motor_get_current(void) { return current_ma; }
int32_t       motor_get_velocity(void){ return measured_velocity; }

int32_t motor_get_target(void)
{
    if (ctrl_mode == CTRL_DUTY) return target_duty;
    if (ctrl_mode == CTRL_TORQUE) return target_current;
    return target_velocity;
}

/* ── Fast poll (main loop, every iteration) ──────────────────────────── */

void motor_poll_fast(void)
{
    uint8_t result = hall_poll();

    if (result == HALL_POLL_INVALID) {
        pwm_fault_brake();
        state = MOTOR_FAULT;
        fault = FAULT_HALL_INVALID;
        return;
    }

    if (result != HALL_POLL_TRANSITION)
        return;

    if (state == MOTOR_RUN)
        commutation_update(drive_dir);
}

/* ── Control tick ────────────────────────────────────────────────────── */

void motor_tick_control(void)
{
    adc_snapshot_t adc_snapshot;
    uint32_t avg_current_ma;
    uint32_t peak_current_ma;
    int32_t duty;

    /* Consume one ADC window accumulated at PWM rate since the previous control tick. */
    adc_consume_snapshot(&adc_snapshot);
    avg_current_ma = adc_current_raw_to_ma(adc_snapshot.avg_current_raw);
    peak_current_ma = adc_current_raw_to_ma(adc_snapshot.peak_current_raw);

    if (!current_filt_initialized) {
        current_filt_q8 = (int32_t)(avg_current_ma << PID_SCALE_SHIFT);
        current_filt_initialized = 1u;
    } else {
        int32_t avg_current_q8 = (int32_t)(avg_current_ma << PID_SCALE_SHIFT);
        current_filt_q8 += (avg_current_q8 - current_filt_q8) / (1 << CURRENT_FILTER_SHIFT);
    }
    current_ma = (uint32_t)(current_filt_q8 / PID_SCALE);

    /* Use a sustained peak-current detector for normal overcurrent trips so
     * brief motion spikes do not fault immediately, while still preserving a
     * higher instantaneous ceiling for real shoot-through / stall events. */
    if (state == MOTOR_RUN) {
        if (peak_current_ma > CURRENT_PEAK_LIMIT_MA) {
            pwm_fault_brake();
            enter_fault(FAULT_OVERCURRENT);
            return;
        }

        if (peak_current_ma > CURRENT_LIMIT_MA) {
            if (overcurrent_ticks < 0xFFFFu)
                overcurrent_ticks++;
        } else {
            overcurrent_ticks = 0u;
        }

        if (overcurrent_ticks >= OVERCURRENT_FAULT_TICKS) {
            pwm_fault_brake();
            enter_fault(FAULT_OVERCURRENT);
            return;
        }
    } else {
        overcurrent_ticks = 0u;
    }

    /* Always update measured velocity (useful for status reporting) */
    measured_velocity = estimate_velocity();

    /* Coast detection at CONTROL_LOOP_HZ — armed by strike module.
     * Must cut power before the mallet hits the drum. */
    if (coast_armed && state == MOTOR_RUN) {
        int32_t pos = -encoder_get_position();
        uint8_t crossed = (coast_trip_dir > 0)
            ? (pos >= coast_trip_pos)
            : (pos <= coast_trip_pos);
        if (crossed) {
            motor_coast();
            coast_armed = 0;
        }
    }

    if (state != MOTOR_RUN)
        return;

    /* ── Control mode dispatch ──────────────────────────────────────── */
    switch (ctrl_mode) {

    case CTRL_DUTY:
        if (coasting)
            return;
        if (target_duty == 0) {
            motor_stop();
            return;
        }
        duty = torque_clamp(target_duty);
        apply_duty(duty);
        return;

    case CTRL_TORQUE:
        if (coasting)
            return;
        if (++cur_div >= CUR_LOOP_DIVIDER) {
            cur_div = 0;
            duty = run_current_loop(target_current);
            apply_duty(duty);
        }
        return;

    case CTRL_VELOCITY:
    case CTRL_POSITION:
        if (coasting)
            return;
        break;

    default:
        apply_duty(0);
        return;
    }

    /* ── Cascaded loops (velocity / position modes) ────────────────── */

    /* Position PID — outermost, only in CTRL_POSITION */
    if (ctrl_mode == CTRL_POSITION && ++pos_div >= POS_LOOP_DIVIDER) {
        pos_div = 0;
        int32_t pos = -encoder_get_position();
        int32_t pid_out = pid_update(&pos_pid, target_position, pos);
        target_velocity = pid_out / POS_ERROR_PRESCALE;
    }

    /* Velocity PID — outputs current setpoint in mA */
    if (++vel_div >= VEL_LOOP_DIVIDER) {
        vel_div = 0;
        current_setpoint = run_velocity_loop();
    }

    /* Current PI — innermost, outputs duty */
    if (++cur_div >= CUR_LOOP_DIVIDER) {
        cur_div = 0;
        duty = run_current_loop(current_setpoint);
        apply_duty(duty);
    }
}
