/*
 * Motor Control for Nuvoton M2003
 *
 * Supports three control modes:
 *   CTRL_DUTY     — open-loop signed duty cycle (pass-through)
 *   CTRL_VELOCITY — closed-loop RPM via encoder-derived velocity PID (5 kHz)
 *   CTRL_POSITION — cascaded: position PID (1 kHz) → velocity PID (5 kHz)
 *
 * Control loop runs from the main loop tick, NOT from ISR.
 * ADC ISR only captures raw samples.
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
static int32_t       measured_velocity;  /* signed RPM */
static int8_t        drive_dir;          /* +1 / -1, commutation direction */
static uint32_t      torque_limit_ma;
static uint32_t      current_ma;

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
static uint8_t       pos_div;            /* divider counter for 1 kHz position loop */

#define DEFAULT_TORQUE_LIMIT_MA 800
#define CURRENT_LIMIT_MA        5000

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
    int32_t raw_q8 = raw_rpm << 8;
    /* Use division to avoid asymmetric deadband and -1 RPM stuck state with signed shift */
    vel_filt_q8 += (raw_q8 - vel_filt_q8) / (1 << VEL_FILTER_SHIFT);

    return vel_filt_q8 / 256;
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

/* ── Public API ──────────────────────────────────────────────────────── */

void motor_init(void)
{
    state = MOTOR_IDLE;
    fault = FAULT_NONE;
    ctrl_mode = CTRL_DUTY;
    target_duty = 0;
    target_velocity = 0;
    target_position = 0;
    measured_velocity = 0;
    drive_dir = 1;
    torque_limit_ma = DEFAULT_TORQUE_LIMIT_MA;
    current_ma = 0;
    prev_enc_position = 0;
    vel_filt_q8 = 0;
    vel_initialized = 0;
    last_applied_duty = 0;
    coasting = 0;
    coast_armed = 0;
    pos_div = 0;

    pid_init(&vel_pid,
             VEL_PID_KP_DEFAULT, VEL_PID_KI_DEFAULT, VEL_PID_KD_DEFAULT, VEL_FF_DEFAULT,
             -(int32_t)PWM_MAX_DUTY, (int32_t)PWM_MAX_DUTY);

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
            if (ctrl_mode == CTRL_DUTY) {
                /* vel_pid wasn't running — seed it */
                pid_preload(&vel_pid, last_applied_duty, measured_velocity);
            }
            /* else: vel_pid was inner loop, keep its state */
            pid_reset(&pos_pid);
            break;
        case CTRL_POSITION:
            if (ctrl_mode == CTRL_DUTY) {
                /* Neither PID was running — seed vel_pid */
                pid_preload(&vel_pid, last_applied_duty, measured_velocity);
            }
            /* Seed pos_pid to hold current velocity */
            target_velocity = measured_velocity;
            pid_preload(&pos_pid,
                        measured_velocity * POS_ERROR_PRESCALE,
                        -encoder_get_position());
            break;
        default:
            pid_reset(&vel_pid);
            pid_reset(&pos_pid);
            break;
        }
    } else {
        pid_reset(&vel_pid);
        pid_reset(&pos_pid);
    }

    ctrl_mode = mode;
    pos_div = 0;
}

ctrl_mode_t motor_get_mode(void) { return ctrl_mode; }

void motor_set_duty(int32_t duty)        { target_duty = duty; }
void motor_set_velocity(int32_t rpm)     { target_velocity = rpm; }
void motor_set_position(int32_t counts)  { target_position = counts; }

void motor_set_torque_limit(uint32_t ma)
{
    if (ma > CURRENT_LIMIT_MA)
        ma = CURRENT_LIMIT_MA;
    torque_limit_ma = ma;
}

void motor_set_vel_pid(int32_t kp, int32_t ki, int32_t kd)
{
    pid_init(&vel_pid, kp, ki, kd, vel_pid.kf,
             -(int32_t)PWM_MAX_DUTY, (int32_t)PWM_MAX_DUTY);
}

void motor_set_vel_ff(int32_t gain)
{
    vel_pid.kf = gain;
}

void motor_set_pos_pid(int32_t kp, int32_t ki, int32_t kd)
{
    pid_init(&pos_pid, kp, ki, kd, 0,
             -(int32_t)(POS_MAX_VEL_RPM * POS_ERROR_PRESCALE),
             (int32_t)(POS_MAX_VEL_RPM * POS_ERROR_PRESCALE));
    pos_pid.int_max = (int64_t)POS_INT_MAX_RPM * POS_ERROR_PRESCALE * PID_SCALE;
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
    pos_div = 0;
    coasting = 0;
    state = MOTOR_RUN;

    /* Set initial commutation direction */
    if (ctrl_mode == CTRL_DUTY)
        drive_dir = (target_duty >= 0) ? 1 : -1;
    else if (ctrl_mode == CTRL_VELOCITY)
        drive_dir = (target_velocity >= 0) ? 1 : -1;
    else
        drive_dir = 1;  /* position PID will sort it out */

    commutation_update(drive_dir);
    pwm_enable();
}

void motor_stop(void)
{
    target_duty = 0;
    target_velocity = 0;
    pwm_set_duty(0);
    pwm_disable();
    pid_reset(&vel_pid);
    pid_reset(&pos_pid);
    coasting = 0;
    coast_armed = 0;
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
    fault = FAULT_NONE;
    pid_reset(&vel_pid);
    pid_reset(&pos_pid);
    pwm_disable();
    coasting = 0;
    coast_armed = 0;
    state = MOTOR_IDLE;
}

motor_state_t motor_get_state(void)   { return state; }
fault_code_t  motor_get_fault(void)   { return fault; }
uint32_t      motor_get_current(void) { return current_ma; }
int32_t       motor_get_velocity(void){ return measured_velocity; }

int32_t motor_get_target(void)
{
    /* In position mode, report the velocity setpoint (pos PID output) */
    return (ctrl_mode == CTRL_DUTY) ? target_duty : target_velocity;
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

void motor_tick_2khz(void)
{
    int32_t duty;

    /* ADC sample */
    current_ma = adc_read_current_ma();

    /* Hard overcurrent fault */
    if (current_ma > CURRENT_LIMIT_MA) {
        pwm_fault_brake();
        state = MOTOR_FAULT;
        fault = FAULT_OVERCURRENT;
        return;
    }

    /* Always update measured velocity (useful for status reporting) */
    measured_velocity = estimate_velocity();

    /* Fast coast detection at 5 kHz — armed by strike module.
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
        break;

    case CTRL_VELOCITY:
        duty = torque_clamp(run_velocity_loop());
        break;

    case CTRL_POSITION: {
        /* Position PID at 1 kHz (every POS_LOOP_DIVIDER ticks) */
        if (++pos_div >= POS_LOOP_DIVIDER) {
            pos_div = 0;
            /* Calculate full 32-bit error to eliminate quantization deadband limit cycling.
             * The PID math scales the output, which is then divided by POS_ERROR_PRESCALE
             * to maintain the original gain scaling for backwards compatibility. */
            int32_t pos = -encoder_get_position();
            
            /* Position PID outputs velocity setpoint (RPM) scaled by POS_ERROR_PRESCALE */
            int32_t pid_out = pid_update(&pos_pid, target_position, pos);
            target_velocity = pid_out / POS_ERROR_PRESCALE;
        }
        /* Velocity loop at full rate */
        duty = torque_clamp(run_velocity_loop());
        break;
    }

    default:
        duty = 0;
        break;
    }

    apply_duty(duty);
}