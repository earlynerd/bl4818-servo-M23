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

static int32_t       vel_ff_gain;        /* Q8 feedforward: duty per RPM */
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
    int32_t error = target_velocity - measured_velocity;
    int32_t pid_out = pid_update(&vel_pid, error, measured_velocity);
    int32_t ff = (vel_ff_gain * target_velocity) / PID_SCALE;
    int32_t sum = pid_out + ff;
    if (sum > PWM_MAX_DUTY) sum = PWM_MAX_DUTY;
    else if (sum < -(int32_t)PWM_MAX_DUTY) sum = -(int32_t)PWM_MAX_DUTY;
    return sum;
}

static void apply_duty(int32_t duty)
{
    uint16_t abs_duty;

    if (duty >= 0) {
        drive_dir = 1;
        abs_duty = (uint16_t)duty;
    } else {
        drive_dir = -1;
        abs_duty = (uint16_t)(-duty);
    }

    if (abs_duty > PWM_MAX_DUTY)
        abs_duty = PWM_MAX_DUTY;

    pwm_set_duty(abs_duty);
}

static int32_t torque_clamp(int32_t duty)
{
    if (current_ma <= torque_limit_ma)
        return duty;

    return (duty * torque_limit_ma) / current_ma;
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
    vel_ff_gain = VEL_FF_DEFAULT;
    pos_div = 0;

    pid_init(&vel_pid,
             VEL_PID_KP_DEFAULT, VEL_PID_KI_DEFAULT, VEL_PID_KD_DEFAULT,
             -(int32_t)PWM_MAX_DUTY, (int32_t)PWM_MAX_DUTY);

    pid_init(&pos_pid,
             POS_PID_KP_DEFAULT, POS_PID_KI_DEFAULT, POS_PID_KD_DEFAULT,
             -(int32_t)(POS_MAX_VEL_RPM * POS_ERROR_PRESCALE),
             (int32_t)(POS_MAX_VEL_RPM * POS_ERROR_PRESCALE));
}

void motor_set_mode(ctrl_mode_t mode)
{
    if (mode == ctrl_mode)
        return;
    ctrl_mode = mode;
    pid_reset(&vel_pid);
    pid_reset(&pos_pid);
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
    pid_init(&vel_pid, kp, ki, kd,
             -(int32_t)PWM_MAX_DUTY, (int32_t)PWM_MAX_DUTY);
}

void motor_set_vel_ff(int32_t gain)
{
    vel_ff_gain = gain;
}

void motor_set_pos_pid(int32_t kp, int32_t ki, int32_t kd)
{
    pid_init(&pos_pid, kp, ki, kd,
             -(int32_t)(POS_MAX_VEL_RPM * POS_ERROR_PRESCALE),
             (int32_t)(POS_MAX_VEL_RPM * POS_ERROR_PRESCALE));
}

void motor_start(void)
{
    if (state == MOTOR_FAULT)
        return;

    if (ctrl_mode == CTRL_DUTY && target_duty == 0)
        return;

    pid_reset(&vel_pid);
    pid_reset(&pos_pid);
    pos_div = 0;
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
    state = MOTOR_IDLE;
}

void motor_clear_fault(void)
{
    target_duty = 0;
    target_velocity = 0;
    fault = FAULT_NONE;
    pid_reset(&vel_pid);
    pid_reset(&pos_pid);
    pwm_disable();
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

    if (state != MOTOR_RUN)
        return;

    /* ── Control mode dispatch ──────────────────────────────────────── */
    switch (ctrl_mode) {

    case CTRL_DUTY:
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
            int32_t error = target_position - pos;
            
            /* Position PID outputs velocity setpoint (RPM) scaled by POS_ERROR_PRESCALE */
            int32_t pid_out = pid_update(&pos_pid, error, pos);
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