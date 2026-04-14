/*
 * Motor Control for Nuvoton M2003
 *
 * Supports three control modes:
 *   CTRL_DUTY     — open-loop signed duty cycle (torque-clamped)
 *   CTRL_VELOCITY — closed-loop RPM: velocity PID → current PI → duty
 *   CTRL_POSITION — cascaded: position PID → velocity PID → current PI → duty
 *
 * Loop rates are configured in m2003_config.h (_LOOP_HZ defines).
 * The fast tick runs at CURRENT_LOOP_HZ for current PI and protection, while
 * velocity and position work are scheduled more slowly from it.
 * ADC ISR oversamples at PWM rate and hands the fast loop an averaged window.
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
#include "timing.h"

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
static uint16_t      overcurrent_samples;
static uint16_t      peak_overcurrent_samples;

static int32_t       prev_enc_position;
static int32_t       vel_filt_q8;       /* IIR-filtered velocity in Q8 */
static uint8_t       vel_initialized;

static int32_t       last_applied_duty;  /* for bumpless transfer on mode switch */
static uint8_t       coasting;           /* 1 = phases floating, skip duty dispatch */
static int32_t       coast_trip_pos;     /* auto-coast position threshold */
static int8_t        coast_trip_dir;     /* +1: coast when pos >= trip, -1: when <= */
static uint8_t       coast_armed;        /* 1 = watching for coast trigger on encoder samples */
static pid_t         vel_pid;
static pid_t         pos_pid;
static pid_t         cur_pid;
static int32_t       current_setpoint;   /* mA, output of velocity PID for current loop */
static uint32_t      velocity_sched_accum;
static uint32_t      position_sched_accum;
static uint32_t      velocity_elapsed_samples;

/*
 * Encoder-based velocity:
 *   RPM = delta_counts * 60 * PWM_FREQ_HZ /
 *         (ENCODER_COUNTS_PER_REV * elapsed_pwm_samples)
 */
/* ── Helpers ─────────────────────────────────────────────────────────── */

static uint32_t irq_save(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void irq_restore(uint32_t primask)
{
    if ((primask & 1u) == 0u)
        __enable_irq();
}

static void reset_scheduler_state(void)
{
    velocity_sched_accum = 0u;
    position_sched_accum = 0u;
    velocity_elapsed_samples = 0u;
}

static uint8_t schedule_latest_from_samples(uint32_t *accum, uint32_t elapsed_samples,
                                            uint32_t target_hz, uint32_t *dropped_updates)
{
    uint32_t extra_due;

    *accum += elapsed_samples * target_hz;
    if (*accum < PWM_FREQ_HZ) {
        if (dropped_updates != 0)
            *dropped_updates = 0u;
        return 0u;
    }

    *accum -= PWM_FREQ_HZ;
    if (*accum < PWM_FREQ_HZ) {
        if (dropped_updates != 0)
            *dropped_updates = 0u;
        return 1u;
    }

    extra_due = *accum / PWM_FREQ_HZ;
    *accum -= extra_due * PWM_FREQ_HZ;

    if (dropped_updates != 0)
        *dropped_updates = extra_due;

    return 1u;
}

static int32_t estimate_velocity(uint32_t elapsed_samples)
{
    int32_t pos = encoder_get_position();
    int32_t delta = pos - prev_enc_position;
    int64_t numerator;
    int64_t denominator;
    prev_enc_position = pos;

    if (!vel_initialized) {
        vel_initialized = 1;
        return 0;
    }

    if (elapsed_samples == 0u)
        elapsed_samples = 1u;

    /* Negate: encoder counts positive in opposite direction to commutation forward */
    numerator = -(int64_t)delta * 60LL * (int64_t)PWM_FREQ_HZ;
    denominator = (int64_t)ENCODER_COUNTS_PER_REV * (int64_t)elapsed_samples;
    int32_t raw_rpm = (int32_t)(numerator / denominator);

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

static void reset_current_protection_state(void)
{
    current_filt_q8 = 0;
    current_filt_initialized = 0;
    overcurrent_samples = 0;
    peak_overcurrent_samples = 0;
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
    reset_scheduler_state();
    coasting = 0;
    coast_armed = 0;
    reset_current_protection_state();
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
    overcurrent_samples = 0;
    peak_overcurrent_samples = 0;
    prev_enc_position = 0;
    vel_filt_q8 = 0;
    vel_initialized = 0;
    last_applied_duty = 0;
    coasting = 0;
    coast_armed = 0;
    current_setpoint = 0;
    reset_scheduler_state();

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
    uint32_t irq_state = irq_save();
    uint8_t was_coasting;

    if (mode == ctrl_mode) {
        irq_restore(irq_state);
        return;
    }

    was_coasting = coasting;
    coasting = 0;

    if (state == MOTOR_RUN) {
        if (was_coasting) {
            pid_reset(&vel_pid);
            pid_reset(&pos_pid);
            pid_reset(&cur_pid);
        } else {
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
        }
    } else {
        pid_reset(&vel_pid);
        pid_reset(&pos_pid);
        pid_reset(&cur_pid);
    }

    ctrl_mode = mode;
    current_setpoint = 0;
    reset_scheduler_state();
    irq_restore(irq_state);
}

ctrl_mode_t motor_get_mode(void) { return ctrl_mode; }

void motor_set_duty(int32_t duty)        { target_duty = duty; }
void motor_set_velocity(int32_t rpm)     { target_velocity = rpm; }
void motor_set_position(int32_t counts)  { target_position = counts; }
void motor_set_current(int32_t ma)       { target_current = ma; }
void motor_shift_position_reference(int32_t delta)
{
    uint32_t irq_state = irq_save();
    target_position -= delta;
    pos_pid.prev_meas -= delta;
    coast_trip_pos -= delta;

    /* encoder_position moves to encoder_position + delta when re-zeroed */
    prev_enc_position += delta;
    irq_restore(irq_state);
}

void motor_set_torque_limit(uint32_t ma)
{
    uint32_t irq_state = irq_save();
    if (ma > CURRENT_LIMIT_MA)
        ma = CURRENT_LIMIT_MA;
    torque_limit_ma = ma;
    irq_restore(irq_state);
}

uint32_t motor_get_torque_limit(void)
{
    return torque_limit_ma;
}

void motor_set_vel_pid(int32_t kp, int32_t ki, int32_t kd)
{
    uint32_t irq_state = irq_save();
    pid_set_gains(&vel_pid, kp, ki, kd, vel_pid.kf,
                  -(int32_t)CURRENT_LIMIT_MA, (int32_t)CURRENT_LIMIT_MA);
    irq_restore(irq_state);
}

void motor_set_vel_ff(int32_t gain)
{
    uint32_t irq_state = irq_save();
    vel_pid.kf = gain;
    irq_restore(irq_state);
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
    uint32_t irq_state = irq_save();
    pid_set_gains(&pos_pid, kp, ki, kd, 0,
                  -(int32_t)(POS_MAX_VEL_RPM * POS_ERROR_PRESCALE),
                  (int32_t)(POS_MAX_VEL_RPM * POS_ERROR_PRESCALE));
    pos_pid.int_max = (int64_t)POS_INT_MAX_RPM * POS_ERROR_PRESCALE * PID_SCALE;
    irq_restore(irq_state);
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
    uint32_t irq_state = irq_save();
    pid_set_gains(&cur_pid, kp, ki, 0, 0,
                  0, (int32_t)PWM_MAX_DUTY);
    irq_restore(irq_state);
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
    uint32_t irq_state = irq_save();

    if (state == MOTOR_FAULT)
    {
        irq_restore(irq_state);
        return;
    }

    if (ctrl_mode == CTRL_DUTY && target_duty == 0)
    {
        irq_restore(irq_state);
        return;
    }

    if (state == MOTOR_RUN)
    {
        irq_restore(irq_state);
        return;
    }

    pid_reset(&vel_pid);
    pid_reset(&pos_pid);
    pid_reset(&cur_pid);
    current_setpoint = 0;
    reset_scheduler_state();
    coasting = 0;
    reset_current_protection_state();
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
    irq_restore(irq_state);
}

void motor_stop(void)
{
    uint32_t irq_state = irq_save();
    target_duty = 0;
    target_velocity = 0;
    target_current = 0;
    pwm_set_duty(0);
    pwm_disable();
    pid_reset(&vel_pid);
    pid_reset(&pos_pid);
    pid_reset(&cur_pid);
    current_setpoint = 0;
    reset_scheduler_state();
    coasting = 0;
    coast_armed = 0;
    reset_current_protection_state();
    state = MOTOR_IDLE;
    irq_restore(irq_state);
}

void motor_coast(void)
{
    commutation_coast();
    pwm_set_duty(0);
    pid_reset(&vel_pid);
    pid_reset(&pos_pid);
    pid_reset(&cur_pid);
    current_setpoint = 0;
    reset_scheduler_state();
    overcurrent_samples = 0;
    peak_overcurrent_samples = 0;
    last_applied_duty = 0;
    coasting = 1;
}

void motor_arm_coast(int32_t threshold, int8_t direction)
{
    uint32_t irq_state = irq_save();
    coast_trip_pos = threshold;
    coast_trip_dir = direction;
    coast_armed = 1;
    irq_restore(irq_state);
}

void motor_disarm_coast(void)
{
    uint32_t irq_state = irq_save();
    coast_armed = 0;
    irq_restore(irq_state);
}

uint8_t motor_is_coasting(void)
{
    return coasting;
}

void motor_clear_fault(void)
{
    uint32_t irq_state = irq_save();
    target_duty = 0;
    target_velocity = 0;
    target_current = 0;
    fault = FAULT_NONE;
    pid_reset(&vel_pid);
    pid_reset(&pos_pid);
    pid_reset(&cur_pid);
    current_setpoint = 0;
    reset_scheduler_state();
    pwm_disable();
    coasting = 0;
    coast_armed = 0;
    reset_current_protection_state();
    state = MOTOR_IDLE;
    irq_restore(irq_state);
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

/* ── Hall transition handling ────────────────────────────────────────── */

void motor_handle_hall_transition(uint8_t result)
{
    if (result == HALL_POLL_INVALID) {
        pwm_fault_brake();
        enter_fault(FAULT_HALL_INVALID);
        return;
    }

    if (result != HALL_POLL_TRANSITION)
        return;

    if (state == MOTOR_RUN && !coasting)
        commutation_update(drive_dir);
}

/* ── Fast poll (main loop, every iteration) ──────────────────────────── */

void motor_poll_fast(void)
{
    /* Hall transitions are processed in the GPIO IRQ path. Keep the hook so
     * the main loop structure does not need to change. */
}

/* ── Control tick ────────────────────────────────────────────────────── */

uint16_t motor_tick_control(void)
{
    adc_snapshot_t adc_snapshot;
    uint16_t elapsed_samples;
    uint32_t avg_current_ma;
    uint32_t peak_current_ma;
    uint32_t dropped_updates;
    int32_t duty;
    uint8_t velocity_due = 0u;
    uint8_t position_due = 0u;

    /* Consume one ADC window accumulated at PWM rate since the previous fast tick. */
    adc_consume_snapshot(&adc_snapshot);
    elapsed_samples = adc_snapshot.sample_count;
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

    /* Skip overcurrent faulting during intentional coast: outputs are masked,
     * and impact/rebound transients are expected while the strike is floating. */
    if (state == MOTOR_RUN && !coasting) {
        if (peak_current_ma > CURRENT_PEAK_LIMIT_MA) {
            uint32_t total = (uint32_t)peak_overcurrent_samples + elapsed_samples;
            peak_overcurrent_samples = (total > 0xFFFFu) ? 0xFFFFu : (uint16_t)total;
        } else {
            peak_overcurrent_samples = 0u;
        }

        if (peak_overcurrent_samples >= CURRENT_PEAK_FAULT_SAMPLES) {
            pwm_fault_brake();
            enter_fault(FAULT_OVERCURRENT);
            return elapsed_samples;
        }

        if (peak_current_ma > CURRENT_LIMIT_MA) {
            uint32_t total = (uint32_t)overcurrent_samples + elapsed_samples;
            overcurrent_samples = (total > 0xFFFFu) ? 0xFFFFu : (uint16_t)total;
        } else {
            overcurrent_samples = 0u;
        }

        if (overcurrent_samples >= OVERCURRENT_FAULT_SAMPLES) {
            pwm_fault_brake();
            enter_fault(FAULT_OVERCURRENT);
            return elapsed_samples;
        }
    } else {
        overcurrent_samples = 0u;
        peak_overcurrent_samples = 0u;
    }

    velocity_elapsed_samples += elapsed_samples;
    if (schedule_latest_from_samples(&velocity_sched_accum, elapsed_samples,
                                     VELOCITY_LOOP_HZ, &dropped_updates)) {
        velocity_due = 1u;
        if (dropped_updates != 0u)
            timing_note_velocity_drops(dropped_updates);

        encoder_poll();
        measured_velocity = estimate_velocity(velocity_elapsed_samples);
        velocity_elapsed_samples = 0u;

        /* Coast detection follows the encoder sampling cadence. Bit-banged SSI
         * reads are too expensive to do at the current-loop rate. */
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
    }

    if (schedule_latest_from_samples(&position_sched_accum, elapsed_samples,
                                     POSITION_LOOP_HZ, &dropped_updates)) {
        position_due = 1u;
        if (dropped_updates != 0u)
            timing_note_position_drops(dropped_updates);
    }

    if (state != MOTOR_RUN)
        return elapsed_samples;

    /* ── Control mode dispatch ──────────────────────────────────────── */
    switch (ctrl_mode) {

    case CTRL_DUTY:
        if (coasting)
            return elapsed_samples;
        if (target_duty == 0) {
            motor_stop();
            return elapsed_samples;
        }
        duty = torque_clamp(target_duty);
        apply_duty(duty);
        return elapsed_samples;

    case CTRL_TORQUE:
        if (coasting)
            return elapsed_samples;
        duty = run_current_loop(target_current);
        apply_duty(duty);
        return elapsed_samples;

    case CTRL_VELOCITY:
    case CTRL_POSITION:
        if (coasting)
            return elapsed_samples;
        break;

    default:
        apply_duty(0);
        return elapsed_samples;
    }

    /* ── Cascaded loops (velocity / position modes) ────────────────── */

    /* Position PID — outermost, only in CTRL_POSITION */
    if (ctrl_mode == CTRL_POSITION && position_due) {
        int32_t pos = -encoder_get_position();
        int32_t pid_out = pid_update(&pos_pid, target_position, pos);
        target_velocity = pid_out / POS_ERROR_PRESCALE;
    }

    /* Velocity PID — outputs current setpoint in mA */
    if (velocity_due) {
        current_setpoint = run_velocity_loop();
    }

    /* Current PI — innermost, outputs duty */
    duty = run_current_loop(current_setpoint);
    apply_duty(duty);
    return elapsed_samples;
}
