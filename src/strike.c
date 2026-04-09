/*
 * Strike Module — Mallet strike, rebound, and catch sequencer
 *
 * Orchestrates the motor control layer through a strike cycle:
 *   1. DRIVING   — open-loop duty toward drum (loudness control)
 *   2. COASTING  — phases floating, momentum carries mallet to drum
 *   3. RETURNING — closed-loop velocity back toward home
 *   4. CATCHING  — position servo settles onto home
 *
 * A new strike command received during an active cycle aborts the current
 * recovery path and immediately launches a fresh strike attempt.
 *
 * Homing learns the drum surface position by stall detection at low duty,
 * then lifts to a configurable home offset.
 *
 * Position convention: pos = -encoder_get_position() throughout,
 * matching motor.c.
 */

#include <stdint.h>
#include "m2003_config.h"
#include "strike.h"
#include "motor.h"
#include "encoder.h"

/* ── Helpers ─────────────────────────────────────────────────────────── */

static int32_t abs_i32(int32_t x) { return (x < 0) ? -x : x; }

/* ── State ───────────────────────────────────────────────────────────── */

typedef enum {
    HOME_SEEK_DRUM = 0,
    HOME_MOVE_HOME
} home_phase_t;

static strike_state_t state;
static home_phase_t   home_phase;
static uint8_t        homed;

/* Learned geometry */
static int32_t        drum_position;
static int32_t        home_position;
static int8_t         drum_dir;         /* +1/-1: sign of "toward drum" */

/* Configurable parameters */
static int32_t        home_offset;      /* counts above drum (always positive) */
static int32_t        coast_distance;   /* counts from drum to cut power */
static int32_t        homing_duty;      /* signed duty toward drum */

/* Runtime */
static int32_t        coast_threshold;  /* absolute position of coast point */
static int32_t        brake_target;     /* velocity ramp target during braking */
static uint16_t       settle_counter;
static uint16_t       stall_counter;
static int32_t        stall_prev_pos;
static uint16_t       coast_timeout;
static uint32_t       timebase_ms;
static uint32_t       active_start_ms;
static uint16_t       strike_sequence;
static uint8_t        timing_flags;
static int16_t        last_duty;
static uint16_t       trigger_to_coast_ms;
static uint16_t       trigger_to_rebound_ms;
static uint16_t       trigger_to_retrigger_ready_ms;
static uint16_t       trigger_to_ready_ms;
static uint16_t       estimated_strike_velocity_dps;

/* ── Position helper ─────────────────────────────────────────────────── */

static int32_t get_pos(void)
{
    return -encoder_get_position();
}

static uint16_t elapsed_ms_since(uint32_t start_ms)
{
    uint32_t elapsed = timebase_ms - start_ms;

    if (elapsed > 0xFFFFu)
        return 0xFFFFu;

    return (uint16_t)elapsed;
}

static int32_t orient_strike_duty(int32_t duty)
{
    if ((duty > 0 && drum_dir < 0) || (duty < 0 && drum_dir > 0))
        return -duty;

    return duty;
}

static int32_t home_direction_velocity(int32_t magnitude_rpm)
{
    return -drum_dir * magnitude_rpm;
}

static int32_t home_side_error(int32_t pos)
{
    return drum_dir * (pos - home_position);
}

static uint8_t retrigger_ready_now(int32_t pos, int32_t vel)
{
    if (abs_i32(pos - home_position) > STRIKE_RETRIGGER_DEADBAND)
        return 0u;

    if (abs_i32(vel) > STRIKE_RETRIGGER_VEL_THRESHOLD)
        return 0u;

    return 1u;
}

static uint16_t rpm_to_dps_clamped(int32_t rpm)
{
    uint32_t dps;

    if (rpm <= 0)
        return 0u;

    dps = (uint32_t)rpm * 6u;
    if (dps > 0xFFFFu)
        return 0xFFFFu;

    return (uint16_t)dps;
}

static void update_home_position_target(void)
{
    if (drum_dir == 0)
        return;

    home_position = drum_position - drum_dir * home_offset;
}

static void command_home_position_if_safe(void)
{
    if (motor_get_state() == MOTOR_FAULT)
        return;

    if (state == STRIKE_HOMING && home_phase == HOME_MOVE_HOME) {
        motor_set_position(home_position);
        motor_set_mode(CTRL_POSITION);
        settle_counter = 0;
        return;
    }

    if (!homed)
        return;

    if (state == STRIKE_IDLE || state == STRIKE_CATCHING) {
        motor_set_position(home_position);
        motor_set_mode(CTRL_POSITION);
        motor_start();
        settle_counter = 0;
        state = STRIKE_CATCHING;
    }
}

static void start_catching(void)
{
    motor_set_position(home_position);
    motor_set_mode(CTRL_POSITION);
    settle_counter = 0;
    state = STRIKE_CATCHING;
}

static void force_home_capture(void)
{
    motor_stop();
    motor_set_position(home_position);
    motor_set_mode(CTRL_POSITION);
    motor_start();
    settle_counter = 0;
    state = STRIKE_CATCHING;
}

static void maybe_mark_retrigger_ready(int32_t pos, int32_t vel)
{
    if ((timing_flags & STRIKE_TIMING_RETRIGGER_READY_VALID) != 0u)
        return;

    if (!retrigger_ready_now(pos, vel))
        return;

    trigger_to_retrigger_ready_ms = elapsed_ms_since(active_start_ms);
    timing_flags |= STRIKE_TIMING_RETRIGGER_READY_VALID;
}

static void begin_strike(int32_t duty, uint8_t retriggered)
{
    duty = orient_strike_duty(duty);

    strike_sequence++;
    timing_flags = STRIKE_TIMING_ACTIVE;
    if (retriggered)
        timing_flags |= STRIKE_TIMING_RETRIGGERED;

    last_duty = (int16_t)duty;
    trigger_to_coast_ms = 0;
    trigger_to_rebound_ms = 0;
    trigger_to_retrigger_ready_ms = 0;
    trigger_to_ready_ms = 0;
    estimated_strike_velocity_dps = 0;
    timing_flags |= STRIKE_TIMING_VELOCITY_VALID;
    active_start_ms = timebase_ms;

    /* Coast threshold: coast_distance counts from drum, on the home side */
    coast_threshold = drum_position - drum_dir * coast_distance;

    motor_set_mode(CTRL_DUTY);
    motor_set_duty(duty);
    /* motor_start() is a no-op if already running (position hold) */
    motor_start();

    /* Arm 5 kHz coast detection in motor tick — 1 kHz is too slow */
    motor_arm_coast(coast_threshold, drum_dir);

    state = STRIKE_DRIVING;
}

/* ── Init ────────────────────────────────────────────────────────────── */

void strike_init(void)
{
    state = STRIKE_IDLE;
    homed = 0;
    drum_position = 0;
    home_position = 0;
    drum_dir = 0;

    home_offset    = STRIKE_HOME_OFFSET_DEFAULT;
    coast_distance = STRIKE_COAST_DISTANCE_DEFAULT;
    homing_duty    = STRIKE_HOMING_DUTY_DEFAULT;

    timebase_ms = 0;
    active_start_ms = 0;
    strike_sequence = 0;
    timing_flags = 0;
    last_duty = 0;
    trigger_to_coast_ms = 0;
    trigger_to_rebound_ms = 0;
    trigger_to_retrigger_ready_ms = 0;
    trigger_to_ready_ms = 0;
    estimated_strike_velocity_dps = 0;
}

/* ── Configuration ───────────────────────────────────────────────────── */

void strike_set_home_offset(int32_t counts)
{
    home_offset = counts;

    if (homed || (state == STRIKE_HOMING && home_phase == HOME_MOVE_HOME)) {
        update_home_position_target();
        command_home_position_if_safe();
    }
}
void strike_set_coast_distance(int32_t counts) { coast_distance = counts; }
void strike_set_homing_duty(int32_t duty)      { homing_duty = duty; }
int32_t strike_get_home_offset(void)           { return home_offset; }
int32_t strike_get_coast_distance(void)        { return coast_distance; }
int32_t strike_get_homing_duty(void)           { return homing_duty; }

/* ── Commands ────────────────────────────────────────────────────────── */

void strike_shift_position_reference(int32_t delta)
{
    if (homed || (state == STRIKE_HOMING && home_phase == HOME_MOVE_HOME)) {
        drum_position -= delta;
        home_position -= delta;
    }

    if (state == STRIKE_HOMING)
        stall_prev_pos -= delta;

    if (state == STRIKE_DRIVING)
        coast_threshold -= delta;
}

void strike_restore_calibration(int32_t drum_pos, int32_t home_pos)
{
    if (homing_duty == 0)
        return;

    drum_position = drum_pos;
    home_position = home_pos;
    drum_dir = (homing_duty > 0) ? 1 : -1;
    homed = 1;
    state = STRIKE_IDLE;
}

void strike_home(void)
{
    if (state != STRIKE_IDLE)
        return;
    if (homing_duty == 0)
        return;

    drum_dir = (homing_duty > 0) ? 1 : -1;
    homed = 0;
    drum_position = 0;
    home_position = 0;

    motor_set_mode(CTRL_DUTY);
    motor_set_duty(homing_duty);
    motor_start();

    stall_counter = 0;
    settle_counter = 0;
    stall_prev_pos = get_pos();

    state = STRIKE_HOMING;
    home_phase = HOME_SEEK_DRUM;
}

strike_trigger_result_t strike_trigger(int32_t duty)
{
    int32_t pos;
    int32_t vel;
    uint8_t retriggered;

    if (!homed || state == STRIKE_HOMING)
        return STRIKE_TRIGGER_REJECT_NOT_HOMED;
    if (motor_get_state() == MOTOR_FAULT)
        return STRIKE_TRIGGER_REJECT_FAULT;
    if (duty == 0)
        return STRIKE_TRIGGER_REJECT_ZERO;

    retriggered = (state != STRIKE_IDLE);
    pos = get_pos();
    vel = motor_get_velocity();

    if (retriggered) {
        if (!retrigger_ready_now(pos, vel))
            return STRIKE_TRIGGER_REJECT_NOT_READY;

        if ((timing_flags & STRIKE_TIMING_RETRIGGER_READY_VALID) == 0u) {
            trigger_to_retrigger_ready_ms = elapsed_ms_since(active_start_ms);
            timing_flags |= STRIKE_TIMING_RETRIGGER_READY_VALID;
        }
    }

    motor_disarm_coast();
    if (retriggered)
        motor_stop();

    begin_strike(duty, retriggered);
    return retriggered ? STRIKE_TRIGGER_RETRIGGERED : STRIKE_TRIGGER_ACCEPTED;
}

void strike_cancel(void)
{
    motor_disarm_coast();
    if (homed) {
        motor_set_position(home_position);
        motor_set_mode(CTRL_POSITION);
        motor_start();
    } else {
        motor_stop();
    }
    timing_flags &= (uint8_t)~STRIKE_TIMING_ACTIVE;
    state = STRIKE_IDLE;
}

/* ── Status ──────────────────────────────────────────────────────────── */

strike_state_t strike_get_state(void)        { return state; }
int32_t  strike_get_drum_position(void)      { return drum_position; }
int32_t  strike_get_home_position(void)      { return home_position; }
uint8_t  strike_is_homed(void)               { return homed; }
uint16_t strike_get_sequence(void)           { return strike_sequence; }
void strike_get_metrics(strike_metrics_t *metrics)
{
    if (metrics == 0)
        return;

    metrics->flags = timing_flags;
    metrics->sequence = strike_sequence;
    metrics->last_duty = last_duty;
    metrics->trigger_to_coast_ms = trigger_to_coast_ms;
    metrics->trigger_to_rebound_ms = trigger_to_rebound_ms;
    metrics->trigger_to_retrigger_ready_ms = trigger_to_retrigger_ready_ms;
    metrics->trigger_to_ready_ms = trigger_to_ready_ms;
    metrics->estimated_strike_velocity_dps = estimated_strike_velocity_dps;
    metrics->home_offset = (int16_t)home_offset;
    metrics->coast_distance = (int16_t)coast_distance;
    metrics->homing_duty = (int16_t)homing_duty;
}

/* ── Tick (1 kHz) ────────────────────────────────────────────────────── */

void strike_tick(void)
{
    int32_t pos = get_pos();
    int32_t vel = motor_get_velocity();
    int32_t toward_drum_rpm = vel * drum_dir;
    int32_t home_error = home_side_error(pos);

    timebase_ms++;

    /* Abort on motor fault */
    if (motor_get_state() == MOTOR_FAULT) {
        motor_disarm_coast();
        timing_flags &= (uint8_t)~STRIKE_TIMING_ACTIVE;
        state = STRIKE_IDLE;
        return;
    }

    switch (state) {

    case STRIKE_IDLE:
        break;

    /* ── Homing ──────────────────────────────────────────────────────── */
    case STRIKE_HOMING:
        switch (home_phase) {

        case HOME_SEEK_DRUM:
            /* Stall detection: position unchanged for HOMING_STALL_TICKS */
            if (abs_i32(pos - stall_prev_pos) > STRIKE_HOMING_STALL_THRESHOLD) {
                stall_prev_pos = pos;
                stall_counter = 0;
            } else {
                stall_counter++;
            }

            if (stall_counter >= STRIKE_HOMING_STALL_TICKS) {
                drum_position = pos;
                /* Home is opposite the drum direction by home_offset */
                update_home_position_target();

                motor_set_position(home_position);
                motor_set_mode(CTRL_POSITION);

                settle_counter = 0;
                home_phase = HOME_MOVE_HOME;
            }
            break;

        case HOME_MOVE_HOME:
            if (abs_i32(pos - home_position) < STRIKE_SETTLE_DEADBAND)
                settle_counter++;
            else
                settle_counter = 0;

            if (settle_counter >= STRIKE_SETTLE_TICKS) {
                homed = 1;
                state = STRIKE_IDLE;
                /* Motor stays in CTRL_POSITION holding home */
            }
            break;
        }
        break;

    /* ── Driving toward drum ─────────────────────────────────────────── */
    case STRIKE_DRIVING:
        if (toward_drum_rpm > 0) {
            uint16_t velocity_dps = rpm_to_dps_clamped(toward_drum_rpm);
            if (velocity_dps > estimated_strike_velocity_dps)
                estimated_strike_velocity_dps = velocity_dps;
        }

        /* Coast is triggered by motor_tick at 5 kHz (motor_arm_coast).
         * We just detect the transition here at 1 kHz. */
        if (motor_is_coasting()) {
            trigger_to_coast_ms = elapsed_ms_since(active_start_ms);
            timing_flags |= STRIKE_TIMING_COAST_VALID;
            coast_timeout = 0;
            state = STRIKE_COASTING;
        }
        break;

    /* ── Coasting through impact ─────────────────────────────────────── */
    case STRIKE_COASTING:
        if (toward_drum_rpm > 0) {
            uint16_t velocity_dps = rpm_to_dps_clamped(toward_drum_rpm);
            if (velocity_dps > estimated_strike_velocity_dps)
                estimated_strike_velocity_dps = velocity_dps;
        }

        coast_timeout++;

        /* Rebound: velocity reversed away from drum past threshold */
        {
            int32_t home_vel = -vel * drum_dir;  /* positive = toward home */
            if (home_vel > STRIKE_REBOUND_THRESHOLD ||
                coast_timeout >= STRIKE_COAST_TIMEOUT_TICKS) {
                trigger_to_rebound_ms = elapsed_ms_since(active_start_ms);
                timing_flags |= STRIKE_TIMING_REBOUND_VALID;
                if (coast_timeout >= STRIKE_COAST_TIMEOUT_TICKS)
                    timing_flags |= STRIKE_TIMING_REBOUND_TIMEOUT;
                else
                    timing_flags &= (uint8_t)~STRIKE_TIMING_REBOUND_TIMEOUT;

                /* Re-enter control in velocity mode first, then hand off to
                 * position hold once the mallet is close to home. This avoids
                 * the immediate position-mode current spike that used to fault. */
                brake_target = vel;
                motor_set_velocity(brake_target);
                motor_set_mode(CTRL_VELOCITY);
                state = STRIKE_RETURNING;
            }
        }
        break;

    /* ── Returning: ramp into an aggressive homeward velocity target ── */
    case STRIKE_RETURNING:
    {
        int32_t desired_home_vel = home_direction_velocity(STRIKE_RETURN_VELOCITY_RPM);

        if (brake_target < desired_home_vel - STRIKE_RETURN_RAMP_RATE)
            brake_target += STRIKE_RETURN_RAMP_RATE;
        else if (brake_target > desired_home_vel + STRIKE_RETURN_RAMP_RATE)
            brake_target -= STRIKE_RETURN_RAMP_RATE;
        else
            brake_target = desired_home_vel;

        motor_set_velocity(brake_target);

        /* The 1 kHz strike tick can skip over a small catch window. Once the
         * mallet reaches or passes home, switch to position capture instead of
         * continuing to drive away from home in velocity mode. */
        if (home_error < -STRIKE_MAX_REBOUND_OVERSHOOT &&
            toward_drum_rpm < 0) {
            force_home_capture();
        } else if (home_error <= STRIKE_CATCH_ENTRY_WINDOW) {
            start_catching();
        }
        break;
    }

    /* ── Catching: position hold at home ─────────────────────────────── */
    case STRIKE_CATCHING:
        if (home_error < -STRIKE_MAX_REBOUND_OVERSHOOT &&
            toward_drum_rpm < 0) {
            force_home_capture();
            break;
        }

        maybe_mark_retrigger_ready(pos, vel);

        if (abs_i32(pos - home_position) < STRIKE_SETTLE_DEADBAND)
            settle_counter++;
        else
            settle_counter = 0;

        if (settle_counter >= STRIKE_SETTLE_TICKS)
        {
            trigger_to_ready_ms = elapsed_ms_since(active_start_ms);
            timing_flags |= STRIKE_TIMING_READY_VALID;
            timing_flags &= (uint8_t)~STRIKE_TIMING_ACTIVE;
            state = STRIKE_IDLE;
        }
        break;
    }
}
