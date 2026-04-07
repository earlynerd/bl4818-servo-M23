/*
 * Strike Module — Mallet strike, rebound, and catch sequencer
 *
 * Orchestrates the motor control layer through a strike cycle:
 *   1. DRIVING  — open-loop duty toward drum (loudness control)
 *   2. COASTING — phases floating, momentum carries mallet to drum
 *   3. CATCHING — position servo returns mallet to home
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

/* ── Position helper ─────────────────────────────────────────────────── */

static int32_t get_pos(void)
{
    return -encoder_get_position();
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
}

/* ── Configuration ───────────────────────────────────────────────────── */

void strike_set_home_offset(int32_t counts)   { home_offset = counts; }
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

void strike_trigger(int32_t duty)
{
    if (state != STRIKE_IDLE || !homed)
        return;
    if (duty == 0)
        return;

    if ((duty > 0 && drum_dir < 0) || (duty < 0 && drum_dir > 0))
        duty = -duty;

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

void strike_cancel(void)
{
    motor_disarm_coast();
    if (homed) {
        motor_set_mode(CTRL_POSITION);
        motor_set_position(home_position);
        motor_start();
    } else {
        motor_stop();
    }
    state = STRIKE_IDLE;
}

/* ── Status ──────────────────────────────────────────────────────────── */

strike_state_t strike_get_state(void)        { return state; }
int32_t  strike_get_drum_position(void)      { return drum_position; }
int32_t  strike_get_home_position(void)      { return home_position; }
uint8_t  strike_is_homed(void)               { return homed; }

/* ── Tick (1 kHz) ────────────────────────────────────────────────────── */

void strike_tick(void)
{
    int32_t pos = get_pos();
    int32_t vel = motor_get_velocity();

    /* Abort on motor fault */
    if (motor_get_state() == MOTOR_FAULT) {
        motor_disarm_coast();
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
                home_position = drum_position - drum_dir * home_offset;

                motor_set_mode(CTRL_POSITION);
                motor_set_position(home_position);

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
        /* Coast is triggered by motor_tick at 5 kHz (motor_arm_coast).
         * We just detect the transition here at 1 kHz. */
        if (motor_is_coasting()) {
            coast_timeout = 0;
            state = STRIKE_COASTING;
        }
        break;

    /* ── Coasting through impact ─────────────────────────────────────── */
    case STRIKE_COASTING:
        coast_timeout++;

        /* Rebound: velocity reversed away from drum past threshold */
        {
            int32_t home_vel = -vel * drum_dir;  /* positive = toward home */
            if (home_vel > STRIKE_REBOUND_THRESHOLD ||
                coast_timeout >= STRIKE_COAST_TIMEOUT_TICKS) {
                /* Start braking from current velocity to avoid slamming
                 * full reverse duty on a spinning motor (overcurrent) */
                brake_target = vel;
                motor_set_mode(CTRL_VELOCITY);
                motor_set_velocity(brake_target);
                state = STRIKE_BRAKING;
            }
        }
        break;

    /* ── Braking: ramp velocity toward zero, then position hold ─────── */
    case STRIKE_BRAKING:
        /* Ramp brake_target toward zero */
        if (brake_target > STRIKE_BRAKE_RAMP_RATE)
            brake_target -= STRIKE_BRAKE_RAMP_RATE;
        else if (brake_target < -STRIKE_BRAKE_RAMP_RATE)
            brake_target += STRIKE_BRAKE_RAMP_RATE;
        else
            brake_target = 0;
        motor_set_velocity(brake_target);

        if (abs_i32(vel) < STRIKE_BRAKE_VEL_THRESHOLD) {
            motor_set_mode(CTRL_POSITION);
            motor_set_position(home_position);
            settle_counter = 0;
            state = STRIKE_CATCHING;
        }
        break;

    /* ── Catching: position hold at home ─────────────────────────────── */
    case STRIKE_CATCHING:
        if (abs_i32(pos - home_position) < STRIKE_SETTLE_DEADBAND)
            settle_counter++;
        else
            settle_counter = 0;

        if (settle_counter >= STRIKE_SETTLE_TICKS)
            state = STRIKE_IDLE;
        break;
    }
}
