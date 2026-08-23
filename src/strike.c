/*
 * Strike Module — Mallet strike, rebound, and catch sequencer
 *
 * Orchestrates the motor control layer through a strike cycle:
 *   1. DRIVING   — current-controlled acceleration toward drum
 *   2. COASTING  — phases floating, momentum carries mallet to drum
 *   3. CATCHING  — position servo captures the rebound back to home
 *
 * Dead strikes (STRIKE_TYPE_DEAD) insert a MUTING state between COASTING
 * and CATCHING: at detected impact the velocity loop brakes to setpoint 0
 * (braking current self-scales to rebound energy), then a small torque
 * presses the mallet into the surface for the commanded dwell to mute the
 * note before the normal catch.
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
#include "M2003.h"
#include "strike.h"
#include "motor.h"
#include "encoder.h"
#include "irq_util.h"
#include "timing.h"

#if (ENCODER_COUNTS_PER_REV & (ENCODER_COUNTS_PER_REV - 1u)) != 0u
#error "absolute_angle_distance requires a power-of-two encoder count"
#endif

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
static uint8_t        calibration_valid;

/* Learned geometry */
static int32_t        drum_position;
static int32_t        home_position;
static int8_t         drum_dir;         /* +1/-1: sign of "toward drum" */
static uint16_t       drum_absolute_angle;
static uint16_t       homing_reference_angle;
static uint8_t        drum_absolute_angle_valid;
static uint8_t        homing_reference_angle_valid;

/* Configurable parameters */
static int32_t        home_offset;      /* counts above drum (always positive) */
static int32_t        coast_distance;   /* counts from drum to cut power */
static int32_t        homing_duty;      /* signed duty toward drum */
static int32_t        mute_brake_ms;    /* dead strike: velocity-0 brake phase duration */
static int32_t        mute_press_ma;    /* dead strike: contact-press current (0 = velocity hold) */
static int32_t        mute_engage_offset; /* dead strike: brake trip point, counts before drum surface (negative = past it) */

/* Runtime */
static int32_t        coast_threshold;  /* absolute position of coast point */
static uint8_t        active_type;      /* strike_type_t of the in-flight strike */
static uint16_t       mute_total_ticks; /* total MUTING dwell for this strike */
static uint16_t       mute_brake_ticks; /* brake-phase end within that dwell */
static uint16_t       mute_counter;
static uint8_t        mute_braking;     /* 1 = velocity-0 brake phase, 0 = press phase */
static uint16_t       settle_counter;
static uint16_t       stall_counter;
static int32_t        stall_prev_pos;
static uint16_t       coast_timeout;
static uint32_t       timebase_ticks;
static uint32_t       active_start_tick;
static uint16_t       strike_sequence;
static uint16_t       timing_flags;
static uint16_t       warning_flags;
static uint8_t        motion_guard_active;
static int32_t        motion_guard_last_pos;
static uint32_t       motion_guard_travel;
static uint32_t       homing_start_stamp;
static int16_t        last_current_ma;
static uint16_t       trigger_to_coast_ms;
static uint16_t       trigger_to_impact_ms;
static uint16_t       trigger_to_rebound_ms;
static uint16_t       trigger_to_retrigger_ready_ms;
static uint16_t       trigger_to_ready_ms;
static uint16_t       estimated_strike_velocity_dps;

/* "Last completed strike" snapshot — see strike.h. Updated at rebound
 * detection and at each begin_strike so the most useful data is always
 * one cheap read away even if the in-progress strike has overwritten the
 * live counters. */
static strike_compact_t prev_compact;

/* ── Position helper ─────────────────────────────────────────────────── */

static int32_t get_pos(void)
{
    return -encoder_get_position();
}

/* Magnitude of a continuous-position step, including signed int32 wrap.
 * The encoder cannot physically move 2^31 counts between 500 Hz samples. */
static uint32_t position_step(int32_t pos, int32_t previous)
{
    uint32_t delta = (uint32_t)pos - (uint32_t)previous;

    if (delta > 0x80000000u)
        delta = 0u - delta;

    return delta;
}

/* Shortest distance between two raw 14-bit absolute encoder angles. Unlike
 * the continuous logical position, this coordinate survives a power cycle
 * even when no logical zero reference has been configured. */
static uint32_t absolute_angle_distance(uint16_t a, uint16_t b)
{
    uint32_t delta = ((uint32_t)a - (uint32_t)b) &
                     (ENCODER_COUNTS_PER_REV - 1u);

    if (delta > (ENCODER_COUNTS_PER_REV / 2u))
        delta = ENCODER_COUNTS_PER_REV - delta;

    return delta;
}

static void motion_guard_start(int32_t pos)
{
    motion_guard_last_pos = pos;
    motion_guard_travel = 0u;
    active_start_tick = timebase_ticks;
    motion_guard_active = 1u;
}

static uint8_t motion_limit_reached(int32_t pos)
{
    uint32_t step = position_step(pos, motion_guard_last_pos);

    motion_guard_last_pos = pos;
    if (step >= (STRIKE_MOTION_LIMIT_COUNTS - motion_guard_travel))
        return 1u;

    motion_guard_travel += step;
    return 0u;
}

static void abort_motion(fault_code_t code)
{
    motor_raise_fault(code);
    timing_flags &= (uint16_t)~STRIKE_TIMING_ACTIVE;
    motion_guard_active = 0u;
    homed = 0u;
    state = STRIKE_IDLE;
}

static uint16_t elapsed_ms_since(uint32_t start_tick)
{
    uint32_t elapsed_ticks = timebase_ticks - start_tick;
    uint64_t elapsed_ms = ((uint64_t)elapsed_ticks * 1000ULL) / STRIKE_LOOP_HZ;

    if (elapsed_ms > 0xFFFFu)
        return 0xFFFFu;

    return (uint16_t)elapsed_ms;
}

static int32_t orient_strike_current(int32_t current_ma)
{
    if ((current_ma > 0 && drum_dir < 0) || (current_ma < 0 && drum_dir > 0))
        return -current_ma;

    return current_ma;
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

static int32_t toward_drum_current(int32_t magnitude_ma)
{
    return (drum_dir < 0) ? -magnitude_ma : magnitude_ma;
}

static uint16_t ms_to_strike_ticks(uint32_t ms)
{
    uint32_t ticks = (ms * STRIKE_LOOP_HZ) / 1000u;

    if (ticks > 0xFFFFu)
        return 0xFFFFu;

    return (uint16_t)ticks;
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

static void snapshot_prev_compact(void);

/* MUTING-state bookkeeping shared by all entry paths. The major timing
 * fields (coast, impact, peak velocity) are in by now; rebound never
 * becomes valid for a dead strike. */
static void enter_muting(void)
{
    snapshot_prev_compact();
    mute_counter = 0;
    mute_braking = 1u;
    state = STRIKE_MUTING;
}

/* Fallback mute entry (mallet stalled short of the brake trip point, or
 * coast timeout): command the velocity-0 hold from the strike tick. The
 * primary entry for a clean hit is the position-armed brake in motor.c —
 * by the time the filtered-velocity zero-cross is visible here, the mallet
 * has already begun rebounding. The mode switch clears any stale coast
 * state and resets the loop PIDs for a clean brake start. */
static void start_muting(void)
{
    motor_disarm_coast();

    motor_set_velocity(0);
    motor_set_mode(CTRL_VELOCITY);
    motor_start();

    enter_muting();
}

/* Primary mute entry: the motor layer tripped the velocity-0 brake on the
 * raw encoder crossing of the contact point. The motor is already braking;
 * we only record timing and follow the state machine. Coast/impact stamps
 * are best-effort here (≤ one strike tick late) — the brake itself was not
 * delayed by this tick's cadence. */
static void on_brake_engaged(void)
{
    if ((timing_flags & STRIKE_TIMING_COAST_VALID) == 0u) {
        trigger_to_coast_ms = elapsed_ms_since(active_start_tick);
        timing_flags |= STRIKE_TIMING_COAST_VALID;
    }
    if ((timing_flags & STRIKE_TIMING_IMPACT_VALID) == 0u) {
        trigger_to_impact_ms = elapsed_ms_since(active_start_tick);
        timing_flags |= STRIKE_TIMING_IMPACT_VALID;
    }

    enter_muting();
}

/* Mute press phase: a small steady toward-drum current keeps the rubber
 * tip seated against the surface with a defined force. mute_press_ma == 0
 * means "stay in the velocity-0 hold" instead. */
static void start_mute_press(void)
{
    mute_braking = 0u;

    if (mute_press_ma <= 0)
        return;

    motor_set_current(toward_drum_current(mute_press_ma));
    motor_set_mode(CTRL_TORQUE);
    motor_start();
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

    trigger_to_retrigger_ready_ms = elapsed_ms_since(active_start_tick);
    timing_flags |= STRIKE_TIMING_RETRIGGER_READY_VALID;
}

/* Record drum impact: the first sample where toward-drum velocity falls to
 * (or below) zero after the mallet has had a meaningful forward swing.
 * The VELOCITY_VALID guard ensures we don't latch impact on a noisy first
 * tick before the mallet has actually accelerated. Cost per coast tick is
 * one already-computed-int compare and one branch. */
static void maybe_record_impact(int32_t toward_drum_rpm)
{
    if ((timing_flags & STRIKE_TIMING_IMPACT_VALID) != 0u)
        return;
    if ((timing_flags & STRIKE_TIMING_VELOCITY_VALID) == 0u)
        return;
    if (toward_drum_rpm > STRIKE_IMPACT_VEL_THRESHOLD)
        return;

    trigger_to_impact_ms = elapsed_ms_since(active_start_tick);
    timing_flags |= STRIKE_TIMING_IMPACT_VALID;
}

static void snapshot_prev_compact(void)
{
    prev_compact.flags = timing_flags;
    prev_compact.sequence = strike_sequence;
    prev_compact.last_current_ma = last_current_ma;
    prev_compact.trigger_to_coast_ms = trigger_to_coast_ms;
    prev_compact.trigger_to_impact_ms = trigger_to_impact_ms;
    prev_compact.trigger_to_rebound_ms = trigger_to_rebound_ms;
    prev_compact.estimated_strike_velocity_dps = estimated_strike_velocity_dps;
}

static void begin_strike(int32_t current_ma, uint8_t retriggered,
                         uint8_t type, uint16_t mute_ms, int32_t start_pos)
{
    current_ma = orient_strike_current(current_ma);

    /* Snapshot whatever metrics the previous strike accumulated before we
     * overwrite them. Even partial data from a retrigger is more useful to
     * the host than nothing; rebound-detection also takes a snapshot once
     * a strike completes naturally. */
    snapshot_prev_compact();

    strike_sequence++;
    timing_flags = STRIKE_TIMING_ACTIVE;
    if (retriggered)
        timing_flags |= STRIKE_TIMING_RETRIGGERED;

    active_type = type;
    if (type == STRIKE_TYPE_DEAD) {
        uint32_t hold_ms = (mute_ms == 0u) ? STRIKE_MUTE_HOLD_DEFAULT_MS
                                           : (uint32_t)mute_ms;
        if (hold_ms > STRIKE_MUTE_MAX_MS)
            hold_ms = STRIKE_MUTE_MAX_MS;

        mute_total_ticks = ms_to_strike_ticks(hold_ms);
        if (mute_total_ticks == 0u)
            mute_total_ticks = 1u;

        mute_brake_ticks = ms_to_strike_ticks((uint32_t)mute_brake_ms);
        if (mute_brake_ticks > mute_total_ticks)
            mute_brake_ticks = mute_total_ticks;

        timing_flags |= STRIKE_TIMING_DEAD;
    }

    last_current_ma = (int16_t)current_ma;
    trigger_to_coast_ms = 0;
    trigger_to_impact_ms = 0;
    trigger_to_rebound_ms = 0;
    trigger_to_retrigger_ready_ms = 0;
    trigger_to_ready_ms = 0;
    estimated_strike_velocity_dps = 0;
    motion_guard_start(start_pos);

    /* Coast threshold: coast_distance counts from drum, on the home side */
    coast_threshold = drum_position - drum_dir * coast_distance;

    motor_set_current(current_ma);
    motor_set_mode(CTRL_TORQUE);
    /* motor_start() is a no-op if already running (position hold) */
    motor_start();

    /* Arm coast detection on the encoder/velocity cadence so power cuts
     * before impact. Dead strikes arm the second stage too: the velocity-0
     * brake trips on the raw position crossing of the contact point, while
     * the ball is still compressing — waiting for the strike-tick velocity
     * zero-cross lets the mallet rebound first (audible double hit). */
    if (type == STRIKE_TYPE_DEAD) {
        int32_t brake_threshold = drum_position - drum_dir * mute_engage_offset;
        motor_arm_coast_then_brake(coast_threshold, brake_threshold, drum_dir);
    } else {
        motor_arm_coast(coast_threshold, drum_dir);
    }

    state = STRIKE_DRIVING;
}

/* ── Init ────────────────────────────────────────────────────────────── */

void strike_init(void)
{
    state = STRIKE_IDLE;
    homed = 0;
    calibration_valid = 0;
    drum_position = 0;
    home_position = 0;
    drum_dir = 0;
    drum_absolute_angle = 0u;
    homing_reference_angle = 0u;
    drum_absolute_angle_valid = 0u;
    homing_reference_angle_valid = 0u;

    home_offset    = STRIKE_HOME_OFFSET_DEFAULT;
    coast_distance = STRIKE_COAST_DISTANCE_DEFAULT;
    homing_duty    = STRIKE_HOMING_DUTY_DEFAULT;
    mute_brake_ms  = STRIKE_MUTE_BRAKE_MS_DEFAULT;
    mute_press_ma  = STRIKE_MUTE_PRESS_MA_DEFAULT;
    mute_engage_offset = STRIKE_MUTE_ENGAGE_OFFSET_DEFAULT;

    active_type      = STRIKE_TYPE_NORMAL;
    mute_total_ticks = 0;
    mute_brake_ticks = 0;
    mute_counter     = 0;
    mute_braking     = 0;

    timebase_ticks = 0;
    active_start_tick = 0;
    strike_sequence = 0;
    timing_flags = 0;
    warning_flags = 0;
    motion_guard_active = 0;
    motion_guard_last_pos = 0;
    motion_guard_travel = 0;
    homing_start_stamp = 0;
    last_current_ma = 0;
    trigger_to_coast_ms = 0;
    trigger_to_impact_ms = 0;
    trigger_to_rebound_ms = 0;
    trigger_to_retrigger_ready_ms = 0;
    trigger_to_ready_ms = 0;
    estimated_strike_velocity_dps = 0;

    prev_compact.flags = 0;
    prev_compact.sequence = 0;
    prev_compact.last_current_ma = 0;
    prev_compact.trigger_to_coast_ms = 0;
    prev_compact.trigger_to_impact_ms = 0;
    prev_compact.trigger_to_rebound_ms = 0;
    prev_compact.estimated_strike_velocity_dps = 0;
}

/* ── Configuration ───────────────────────────────────────────────────── */

void strike_set_home_offset(int32_t counts)
{
    uint32_t irq_state = irq_save();

    home_offset = counts;

    if (homed || (state == STRIKE_HOMING && home_phase == HOME_MOVE_HOME)) {
        update_home_position_target();
        command_home_position_if_safe();
    }

    irq_restore(irq_state);
}
void strike_set_coast_distance(int32_t counts)
{
    uint32_t irq_state = irq_save();
    coast_distance = counts;
    irq_restore(irq_state);
}
void strike_set_homing_duty(int32_t duty)
{
    uint32_t irq_state = irq_save();
    homing_duty = duty;
    irq_restore(irq_state);
}
void strike_set_mute_brake_ms(int32_t ms)
{
    uint32_t irq_state = irq_save();
    mute_brake_ms = (ms < 0) ? 0 : ms;
    irq_restore(irq_state);
}
void strike_set_mute_press_ma(int32_t ma)
{
    uint32_t irq_state = irq_save();
    mute_press_ma = (ma < 0) ? 0 : ma;
    irq_restore(irq_state);
}
void strike_set_mute_engage_offset(int32_t counts)
{
    uint32_t irq_state = irq_save();
    mute_engage_offset = counts;
    irq_restore(irq_state);
}
int32_t strike_get_home_offset(void)           { return home_offset; }
int32_t strike_get_coast_distance(void)        { return coast_distance; }
int32_t strike_get_homing_duty(void)           { return homing_duty; }
int32_t strike_get_mute_brake_ms(void)         { return mute_brake_ms; }
int32_t strike_get_mute_press_ma(void)         { return mute_press_ma; }
int32_t strike_get_mute_engage_offset(void)    { return mute_engage_offset; }

/* ── Commands ────────────────────────────────────────────────────────── */

void strike_shift_position_reference(int32_t delta)
{
    uint32_t irq_state = irq_save();

    if (calibration_valid ||
        (state == STRIKE_HOMING && home_phase == HOME_MOVE_HOME)) {
        drum_position -= delta;
        home_position -= delta;
    }

    if (state == STRIKE_HOMING) {
        stall_prev_pos -= delta;
    }

    if (motion_guard_active)
        motion_guard_last_pos -= delta;

    if (state == STRIKE_DRIVING)
        coast_threshold -= delta;

    irq_restore(irq_state);
}

void strike_restore_calibration(int32_t drum_pos, int32_t home_pos)
{
    uint32_t irq_state = irq_save();

    if (homing_duty == 0)
    {
        irq_restore(irq_state);
        return;
    }

    drum_position = drum_pos;
    home_position = home_pos;
    drum_dir = (homing_duty > 0) ? 1 : -1;
    calibration_valid = 1;
    homed = 1;
    state = STRIKE_IDLE;
    motion_guard_active = 0u;

    irq_restore(irq_state);
}

void strike_restore_drum_angle(uint16_t angle)
{
    uint32_t irq_state = irq_save();

    drum_absolute_angle = (uint16_t)(angle & (ENCODER_COUNTS_PER_REV - 1u));
    drum_absolute_angle_valid = 1u;

    irq_restore(irq_state);
}

strike_home_result_t strike_home(void)
{
    uint32_t irq_state = irq_save();

    if (state != STRIKE_IDLE)
    {
        irq_restore(irq_state);
        return STRIKE_HOME_REJECT_BUSY;
    }
    if (motor_get_state() == MOTOR_FAULT)
    {
        irq_restore(irq_state);
        return STRIKE_HOME_REJECT_FAULT;
    }
    if (homing_duty == 0)
    {
        irq_restore(irq_state);
        return STRIKE_HOME_REJECT_DISABLED;
    }

    homing_reference_angle_valid = drum_absolute_angle_valid;
    if (homing_reference_angle_valid)
        homing_reference_angle = drum_absolute_angle;

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
    homing_start_stamp = timing_capture_stamp();
    motion_guard_start(stall_prev_pos);

    state = STRIKE_HOMING;
    home_phase = HOME_SEEK_DRUM;

    irq_restore(irq_state);
    return STRIKE_HOME_STARTED;
}

strike_trigger_result_t strike_trigger(int32_t current_ma)
{
    return strike_trigger_ex(current_ma, STRIKE_TYPE_NORMAL, 0u);
}

strike_trigger_result_t strike_trigger_ex(int32_t current_ma, uint8_t type, uint16_t mute_ms)
{
    uint32_t irq_state = irq_save();
    int32_t pos;
    int32_t vel;
    uint8_t retriggered;

    if (type > STRIKE_TYPE_DEAD)
    {
        irq_restore(irq_state);
        return STRIKE_TRIGGER_REJECT_BAD_TYPE;
    }
    if (!homed || state == STRIKE_HOMING)
    {
        irq_restore(irq_state);
        return STRIKE_TRIGGER_REJECT_NOT_HOMED;
    }
    if (motor_get_state() == MOTOR_FAULT)
    {
        irq_restore(irq_state);
        return STRIKE_TRIGGER_REJECT_FAULT;
    }
    if (current_ma == 0)
    {
        irq_restore(irq_state);
        return STRIKE_TRIGGER_REJECT_ZERO;
    }

    retriggered = (state != STRIKE_IDLE);
    pos = get_pos();
    vel = motor_get_velocity();

    if (retriggered) {
        if (!retrigger_ready_now(pos, vel))
        {
            irq_restore(irq_state);
            return STRIKE_TRIGGER_REJECT_NOT_READY;
        }

        if ((timing_flags & STRIKE_TIMING_RETRIGGER_READY_VALID) == 0u) {
            trigger_to_retrigger_ready_ms = elapsed_ms_since(active_start_tick);
            timing_flags |= STRIKE_TIMING_RETRIGGER_READY_VALID;
        }
    }

    motor_disarm_coast();
    if (retriggered)
        motor_stop();

    begin_strike(current_ma, retriggered, type, mute_ms, pos);
    irq_restore(irq_state);
    return retriggered ? STRIKE_TRIGGER_RETRIGGERED : STRIKE_TRIGGER_ACCEPTED;
}

void strike_cancel(void)
{
    uint32_t irq_state = irq_save();
    uint8_t was_active;

    was_active = (state != STRIKE_IDLE) ? 1u : 0u;
    motor_disarm_coast();
    if (homed) {
        motor_set_position(home_position);
        motor_set_mode(CTRL_POSITION);
        motor_start();
        settle_counter = 0;
        if (was_active)
            state = STRIKE_CATCHING;
    } else {
        motor_stop();
        state = STRIKE_IDLE;
        motion_guard_active = 0u;
    }
    if (!was_active)
        motion_guard_active = 0u;
    timing_flags &= (uint16_t)~STRIKE_TIMING_ACTIVE;

    irq_restore(irq_state);
}

void strike_stop(void)
{
    uint32_t irq_state = irq_save();

    motor_disarm_coast();
    motor_stop();
    state = STRIKE_IDLE;
    homed = 0;
    motion_guard_active = 0u;
    timing_flags &= (uint16_t)~STRIKE_TIMING_ACTIVE;

    irq_restore(irq_state);
}

void strike_clear_fault(void)
{
    uint32_t irq_state = irq_save();

    if (motor_get_state() == MOTOR_FAULT) {
        motor_clear_fault();
        state = STRIKE_IDLE;
        homed = 0u;
        motion_guard_active = 0u;
        timing_flags &= (uint16_t)~STRIKE_TIMING_ACTIVE;
    }

    irq_restore(irq_state);
}

uint8_t strike_pause_for_persist(void)
{
    uint32_t irq_state = irq_save();
    uint8_t resume_hold = 0u;

    if (state == STRIKE_IDLE && homed &&
        motor_get_state() == MOTOR_RUN &&
        motor_get_mode() == CTRL_POSITION) {
        motor_stop();
        resume_hold = 1u;
    }

    irq_restore(irq_state);
    return resume_hold;
}

void strike_resume_after_persist(uint8_t resume_hold)
{
    uint32_t irq_state;

    if (!resume_hold)
        return;

    irq_state = irq_save();
    if (state == STRIKE_IDLE && homed && motor_get_state() != MOTOR_FAULT) {
        motor_set_position(home_position);
        motor_set_mode(CTRL_POSITION);
        motor_start();
    }
    irq_restore(irq_state);
}

/* ── Status ──────────────────────────────────────────────────────────── */

strike_state_t strike_get_state(void)        { return state; }
int32_t  strike_get_drum_position(void)      { return drum_position; }
int32_t  strike_get_home_position(void)      { return home_position; }
uint16_t strike_get_drum_angle(void)         { return drum_absolute_angle; }
uint8_t  strike_has_drum_angle(void)         { return drum_absolute_angle_valid; }
uint8_t  strike_is_homed(void)               { return homed; }
uint16_t strike_get_sequence(void)           { return strike_sequence; }
void strike_get_metrics(strike_metrics_t *metrics)
{
    uint32_t irq_state;

    if (metrics == 0)
        return;

    irq_state = irq_save();
    metrics->flags = timing_flags | warning_flags;
    metrics->sequence = strike_sequence;
    metrics->last_current_ma = last_current_ma;
    metrics->trigger_to_coast_ms = trigger_to_coast_ms;
    metrics->trigger_to_impact_ms = trigger_to_impact_ms;
    metrics->trigger_to_rebound_ms = trigger_to_rebound_ms;
    metrics->trigger_to_retrigger_ready_ms = trigger_to_retrigger_ready_ms;
    metrics->trigger_to_ready_ms = trigger_to_ready_ms;
    metrics->estimated_strike_velocity_dps = estimated_strike_velocity_dps;
    metrics->home_offset = (int16_t)home_offset;
    metrics->coast_distance = (int16_t)coast_distance;
    metrics->homing_duty = (int16_t)homing_duty;
    irq_restore(irq_state);
}

void strike_get_compact_metrics(strike_compact_t *compact)
{
    uint32_t irq_state;

    if (compact == 0)
        return;

    irq_state = irq_save();
    *compact = prev_compact;
    compact->flags |= warning_flags;
    irq_restore(irq_state);
}

/* ── Tick (configured by STRIKE_LOOP_HZ) ─────────────────────────────── */

void strike_tick(void)
{
    int32_t pos = get_pos();
    int32_t vel = motor_get_velocity();
    int32_t toward_drum_rpm = vel * drum_dir;
    int32_t home_error = home_side_error(pos);

    timebase_ticks++;

    /* Abort on motor fault */
    if (motor_get_state() == MOTOR_FAULT) {
        motor_disarm_coast();
        timing_flags &= (uint16_t)~STRIKE_TIMING_ACTIVE;
        motion_guard_active = 0u;
        homed = 0u;
        state = STRIKE_IDLE;
        return;
    }

    /* Cumulative travel is checked only while a homing/strike sequence is
     * active, and only at this 500 Hz state-machine cadence. */
    if (motion_guard_active) {
        if (motion_limit_reached(pos)) {
            abort_motion((state == STRIKE_HOMING) ? FAULT_HOMING_LIMIT
                                                   : FAULT_STRIKE_LIMIT);
            return;
        }
        if (state == STRIKE_HOMING &&
            timing_elapsed_us(homing_start_stamp) >=
            (STRIKE_HOMING_TIMEOUT_MS * 1000u)) {
            abort_motion(FAULT_HOMING_LIMIT);
            return;
        }
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
                uint16_t detected_drum_angle = encoder_get_angle();

                if (homing_reference_angle_valid &&
                    absolute_angle_distance(detected_drum_angle,
                                            homing_reference_angle) >=
                    STRIKE_HOME_SHIFT_WARN_COUNTS) {
                    warning_flags |= STRIKE_WARNING_HOME_SHIFT;
                } else {
                    warning_flags &= (uint16_t)~STRIKE_WARNING_HOME_SHIFT;
                }

                drum_position = pos;
                drum_absolute_angle = detected_drum_angle;
                drum_absolute_angle_valid = 1u;
                calibration_valid = 1u;
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
                motion_guard_active = 0u;
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
            if (velocity_dps > estimated_strike_velocity_dps) {
                estimated_strike_velocity_dps = velocity_dps;
                timing_flags |= STRIKE_TIMING_VELOCITY_VALID;
            }
        }

        /* Rare but possible: impact happens before the coast-distance
         * threshold (e.g. if coast is set wide). Catch it here too. */
        maybe_record_impact(toward_drum_rpm);

        if (active_type == STRIKE_TYPE_DEAD) {
            /* Coast and brake can both trip between strike ticks; the
             * armed brake in motor.c is the primary entry. */
            if (motor_is_brake_engaged()) {
                on_brake_engaged();
                break;
            }
            if ((timing_flags & STRIKE_TIMING_IMPACT_VALID) != 0u) {
                start_muting();
                break;
            }
        }

        /* Coast is triggered by the faster encoder/velocity cadence
         * (motor_arm_coast). We just detect the transition here. */
        if (motor_is_coasting()) {
            trigger_to_coast_ms = elapsed_ms_since(active_start_tick);
            timing_flags |= STRIKE_TIMING_COAST_VALID;
            coast_timeout = 0;
            state = STRIKE_COASTING;
        }
        break;

    /* ── Coasting through impact ─────────────────────────────────────── */
    case STRIKE_COASTING:
        if (toward_drum_rpm > 0) {
            uint16_t velocity_dps = rpm_to_dps_clamped(toward_drum_rpm);
            if (velocity_dps > estimated_strike_velocity_dps) {
                estimated_strike_velocity_dps = velocity_dps;
                timing_flags |= STRIKE_TIMING_VELOCITY_VALID;
            }
        }

        maybe_record_impact(toward_drum_rpm);

        coast_timeout++;

        /* Dead strike: the position-armed brake in motor.c is the primary
         * entry — it pins the mallet while the ball is still compressing.
         * The filtered-velocity zero-cross only covers a mallet that stalls
         * short of the brake trip point; the timeout path still presses
         * gently into the surface, which doubles as a damp. */
        if (active_type == STRIKE_TYPE_DEAD) {
            if (motor_is_brake_engaged()) {
                on_brake_engaged();
            } else if ((timing_flags & STRIKE_TIMING_IMPACT_VALID) != 0u) {
                start_muting();
            } else if (coast_timeout >= STRIKE_COAST_TIMEOUT_TICKS) {
                timing_flags |= STRIKE_TIMING_REBOUND_TIMEOUT;
                start_muting();
            }
            break;
        }

        /* Rebound: velocity reversed away from drum past threshold */
        {
            int32_t home_vel = -vel * drum_dir;  /* positive = toward home */
            if (home_vel > STRIKE_REBOUND_THRESHOLD ||
                coast_timeout >= STRIKE_COAST_TIMEOUT_TICKS) {
                trigger_to_rebound_ms = elapsed_ms_since(active_start_tick);
                timing_flags |= STRIKE_TIMING_REBOUND_VALID;
                if (coast_timeout >= STRIKE_COAST_TIMEOUT_TICKS)
                    timing_flags |= STRIKE_TIMING_REBOUND_TIMEOUT;
                else
                    timing_flags &= (uint16_t)~STRIKE_TIMING_REBOUND_TIMEOUT;

                /* Take a snapshot now: the major timing fields (coast,
                 * impact, rebound, peak velocity) are all in. The host
                 * gets fully-formed metrics from the next ACK_TIMED or a
                 * compact STRIKE_TIMING query without us having to wait
                 * for catch/settle. */
                snapshot_prev_compact();

                /* Re-enter control directly in position mode and let the
                 * cascaded position -> velocity -> current loops catch the
                 * rebound at home. motor_set_mode() resets stale coast state. */
                start_catching();
            }
        }
        break;

    /* ── Muting: dead strike holds the mallet against the surface ────── */
    case STRIKE_MUTING:
        mute_counter++;

        if (mute_braking != 0u && mute_counter >= mute_brake_ticks)
            start_mute_press();

        if (mute_counter >= mute_total_ticks)
            start_catching();
        break;

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
            trigger_to_ready_ms = elapsed_ms_since(active_start_tick);
            timing_flags |= STRIKE_TIMING_READY_VALID;
            timing_flags &= (uint16_t)~STRIKE_TIMING_ACTIVE;
            motion_guard_active = 0u;
            state = STRIKE_IDLE;
        }
        break;
    }
}
