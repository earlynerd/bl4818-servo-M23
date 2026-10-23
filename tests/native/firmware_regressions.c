#include <assert.h>
#include <stdio.h>
#include "m2003_config.h"
#include "app_adc.h"
#include "app_pwm.h"
#include "commutation.h"
#include "timing.h"

/* Include the implementation to test private record migration/capture without
 * replacing any of that logic or attempting physical flash operations. */
#include "../../src/persist.c"

uint32_t test_primask;
static uint16_t test_angle, test_zero, test_duty;
static int32_t test_position;
static uint8_t test_zero_valid, test_csn, test_power;
static int8_t test_direction;
static uint32_t test_clock;

/* Peripheral substitutes retain observable state; motor/strike/PID/persistence
 * logic is compiled directly from the production sources. */
uint16_t encoder_get_angle(void) { return test_angle; }
int32_t encoder_get_position(void) { return test_position; }
uint8_t encoder_has_zero_reference(void) { return test_zero_valid; }
uint16_t encoder_get_zero_reference(void) { return test_zero; }
uint8_t encoder_has_csn_polarity(void) { return 1u; }
uint8_t encoder_get_csn_polarity(void) { return test_csn; }
void encoder_set_csn_polarity(uint8_t level) { test_csn = level; }
void encoder_set_zero_reference(uint16_t angle)
{
    test_zero = angle;
    test_zero_valid = 1u;
    test_position = (int32_t)test_angle - angle;
    if (test_position > 8192) test_position -= 16384;
    if (test_position < -8192) test_position += 16384;
}
void pwm_set_duty(uint16_t duty) { test_duty = duty; }
void pwm_enable(void) { test_power = 1u; }
void pwm_disable(void) { test_power = 0u; }
void pwm_fault_brake(void) { test_power = 0u; }
void commutation_update(int8_t direction) { test_direction = direction; }
void commutation_coast(void) { }
/* TIMER2 is a 24-bit free-running 1 MHz counter. */
uint32_t timing_capture_stamp(void) { return test_clock & 0xFFFFFFu; }
uint32_t timing_elapsed_us(uint32_t start) { return (test_clock - start) & 0xFFFFFFu; }
uint32_t timing_stamp_delta_us(uint32_t start, uint32_t end) { return (end - start) & 0xFFFFFFu; }
void timing_note_velocity_drops(uint32_t dropped) { (void)dropped; }
void timing_note_position_drops(uint32_t dropped) { (void)dropped; }

/* Strike simulation: a scripted mallet path (motor coordinates, counts
 * toward the drum from home) sampled whenever the motor layer polls the
 * encoder, with SysTick ordering of the 6 kHz control and 500 Hz strike ticks. */
static int32_t (*sim_path)(uint32_t us);
static uint32_t sim_start, sim_ticks;

void encoder_poll(void)
{
    if (sim_path)
        test_position = -sim_path((test_clock - sim_start) & 0xFFFFFFu);
}
void adc_consume_snapshot(adc_snapshot_t *snapshot)
{
    static const uint16_t samples[3] = {3u, 3u, 4u};  /* 20 kHz PWM / 6 kHz loop */
    snapshot->latest_current_raw = snapshot->avg_current_raw = snapshot->peak_current_raw = 0u;
    snapshot->latest_voltage_raw = snapshot->avg_voltage_raw = 0u;
    snapshot->sample_count = samples[sim_ticks % 3u];
}
uint32_t adc_current_raw_to_ma(uint16_t raw) { return raw; }

static void reset_runtime(uint16_t angle, int32_t position)
{
    test_angle = angle;
    test_position = position;
    test_zero_valid = 0u;
    test_power = 0u;
    test_primask = 0u;
    motor_init();
    strike_init();
}

static uint16_t physical_angle_of(int32_t motor_position)
{
    return (uint16_t)(test_angle - test_position - motor_position) & 0x3FFFu;
}

static void assert_geometry(uint16_t contact, int direction, int clearance)
{
    uint16_t expected_home = (uint16_t)(contact + direction * clearance) & 0x3FFFu;
    assert(strike_is_homed());
    assert(strike_get_drum_angle() == contact);
    assert(physical_angle_of(strike_get_drum_position()) == contact);
    assert(physical_angle_of(strike_get_home_position()) == expected_home);
    assert(strike_get_drum_position() - strike_get_home_position() == direction * clearance);
    assert(motor_get_state() == MOTOR_IDLE);
    assert(!test_power);  /* restoring geometry must not initiate motion */
}

static void test_absolute_calibration(void)
{
    const uint16_t contacts[] = {0, 100, 8000, 16000, 16383};
    const int displacements[] = {-2000, -1, 0, 1, 2000};
    for (unsigned c = 0; c < sizeof(contacts) / sizeof(contacts[0]); ++c) {
        for (int direction = -1; direction <= 1; direction += 2) {
            for (unsigned d = 0; d < sizeof(displacements) / sizeof(displacements[0]); ++d) {
                uint16_t home = (uint16_t)(contacts[c] + direction * 1024) & 0x3FFFu;
                uint16_t boot = (uint16_t)(home + displacements[d]) & 0x3FFFu;
                persist_record_t record;
                reset_runtime(boot, 0);
                strike_set_homing_duty(direction * 100);
                strike_restore_calibration(contacts[c]);
                assert_geometry(contacts[c], direction, 1024);
                assert(strike_get_home_position() == displacements[d]);
                persist_capture_runtime(&record, 7u);
                assert(record.version == 6u && persist_record_is_valid(&record));
                assert(record.strike_drum_position == 0 && record.strike_home_position == 0);

                /* Reboot at a different angle and a different logical origin. */
                reset_runtime((uint16_t)(boot + 333) & 0x3FFFu, 4567);
                persist_apply_record(&record);
                assert(!strike_is_homed());
                persist_restore_calibration();
                assert_geometry(contacts[c], direction, 1024);
            }
        }
    }
}

static void test_record_migration(void)
{
    persist_record_t record;
    reset_runtime(1200, 0);
    strike_restore_calibration(100);
    persist_capture_runtime(&record, 1u);

    /* v5 absolute angle takes precedence over stale continuous coordinates. */
    record.version = 5u;
    record.strike_drum_position = 70000;
    record.strike_home_position = -90000;
    record.crc = persist_crc(&record);
    assert(persist_record_is_valid(&record));
    reset_runtime(1400, 0);
    persist_apply_record(&record);
    persist_restore_calibration();
    assert_geometry(100, 1, 1024);

    for (uint16_t version = 3; version <= 5; ++version) {
        record.version = version;
        record.flags = PERSIST_FLAG_STRIKE_CAL_VALID | PERSIST_FLAG_ZERO_VALID;
        record.zero_angle = 16000;
        record.strike_drum_position = -484;  /* wraps to absolute contact 100 */
        reset_runtime(1300, 0);
        persist_apply_record(&record);
        persist_restore_calibration();
        assert_geometry(100, 1, 1024);

        record.flags = PERSIST_FLAG_STRIKE_CAL_VALID;
        reset_runtime(1300, 0);
        persist_apply_record(&record);
        persist_restore_calibration();
        assert(!strike_is_homed());  /* no recoverable physical reference */
    }
}

static void test_stop_preserves_fault(void)
{
    for (int code = FAULT_OVERCURRENT; code <= FAULT_STRIKE_LIMIT; ++code) {
        reset_runtime(1200, 0);
        motor_set_duty(100);
        motor_start();
        assert(test_power);
        motor_raise_fault((fault_code_t)code);
        strike_stop();
        strike_cancel();
        assert(motor_get_state() == MOTOR_FAULT);
        assert(motor_get_fault() == (fault_code_t)code);
        motor_set_duty(100);
        motor_start();
        assert(!test_power);
        assert(strike_home() == STRIKE_HOME_REJECT_FAULT);
        strike_clear_fault();
        assert(motor_get_fault() == FAULT_NONE && !strike_is_homed());
        assert(strike_home() == STRIKE_HOME_STARTED);
        assert(test_power);
    }
}

/* Geometry for the strike paths: home 0, drum surface 1024, contact point
 * 1024 - STRIKE_CONTACT_MARGIN_COUNTS = 992, coast point 724. */
static int32_t path_rebound(uint32_t us)    /* 100 counts/ms in, 40 compression, 60 counts/ms out */
{
    int32_t out;
    if (us <= 10640u)
        return (int32_t)(us / 10u);
    out = 1064 - (int32_t)((us - 10640u) * 6u / 100u);
    return out > 0 ? out : 0;
}
static int32_t path_rest(uint32_t us)       /* soft hit: 30 counts/ms, then rests in contact */
{
    if (us <= 34466u)
        return (int32_t)(us * 3u / 100u);
    return 1034 + (int32_t)((us / 667u) & 1u);   /* +-1 count encoder noise */
}
static int32_t path_short(uint32_t us)      /* stalls 200 counts short of the surface */
{
    return (us <= 27466u) ? (int32_t)(us * 3u / 100u) : 824;
}
static int32_t path_backswing(uint32_t us)  /* retrigger-like start moving away */
{
    int32_t out;
    if (us <= 3000u)
        return -(int32_t)(us / 50u);                       /* away at 20 counts/ms */
    if (us <= 14240u)
        return -60 + (int32_t)((us - 3000u) / 10u);        /* in at 100 counts/ms */
    out = 1064 - (int32_t)((us - 14240u) * 6u / 100u);
    return out > 0 ? out : 0;
}

static int32_t path_dead_slow(uint32_t us)  /* 10 counts/ms: contact-to-brake spans strike ticks */
{
    int32_t out;
    if (us <= 106400u)
        return (int32_t)(us / 100u);
    out = 1064 - (int32_t)((us - 106400u) / 100u);
    return out > 0 ? out : 0;
}

static void sim_tick(void)
{
    static const uint16_t step_us[3] = {167u, 167u, 166u};
    test_clock += step_us[sim_ticks % 3u];
    (void)motor_tick_control();
    if (++sim_ticks % (CONTROL_LOOP_HZ / STRIKE_LOOP_HZ) == 0u)
        strike_tick();
}

static void start_strike_sim(int32_t (*path)(uint32_t), uint32_t clock, uint8_t type)
{
    reset_runtime(9024, 0);         /* home angle for contact 8000: home = 0 */
    strike_set_homing_duty(100);
    strike_restore_calibration(8000);
    assert(strike_get_home_position() == 0 && strike_get_drum_position() == 1024);
    test_clock = clock;
    sim_start = clock;
    sim_ticks = 0u;
    sim_path = path;
    assert(strike_trigger_ex(800, type, 0u) == STRIKE_TRIGGER_ACCEPTED);
}

/* Elapsed us since the trigger at the strike tick that reached `target`. */
static uint32_t sim_until(strike_state_t target, uint32_t limit_us)
{
    while (strike_get_state() != target) {
        assert(((test_clock - sim_start) & 0xFFFFFFu) < limit_us);
        sim_tick();
    }
    return (test_clock - sim_start) & 0xFFFFFFu;
}

static void test_strike_contact_watch(void)
{
    const uint16_t timed = STRIKE_TIMING_COAST_VALID | STRIKE_TIMING_IMPACT_VALID |
                           STRIKE_TIMING_REBOUND_VALID | STRIKE_TIMING_VELOCITY_VALID;
    const uint16_t lifted = STRIKE_TIMING_STALLED | STRIKE_TIMING_NO_CONTACT |
                            STRIKE_TIMING_REBOUND_TIMEOUT;
    const uint32_t clocks[2] = {1000u, 0xFFFFFFu - 4000u};   /* second wraps TIMER2 mid-strike */
    strike_compact_t c;
    uint32_t caught;

    /* Clean hit: impact at the interpolated 992-count crossing (9.92 ms),
     * raw contact speed (100 counts/ms = 2197 deg/s), and the catch within
     * one encoder sample plus one strike tick of the 12-count retreat. */
    for (unsigned i = 0; i < 2u; ++i) {
        start_strike_sim(path_rebound, clocks[i], STRIKE_TYPE_NORMAL);
        caught = sim_until(STRIKE_CATCHING, 100000u);
        strike_get_compact_metrics(&c);
        assert((c.flags & timed) == timed && (c.flags & lifted) == 0u);
        assert(c.trigger_to_impact_01ms == 99u && c.trigger_to_impact_ms == 10u);
        assert(c.estimated_strike_velocity_dps >= 2160u && c.estimated_strike_velocity_dps <= 2240u);
        assert(c.trigger_to_rebound_ms >= 11u && c.trigger_to_rebound_ms <= 12u);
        assert(caught > 10840u && caught <= 10840u + 667u + 2000u + 167u);
    }

    /* Moving away at trigger must not read as a rebound before the hit. */
    start_strike_sim(path_backswing, 1000u, STRIKE_TYPE_NORMAL);
    caught = sim_until(STRIKE_CATCHING, 100000u);
    strike_get_compact_metrics(&c);
    assert((c.flags & timed) == timed && (c.flags & lifted) == 0u);
    assert(c.trigger_to_impact_01ms == 135u && caught > 14240u);

    /* Soft hit resting on the surface: lifted ~10 ms after it stops, not at
     * the 500 ms coast timeout. Impact is still timed. */
    start_strike_sim(path_rest, 1000u, STRIKE_TYPE_NORMAL);
    caught = sim_until(STRIKE_CATCHING, 100000u);
    strike_get_compact_metrics(&c);
    assert((c.flags & STRIKE_TIMING_IMPACT_VALID) && (c.flags & STRIKE_TIMING_STALLED));
    assert((c.flags & (STRIKE_TIMING_NO_CONTACT | STRIKE_TIMING_REBOUND_TIMEOUT)) == 0u);
    assert(c.trigger_to_impact_01ms == 331u);
    assert(caught >= 34466u + 8000u && caught <= 34466u + 14000u);

    /* Stalled short: a missed note, reported and lifted early. */
    start_strike_sim(path_short, 1000u, STRIKE_TYPE_NORMAL);
    caught = sim_until(STRIKE_CATCHING, 100000u);
    strike_get_compact_metrics(&c);
    assert((c.flags & (STRIKE_TIMING_STALLED | STRIKE_TIMING_NO_CONTACT)) ==
           (STRIKE_TIMING_STALLED | STRIKE_TIMING_NO_CONTACT));
    assert((c.flags & STRIKE_TIMING_IMPACT_VALID) == 0u && caught <= 27466u + 14000u);

    /* Dead strike: crossing the contact point (992, 99.2 ms) must not pre-empt
     * the position-armed brake at the surface (1024, 102.4 ms) even though
     * strike ticks fall between them; MUTING is entered by the brake (still
     * latched), with the contact-point impact time. */
    start_strike_sim(path_dead_slow, 1000u, STRIKE_TYPE_DEAD);
    caught = sim_until(STRIKE_MUTING, 200000u);
    assert(motor_is_brake_engaged() && caught > 102400u);
    strike_get_compact_metrics(&c);
    assert((c.flags & (STRIKE_TIMING_DEAD | STRIKE_TIMING_IMPACT_VALID)) ==
           (STRIKE_TIMING_DEAD | STRIKE_TIMING_IMPACT_VALID));
    assert((c.flags & STRIKE_TIMING_NO_CONTACT) == 0u && c.trigger_to_impact_01ms == 992u);
    sim_path = 0;
}

int main(void)
{
    test_absolute_calibration();
    test_record_migration();
    test_stop_preserves_fault();
    test_strike_contact_watch();
    puts("Firmware regressions passed: 50 reboot geometries, legacy migration, all six fault latches, "
         "6 simulated strike paths.");
    return 0;
}
