#include <assert.h>
#include <stdio.h>
#include "m2003_config.h"
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
uint32_t timing_capture_stamp(void) { return test_clock; }

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

int main(void)
{
    test_absolute_calibration();
    test_record_migration();
    test_stop_preserves_fault();
    puts("Firmware regressions passed: 50 reboot geometries, legacy migration, all six fault latches.");
    return 0;
}
