#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "M2003.h"
#include "fmc.h"
#include "crc16.h"
#include "encoder.h"
#include "motor.h"
#include "persist.h"
#include "strike.h"

/*
 * Two-page ping-pong wear-leveling layout:
 *
 *   page A @ FMC_APROM_BASE + FMC_APROM_SIZE - 2*FMC_FLASH_PAGE_SIZE
 *   page B @ FMC_APROM_BASE + FMC_APROM_SIZE - 1*FMC_FLASH_PAGE_SIZE
 *
 * Each record carries a monotonic sequence number.  On load the page with
 * the higher valid sequence is used.  On save we erase+write the OTHER
 * page first; only after that succeeds do we adopt it as current — so if
 * power is lost mid-write the previous record is still intact on the
 * untouched page, and the device boots cleanly with prior settings.
 *
 * The linker carves out both pages in PERSIST (see m2003.ld).  The record
 * Version 5 appends the raw absolute drum-contact angle used by the home-shift
 * warning. Versions 3 and 4 remain readable; their missing absolute-angle
 * reference is learned by the next successful home and stored on the next save.
 */

#define PERSIST_PAGE_COUNT   2u
#define PERSIST_REGION_BASE  (FMC_APROM_BASE + FMC_APROM_SIZE - \
                              PERSIST_PAGE_COUNT * FMC_FLASH_PAGE_SIZE)
#define PERSIST_MAGIC        0x31545350UL  /* "PST1" */
#define PERSIST_VERSION      5u
#define PERSIST_VERSION_V4   4u
#define PERSIST_VERSION_V3   3u

#define PERSIST_FLAG_ZERO_VALID         0x0001u
#define PERSIST_FLAG_STRIKE_CAL_VALID   0x0002u
#define PERSIST_FLAG_CSN_POLARITY_VALID 0x0004u
#define PERSIST_FLAG_STRIKE_ANGLE_VALID 0x0008u

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t flags;
    uint32_t sequence;
    uint16_t zero_angle;
    /* Splitting the previous reserved0 word in place keeps the layout
     * (and CRC coverage) identical for v3 records written before the
     * polarity feature existed — old records still validate, the flag
     * bit is just absent so autodetect runs once and re-saves. */
    uint8_t  encoder_csn_assert_level;
    uint8_t  reserved0;
    int32_t strike_drum_position;
    int32_t strike_home_position;
    int32_t strike_home_offset;
    int32_t strike_coast_distance;
    int32_t strike_homing_duty;
    int32_t vel_kp;
    int32_t vel_ki;
    int32_t vel_kd;
    int32_t vel_ff;
    int32_t pos_kp;
    int32_t pos_ki;
    int32_t pos_kd;
    uint32_t torque_limit_ma;
    int32_t cur_kp;
    int32_t cur_ki;
    int32_t strike_mute_brake_ms;
    int32_t strike_mute_press_ma;
    int32_t strike_mute_engage_offset;
    uint16_t strike_drum_angle;
    uint16_t crc;
    uint16_t reserved1;
    uint16_t reserved2;
} persist_record_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t flags;
    uint32_t sequence;
    uint16_t zero_angle;
    uint8_t  encoder_csn_assert_level;
    uint8_t  reserved0;
    int32_t strike_drum_position;
    int32_t strike_home_position;
    int32_t strike_home_offset;
    int32_t strike_coast_distance;
    int32_t strike_homing_duty;
    int32_t vel_kp;
    int32_t vel_ki;
    int32_t vel_kd;
    int32_t vel_ff;
    int32_t pos_kp;
    int32_t pos_ki;
    int32_t pos_kd;
    uint32_t torque_limit_ma;
    int32_t cur_kp;
    int32_t cur_ki;
    int32_t strike_mute_brake_ms;
    int32_t strike_mute_press_ma;
    int32_t strike_mute_engage_offset;
    uint16_t crc;
    uint16_t reserved1;
} persist_record_v4_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t flags;
    uint32_t sequence;
    uint16_t zero_angle;
    uint8_t  encoder_csn_assert_level;
    uint8_t  reserved0;
    int32_t strike_drum_position;
    int32_t strike_home_position;
    int32_t strike_home_offset;
    int32_t strike_coast_distance;
    int32_t strike_homing_duty;
    int32_t vel_kp;
    int32_t vel_ki;
    int32_t vel_kd;
    int32_t vel_ff;
    int32_t pos_kp;
    int32_t pos_ki;
    int32_t pos_kd;
    uint32_t torque_limit_ma;
    int32_t cur_kp;
    int32_t cur_ki;
    uint16_t crc;
    uint16_t reserved1;
} persist_record_v3_t;

typedef char persist_record_alignment_check[
    (sizeof(persist_record_t) % sizeof(uint32_t) == 0u) ? 1 : -1];

typedef char persist_record_v5_layout_check[
    (sizeof(persist_record_t) == 96u &&
     offsetof(persist_record_t, crc) == 90u) ? 1 : -1];

typedef char persist_record_v4_layout_check[
    (sizeof(persist_record_v4_t) == 92u &&
     offsetof(persist_record_v4_t, crc) == 88u) ? 1 : -1];

typedef char persist_record_v3_layout_check[
    (sizeof(persist_record_v3_t) == 80u &&
     offsetof(persist_record_v3_t, crc) == 76u) ? 1 : -1];

typedef char persist_record_fits_page[
    (sizeof(persist_record_t) <= FMC_FLASH_PAGE_SIZE) ? 1 : -1];

static uint8_t  persist_valid;
static uint8_t  current_page;
static uint32_t current_seq;

static uint32_t persist_page_addr(uint8_t index)
{
    return PERSIST_REGION_BASE + ((uint32_t)index * FMC_FLASH_PAGE_SIZE);
}

static uint16_t persist_crc(const persist_record_t *record)
{
    return crc16_ccitt((const uint8_t *)record,
                       (uint8_t)offsetof(persist_record_t, crc));
}

static uint8_t persist_record_is_valid(const persist_record_t *record)
{
    if (record->magic != PERSIST_MAGIC)
        return 0u;
    if (record->version == PERSIST_VERSION)
        return (persist_crc(record) == record->crc) ? 1u : 0u;
    if (record->version == PERSIST_VERSION_V4) {
        const persist_record_v4_t *v4 = (const persist_record_v4_t *)record;
        uint16_t crc = crc16_ccitt((const uint8_t *)v4,
                                   (uint8_t)offsetof(persist_record_v4_t, crc));
        return (crc == v4->crc) ? 1u : 0u;
    }
    if (record->version == PERSIST_VERSION_V3) {
        const persist_record_v3_t *v3 = (const persist_record_v3_t *)record;
        uint16_t crc = crc16_ccitt((const uint8_t *)v3,
                                   (uint8_t)offsetof(persist_record_v3_t, crc));
        return (crc == v3->crc) ? 1u : 0u;
    }
    return 0u;
}

static void persist_apply_record(const persist_record_t *record)
{
    strike_set_home_offset(record->strike_home_offset);
    strike_set_coast_distance(record->strike_coast_distance);
    strike_set_homing_duty(record->strike_homing_duty);

    motor_set_torque_limit(record->torque_limit_ma);
    motor_set_vel_pid(record->vel_kp, record->vel_ki, record->vel_kd);
    motor_set_vel_ff(record->vel_ff);
    motor_set_pos_pid(record->pos_kp, record->pos_ki, record->pos_kd);
    motor_set_cur_pid(record->cur_kp, record->cur_ki);

    if (record->version >= PERSIST_VERSION_V4) {
        strike_set_mute_brake_ms(record->strike_mute_brake_ms);
        strike_set_mute_press_ma(record->strike_mute_press_ma);
        strike_set_mute_engage_offset(record->strike_mute_engage_offset);
    }

    if ((record->flags & PERSIST_FLAG_ZERO_VALID) != 0u)
        encoder_set_zero_reference(record->zero_angle);

    if ((record->flags & PERSIST_FLAG_CSN_POLARITY_VALID) != 0u)
        encoder_set_csn_polarity(record->encoder_csn_assert_level);

    if ((record->flags & PERSIST_FLAG_STRIKE_CAL_VALID) != 0u)
        strike_restore_calibration(record->strike_drum_position,
                                   record->strike_home_position);

    if (record->version >= PERSIST_VERSION &&
        (record->flags & PERSIST_FLAG_STRIKE_ANGLE_VALID) != 0u)
        strike_restore_drum_angle(record->strike_drum_angle);
}

static void persist_capture_runtime(persist_record_t *record, uint32_t sequence)
{
    memset(record, 0, sizeof(*record));

    record->magic = PERSIST_MAGIC;
    record->version = PERSIST_VERSION;
    record->sequence = sequence;

    if (encoder_has_zero_reference()) {
        record->flags |= PERSIST_FLAG_ZERO_VALID;
        record->zero_angle = encoder_get_zero_reference();
    }

    if (encoder_has_csn_polarity()) {
        record->flags |= PERSIST_FLAG_CSN_POLARITY_VALID;
        record->encoder_csn_assert_level = encoder_get_csn_polarity();
    }

    if (strike_is_homed()) {
        record->flags |= PERSIST_FLAG_STRIKE_CAL_VALID;
        record->strike_drum_position = strike_get_drum_position();
        record->strike_home_position = strike_get_home_position();
        if (strike_has_drum_angle()) {
            record->flags |= PERSIST_FLAG_STRIKE_ANGLE_VALID;
            record->strike_drum_angle = strike_get_drum_angle();
        }
    }

    record->strike_home_offset = strike_get_home_offset();
    record->strike_coast_distance = strike_get_coast_distance();
    record->strike_homing_duty = strike_get_homing_duty();

    motor_get_vel_pid(&record->vel_kp, &record->vel_ki, &record->vel_kd);
    record->vel_ff = motor_get_vel_ff();
    motor_get_pos_pid(&record->pos_kp, &record->pos_ki, &record->pos_kd);
    record->torque_limit_ma = motor_get_torque_limit();
    motor_get_cur_pid(&record->cur_kp, &record->cur_ki);
    record->strike_mute_brake_ms = strike_get_mute_brake_ms();
    record->strike_mute_press_ma = strike_get_mute_press_ma();
    record->strike_mute_engage_offset = strike_get_mute_engage_offset();
    record->crc = persist_crc(record);
}

static int32_t persist_write_page(uint8_t page_index, const persist_record_t *record)
{
    uint32_t primask = __get_PRIMASK();
    const uint32_t *words = (const uint32_t *)record;
    uint32_t page_addr = persist_page_addr(page_index);
    uint32_t i;
    int32_t status = 0;

    __disable_irq();
    SYS_UnlockReg();

    FMC_Open();
    FMC_ENABLE_AP_UPDATE();

    if (FMC_Erase(page_addr) != 0) {
        status = -1;
        goto cleanup;
    }

    for (i = 0; i < (sizeof(*record) / sizeof(uint32_t)); i++) {
        uint32_t addr = page_addr + (i * sizeof(uint32_t));
        if (FMC_Write(addr, words[i]) != 0) {
            status = -1;
            goto cleanup;
        }
        if (FMC_Read(addr) != words[i]) {
            status = -1;
            goto cleanup;
        }
    }

cleanup:
    FMC_DISABLE_AP_UPDATE();
    FMC_Close();
    SYS_LockReg();
    if (primask == 0u)
        __enable_irq();

    return status;
}

static int32_t persist_erase_page(uint8_t page_index)
{
    uint32_t primask = __get_PRIMASK();
    uint32_t page_addr = persist_page_addr(page_index);
    int32_t status = 0;

    __disable_irq();
    SYS_UnlockReg();

    FMC_Open();
    FMC_ENABLE_AP_UPDATE();

    if (FMC_Erase(page_addr) != 0)
        status = -1;

    FMC_DISABLE_AP_UPDATE();
    FMC_Close();
    SYS_LockReg();
    if (primask == 0u)
        __enable_irq();

    return status;
}

void persist_init(void)
{
    const persist_record_t *r0 = (const persist_record_t *)persist_page_addr(0);
    const persist_record_t *r1 = (const persist_record_t *)persist_page_addr(1);
    uint8_t v0 = persist_record_is_valid(r0);
    uint8_t v1 = persist_record_is_valid(r1);
    const persist_record_t *use = NULL;

    if (v0 && v1) {
        /* Signed wraparound-safe compare — whichever seq is 'ahead' wins. */
        if ((int32_t)(r1->sequence - r0->sequence) > 0) {
            use = r1;
            current_page = 1u;
        } else {
            use = r0;
            current_page = 0u;
        }
    } else if (v0) {
        use = r0;
        current_page = 0u;
    } else if (v1) {
        use = r1;
        current_page = 1u;
    } else {
        persist_valid = 0u;
        current_page = 0u;          /* first save will land on page 0 */
        current_seq  = 0u;
        return;
    }

    current_seq = use->sequence;
    persist_valid = 1u;
    persist_apply_record(use);
}

int32_t persist_save_runtime(void)
{
    persist_record_t record;
    uint8_t  next_page = (uint8_t)(1u - current_page);
    uint32_t next_seq  = current_seq + 1u;
    int32_t  status;

    persist_capture_runtime(&record, next_seq);

    status = persist_write_page(next_page, &record);
    if (status == 0) {
        current_page = next_page;
        current_seq  = next_seq;
        persist_valid = 1u;
    }

    return status;
}

int32_t persist_clear(void)
{
    int32_t status_a = persist_erase_page(0);
    int32_t status_b = persist_erase_page(1);

    persist_valid = 0u;
    current_page = 0u;
    current_seq  = 0u;

    return (status_a != 0 || status_b != 0) ? -1 : 0;
}

uint8_t persist_is_valid(void)
{
    return persist_valid;
}
