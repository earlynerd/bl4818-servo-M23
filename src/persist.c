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

#define PERSIST_FLASH_BASE   (FMC_APROM_BASE + FMC_APROM_SIZE - FMC_FLASH_PAGE_SIZE)
#define PERSIST_MAGIC        0x31545350UL  /* "PST1" */
#define PERSIST_VERSION      1u

#define PERSIST_FLAG_ZERO_VALID       0x0001u
#define PERSIST_FLAG_STRIKE_CAL_VALID 0x0002u

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t flags;
    uint16_t zero_angle;
    uint16_t reserved0;
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
    uint16_t crc;
    uint16_t reserved1;
} persist_record_t;

typedef char persist_record_alignment_check[
    (sizeof(persist_record_t) % sizeof(uint32_t) == 0u) ? 1 : -1];

static uint8_t persist_valid;

static uint16_t persist_crc(const persist_record_t *record)
{
    return crc16_ccitt((const uint8_t *)record,
                       (uint8_t)offsetof(persist_record_t, crc));
}

static uint8_t persist_record_is_valid(const persist_record_t *record)
{
    if (record->magic != PERSIST_MAGIC)
        return 0u;
    if (record->version != PERSIST_VERSION)
        return 0u;
    if (persist_crc(record) != record->crc)
        return 0u;
    return 1u;
}

static void persist_capture_runtime(persist_record_t *record)
{
    memset(record, 0, sizeof(*record));

    record->magic = PERSIST_MAGIC;
    record->version = PERSIST_VERSION;

    if (encoder_has_zero_reference()) {
        record->flags |= PERSIST_FLAG_ZERO_VALID;
        record->zero_angle = encoder_get_zero_reference();
    }

    if (strike_is_homed()) {
        record->flags |= PERSIST_FLAG_STRIKE_CAL_VALID;
        record->strike_drum_position = strike_get_drum_position();
        record->strike_home_position = strike_get_home_position();
    }

    record->strike_home_offset = strike_get_home_offset();
    record->strike_coast_distance = strike_get_coast_distance();
    record->strike_homing_duty = strike_get_homing_duty();

    motor_get_vel_pid(&record->vel_kp, &record->vel_ki, &record->vel_kd);
    record->vel_ff = motor_get_vel_ff();
    motor_get_pos_pid(&record->pos_kp, &record->pos_ki, &record->pos_kd);
    record->torque_limit_ma = motor_get_torque_limit();
    record->crc = persist_crc(record);
}

static int32_t persist_write_record(const persist_record_t *record)
{
    uint32_t primask = __get_PRIMASK();
    const uint32_t *words = (const uint32_t *)record;
    uint32_t i;
    int32_t status = 0;

    __disable_irq();
    SYS_UnlockReg();

    FMC_Open();
    FMC_ENABLE_AP_UPDATE();

    if (FMC_Erase(PERSIST_FLASH_BASE) != 0) {
        status = -1;
        goto cleanup;
    }

    for (i = 0; i < (sizeof(*record) / sizeof(uint32_t)); i++) {
        uint32_t addr = PERSIST_FLASH_BASE + (i * sizeof(uint32_t));
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

void persist_init(void)
{
    const persist_record_t *record = (const persist_record_t *)PERSIST_FLASH_BASE;

    persist_valid = persist_record_is_valid(record);
    if (!persist_valid)
        return;

    strike_set_home_offset(record->strike_home_offset);
    strike_set_coast_distance(record->strike_coast_distance);
    strike_set_homing_duty(record->strike_homing_duty);

    motor_set_torque_limit(record->torque_limit_ma);
    motor_set_vel_pid(record->vel_kp, record->vel_ki, record->vel_kd);
    motor_set_vel_ff(record->vel_ff);
    motor_set_pos_pid(record->pos_kp, record->pos_ki, record->pos_kd);

    if ((record->flags & PERSIST_FLAG_ZERO_VALID) != 0u)
        encoder_set_zero_reference(record->zero_angle);

    if ((record->flags & PERSIST_FLAG_STRIKE_CAL_VALID) != 0u)
        strike_restore_calibration(record->strike_drum_position,
                                   record->strike_home_position);
}

int32_t persist_save_runtime(void)
{
    persist_record_t record;
    int32_t status;

    persist_capture_runtime(&record);
    status = persist_write_record(&record);
    persist_valid = (status == 0) ? 1u : 0u;

    return status;
}

int32_t persist_clear(void)
{
    uint32_t primask = __get_PRIMASK();
    int32_t status = 0;

    __disable_irq();
    SYS_UnlockReg();

    FMC_Open();
    FMC_ENABLE_AP_UPDATE();

    if (FMC_Erase(PERSIST_FLASH_BASE) != 0)
        status = -1;

    FMC_DISABLE_AP_UPDATE();
    FMC_Close();
    SYS_LockReg();
    if (primask == 0u)
        __enable_irq();

    persist_valid = 0;
    return status;
}

uint8_t persist_is_valid(void)
{
    return persist_valid;
}
