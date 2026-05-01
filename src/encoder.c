/*
 * Bit-Banged SSI Encoder Driver for Nuvoton M2003
 *
 * Designed for MT6701 or similar 24-bit SSI absolute magnetic encoders.
 */

#include "m2003_config.h"
#include "M2003.h"
#include "encoder.h"
#include "irq_util.h"

static uint32_t encoder_raw = 0;
static uint16_t encoder_angle = 0;
static uint16_t encoder_prev_angle = 0;
static uint16_t encoder_zero_angle = 0;
static int32_t  encoder_position_raw = 0;      /* continuous unwrapped position */
static int32_t  encoder_position_offset = 0;   /* logical zero reference */
static uint8_t  encoder_initialized = 0;
static uint8_t  encoder_zero_valid = 0;

/* PB1 level that asserts CSn at the encoder.  0 matches the FET-inverting
 * PCB variant; 1 matches the modchip variant.  csn_polarity_set tracks
 * whether the value was learned (autodetect) or applied from persist —
 * boot uses it to decide whether to run autodetect. */
static uint8_t  csn_assert_level = 0u;
static uint8_t  csn_polarity_set = 0u;

static int16_t angle_delta(uint16_t current, uint16_t reference)
{
    int16_t delta = (int16_t)(current - reference);

    if (delta > 8192)
        delta -= 16384;
    else if (delta < -8192)
        delta += 16384;

    return delta;
}

static void recompute_position_offset(void)
{
    if (!encoder_initialized || !encoder_zero_valid)
        return;

    encoder_position_offset =
        (int32_t)angle_delta(encoder_angle, encoder_zero_angle) -
        encoder_position_raw;
}

/* Simple delay using NOPs */
static inline void delay_nop(uint32_t n)
{
    while (n--) {
        __NOP();
    }
}

/* MT6701 SSI frame: [23:10] = 14-bit angle, [9:6] = 4-bit status,
 * [5:0] = 6-bit CRC over [23:6].  Polynomial x^6 + x + 1 (0x03), init 0,
 * MSB-first, no reflection, no final XOR.  Used only during polarity
 * autodetect — the per-poll cost is too high for the runtime hot path. */
static uint8_t mt6701_crc6(uint32_t data18)
{
    uint8_t crc = 0u;
    int i;

    for (i = 17; i >= 0; --i) {
        uint8_t in_bit   = (uint8_t)((data18 >> i) & 1u);
        uint8_t feedback = (uint8_t)(((crc >> 5) ^ in_bit) & 1u);
        crc = (uint8_t)((crc << 1) & 0x3Fu);
        if (feedback)
            crc ^= 0x03u;
    }

    return crc;
}

static uint8_t encoder_validate_crc(uint32_t raw)
{
    uint32_t upper18 = (raw >> 6) & 0x3FFFFu;
    uint8_t  expected = (uint8_t)(raw & 0x3Fu);

    return (mt6701_crc6(upper18) == expected) ? 1u : 0u;
}

/* Bit-bang the 24-bit SSI frame using the active csn_assert_level.  Does
 * not touch encoder_angle / encoder_position_raw / encoder_initialized — the
 * caller decides whether to fold the result into the unwrap state.  This
 * lets autodetect probe each polarity without poisoning the position
 * counter when the wrong polarity yields a garbage frame. */
static uint32_t encoder_read_raw(void)
{
    uint32_t raw = 0;
    uint32_t irq_state;
    int i;
    uint8_t assert_level   = csn_assert_level;
    uint8_t deassert_level = (uint8_t)(assert_level ^ 1u);

    irq_state = irq_save();

    PIN_SSI_CSN = assert_level;
    delay_nop(2);

    for (i = 0; i < 24; i++) {
        PIN_SSI_CLK = 0;
        PIN_SSI_CLK = 1;
        raw <<= 1;
        if (PIN_SSI_DAT) {
            raw |= 1;
        }
    }

    PIN_SSI_CSN = deassert_level;
    irq_restore(irq_state);

    return raw;
}

void encoder_init(void)
{
    encoder_raw = 0;
    encoder_angle = 0;
    encoder_prev_angle = 0;
    encoder_position_raw = 0;
    encoder_position_offset = 0;
    encoder_initialized = 0;
    encoder_zero_valid = 0;

    /* Ensure GPIOB clock is enabled */
    CLK->AHBCLK |= CLK_AHBCLK_GPBCKEN_Msk;

    /*
     * Configure Pins:
     * PB.0 (Pin 20) (original function: PWM input) -> SSI CLK (Output)
     * PB.1 (Pin 1) (original function: tach output) -> SSI CSn (Output, inverted via N-FET) 
     * PB.15 (Pin 11) (original function: direction input)-> SSI DAT (Input)
     */
    GPIO_SetMode(PB, BIT0 | BIT1, GPIO_MODE_OUTPUT);
    GPIO_SetSlewCtl(PB, BIT0 | BIT1, GPIO_SLEWCTL_HIGH);
    GPIO_SetMode(PB, BIT15, GPIO_MODE_INPUT);

    /* Enable digital input path for DAT */
    GPIO_ENABLE_DIGITAL_PATH(PB, BIT15);

    /* Idle: CLK high, CSn de-asserted (opposite of csn_assert_level).
     * Polarity defaults to 0 here; persist_apply_record may overwrite it
     * before the first encoder_poll, and main.c runs autodetect after
     * persist_init if no record is stored. */
    PIN_SSI_CLK = 1;
    PIN_SSI_CSN = (uint8_t)(csn_assert_level ^ 1u);
}

void encoder_poll(void)
{
    uint32_t raw = encoder_read_raw();

    encoder_raw = raw;

    /* MT6701's 14-bit angle is in the top 14 bits of the 24-bit frame */
    encoder_angle = (uint16_t)(raw >> 10);

    /* Unwrap into continuous multi-turn position */
    if (!encoder_initialized) {
        encoder_prev_angle = encoder_angle;
        encoder_initialized = 1;
        recompute_position_offset();
    } else {
        encoder_position_raw += angle_delta(encoder_angle, encoder_prev_angle);
        encoder_prev_angle = encoder_angle;
    }
}

uint32_t encoder_get_raw(void)
{
    return encoder_raw;
}

uint16_t encoder_get_angle(void)
{
    return encoder_angle;
}

int32_t encoder_get_position(void)
{
    return encoder_position_raw + encoder_position_offset;
}

void encoder_set_zero_reference(uint16_t angle)
{
    encoder_zero_angle = (uint16_t)(angle & 0x3FFFu);
    encoder_zero_valid = 1;
    recompute_position_offset();
}

uint8_t encoder_has_zero_reference(void)
{
    return encoder_zero_valid;
}

uint16_t encoder_get_zero_reference(void)
{
    return encoder_zero_angle;
}

void encoder_reset_position(void)
{
    if (!encoder_initialized)
        return;

    encoder_zero_angle = encoder_angle;
    encoder_zero_valid = 1;
    recompute_position_offset();
}

void encoder_set_csn_polarity(uint8_t assert_level)
{
    uint32_t irq_state = irq_save();
    csn_assert_level = assert_level ? 1u : 0u;
    csn_polarity_set = 1u;
    /* Park the pin at the new de-assert (idle) level immediately. */
    PIN_SSI_CSN = (uint8_t)(csn_assert_level ^ 1u);
    irq_restore(irq_state);
}

uint8_t encoder_get_csn_polarity(void)
{
    return csn_assert_level;
}

uint8_t encoder_has_csn_polarity(void)
{
    return csn_polarity_set;
}

uint8_t encoder_autodetect_csn_polarity(void)
{
    uint32_t raw_a, raw_b;
    uint8_t  valid_a, valid_b;
    uint8_t  saved_initialized = encoder_initialized;

    /* Probe each polarity once and keep the one whose 24-bit frame
     * passes the MT6701 CRC.  encoder_read_raw deliberately doesn't
     * touch unwrap state, so a wrong-polarity probe can't corrupt
     * encoder_position_raw — but we still suppress the next normal
     * encoder_poll's delta by clearing encoder_initialized, so the
     * first real poll after autodetect re-seeds prev_angle cleanly. */

    csn_assert_level = 0u;
    raw_a = encoder_read_raw();
    valid_a = encoder_validate_crc(raw_a);

    csn_assert_level = 1u;
    raw_b = encoder_read_raw();
    valid_b = encoder_validate_crc(raw_b);

    encoder_initialized = 0u;

    if (valid_a && !valid_b) {
        encoder_set_csn_polarity(0u);
        return 1u;
    }
    if (valid_b && !valid_a) {
        encoder_set_csn_polarity(1u);
        return 1u;
    }

    /* Both passed (anomaly) or neither passed (encoder unresponsive).
     * Restore default polarity and prior init state — caller will see
     * the failure return and can retry, surface a fault, or leave it. */
    csn_assert_level = 0u;
    PIN_SSI_CSN = 1u;
    encoder_initialized = saved_initialized;
    return 0u;
}
