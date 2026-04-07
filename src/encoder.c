/*
 * Bit-Banged SSI Encoder Driver for Nuvoton M2003
 *
 * Designed for MT6701 or similar 24-bit SSI absolute magnetic encoders.
 */

#include "m2003_config.h"
#include "M2003.h"
#include "encoder.h"

static uint32_t encoder_raw = 0;
static uint16_t encoder_angle = 0;
static uint16_t encoder_prev_angle = 0;
static uint16_t encoder_zero_angle = 0;
static int32_t  encoder_position_raw = 0;      /* continuous unwrapped position */
static int32_t  encoder_position_offset = 0;   /* logical zero reference */
static uint8_t  encoder_initialized = 0;
static uint8_t  encoder_zero_valid = 0;

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

    /*
     * Initial state: CLK high (idle), CSn high (idle).
     * CSn pin is inverted via N-FET: writing 0 turns FET OFF,
     * pull-up pulls the pad high (de-asserted).
     */
    PIN_SSI_CLK = 1;
    PIN_SSI_CSN = 0;
}

void encoder_poll(void)
{
    uint32_t raw = 0;
    int i;

    /* Assert CSn to start the frame */
    PIN_SSI_CSN = 1;
    delay_nop(2);

    /* Read 24 bits */
    for (i = 0; i < 24; i++) {
        /* CLK goes LOW  */
        PIN_SSI_CLK = 0;
        //delay_nop(2);

        /* CLK goes HIGH */
        PIN_SSI_CLK = 1;
        //delay_nop(2);

        /* Read data just after the rising edge */
        raw <<= 1;
        if (PIN_SSI_DAT) {
            raw |= 1;
        }
    }
    delay_nop(2); /* Minimum quiet time before next frame */
    /* De-assert CSn to end the frame */
    PIN_SSI_CSN = 0;
    

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
