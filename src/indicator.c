/*
 * Status LED Indicator (drives PB1 shared with SSI_CSN).
 *
 * Patterns:
 *   MOTOR_IDLE   — slow two-blink heartbeat, 1.5 s period
 *   MOTOR_RUN    — solid on
 *   MOTOR_FAULT  — N blinks then a 1 s pause, where N = fault_code_t value
 *                  (1=overcurrent, 2=hall, 3=encoder, 4=adc)
 *
 * The LED lights when PB1 sits at the encoder's CSn *assert* level (the FET
 * that asserts CSn also drives the LED).  That level differs per board
 * variant, so every pattern's logical "lit" is mapped to the physical pin
 * through the autodetected polarity (encoder_get_csn_polarity) rather than a
 * fixed high — otherwise the indicator reads inverted on CSn-active-low
 * boards.
 */

#include <stdint.h>
#include "indicator.h"
#include "m2003_config.h"
#include "M2003.h"
#include "motor.h"
#include "encoder.h"

#define IDLE_PERIOD_TICKS   (INDICATOR_TICK_HZ * 3u / 2u)   /* 1.5 s */
#define FAULT_BLINK_TICKS   (INDICATOR_TICK_HZ / 5u)        /* 200 ms per blink slot */
#define FAULT_BLINK_ON      (FAULT_BLINK_TICKS * 2u / 5u)   /* 80 ms on  */
#define FAULT_PAUSE_TICKS   INDICATOR_TICK_HZ               /* 1 s gap after blink group */

static volatile uint8_t csn_level;
static uint16_t         phase_ticks;
static motor_state_t    prev_state;
static fault_code_t     prev_fault;

/* Translate a logical lit/dark intent into the physical PB1 level for the
 * current board.  "Lit" is the autodetected CSn assert level; "dark" is its
 * complement (the idle/de-assert level the encoder also parks at between
 * frames).  On the modchip variant (assert=1) this is the identity map; on
 * the FET-inverting / CSn-active-low variant (assert=0) it inverts. */
static uint8_t pin_level_for(uint8_t lit)
{
    uint8_t assert_level = encoder_get_csn_polarity();
    return lit ? assert_level : (uint8_t)(assert_level ^ 1u);
}

uint8_t indicator_get_csn_level(void)
{
    return csn_level;
}

void indicator_init(void)
{
    /* Park dark (LED off) at the de-assert level for this board's polarity. */
    uint8_t off_level = pin_level_for(0u);

    csn_level = off_level;
    phase_ticks = 0u;
    prev_state = MOTOR_IDLE;
    prev_fault = FAULT_NONE;
    PIN_SSI_CSN = off_level;
}

static uint8_t pattern_idle(uint16_t t)
{
    /* Double-tap heartbeat inside the 1.5 s window.  Values in ticks at
     * INDICATOR_TICK_HZ (default 100 Hz = 10 ms/tick). */
    uint16_t on1_end  = INDICATOR_TICK_HZ / 25u;            /* 40 ms  */
    uint16_t gap_end  = INDICATOR_TICK_HZ * 4u / 25u;       /* 160 ms */
    uint16_t on2_end  = INDICATOR_TICK_HZ / 5u;             /* 200 ms */

    if (t < on1_end)  return 1u;
    if (t < gap_end)  return 0u;
    if (t < on2_end)  return 1u;
    return 0u;
}

static uint8_t pattern_fault(uint16_t t, fault_code_t code)
{
    uint8_t  blinks = (uint8_t)code;
    uint16_t blink_window;
    uint16_t period;
    uint16_t p;

    if (blinks == 0u || blinks > 9u)
        blinks = 1u;

    blink_window = (uint16_t)blinks * FAULT_BLINK_TICKS;
    period = blink_window + FAULT_PAUSE_TICKS;
    p = (period != 0u) ? (uint16_t)(t % period) : 0u;

    if (p < blink_window) {
        uint16_t within = (uint16_t)(p % FAULT_BLINK_TICKS);
        return (within < FAULT_BLINK_ON) ? 1u : 0u;
    }
    return 0u;
}

void indicator_tick(void)
{
    motor_state_t s = motor_get_state();
    fault_code_t  f = motor_get_fault();
    uint8_t       level;

    if (s != prev_state || f != prev_fault) {
        phase_ticks = 0u;
        prev_state = s;
        prev_fault = f;
    }

    switch (s) {
    case MOTOR_RUN:
        level = 1u;
        break;
    case MOTOR_FAULT:
        level = pattern_fault(phase_ticks, f);
        break;
    case MOTOR_IDLE:
    default:
        level = pattern_idle(phase_ticks);
        break;
    }

    /* Map logical on/off onto the pin so the pattern reads the same on both
     * board variants; csn_level publishes the physical level for the encoder
     * to restore on frame exit. */
    level = pin_level_for(level);
    csn_level = level;
    PIN_SSI_CSN = level;

    /* Wrap well above the longest pattern period (fault at code 9 ≈ 2.8 s). */
    if (++phase_ticks >= (uint16_t)(INDICATOR_TICK_HZ * 10u)) {
        phase_ticks = 0u;
    }
}
