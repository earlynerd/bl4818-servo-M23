/*
 * Status LED Indicator on the shared SSI_CSN pin.
 *
 * The external status LED is wired to PB1 — the same pin that the MT6701
 * encoder driver uses for CSn.  Lighting the LED and asserting CSn are the
 * same electrical event, so the LED lights at PB1's CSn *assert* level.  That
 * level is board-dependent (PB1=1 on the modchip variant, PB1=0 on the
 * FET-inverting / CSn-active-low variant), so the indicator maps its patterns
 * through the autodetected polarity (encoder_get_csn_polarity).
 *
 * This module cooperates with encoder_poll().  encoder_poll() owns the pin
 * for the ~15 µs duration of each 24-bit SSI frame and restores PB1 to the
 * indicator's published idle level on frame exit, so the LED goes dark
 * only for the frame itself (~3.7% duty at 2.5 kHz polling).  Between
 * frames the indicator drives the pin according to a pattern selected from
 * motor_get_state() / motor_get_fault().
 */

#ifndef INDICATOR_H
#define INDICATOR_H

#include <stdint.h>

void indicator_init(void);

/* Step the pattern state machine.  Expected cadence: INDICATOR_TICK_HZ
 * (configured in m2003_config.h).  Writes PIN_SSI_CSN and publishes the
 * level that encoder_poll() should restore on frame exit. */
void indicator_tick(void);

/* Desired physical PB1 level between SSI frames (already mapped through the
 * board's CSn polarity — equals the assert level when the LED should be lit).
 * Read by encoder_poll() at frame end. */
uint8_t indicator_get_csn_level(void);

#endif /* INDICATOR_H */
