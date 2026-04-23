/*
 * Status LED Indicator on the shared SSI_CSN pin.
 *
 * The external status LED is wired to PB1 — the same pin that the MT6701
 * encoder driver uses for CSn (via an N-FET inversion: PB1=1 turns the
 * FET on, which both asserts CSn at the encoder and lights the LED).
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

/* Desired PB1 level between SSI frames (0 = off, 1 = on).  Read by
 * encoder_poll() at frame end. */
uint8_t indicator_get_csn_level(void);

#endif /* INDICATOR_H */
