/*
 * Strike Module — Mallet strike, rebound, and catch sequencer
 *
 * State machine:
 *   IDLE (position hold at home) → DRIVING (open-loop toward drum)
 *   → COASTING (phases floating) → CATCHING (position servo to home)
 *   → IDLE
 *
 * Homing: low duty toward drum until stall → record drum surface →
 *         position servo to home offset above drum.
 */

#ifndef STRIKE_H
#define STRIKE_H

#include <stdint.h>

typedef enum {
    STRIKE_IDLE     = 0,
    STRIKE_HOMING   = 1,
    STRIKE_DRIVING  = 2,
    STRIKE_COASTING = 3,
    STRIKE_BRAKING  = 4,
    STRIKE_CATCHING = 5
} strike_state_t;

void strike_init(void);
void strike_tick(void);             /* call at 1 kHz */

/* Commands */
void strike_trigger(int32_t duty);  /* fire strike with given duty (loudness) */
void strike_home(void);             /* run homing sequence */
void strike_cancel(void);           /* abort sequence, return to idle */

/* Configuration (encoder counts) */
void strike_set_home_offset(int32_t counts);
void strike_set_coast_distance(int32_t counts);
void strike_set_homing_duty(int32_t duty);
int32_t strike_get_home_offset(void);
int32_t strike_get_coast_distance(void);
int32_t strike_get_homing_duty(void);
void strike_restore_calibration(int32_t drum_position, int32_t home_position);
void strike_shift_position_reference(int32_t delta);

/* Status */
strike_state_t strike_get_state(void);
int32_t  strike_get_drum_position(void);
int32_t  strike_get_home_position(void);
uint8_t  strike_is_homed(void);

#endif /* STRIKE_H */
