/*
 * Strike Module — Mallet strike, rebound, and catch sequencer
 *
 * State machine:
 *   IDLE (position hold at home) → DRIVING (current-controlled toward drum)
 *   → COASTING (phases floating through impact) → CATCHING
 *   (position servo captures the rebound back to home) → IDLE
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
    STRIKE_CATCHING = 5  /* wire value 4 is left unused to preserve legacy protocol numbering */
} strike_state_t;

typedef enum {
    STRIKE_TRIGGER_ACCEPTED = 0,
    STRIKE_TRIGGER_RETRIGGERED,
    STRIKE_TRIGGER_REJECT_NOT_HOMED,
    STRIKE_TRIGGER_REJECT_FAULT,
    STRIKE_TRIGGER_REJECT_ZERO,
    STRIKE_TRIGGER_REJECT_NOT_READY,
} strike_trigger_result_t;

/* Validity / state flags packed into a uint16. The low byte (0x01..0x80)
 * keeps the legacy bit assignments so the on-wire low-byte parsing is
 * unchanged for hosts that haven't been updated yet. */
#define STRIKE_TIMING_COAST_VALID      0x0001u
#define STRIKE_TIMING_REBOUND_VALID    0x0002u
#define STRIKE_TIMING_READY_VALID      0x0004u
#define STRIKE_TIMING_ACTIVE           0x0008u
#define STRIKE_TIMING_RETRIGGERED      0x0010u
#define STRIKE_TIMING_REBOUND_TIMEOUT  0x0020u
#define STRIKE_TIMING_VELOCITY_VALID   0x0040u
#define STRIKE_TIMING_RETRIGGER_READY_VALID 0x0080u
#define STRIKE_TIMING_IMPACT_VALID     0x0100u  /* trigger->mallet-stops time */

typedef struct {
    uint16_t flags;
    uint16_t sequence;
    int16_t  last_current_ma;
    uint16_t trigger_to_coast_ms;
    uint16_t trigger_to_impact_ms;
    uint16_t trigger_to_rebound_ms;
    uint16_t trigger_to_retrigger_ready_ms;
    uint16_t trigger_to_ready_ms;
    uint16_t estimated_strike_velocity_dps;
    int16_t  home_offset;
    int16_t  coast_distance;
    int16_t  homing_duty;
} strike_metrics_t;

/* Compact "last completed strike" snapshot. Captured at rebound detection
 * (when the major timing fields are all valid) and again whenever a new
 * strike begins (so a retriggered cycle still publishes whatever data we
 * had at the moment of retrigger). Used by piggyback-ACK and the compact
 * STRIKE_TIMING query to keep ring traffic small. */
typedef struct {
    uint16_t flags;
    uint16_t sequence;
    int16_t  last_current_ma;
    uint16_t trigger_to_coast_ms;
    uint16_t trigger_to_impact_ms;
    uint16_t trigger_to_rebound_ms;
    uint16_t estimated_strike_velocity_dps;
} strike_compact_t;

void strike_init(void);
void strike_tick(void);             /* call at STRIKE_LOOP_HZ */

/* Commands */
strike_trigger_result_t strike_trigger(int32_t current_ma);  /* fire strike with given current magnitude in mA */
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
uint16_t strike_get_sequence(void);
void strike_get_metrics(strike_metrics_t *metrics);
void strike_get_compact_metrics(strike_compact_t *compact);

#endif /* STRIKE_H */
