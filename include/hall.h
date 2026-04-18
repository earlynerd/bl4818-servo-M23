/*
 * Hall Sensor Interface
 */
#ifndef HALL_H
#define HALL_H

#include <stdint.h>

void hall_init(void);
uint8_t hall_read_raw(void);
uint8_t hall_decode_state(uint8_t raw_state);
uint8_t hall_read(void);
uint8_t hall_sector(void);
int8_t hall_direction(void);
uint32_t hall_period(void);
int32_t hall_count(void);
void hall_count_reset(void);

#define HALL_POLL_NO_CHANGE  0
#define HALL_POLL_TRANSITION 1
#define HALL_POLL_INVALID    2
uint8_t hall_poll(void);

/* Called periodically from a slower context to prevent the 24-bit timing
 * baseline from aliasing after long idle.  Cheap: one register read plus an
 * IRQ-protected compare in the common path. */
void hall_refresh_baseline(void);

#endif /* HALL_H */
