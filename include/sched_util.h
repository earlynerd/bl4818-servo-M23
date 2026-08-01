/*
 * Fractional-rate scheduler — shared between main.c and motor.c.
 *
 * Converts elapsed source units into sub-rate ticks using a Bresenham-style
 * accumulator. Returns 1 when a tick is due, 0 otherwise. If more than one
 * tick was due (i.e. the caller fell behind), the extra count is reported
 * through *dropped_updates so the caller can log or compensate.
 */

#ifndef SCHED_UTIL_H
#define SCHED_UTIL_H

#include <stdint.h>
#include "m2003_config.h"

static inline uint8_t schedule_latest_from_rate(uint32_t *accum,
                                                uint32_t elapsed_units,
                                                uint32_t target_hz,
                                                uint32_t source_hz,
                                                uint32_t *dropped_updates)
{
    uint32_t extra_due;

    *accum += elapsed_units * target_hz;
    if (*accum < source_hz) {
        if (dropped_updates != 0)
            *dropped_updates = 0u;
        return 0u;
    }

    *accum -= source_hz;
    if (*accum < source_hz) {
        if (dropped_updates != 0)
            *dropped_updates = 0u;
        return 1u;
    }

    extra_due = *accum / source_hz;
    *accum -= extra_due * source_hz;

    if (dropped_updates != 0)
        *dropped_updates = extra_due;

    return 1u;
}

static inline uint8_t schedule_latest_from_samples(uint32_t *accum,
                                                   uint32_t elapsed_samples,
                                                   uint32_t target_hz,
                                                   uint32_t *dropped_updates)
{
    return schedule_latest_from_rate(accum, elapsed_samples, target_hz,
                                     PWM_FREQ_HZ, dropped_updates);
}

static inline uint8_t schedule_latest_from_control_ticks(uint32_t *accum,
                                                         uint32_t elapsed_ticks,
                                                         uint32_t target_hz,
                                                         uint32_t *dropped_updates)
{
    return schedule_latest_from_rate(accum, elapsed_ticks, target_hz,
                                     CONTROL_LOOP_HZ, dropped_updates);
}

#endif /* SCHED_UTIL_H */
