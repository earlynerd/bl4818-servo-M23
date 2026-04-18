/*
 * Fractional-tick scheduler — shared between main.c and motor.c.
 *
 * Converts elapsed PWM samples into sub-rate ticks using a Bresenham-style
 * accumulator.  Returns 1 when a tick is due, 0 otherwise.  If more than one
 * tick was due (i.e. the caller fell behind), the extra count is reported
 * through *dropped_updates so the caller can log or compensate.
 */

#ifndef SCHED_UTIL_H
#define SCHED_UTIL_H

#include <stdint.h>
#include "m2003_config.h"

static inline uint8_t schedule_latest_from_samples(uint32_t *accum,
                                                   uint32_t elapsed_samples,
                                                   uint32_t target_hz,
                                                   uint32_t *dropped_updates)
{
    uint32_t extra_due;

    *accum += elapsed_samples * target_hz;
    if (*accum < PWM_FREQ_HZ) {
        if (dropped_updates != 0)
            *dropped_updates = 0u;
        return 0u;
    }

    *accum -= PWM_FREQ_HZ;
    if (*accum < PWM_FREQ_HZ) {
        if (dropped_updates != 0)
            *dropped_updates = 0u;
        return 1u;
    }

    extra_due = *accum / PWM_FREQ_HZ;
    *accum -= extra_due * PWM_FREQ_HZ;

    if (dropped_updates != 0)
        *dropped_updates = extra_due;

    return 1u;
}

#endif /* SCHED_UTIL_H */
