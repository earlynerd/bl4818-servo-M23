/*
 * IRQ save/restore helpers — shared across all modules.
 */

#ifndef IRQ_UTIL_H
#define IRQ_UTIL_H

#include <stdint.h>
#include "M2003.h"

static inline uint32_t irq_save(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static inline void irq_restore(uint32_t primask)
{
    if ((primask & 1u) == 0u)
        __enable_irq();
}

#endif /* IRQ_UTIL_H */
