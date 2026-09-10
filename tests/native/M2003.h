#ifndef TEST_M2003_H
#define TEST_M2003_H
#include <stdint.h>

/* Test IRQ model; the harness never accesses physical registers. */
extern uint32_t test_primask;
static inline uint32_t __get_PRIMASK(void) { return test_primask; }
static inline void __disable_irq(void) { test_primask = 1u; }
static inline void __enable_irq(void) { test_primask = 0u; }
void SYS_UnlockReg(void);
void SYS_LockReg(void);
#endif
