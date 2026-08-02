#include <stdint.h>
#include "bl_internal.h"

extern uint32_t _sidata;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;
extern uint32_t _estack;
extern int main(void);

void Reset_Handler(void)
{
    uint32_t *src;
    uint32_t *dst;

    bl_early_safe_outputs();

    src = &_sidata;
    dst = &_sdata;
    while (dst < &_edata)
        *dst++ = *src++;

    dst = &_sbss;
    while (dst < &_ebss)
        *dst++ = 0u;

    (void)main();
    for (;;) {
    }
}

void Default_Handler(void)
{
    bl_early_safe_outputs();
    for (;;) {
    }
}

void UART1_IRQHandler(void);

__attribute__((section(".vectors"), used))
const uintptr_t bl_vectors[16u + 38u] =
{
    [0] = (uintptr_t)&_estack,
    [1] = (uintptr_t)Reset_Handler,
    [2] = (uintptr_t)Default_Handler,
    [3] = (uintptr_t)Default_Handler,
    [16u + 37u] = (uintptr_t)UART1_IRQHandler
};
