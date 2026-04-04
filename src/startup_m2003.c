/*
 * M2003 Startup Code (Minimal)
 */

#include <stdint.h>

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;
extern int main(void);
extern void SystemInit(void);

void Reset_Handler(void)
{
    /* Run the vendor early-init hook before touching runtime state. */
    SystemInit();

    // Copy data from Flash to RAM
    uint32_t *src = &_sidata;
    uint32_t *dst = &_sdata;
    while (dst < &_edata) {
        *dst++ = *src++;
    }

    // Zero out BSS
    dst = &_sbss;
    while (dst < &_ebss) {
        *dst++ = 0;
    }

    main();
    while (1);
}

// Default Handlers
void Default_Handler(void) { while (1); }

// Core Exceptions
void NMI_Handler(void)          __attribute__ ((weak, alias("Default_Handler")));
void HardFault_Handler(void)    __attribute__ ((weak, alias("Default_Handler")));
void SVC_Handler(void)          __attribute__ ((weak, alias("Default_Handler")));
void PendSV_Handler(void)       __attribute__ ((weak, alias("Default_Handler")));
void SysTick_Handler(void)      __attribute__ ((weak, alias("Default_Handler")));

// Peripheral Exceptions
void BOD_IRQHandler(void)       __attribute__ ((weak, alias("Default_Handler")));
void PWRWU_IRQHandler(void)     __attribute__ ((weak, alias("Default_Handler")));
void ISP_IRQHandler(void)       __attribute__ ((weak, alias("Default_Handler")));
void WDT_IRQHandler(void)       __attribute__ ((weak, alias("Default_Handler")));
void WWDT_IRQHandler(void)      __attribute__ ((weak, alias("Default_Handler")));
void EINT0_IRQHandler(void)     __attribute__ ((weak, alias("Default_Handler")));
void EINT1_IRQHandler(void)     __attribute__ ((weak, alias("Default_Handler")));
void EINT2_IRQHandler(void)     __attribute__ ((weak, alias("Default_Handler")));
void EINT3_IRQHandler(void)     __attribute__ ((weak, alias("Default_Handler")));
void EINT5_IRQHandler(void)     __attribute__ ((weak, alias("Default_Handler")));
void GPB_IRQHandler(void)       __attribute__ ((weak, alias("Default_Handler")));
void GPC_IRQHandler(void)       __attribute__ ((weak, alias("Default_Handler")));
void GPE_IRQHandler(void)       __attribute__ ((weak, alias("Default_Handler")));
void GPF_IRQHandler(void)       __attribute__ ((weak, alias("Default_Handler")));
void PWM0_IRQHandler(void)      __attribute__ ((weak, alias("Default_Handler")));
void TMR0_IRQHandler(void)      __attribute__ ((weak, alias("Default_Handler")));
void TMR1_IRQHandler(void)      __attribute__ ((weak, alias("Default_Handler")));
void TMR2_IRQHandler(void)      __attribute__ ((weak, alias("Default_Handler")));
void TMR3_IRQHandler(void)      __attribute__ ((weak, alias("Default_Handler")));
void UART0_IRQHandler(void)     __attribute__ ((weak, alias("Default_Handler")));
void UART1_IRQHandler(void)     __attribute__ ((weak, alias("Default_Handler")));
void I2C0_IRQHandler(void)      __attribute__ ((weak, alias("Default_Handler")));
void ADC_IRQHandler(void)       __attribute__ ((weak, alias("Default_Handler")));
void USCI0_IRQHandler(void)     __attribute__ ((weak, alias("Default_Handler")));
void ECAP0_IRQHandler(void)     __attribute__ ((weak, alias("Default_Handler")));

// Vector Table
__attribute__ ((section(".vectors"), used))
uint32_t const * const g_pfnVectors[] = {
    // Core Exceptions
    (uint32_t *)&_estack,
    (uint32_t *)Reset_Handler,
    (uint32_t *)NMI_Handler,
    (uint32_t *)HardFault_Handler,
    0, 0, 0, 0, 0, 0, 0,
    (uint32_t *)SVC_Handler,
    0, 0,
    (uint32_t *)PendSV_Handler,
    (uint32_t *)SysTick_Handler,

    // Peripheral Interrupts (IRQs 0-60)
    (uint32_t *)BOD_IRQHandler,    // 0
    0,
    (uint32_t *)PWRWU_IRQHandler,  // 2
    0,
    0,
    (uint32_t *)ISP_IRQHandler,    // 5
    0,
    0,
    (uint32_t *)WDT_IRQHandler,    // 8
    (uint32_t *)WWDT_IRQHandler,   // 9
    (uint32_t *)EINT0_IRQHandler,  // 10
    (uint32_t *)EINT1_IRQHandler,  // 11
    (uint32_t *)EINT2_IRQHandler,  // 12
    (uint32_t *)EINT3_IRQHandler,  // 13
    0,
    (uint32_t *)EINT5_IRQHandler,  // 15
    0,
    (uint32_t *)GPB_IRQHandler,    // 17
    (uint32_t *)GPC_IRQHandler,    // 18
    0,
    (uint32_t *)GPE_IRQHandler,    // 20
    (uint32_t *)GPF_IRQHandler,    // 21
    0, 0, 0,
    (uint32_t *)PWM0_IRQHandler,   // 25
    0, 0, 0, 0, 0, 0,
    (uint32_t *)TMR0_IRQHandler,   // 32
    (uint32_t *)TMR1_IRQHandler,   // 33
    (uint32_t *)TMR2_IRQHandler,   // 34
    (uint32_t *)TMR3_IRQHandler,   // 35
    (uint32_t *)UART0_IRQHandler,  // 36
    (uint32_t *)UART1_IRQHandler,  // 37
    (uint32_t *)I2C0_IRQHandler,   // 38
    0, 0, 0,
    (uint32_t *)ADC_IRQHandler,    // 42
    0, 0, 0, 0, 0, 0, 0, 0, 0,
    (uint32_t *)USCI0_IRQHandler,  // 52
    0, 0, 0, 0, 0, 0, 0,
    (uint32_t *)ECAP0_IRQHandler   // 60
};
