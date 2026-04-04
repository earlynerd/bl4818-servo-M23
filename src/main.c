/*
 * Main Control Loop for Nuvoton M2003
 */

#include <stdint.h>
#include "m2003_config.h"
#include "M2003.h"
#include "app_pwm.h"
#include "app_adc.h"
#include "hall.h"
#include "motor.h"
#include "protocol.h"

extern uint32_t SystemCoreClock;
extern uint32_t CyclesPerUs;
extern uint32_t PllClock;

static void debug_pin_init(void)
{
    CLK->AHBCLK |= CLK_AHBCLK_GPFCKEN_Msk;
    SYS->GPF_MFPL = (SYS->GPF_MFPL & ~SYS_GPF_MFPL_PF0MFP_Msk) | SYS_GPF_MFPL_PF0MFP_GPIO;
    SYS->GPF_MFOS &= ~SYS_GPF_MFOS_PF0MFOS_Msk;

    GPIO_SetMode(PF, BIT0, GPIO_MODE_OUTPUT);
    PF0 = 1;
}

static void delay_timer_init(void)
{
    CLK->APBCLK0 |= CLK_APBCLK0_TMR0CKEN_Msk;
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_TMR0SEL_Msk) | CLK_CLKSEL1_TMR0SEL_HIRC;

    TIMER_Open(TIMER0, TIMER_CONTINUOUS_MODE, 1000000);
    TIMER_Start(TIMER0);
    while (!TIMER_IS_ACTIVE(TIMER0)) {
    }
}

static void delay_us(uint32_t usec)
{
    TIMER_ResetCounter(TIMER0);
    while (TIMER_GetCounter(TIMER0) < usec) {
    }
}

#define SOFT_UART_BIT_US 104U

static void soft_uart_tx_byte(uint8_t value)
{
    PF0 = 0;
    delay_us(SOFT_UART_BIT_US);

    for (uint32_t i = 0; i < 8; ++i) {
        PF0 = (value & 0x01u) ? 1 : 0;
        delay_us(SOFT_UART_BIT_US);
        value >>= 1;
    }

    PF0 = 1;
    delay_us(SOFT_UART_BIT_US);
}

static void soft_uart_tx_string(const char *s)
{
    while (*s != '\0') {
        soft_uart_tx_byte((uint8_t)*s++);
    }
}

int main(void)
{
    /* 1. Unlock Protected Registers */
    SYS_UnlockReg();

    /* 2. Enable HIRC (24MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;
    while (!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(1);

    /* Avoid SystemCoreClockUpdate() here until CPU divide support is verified. */
    SystemCoreClock = CLK_HIRC_24M;
    PllClock = CLK_HIRC_24M;
    CyclesPerUs = CLK_HIRC_24M / 1000000UL;

    /* 3. Use a timer-driven software UART on PF.0, the one proven-good header pad. */
    debug_pin_init();
    delay_timer_init();

    /* Lock Protected Registers after all clock and pin mux changes. */
    SYS_LockReg();
    
    while (1) {
        soft_uart_tx_byte(0x55);
        soft_uart_tx_byte(0x55);
        soft_uart_tx_byte(0x55);
        soft_uart_tx_string(" SOFTUART PF0 OK\r\n");

        delay_us(50000);
    }
}
