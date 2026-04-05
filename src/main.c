/*
 * Main Control Loop for Nuvoton M2003
 */

#include <stdint.h>
#include "m2003_config.h"
#include "M2003.h"
#include "app_pwm.h"
#include "app_uart.h"
#include "app_adc.h"
#include "hall.h"
#include "encoder.h"
#include "motor.h"
#include "protocol.h"

extern uint32_t SystemCoreClock;
extern uint32_t CyclesPerUs;
extern uint32_t PllClock;

void Uart0DefaultMPF(void) {}

#define UART_BAUD              250000UL
#define ENCODER_DIVIDER        1u     /* encoder_poll every tick = 2kHz */

static volatile uint32_t systick_count;

static void clock_init(void)
{
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;
    while (!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk)) {
    }

    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLKDIV_Msk) | CLK_CLKDIV0_HCLK(1);

    CLK->AHBCLK |= CLK_AHBCLK_GPBCKEN_Msk | CLK_AHBCLK_GPCCKEN_Msk |
                   CLK_AHBCLK_GPECKEN_Msk | CLK_AHBCLK_GPFCKEN_Msk;

    SystemCoreClock = CLK_HIRC_24M;
    PllClock = CLK_HIRC_24M;
    CyclesPerUs = CLK_HIRC_24M / 1000000UL;

    SysTick_Config(SystemCoreClock / CONTROL_LOOP_HZ);
}

void SysTick_Handler(void)
{
    systick_count++;
}

int main(void)
{
    uint32_t handled_sys_ticks = 0;
    uint8_t encoder_div = 0;

    SYS_UnlockReg();

    clock_init();
    pwm_init();
    adc_init();
    hall_init();
    encoder_init();
    uart_init(UART_BAUD);
    protocol_init();
    motor_init();

    adc_irq_enable();

    SYS_LockReg();

    while (1) {
        motor_poll_fast();
        protocol_poll();

        while (handled_sys_ticks != systick_count) {
            handled_sys_ticks++;

            motor_tick_2khz();
            protocol_tick();

            if (++encoder_div >= ENCODER_DIVIDER) {
                encoder_div = 0;
                encoder_poll();
            }
        }
    }
}
