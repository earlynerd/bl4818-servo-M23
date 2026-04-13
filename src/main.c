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
#include "persist.h"
#include "protocol.h"
#include "strike.h"
#include "timing.h"

extern uint32_t SystemCoreClock;
extern uint32_t CyclesPerUs;
extern uint32_t PllClock;

void Uart0DefaultMPF(void) {}

#define UART_BAUD              250000UL

static volatile uint32_t protocol_tick_count;
static volatile uint32_t strike_tick_accum;
static volatile uint32_t protocol_tick_accum;

static uint8_t schedule_latest_from_samples(volatile uint32_t *accum, uint32_t elapsed_samples, uint32_t target_hz)
{
    *accum += elapsed_samples * target_hz;
    if (*accum < PWM_FREQ_HZ)
        return 0u;

    *accum -= PWM_FREQ_HZ;
    if (*accum >= PWM_FREQ_HZ)
        *accum = 0u;

    return 1u;
}

static uint32_t schedule_protocol_timeout_from_samples(volatile uint32_t *accum, uint32_t elapsed_samples)
{
    *accum += elapsed_samples * PROTOCOL_TICK_HZ;
    if (*accum < PWM_FREQ_HZ)
        return 0u;

    *accum -= PWM_FREQ_HZ;
    if (*accum >= PWM_FREQ_HZ) {
        *accum = 0u;
        return HZ_TICKS_FROM_MS(PROTOCOL_TICK_HZ, PROTOCOL_FRAME_TIMEOUT_MS);
    }

    return 1u;
}

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

    timing_init();
    SysTick_Config(SystemCoreClock / CONTROL_LOOP_HZ);
    NVIC_SetPriority(SysTick_IRQn, 1u);
}

void SysTick_Handler(void)
{
    uint32_t timing_start = timing_capture_stamp();
    uint32_t protocol_ticks;
    uint16_t elapsed_samples;

    elapsed_samples = motor_tick_control();

    if (schedule_latest_from_samples(&strike_tick_accum, elapsed_samples, STRIKE_LOOP_HZ)) {
        strike_tick();
    }

    protocol_ticks = schedule_protocol_timeout_from_samples(&protocol_tick_accum, elapsed_samples);
    if (protocol_ticks != 0u) {
        protocol_tick_count += protocol_ticks;
    }

    timing_record_control_tick(timing_start);
}

int main(void)
{
    uint32_t handled_protocol_ticks = 0;

    SYS_UnlockReg();

    clock_init();
    pwm_init();
    adc_init();
    hall_init();
    encoder_init();
    uart_init(UART_BAUD);
    protocol_init();
    motor_init();
    strike_init();
    persist_init();

    adc_irq_enable();

    SYS_LockReg();

    while (1) {
        uint32_t protocol_poll_start;
        uint32_t pending_protocol_ticks;

        motor_poll_fast();
        protocol_poll_start = timing_capture_stamp();
        protocol_poll();
        timing_record_protocol_poll(protocol_poll_start);

        pending_protocol_ticks = protocol_tick_count - handled_protocol_ticks;
        timing_note_protocol_backlog(pending_protocol_ticks);

        if (pending_protocol_ticks != 0u) {
            handled_protocol_ticks = protocol_tick_count;
            protocol_tick(pending_protocol_ticks);
        }
    }
}
