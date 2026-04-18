/*
 * Main Control Loop for Nuvoton M2003
 */

#include <stdint.h>
#include "m2003_config.h"
#include "M2003.h"
#include "wdt.h"
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
#include "sched_util.h"

extern uint32_t SystemCoreClock;
extern uint32_t CyclesPerUs;
extern uint32_t PllClock;

void Uart0DefaultMPF(void) {}

#define UART_BAUD              250000UL

static volatile uint32_t protocol_tick_count;
static uint32_t strike_tick_accum;
static uint32_t protocol_tick_accum;
static uint32_t hall_baseline_divider;

/* Refresh the hall timing baseline once per second to prevent the
 * 24-bit delta in hall_process_transition from ever aliasing. */
#define HALL_BASELINE_REFRESH_PERIOD  CONTROL_LOOP_HZ

/* WDT liveness handshake: main loop sets this each iteration; SysTick only
 * kicks the watchdog when it is set, then clears it.  If either the main
 * loop or SysTick stalls for longer than the WDT timeout, the MCU resets
 * and the PWM braking handlers run on the boot path. */
static volatile uint8_t main_loop_alive;

static uint32_t schedule_protocol_timeout_from_samples(uint32_t *accum, uint32_t elapsed_samples,
                                                       uint32_t *dropped_ticks)
{
    uint32_t extra_due;
    uint32_t total_due;
    uint32_t timeout_ticks;

    *accum += elapsed_samples * PROTOCOL_TICK_HZ;
    if (*accum < PWM_FREQ_HZ) {
        if (dropped_ticks != 0u)
            *dropped_ticks = 0u;
        return 0u;
    }

    *accum -= PWM_FREQ_HZ;
    extra_due = *accum / PWM_FREQ_HZ;
    *accum -= extra_due * PWM_FREQ_HZ;
    total_due = 1u + extra_due;
    timeout_ticks = HZ_TICKS_FROM_MS(PROTOCOL_TICK_HZ, PROTOCOL_FRAME_TIMEOUT_MS);

    if (total_due > timeout_ticks) {
        if (dropped_ticks != 0u)
            *dropped_ticks = total_due - timeout_ticks;
        return timeout_ticks;
    }

    if (dropped_ticks != 0u)
        *dropped_ticks = 0u;

    return total_due;
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
    NVIC_SetPriority(SysTick_IRQn, 2u);
}

static void watchdog_init(void)
{
    /* WDT runs from LIRC (~10 kHz), so 2^14 clocks ≈ 1.6 s — comfortably
     * longer than the worst-case blocking op on this MCU (flash erase is
     * ~20 ms with IRQs disabled) and still fast enough to rescue a runaway. */
    CLK->PWRCTL |= CLK_PWRCTL_LIRCEN_Msk;
    while (!(CLK->STATUS & CLK_STATUS_LIRCSTB_Msk)) {
    }

    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_WDTSEL_Msk) | CLK_CLKSEL1_WDTSEL_LIRC;
    CLK->APBCLK0 |= CLK_APBCLK0_WDTCKEN_Msk;

    WDT_Open(WDT_TIMEOUT_2POW14, WDT_RESET_DELAY_18CLK, TRUE, FALSE);
}

static void bod_init(void)
{
    /* Brown-out threshold 2.7 V: well below the 3.3 V nominal rail so normal
     * strike-current sag won't false-trigger, but high enough that the MCU is
     * still comfortably inside its 1.8 V operating window when we trip.  On
     * trip the chip goes through a clean reset — Reset_Handler → pwm_init
     * masks all phases — so the bridge lands in a safe state. */
    SYS_SET_BOD_LEVEL(SYS_BODCTL_BODVL_2_7V);
    SYS_ENABLE_BOD_RST();
    SYS_ENABLE_BOD();
}

/*
 * HardFault / NMI handlers: brake the PWM bridge immediately (direct register
 * writes — no function calls in case the stack is corrupted) and force a
 * system reset.  The WDT is the backstop if even this path can't run.
 */
static inline void fault_brake_pwm(void)
{
    PWM0->CNTEN = 0u;
    PWM0->MSKEN = 0x3Fu;
    PWM0->MSK   = 0x00u;
}

void HardFault_Handler(void)
{
    fault_brake_pwm();
    NVIC_SystemReset();
    while (1) { }
}

void NMI_Handler(void)
{
    fault_brake_pwm();
    NVIC_SystemReset();
    while (1) { }
}

void SysTick_Handler(void)
{
    uint32_t timing_start = timing_capture_stamp();
    uint32_t dropped_updates;
    uint32_t dropped_ticks;
    uint32_t protocol_ticks;
    uint16_t elapsed_samples;

    /* Kick WDT only when the main loop has set its liveness flag since the
     * last tick — this way a hung main loop still trips the watchdog. */
    if (main_loop_alive != 0u) {
        main_loop_alive = 0u;
        WDT_RESET_COUNTER();
    }

    if (++hall_baseline_divider >= HALL_BASELINE_REFRESH_PERIOD) {
        hall_baseline_divider = 0u;
        hall_refresh_baseline();
    }

    elapsed_samples = motor_tick_control();

    if (schedule_latest_from_samples(&strike_tick_accum, elapsed_samples,
                                     STRIKE_LOOP_HZ, &dropped_updates)) {
        if (dropped_updates != 0u)
            timing_note_strike_drops(dropped_updates);
        strike_tick();
    }

    protocol_ticks = schedule_protocol_timeout_from_samples(&protocol_tick_accum,
                                                            elapsed_samples,
                                                            &dropped_ticks);
    if (protocol_ticks != 0u) {
        if (dropped_ticks != 0u)
            timing_note_protocol_drops(dropped_ticks);
        protocol_tick_count += protocol_ticks;
    }

    timing_record_control_tick(timing_start);
}

int main(void)
{
    uint32_t handled_protocol_ticks = 0;

    SYS_UnlockReg();

    bod_init();
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

    /* Capture the INA180 zero-current offset while all phases are still
     * masked off (motor_init does not start driving).  ~64 samples at
     * 20 kHz is ~3.2 ms of averaging — plenty to average out LSB noise. */
    adc_calibrate_current_offset(64u);

    /* Enable watchdog last so any earlier initialisation blocking is
     * covered by the startup window instead of the WDT timeout. */
    watchdog_init();

    SYS_LockReg();

    while (1) {
        uint32_t protocol_poll_start;
        uint32_t pending_protocol_ticks;

        main_loop_alive = 1u;

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
