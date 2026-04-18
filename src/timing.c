/*
 * Lightweight timing instrumentation for control/ISR service visibility.
 *
 * Uses TIMER2 as a free-running 1 MHz timebase so the firmware can report
 * control-loop duration, hall IRQ service time, protocol polling latency,
 * and whether the foreground loop is falling behind scheduled protocol ticks.
 */

#include <stdint.h>
#include "m2003_config.h"
#include "M2003.h"
#include "timing.h"
#include "irq_util.h"
#include "app_uart.h"
#include "app_adc.h"

#define TIMING_TIMER_FREQ   1000000UL
#define TIMING_TIMER_MASK   0x00FFFFFFUL
#define CONTROL_BUDGET_US   ((1000000UL + (CONTROL_LOOP_HZ / 2u)) / CONTROL_LOOP_HZ)

static uint32_t timing_counter_hz;
static uint32_t control_budget_counts;
static volatile uint32_t control_last_counts;
static volatile uint32_t control_max_counts;
static volatile uint32_t control_overrun_count;
static volatile uint32_t velocity_drop_count;
static volatile uint32_t position_drop_count;
static volatile uint32_t strike_drop_count;
static volatile uint32_t protocol_drop_count;
static volatile uint32_t hall_last_counts;
static volatile uint32_t hall_max_counts;
static volatile uint32_t uart_last_counts;
static volatile uint32_t uart_max_counts;
static volatile uint32_t adc_last_counts;
static volatile uint32_t adc_max_counts;
static volatile uint32_t protocol_poll_last_counts;
static volatile uint32_t protocol_poll_max_counts;
static volatile uint16_t protocol_backlog_max;
static volatile uint64_t uptime_us;
static uint32_t last_uptime_stamp;

static uint32_t timing_now(void)
{
    return TIMER_GetCounter(TIMER2) & TIMING_TIMER_MASK;
}

static uint32_t timing_delta_counts(uint32_t start, uint32_t end)
{
    return (end - start) & TIMING_TIMER_MASK;
}

static uint32_t timing_counts_to_us(uint32_t counts)
{
#if TIMING_TIMER_FREQ == 1000000UL
    /* Timer prescaler divides 24 MHz HIRC to exactly 1 MHz, so one count
     * equals one microsecond.  Compile-time guard avoids a 64-bit software
     * divide (~300 cycles on Cortex-M23) in the SysTick hot path. */
    (void)timing_counter_hz;
    return counts;
#else
    if (timing_counter_hz == 0u)
        return 0u;

    return (uint32_t)((((uint64_t)counts * 1000000ULL) + (timing_counter_hz / 2u)) / timing_counter_hz);
#endif
}

static uint16_t clamp_u16(uint32_t value)
{
    if (value > 0xFFFFu)
        return 0xFFFFu;

    return (uint16_t)value;
}

static void update_max_u16(volatile uint16_t *dest, uint16_t value)
{
    if (value > *dest)
        *dest = value;
}

static void update_max_u32(volatile uint32_t *dest, uint32_t value)
{
    if (value > *dest)
        *dest = value;
}

static void saturating_add_u32(volatile uint32_t *dest, uint32_t value)
{
    uint32_t next = *dest + value;

    if (next < *dest)
        *dest = 0xFFFFFFFFu;
    else
        *dest = next;
}

static void sync_uptime(uint32_t now)
{
    uptime_us += timing_counts_to_us(timing_delta_counts(last_uptime_stamp, now));
    last_uptime_stamp = now;
}

void timing_init(void)
{
    CLK->APBCLK0 |= CLK_APBCLK0_TMR2CKEN_Msk;
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_TMR2SEL_Msk) | CLK_CLKSEL1_TMR2SEL_HIRC;
    SYS_ResetModule(TMR2_RST);
    TIMER_Open(TIMER2, TIMER_CONTINUOUS_MODE, TIMING_TIMER_FREQ);
    /* TIMER_Open sets PSC so TIF fires at TIMING_TIMER_FREQ, but in continuous
     * mode the counter itself ticks at the undivided module clock.  Override
     * PSC to divide the 24 MHz HIRC down to TIMING_TIMER_FREQ so CNT really
     * advances at 1 µs per count.  CMP is widened to the full 24 bits — we
     * don't use TIF on this timer, only CNT as a free-running timebase. */
    TIMER2->CTL = (TIMER2->CTL & ~TIMER_CTL_PSC_Msk) |
                  (((CLK_HIRC_24M / TIMING_TIMER_FREQ) - 1u) << TIMER_CTL_PSC_Pos);
    TIMER2->CMP = TIMING_TIMER_MASK;
    TIMER_Start(TIMER2);
    while (!TIMER_IS_ACTIVE(TIMER2)) {
    }

    timing_counter_hz = TIMER_GetModuleClock(TIMER2) /
                        (((TIMER2->CTL & TIMER_CTL_PSC_Msk) >> TIMER_CTL_PSC_Pos) + 1u);

    control_budget_counts = (timing_counter_hz + (CONTROL_LOOP_HZ / 2u)) / CONTROL_LOOP_HZ;
    control_last_counts = 0u;
    control_max_counts = 0u;
    control_overrun_count = 0u;
    velocity_drop_count = 0u;
    position_drop_count = 0u;
    strike_drop_count = 0u;
    protocol_drop_count = 0u;
    hall_last_counts = 0u;
    hall_max_counts = 0u;
    uart_last_counts = 0u;
    uart_max_counts = 0u;
    adc_last_counts = 0u;
    adc_max_counts = 0u;
    protocol_poll_last_counts = 0u;
    protocol_poll_max_counts = 0u;
    protocol_backlog_max = 0u;
    uptime_us = 0u;
    last_uptime_stamp = timing_now();
}

uint32_t timing_capture_stamp(void)
{
    return timing_now();
}

void timing_record_control_tick(uint32_t start_stamp)
{
    uint32_t end_stamp = timing_now();
    uint32_t elapsed_counts = timing_delta_counts(start_stamp, end_stamp);

    sync_uptime(end_stamp);
    control_last_counts = elapsed_counts;
    update_max_u32(&control_max_counts, elapsed_counts);

    if (elapsed_counts > control_budget_counts)
        control_overrun_count++;
}

void timing_record_hall_isr(uint32_t start_stamp)
{
    uint32_t elapsed_counts = timing_delta_counts(start_stamp, timing_now());

    hall_last_counts = elapsed_counts;
    update_max_u32(&hall_max_counts, elapsed_counts);
}

void timing_record_uart_isr(uint32_t start_stamp)
{
    uint32_t elapsed_counts = timing_delta_counts(start_stamp, timing_now());

    uart_last_counts = elapsed_counts;
    update_max_u32(&uart_max_counts, elapsed_counts);
}

void timing_record_adc_isr(uint32_t start_stamp)
{
    uint32_t elapsed_counts = timing_delta_counts(start_stamp, timing_now());

    adc_last_counts = elapsed_counts;
    update_max_u32(&adc_max_counts, elapsed_counts);
}

void timing_record_protocol_poll(uint32_t start_stamp)
{
    uint32_t elapsed_counts = timing_delta_counts(start_stamp, timing_now());

    protocol_poll_last_counts = elapsed_counts;
    update_max_u32(&protocol_poll_max_counts, elapsed_counts);
}

void timing_note_velocity_drops(uint32_t dropped_updates)
{
    saturating_add_u32(&velocity_drop_count, dropped_updates);
}

void timing_note_position_drops(uint32_t dropped_updates)
{
    saturating_add_u32(&position_drop_count, dropped_updates);
}

void timing_note_strike_drops(uint32_t dropped_updates)
{
    saturating_add_u32(&strike_drop_count, dropped_updates);
}

void timing_note_protocol_drops(uint32_t dropped_ticks)
{
    saturating_add_u32(&protocol_drop_count, dropped_ticks);
}

void timing_note_protocol_backlog(uint32_t pending_ticks)
{
    uint16_t backlog = clamp_u16(pending_ticks);

    update_max_u16(&protocol_backlog_max, backlog);
}

void timing_get_snapshot(timing_snapshot_t *snapshot)
{
    uint32_t irq_state;
    uint32_t now;
    uint64_t uptime_now_us;

    if (snapshot == 0)
        return;

    irq_state = irq_save();
    now = timing_now();
    uptime_now_us = uptime_us + timing_counts_to_us(timing_delta_counts(last_uptime_stamp, now));

    snapshot->control_budget_us = clamp_u16(timing_counts_to_us(control_budget_counts));
    snapshot->control_last_us = clamp_u16(timing_counts_to_us(control_last_counts));
    snapshot->control_max_us = clamp_u16(timing_counts_to_us(control_max_counts));
    snapshot->control_overrun_count = control_overrun_count;
    snapshot->velocity_drop_count = velocity_drop_count;
    snapshot->position_drop_count = position_drop_count;
    snapshot->strike_drop_count = strike_drop_count;
    snapshot->protocol_drop_count = protocol_drop_count;
    snapshot->hall_last_us = clamp_u16(timing_counts_to_us(hall_last_counts));
    snapshot->hall_max_us = clamp_u16(timing_counts_to_us(hall_max_counts));
    snapshot->uart_last_us = clamp_u16(timing_counts_to_us(uart_last_counts));
    snapshot->uart_max_us = clamp_u16(timing_counts_to_us(uart_max_counts));
    snapshot->adc_last_us = clamp_u16(timing_counts_to_us(adc_last_counts));
    snapshot->adc_max_us = clamp_u16(timing_counts_to_us(adc_max_counts));
    snapshot->protocol_poll_last_us = clamp_u16(timing_counts_to_us(protocol_poll_last_counts));
    snapshot->protocol_poll_max_us = clamp_u16(timing_counts_to_us(protocol_poll_max_counts));
    snapshot->protocol_backlog_max = protocol_backlog_max;
    snapshot->uptime_ms = (uint32_t)(uptime_now_us / 1000ULL);
    irq_restore(irq_state);

    snapshot->uart_rx_overflow_count = uart_rx_overflow_count();
    snapshot->adc_overrun_count = adc_overrun_count();
}
