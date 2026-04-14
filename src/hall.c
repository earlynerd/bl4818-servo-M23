/*
 * Hall Sensor Interface for Nuvoton M2003
 */

#include "m2003_config.h"
#include "M2003.h"
#include "hall.h"
#include "motor.h"
#include "timing.h"
#include "irq_util.h"

static const uint8_t hall_forward_seq[8] = { 0xFF, 3, 6, 2, 5, 1, 4, 0xFF };
static const uint8_t hall_to_sector[8] = { 0xFF, 0, 2, 1, 4, 5, 3, 0xFF };

#define HALL_TIMER_FREQ 1000000UL
#define HALL_TIMER_MASK 0x00FFFFFFUL
#define HALL_PORTB_MASK (BIT4 | BIT5)
#define HALL_PORTC_MASK BIT14

static uint8_t prev_hall;
static int8_t  detected_direction;
static int32_t hall_position;
static uint32_t transition_period;
static uint32_t last_transition_time;
static uint32_t hall_counter_hz;
static uint32_t hall_min_transition_counts;
static volatile uint8_t hall_pending_result;

static uint8_t hall_process_transition(uint8_t current)
{
    uint32_t now;
    uint32_t period;
    uint8_t forward;
    uint8_t reverse;

    if (current == prev_hall) {
        return HALL_POLL_NO_CHANGE;
    }

    if (current == 0u || current == 7u) {
        return HALL_POLL_INVALID;
    }

    forward = (hall_forward_seq[prev_hall] == current) ? 1u : 0u;
    reverse = (hall_forward_seq[current] == prev_hall) ? 1u : 0u;
    if (!forward && !reverse) {
        /* Ignore impossible state jumps. They are usually chatter or a
         * transient read during two GPIO edge IRQs arriving back-to-back. */
        return HALL_POLL_NO_CHANGE;
    }

    now = TIMER_GetCounter(TIMER1) & HALL_TIMER_MASK;
    period = (now - last_transition_time) & HALL_TIMER_MASK;
    if (period < hall_min_transition_counts) {
        /* Reject transitions faster than the mechanism can plausibly produce. */
        return HALL_POLL_NO_CHANGE;
    }

    if (forward) {
        detected_direction = 1;
        hall_position++;
    } else {
        detected_direction = -1;
        hall_position--;
    }

    transition_period = period;
    last_transition_time = now;
    prev_hall = current;
    return HALL_POLL_TRANSITION;
}

void hall_init(void)
{
    /* Configure Hall pins as Input (PC.14, PB.5, PB.4) */
    GPIO_SetMode(PC, BIT14, GPIO_MODE_INPUT);
    GPIO_SetMode(PB, BIT5 | BIT4, GPIO_MODE_INPUT);
    GPIO_ENABLE_DIGITAL_PATH(PC, BIT14);
    GPIO_ENABLE_DIGITAL_PATH(PB, BIT5 | BIT4);

    /* Free-running 1 MHz timer for hall transition timing. */
    CLK->APBCLK0 |= CLK_APBCLK0_TMR1CKEN_Msk;
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_TMR1SEL_Msk) | CLK_CLKSEL1_TMR1SEL_HIRC;
    TIMER_Open(TIMER1, TIMER_CONTINUOUS_MODE, HALL_TIMER_FREQ);
    TIMER_Start(TIMER1);
    while (!TIMER_IS_ACTIVE(TIMER1)) {
    }
    hall_counter_hz = TIMER_GetModuleClock(TIMER1) /
                      (((TIMER1->CTL & TIMER_CTL_PSC_Msk) >> TIMER_CTL_PSC_Pos) + 1u);
    hall_min_transition_counts =
        (uint32_t)((((uint64_t)hall_counter_hz * HALL_MIN_TRANSITION_US) + 999999ULL) / 1000000ULL);
    if (hall_min_transition_counts == 0u)
        hall_min_transition_counts = 1u;

    GPIO_DISABLE_DEBOUNCE(PB, HALL_PORTB_MASK);
    GPIO_DISABLE_DEBOUNCE(PC, HALL_PORTC_MASK);
    GPIO_EnableInt(PB, 4u, GPIO_INT_BOTH_EDGE);
    GPIO_EnableInt(PB, 5u, GPIO_INT_BOTH_EDGE);
    GPIO_EnableInt(PC, 14u, GPIO_INT_BOTH_EDGE);
    GPIO_CLR_INT_FLAG(PB, HALL_PORTB_MASK);
    GPIO_CLR_INT_FLAG(PC, HALL_PORTC_MASK);
    prev_hall = hall_read();
    detected_direction = 0;
    hall_position = 0;
    transition_period = 0xFFFFFFFFu;
    last_transition_time = TIMER_GetCounter(TIMER1) & HALL_TIMER_MASK;
    hall_pending_result = HALL_POLL_NO_CHANGE;

    NVIC_SetPriority(GPB_IRQn, 0u);
    NVIC_SetPriority(GPC_IRQn, 0u);
    NVIC_EnableIRQ(GPB_IRQn);
    NVIC_EnableIRQ(GPC_IRQn);
}

uint8_t hall_read_raw(void)
{
    uint8_t state = 0;
    if (PC14) state |= 0x01; // Hall 1
    if (PB5)  state |= 0x02; // Hall 2
    if (PB4)  state |= 0x04; // Hall 3
    return state;
}

uint8_t hall_decode_state(uint8_t raw_state)
{
    return raw_state & 0x07u;
}

uint8_t hall_read(void)
{
    return hall_decode_state(hall_read_raw());
}

uint8_t hall_poll(void)
{
    uint8_t result;
    uint32_t primask = irq_save();

    result = hall_pending_result;
    hall_pending_result = HALL_POLL_NO_CHANGE;
    irq_restore(primask);
    return result;
}

int8_t hall_direction(void) { return detected_direction; }
int32_t hall_count(void)     { return hall_position; }
void hall_count_reset(void) { hall_position = 0; }
uint32_t hall_period(void)  { return transition_period; }
uint8_t hall_sector(void)   { return hall_to_sector[hall_read() & 0x07u]; }

void GPB_IRQHandler(void)
{
    uint32_t flags = PB->INTSRC & HALL_PORTB_MASK;

    if (flags != 0u) {
        uint32_t timing_start = timing_capture_stamp();
        uint8_t result;
        GPIO_CLR_INT_FLAG(PB, flags);
        result = hall_process_transition(hall_read());
        hall_pending_result = result;
        motor_handle_hall_transition(result);
        timing_record_hall_isr(timing_start);
    }
}

void GPC_IRQHandler(void)
{
    uint32_t flags = PC->INTSRC & HALL_PORTC_MASK;

    if (flags != 0u) {
        uint32_t timing_start = timing_capture_stamp();
        uint8_t result;
        GPIO_CLR_INT_FLAG(PC, flags);
        result = hall_process_transition(hall_read());
        hall_pending_result = result;
        motor_handle_hall_transition(result);
        timing_record_hall_isr(timing_start);
    }
}
