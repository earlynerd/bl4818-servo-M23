#ifndef TIMING_H
#define TIMING_H

#include <stdint.h>

typedef struct {
    uint16_t control_budget_us;
    uint16_t control_last_us;
    uint16_t control_max_us;
    uint32_t control_overrun_count;
    uint32_t velocity_drop_count;
    uint32_t position_drop_count;
    uint32_t strike_drop_count;
    uint32_t protocol_drop_count;
    uint16_t hall_last_us;
    uint16_t hall_max_us;
    uint16_t uart_last_us;
    uint16_t uart_max_us;
    uint16_t adc_last_us;
    uint16_t adc_max_us;
    uint16_t protocol_poll_last_us;
    uint16_t protocol_poll_max_us;
    uint16_t protocol_backlog_max;
    uint32_t uptime_ms;
    uint32_t uart_rx_overflow_count;
    uint32_t adc_overrun_count;
} timing_snapshot_t;

void timing_init(void);
uint32_t timing_capture_stamp(void);
void timing_record_control_tick(uint32_t start_stamp);
void timing_record_hall_isr(uint32_t start_stamp);
void timing_record_uart_isr(uint32_t start_stamp);
void timing_record_adc_isr(uint32_t start_stamp);
void timing_record_protocol_poll(uint32_t start_stamp);
void timing_note_velocity_drops(uint32_t dropped_updates);
void timing_note_position_drops(uint32_t dropped_updates);
void timing_note_strike_drops(uint32_t dropped_updates);
void timing_note_protocol_drops(uint32_t dropped_ticks);
void timing_note_protocol_backlog(uint32_t pending_ticks);
void timing_get_snapshot(timing_snapshot_t *snapshot);

#endif /* TIMING_H */
