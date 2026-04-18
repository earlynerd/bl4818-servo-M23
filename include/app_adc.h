#ifndef ADC_H
#define ADC_H

#include <stdint.h>

typedef struct {
    uint16_t latest_current_raw;
    uint16_t latest_voltage_raw;
    uint16_t avg_current_raw;
    uint16_t avg_voltage_raw;
    uint16_t peak_current_raw;
    uint16_t sample_count;
} adc_snapshot_t;

void adc_init(void);
void adc_irq_enable(void);
void adc_consume_snapshot(adc_snapshot_t *snapshot);
void adc_calibrate_current_offset(uint16_t settle_samples);
uint16_t adc_get_current_offset(void);
uint32_t adc_overrun_count(void);
uint16_t adc_raw_current(void);
uint16_t adc_raw_voltage(void);
uint32_t adc_current_raw_to_ma(uint16_t raw);
uint16_t adc_voltage_raw_to_mv(uint16_t raw);

#endif /* ADC_H */
