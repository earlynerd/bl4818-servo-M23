#ifndef ADC_H
#define ADC_H

#include <stdint.h>

void adc_init(void);
void adc_irq_enable(void);
uint8_t adc_sample_ready(void);
void adc_sample_clear(void);
uint16_t adc_raw_current(void);
uint16_t adc_raw_voltage(void);
uint16_t adc_read_current_ma(void);
uint16_t adc_read_voltage_mv(void);

#endif /* ADC_H */
