/*
 * ADC Driver — Current sense and voltage measurement
 */
#ifndef ADC_H
#define ADC_H

#include <stdint.h>

void adc_init(void);
uint16_t adc_read(uint8_t channel);
uint16_t adc_read_current_ma(void);
uint16_t adc_read_voltage_mv(void);

#endif /* ADC_H */
