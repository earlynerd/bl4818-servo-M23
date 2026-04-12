/*
 * ADC Driver for Nuvoton M2003
 *
 * Current sense (CH2, PB.2) and Battery Voltage (CH3, PB.3).
 * Hardware-triggered by PWM at 20 kHz.
 * The ISR accumulates a control-window average and tracks peak current.
 */

#include <stdint.h>
#include "m2003_config.h"
#include "M2003.h"
#include "app_adc.h"

static volatile uint16_t raw_current;
static volatile uint16_t raw_voltage;
static volatile uint32_t current_sum_raw;
static volatile uint32_t voltage_sum_raw;
static volatile uint16_t peak_current_raw;
static volatile uint16_t sample_count;

void adc_init(void)
{
    CLK->APBCLK0 |= CLK_APBCLK0_ADCCKEN_Msk;
    CLK->CLKSEL2 = (CLK->CLKSEL2 & ~CLK_CLKSEL2_ADCSEL_Msk) | CLK_CLKSEL2_ADCSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_ADCDIV_Msk) | (6 << CLK_CLKDIV0_ADCDIV_Pos);

    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB2MFP_Msk) | SYS_GPB_MFPL_PB2MFP_ADC0_CH2;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB3MFP_Msk) | SYS_GPB_MFPL_PB3MFP_ADC0_CH3;
    PB->DINOFF |= (GPIO_DINOFF_DINOFF2_Msk | GPIO_DINOFF_DINOFF3_Msk);

    ADC->ADCR = ADC_ADCR_ADEN_Msk | (3 << ADC_ADCR_TRGS_Pos) | ADC_ADCR_TRGEN_Msk | ADC_ADCR_ADIE_Msk;
    ADC->ADCHER = (1 << ADC_CH_CURRENT) | (1 << ADC_CH_VOLTAGE);

    NVIC_SetPriority(ADC_IRQn, 2u);
}

void adc_irq_enable(void)
{
    NVIC_EnableIRQ(ADC_IRQn);
}

void ADC_IRQHandler(void)
{
    uint16_t current_raw;
    uint16_t voltage_raw;

    if (ADC->ADSR0 & ADC_ADSR0_ADF_Msk) {
        ADC->ADSR0 = ADC_ADSR0_ADF_Msk;
    }

    current_raw = (uint16_t)(ADC->ADDR[ADC_CH_CURRENT] & 0xFFFu);
    voltage_raw = (uint16_t)(ADC->ADDR[ADC_CH_VOLTAGE] & 0xFFFu);

    raw_current = current_raw;
    raw_voltage = voltage_raw;
    current_sum_raw += current_raw;
    voltage_sum_raw += voltage_raw;
    if ((sample_count == 0u) || (current_raw > peak_current_raw))
        peak_current_raw = current_raw;
    sample_count++;
}

void adc_consume_snapshot(adc_snapshot_t *snapshot)
{
    uint16_t latest_current;
    uint16_t latest_voltage;
    uint32_t current_sum;
    uint32_t voltage_sum;
    uint16_t peak_current;
    uint16_t count;
    uint32_t irq_state;

    irq_state = __get_PRIMASK();
    __disable_irq();

    latest_current = raw_current;
    latest_voltage = raw_voltage;
    current_sum = current_sum_raw;
    voltage_sum = voltage_sum_raw;
    peak_current = peak_current_raw;
    count = sample_count;

    current_sum_raw = 0u;
    voltage_sum_raw = 0u;
    peak_current_raw = 0u;
    sample_count = 0u;

    if (irq_state == 0u)
        __enable_irq();

    if (count == 0u) {
        snapshot->latest_current_raw = latest_current;
        snapshot->latest_voltage_raw = latest_voltage;
        snapshot->avg_current_raw = latest_current;
        snapshot->avg_voltage_raw = latest_voltage;
        snapshot->peak_current_raw = latest_current;
        snapshot->sample_count = 0u;
        return;
    }

    snapshot->latest_current_raw = latest_current;
    snapshot->latest_voltage_raw = latest_voltage;
    snapshot->avg_current_raw = (uint16_t)(current_sum / count);
    snapshot->avg_voltage_raw = (uint16_t)(voltage_sum / count);
    snapshot->peak_current_raw = peak_current;
    snapshot->sample_count = count;
}

uint16_t adc_raw_current(void)
{
    return raw_current;
}

uint16_t adc_raw_voltage(void)
{
    return raw_voltage;
}

uint32_t adc_current_raw_to_ma(uint16_t raw)
{
    /* I_mA = V_mV * 1000 / (gain * shunt_mΩ), with V_mV derived from ADC counts. */
    uint32_t ma = ((uint32_t)raw * ADC_VREF_MV * 1000UL) /
                  (ADC_FULL_SCALE_COUNTS * CURRENT_SENSE_GAIN * CURRENT_SENSE_SHUNT_MOHM);
    return ma;
}

uint16_t adc_voltage_raw_to_mv(uint16_t raw)
{
    uint32_t pin_mv = ((uint32_t)raw * ADC_VREF_MV) / ADC_FULL_SCALE_COUNTS;
    return (uint16_t)(pin_mv * 11);
}
