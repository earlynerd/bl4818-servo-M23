/*
 * ADC Driver for Nuvoton M2003
 *
 * Current sense (CH2, PB.2) and Battery Voltage (CH3, PB.3).
 * Hardware-triggered by PWM at 20kHz. ISR only captures the raw sample.
 */

#include <stdint.h>
#include "m2003_config.h"
#include "M2003.h"
#include "app_adc.h"

static volatile uint16_t raw_current;
static volatile uint16_t raw_voltage;
static volatile uint8_t  sample_ready;

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
    if (ADC->ADSR0 & ADC_ADSR0_ADF_Msk) {
        ADC->ADSR0 = ADC_ADSR0_ADF_Msk;
    }
    raw_current = (uint16_t)(ADC->ADDR[ADC_CH_CURRENT] & 0xFFFu);
    raw_voltage = (uint16_t)(ADC->ADDR[ADC_CH_VOLTAGE] & 0xFFFu);
    sample_ready = 1u;
}

uint8_t adc_sample_ready(void)
{
    return sample_ready;
}

void adc_sample_clear(void)
{
    sample_ready = 0u;
}

uint16_t adc_raw_current(void)
{
    return raw_current;
}

uint16_t adc_raw_voltage(void)
{
    return raw_voltage;
}

uint16_t adc_read_current_ma(void)
{
    return (uint16_t)(((uint32_t)raw_current * 250000UL) / 4095UL);
}

uint16_t adc_read_voltage_mv(void)
{
    uint32_t pin_mv = ((uint32_t)raw_voltage * 5000UL) / 4095UL;
    return (uint16_t)(pin_mv * 11);
}
