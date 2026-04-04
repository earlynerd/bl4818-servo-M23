/*
 * ADC Driver for Nuvoton M2003
 *
 * Current sense (CH11, PB.11) and Battery Voltage (CH12, PB.12).
 */

#include <stdint.h>
#include "m2003_config.h"
#include "M2003.h"

void adc_init(void)
{
    /* 1. Enable ADC Clock */
    CLK->APBCLK0 |= CLK_APBCLK0_ADCCKEN_Msk;
    
    /* ADC clock source = HIRC (24MHz), divider = 6 (4MHz ADC clock) */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & ~CLK_CLKSEL2_ADCSEL_Msk) | CLK_CLKSEL2_ADCSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_ADCDIV_Msk) | (6 << CLK_CLKDIV0_ADCDIV_Pos);

    /* 2. Configure Pins (PB.2, PB.3) */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB2MFP_Msk) | SYS_GPB_MFPL_PB2MFP_ADC0_CH2;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB3MFP_Msk) | SYS_GPB_MFPL_PB3MFP_ADC0_CH3;

    /* Disable digital input path for these pins */
    PB->DINOFF |= (BIT2 | BIT3);

    /* 3. Configure ADC Peripheral */
    ADC->ADCR = ADC_ADCR_ADEN_Msk; /* Enable ADC */
}

uint16_t adc_read(uint8_t channel)
{
    /* Select channel */
    ADC->ADCHER = (1 << channel);

    /* Start conversion */
    ADC->ADCR |= ADC_ADCR_ADST_Msk;

    /* Wait for completion */
    while (!(ADC->ADSR0 & ADC_ADSR0_ADF_Msk));

    /* Clear flag */
    ADC->ADSR0 = ADC_ADSR0_ADF_Msk;

    /* Return 12-bit result */
    return (uint16_t)(ADC->ADDR[channel] & 0xFFF);
}

uint16_t adc_read_current_ma(void)
{
    uint16_t raw = adc_read(ADC_CH_CURRENT);

    /*
     * Direct shunt (20mΩ, R020), no amplifier. VDD = 5.0V reference.
     *
     * I_mA = raw × 5000 / 4095 / 0.020
     *      = raw × 5000 / 4095 × 50
     *      = raw × 250000 / 4095
     *      ≈ raw × 61.05
     *
     * On the Cortex-M23, we have a hardware 32-bit multiplier and divider,
     * so we can use the full precision formula without a performance hit.
     */
    return (uint16_t)(((uint32_t)raw * 250000UL) / 4095UL);
}

/* Optional: Read battery voltage in millivolts */
uint16_t adc_read_voltage_mv(void)
{
    uint16_t raw = adc_read(ADC_CH_VOLTAGE);

    /*
     * Battery divider on BL4818 board is usually 100k / 10k (approx).
     * Needs verification for exact ratio, but here is the 12-bit calc:
     */
    uint32_t pin_mv = ((uint32_t)raw * 5000UL) / 4095UL;
    
    /* Assuming 11:1 divider (100k+10k)/10k */
    return (uint16_t)(pin_mv * 11);
}
