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
#include "irq_util.h"
#include "timing.h"

static volatile uint16_t raw_current;
static volatile uint16_t raw_voltage;
static volatile uint32_t current_sum_raw;
static volatile uint32_t voltage_sum_raw;
static volatile uint16_t peak_current_raw;
static volatile uint16_t sample_count;
static volatile uint32_t overrun_count;

/* Zero-current ADC offset captured at boot while PWM is idle.  The ADC ISR
 * stays untouched — the offset is applied in adc_consume_snapshot to keep
 * the 20 kHz hot path at a single-store cost.  Hall-transition current
 * checks and peak fault detection therefore see offset-corrected values. */
static uint16_t current_offset_raw;

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
    uint32_t timing_start = timing_capture_stamp();
    uint32_t current_word;
    uint32_t voltage_word;
    uint16_t current_raw;
    uint16_t voltage_raw;

    if (ADC->ADSR0 & ADC_ADSR0_ADF_Msk) {
        ADC->ADSR0 = ADC_ADSR0_ADF_Msk;
    }

    /* Read the full ADDR word so we capture the per-channel OVERRUN bit
     * (position 16) before the read itself clears it.  Overrun means the
     * previous sample hadn't been consumed when a new conversion landed —
     * if we ever see this, ISR latency is starving the ADC pipeline. */
    current_word = ADC->ADDR[ADC_CH_CURRENT];
    voltage_word = ADC->ADDR[ADC_CH_VOLTAGE];
    if ((current_word | voltage_word) & ADC_ADDR_OVERRUN_Msk)
        overrun_count++;
    current_raw = (uint16_t)(current_word & 0xFFFu);
    voltage_raw = (uint16_t)(voltage_word & 0xFFFu);

    raw_current = current_raw;
    raw_voltage = voltage_raw;
    current_sum_raw += current_raw;
    voltage_sum_raw += voltage_raw;
    if ((sample_count == 0u) || (current_raw > peak_current_raw))
        peak_current_raw = current_raw;
    sample_count++;
    timing_record_adc_isr(timing_start);
}

static uint16_t apply_current_offset(uint16_t raw, uint16_t offset)
{
    return (raw > offset) ? (uint16_t)(raw - offset) : 0u;
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
    uint16_t offset = current_offset_raw;
    uint16_t avg_current;

    irq_state = irq_save();

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

    irq_restore(irq_state);

    if (count == 0u) {
        latest_current = apply_current_offset(latest_current, offset);
        snapshot->latest_current_raw = latest_current;
        snapshot->latest_voltage_raw = latest_voltage;
        snapshot->avg_current_raw = latest_current;
        snapshot->avg_voltage_raw = latest_voltage;
        snapshot->peak_current_raw = latest_current;
        snapshot->sample_count = 0u;
        return;
    }

    avg_current = (uint16_t)(current_sum / count);
    snapshot->latest_current_raw = apply_current_offset(latest_current, offset);
    snapshot->latest_voltage_raw = latest_voltage;
    snapshot->avg_current_raw = apply_current_offset(avg_current, offset);
    snapshot->avg_voltage_raw = (uint16_t)(voltage_sum / count);
    snapshot->peak_current_raw = apply_current_offset(peak_current, offset);
    snapshot->sample_count = count;
}

void adc_calibrate_current_offset(uint16_t settle_samples)
{
    adc_snapshot_t snap;
    uint32_t guard;

    /* Flush any pre-existing samples taken before calibration was invoked. */
    current_offset_raw = 0u;
    adc_consume_snapshot(&snap);

    /* Wait for enough fresh samples. PWM is expected to be idle (no phase
     * masking applied) so the INA180 is only reporting its zero-current
     * offset.  Bound the wait to avoid hanging if the ISR is not firing. */
    guard = 2000000u;  /* ~50-100 ms at 24 MHz; bail out if ADC IRQ is silent */
    do {
        if (guard-- == 0u)
            return;
    } while (sample_count < settle_samples);

    adc_consume_snapshot(&snap);
    if (snap.sample_count >= settle_samples)
        current_offset_raw = snap.avg_current_raw;
}

uint16_t adc_get_current_offset(void)
{
    return current_offset_raw;
}

uint32_t adc_overrun_count(void)
{
    uint32_t value;
    uint32_t primask = irq_save();
    value = overrun_count;
    irq_restore(primask);
    return value;
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
