/*
 * PWM Driver for Nuvoton M2003 (EPWM0)
 *
 * This file ports the MS51 six-step commutation masking to the
 * M2003's Enhanced PWM module.
 */

#include <stdint.h>
#include "m2003_config.h"

/* Simplified Register Access (assuming M2003.h standard names) */
#include "M2003.h"

static uint16_t current_duty = 0;

void pwm_init(void)
{
    /* 1. Enable PWM0 Clock */
    CLK->APBCLK1 |= CLK_APBCLK1_PWM0CKEN_Msk;
    CLK->CLKSEL2 = (CLK->CLKSEL2 & ~CLK_CLKSEL2_PWM0SEL_Msk) | CLK_CLKSEL2_PWM0SEL_PCLK0;

    /* 2. Configure Pins (Multi-Function Pins) */
    /* M2003 TSSOP20 Pin Mapping:
     * Pin 13: PB.13 -> PWM0_CH0 (U_LOW)
     * Pin 14: PB.12 -> PWM0_CH1 (U_HIGH)
     * Pin 15: PB.7  -> PWM0_CH2 (V_HIGH)
     * Pin 16: PB.8  -> PWM0_CH3 (V_LOW)
     * Pin 17: PB.9  -> PWM0_CH4 (W_LOW)
     * Pin 19: PB.11 -> PWM0_CH5 (W_HIGH)
     */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB13MFP_Msk) | SYS_GPB_MFPH_PB13MFP_PWM0_CH0;
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB12MFP_Msk) | SYS_GPB_MFPH_PB12MFP_PWM0_CH1;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB7MFP_Msk)  | SYS_GPB_MFPL_PB7MFP_PWM0_CH2;
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB8MFP_Msk)  | SYS_GPB_MFPH_PB8MFP_PWM0_CH3;
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB9MFP_Msk)  | SYS_GPB_MFPH_PB9MFP_PWM0_CH4;
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB11MFP_Msk) | SYS_GPB_MFPH_PB11MFP_PWM0_CH5;

    /* Force push-pull multi-function output and high slew rate for gate drive. */
    SYS->GPB_MFOS &= ~(SYS_GPB_MFOS_PB7MFOS_Msk |
                       SYS_GPB_MFOS_PB8MFOS_Msk |
                       SYS_GPB_MFOS_PB9MFOS_Msk |
                       SYS_GPB_MFOS_PB11MFOS_Msk |
                       SYS_GPB_MFOS_PB12MFOS_Msk |
                       SYS_GPB_MFOS_PB13MFOS_Msk);
    PB->PUSEL &= ~(GPIO_PUSEL_PUSEL7_Msk |
                   GPIO_PUSEL_PUSEL8_Msk |
                   GPIO_PUSEL_PUSEL9_Msk |
                   GPIO_PUSEL_PUSEL11_Msk |
                   GPIO_PUSEL_PUSEL12_Msk |
                   GPIO_PUSEL_PUSEL13_Msk);
    GPIO_SetMode(PB, BIT7 | BIT8 | BIT9 | BIT11 | BIT12 | BIT13, GPIO_MODE_OUTPUT);
    GPIO_SetSlewCtl(PB, BIT7 | BIT8 | BIT9 | BIT11 | BIT12 | BIT13, GPIO_SLEWCTL_HIGH);

    /* 3. Set PWM Period and Mode — Independent output (OUTMODE=0, the default).
     * Up-counter type (CNTTYPE=0). */
    PWM0->CTL1 = 0;
    PWM0->PERIOD[0] = PWM_MAX_DUTY - 1;
    PWM0->PERIOD[1] = PWM_MAX_DUTY - 1;
    PWM0->PERIOD[2] = PWM_MAX_DUTY - 1;
    PWM0->PERIOD[3] = PWM_MAX_DUTY - 1;
    PWM0->PERIOD[4] = PWM_MAX_DUTY - 1;
    PWM0->PERIOD[5] = PWM_MAX_DUTY - 1;

    /* Configure Waveform Generator for independent PWM on all channels:
     * Output HIGH at zero point, Output LOW at compare up match. */
    PWM0->WGCTL0 = 0x00000AAA; /* ZPCTL0-5 = 10 (HIGH) */
    PWM0->WGCTL1 = 0x00000555; /* CMPUCTL0-5 = 01 (LOW) */

    /* 4. Disable Dead-time */
    PWM0->DTCTL[0] = 0;
    PWM0->DTCTL[1] = 0;
    PWM0->DTCTL[2] = 0;

    /* 5. Initialize Mask (All OFF) */
    PWM0->MSKEN = 0x3F;
    PWM0->MSK   = 0x00;

    /* 6. Enable Output Pins */
    PWM0->POEN |= 0x3F;

    /* 7. Configure ADC Triggering */
    /* Select PWM_CH0 zero point to trigger ADC */
    PWM0->ADCTS0 = (PWM0->ADCTS0 & ~PWM_ADCTS0_TRGSEL0_Msk) | (0 << PWM_ADCTS0_TRGSEL0_Pos);
    /* Enable Trigger */
    PWM0->ADCTS0 |= PWM_ADCTS0_TRGEN0_Msk;

    /* 8. Start the counter continuously so ADC triggers fire */
    PWM0->CNTEN |= PWM_CNTEN_CNTEN0_Msk | PWM_CNTEN_CNTEN2_Msk | PWM_CNTEN_CNTEN4_Msk;
}

void pwm_set_duty(uint16_t duty)
{
    if (duty > PWM_MAX_DUTY)
        duty = PWM_MAX_DUTY;

    current_duty = duty;

    PWM0->CMPDAT[0] = duty;
    PWM0->CMPDAT[1] = duty;
    PWM0->CMPDAT[2] = duty;
    PWM0->CMPDAT[3] = duty;
    PWM0->CMPDAT[4] = duty;
    PWM0->CMPDAT[5] = duty;
}

void pwm_set_commutation(uint8_t pmen, uint8_t pmd)
{
    PWM0->MSKEN = pmen & 0x3F;
    PWM0->MSK   = pmd & 0x3F;
}

void pwm_enable(void)
{
    PWM0->CNTEN |= PWM_CNTEN_CNTEN0_Msk | PWM_CNTEN_CNTEN2_Msk | PWM_CNTEN_CNTEN4_Msk;
}

void pwm_disable(void)
{
    PWM0->CNTEN &= ~(PWM_CNTEN_CNTEN0_Msk | PWM_CNTEN_CNTEN2_Msk | PWM_CNTEN_CNTEN4_Msk);
    PWM0->MSKEN = 0x3F;
    PWM0->MSK   = 0x00;
}

void pwm_fault_brake(void)
{
    PWM0->MSKEN = 0x3F;
    PWM0->MSK   = 0x00;
    PWM0->CNTEN &= ~(PWM_CNTEN_CNTEN0_Msk | PWM_CNTEN_CNTEN2_Msk | PWM_CNTEN_CNTEN4_Msk);
}

uint16_t pwm_get_duty(void)
{
    return current_duty;
}
