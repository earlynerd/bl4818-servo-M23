#include "M2003.h"
#include "bl_internal.h"

#define BL_GATE_MASK (BIT7 | BIT8 | BIT9 | BIT11 | BIT12 | BIT13)

static void unlock_protected_registers(void)
{
    SYS->REGLCTL = 0x59u;
    SYS->REGLCTL = 0x16u;
    SYS->REGLCTL = 0x88u;
}

static void wait_for_bod_write(void)
{
    while ((SYS->BODCTL & SYS_BODCTL_WRBUSY_Msk) != 0u) {
    }
}

void bl_early_safe_outputs(void)
{
    uint32_t mode;

    unlock_protected_registers();
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;
    CLK->AHBCLK |= CLK_AHBCLK_GPBCKEN_Msk;
    CLK->APBCLK1 |= CLK_APBCLK1_PWM0CKEN_Msk;

    PWM0->CNTEN = 0u;
    PWM0->POEN = 0u;
    PWM0->MSKEN = 0x3Fu;
    PWM0->MSK = 0u;

    /* PB1 high holds this retrofit's shared CSn/status LED visibly on
     * whenever LDROM is resident. The loader never talks to the encoder. */
    PB->DOUT = (PB->DOUT & ~BL_GATE_MASK) | BIT1;
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB1MFP_Msk |
                       SYS_GPB_MFPL_PB7MFP_Msk);
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB8MFP_Msk |
                       SYS_GPB_MFPH_PB9MFP_Msk |
                       SYS_GPB_MFPH_PB11MFP_Msk |
                       SYS_GPB_MFPH_PB12MFP_Msk |
                       SYS_GPB_MFPH_PB13MFP_Msk);

    mode = PB->MODE;
    mode &= ~(GPIO_MODE_MODE1_Msk | GPIO_MODE_MODE7_Msk |
              GPIO_MODE_MODE8_Msk |
              GPIO_MODE_MODE9_Msk | GPIO_MODE_MODE11_Msk |
              GPIO_MODE_MODE12_Msk | GPIO_MODE_MODE13_Msk);
    mode |= (GPIO_MODE_OUTPUT << GPIO_MODE_MODE1_Pos) |
            (GPIO_MODE_OUTPUT << GPIO_MODE_MODE7_Pos) |
            (GPIO_MODE_OUTPUT << GPIO_MODE_MODE8_Pos) |
            (GPIO_MODE_OUTPUT << GPIO_MODE_MODE9_Pos) |
            (GPIO_MODE_OUTPUT << GPIO_MODE_MODE11_Pos) |
            (GPIO_MODE_OUTPUT << GPIO_MODE_MODE12_Pos) |
            (GPIO_MODE_OUTPUT << GPIO_MODE_MODE13_Pos);
    PB->MODE = mode;

    wait_for_bod_write();
    SYS->BODCTL = (SYS->BODCTL & ~SYS_BODCTL_BODVL_Msk) |
                  SYS_BODCTL_BODVL_2_7V |
                  SYS_BODCTL_BODRSTEN_Msk |
                  SYS_BODCTL_BODEN_Msk;
    wait_for_bod_write();
}

void bl_clock_init(void)
{
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;
    while ((CLK->STATUS & CLK_STATUS_HIRCSTB_Msk) == 0u) {
    }

    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) |
                   CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLKDIV_Msk) |
                   CLK_CLKDIV0_HCLK(1);
    CLK->AHBCLK |= CLK_AHBCLK_ISPCKEN_Msk |
                   CLK_AHBCLK_GPBCKEN_Msk | CLK_AHBCLK_GPFCKEN_Msk;
}

void bl_watchdog_service(void)
{
    /* Harmless when WDT is disabled, essential when the application entered
     * LDROM through a system reset with its watchdog still running. */
    WDT_RESET_COUNTER();
}
