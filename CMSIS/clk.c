/**************************************************************************//**
 * @file     clk.c
 * @version  V0.10
 * @brief    M2003C series Clock Controller (CLK) driver source file
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup CLK_Driver CLK Driver
  @{
*/

int32_t g_CLK_i32ErrCode = 0;   /*!< CLK global error code */

/** @addtogroup CLK_EXPORTED_FUNCTIONS CLK Exported Functions
  @{
*/

/**
  * @brief      Disable frequency output function
  * @param      None
  * @return     None
  * @details    This function disable frequency output function.
  */
void CLK_DisableCKO(void)
{
    /* Disable CKO clock source */
    CLK->APBCLK0 &= (~CLK_APBCLK0_CLKOCKEN_Msk);
}


/**
  * @brief      This function enable frequency divider module clock.
  *             enable frequency divider clock function and configure frequency divider.
  * @param[in]  u32ClkSrc is frequency divider function clock source. Including :
  *             - \ref CLK_CLKSEL1_CLKOSEL_HCLK
  *             - \ref CLK_CLKSEL1_CLKOSEL_HIRC
  * @param[in]  u32ClkDiv is divider output frequency selection.
  * @param[in]  u32ClkDivBy1En is frequency divided by one enable.
  * @return     None
  * @details    Output selected clock to CKO. The output clock frequency is divided by u32ClkDiv.
  *             The formula is:
  *                 CKO frequency = (Clock source frequency) / 2^(u32ClkDiv + 1)
  *             This function is just used to set CKO clock.
  *             User must enable I/O for CKO clock output pin by themselves.
  */
void CLK_EnableCKO(uint32_t u32ClkSrc, uint32_t u32ClkDiv, uint32_t u32ClkDivBy1En)
{
    /* CKO = clock source / 2^(u32ClkDiv + 1) */
    CLK->CLKOCTL = CLK_CLKOCTL_CLKOEN_Msk | u32ClkDiv | (u32ClkDivBy1En << CLK_CLKOCTL_DIV1EN_Pos);

    /* Enable CKO clock source */
    CLK->APBCLK0 |= CLK_APBCLK0_CLKOCKEN_Msk;

    /* Select CKO clock source */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_CLKOSEL_Msk)) | (u32ClkSrc);
}

/**
  * @brief      Enter to Power-down mode
  * @param      None
  * @return     None
  * @details    This function is used to let system enter to Power-down mode. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_PowerDown(void)
{
    volatile uint32_t u32SysTickTICKINT = 0;    /* Backup Systick interrupt enable bit */

    /* Set the processor uses deep sleep as its low power mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* Set system Power-down enabled */
    CLK->PWRCTL |= CLK_PWRCTL_PDEN_Msk;

    /* Backup systick interrupt setting */
    u32SysTickTICKINT = SysTick->CTRL & SysTick_CTRL_TICKINT_Msk;

    /* Disable systick interrupt */
    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;

    /* Chip enter Power-down mode after CPU run WFI instruction */
    __WFI();

    /* Restore systick interrupt setting */
    if(u32SysTickTICKINT) SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
}


/**
  * @brief      Enter to Idle mode
  * @param      None
  * @return     None
  * @details    This function let system enter to Idle mode. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_Idle(void)
{
    /* Set the processor uses sleep as its low power mode */
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

    /* Set chip in idle mode because of WFI command */
    CLK->PWRCTL &= ~CLK_PWRCTL_PDEN_Msk;

    /* Chip enter idle mode after CPU run WFI instruction */
    __WFI();
}


/**
  * @brief      Get HCLK frequency
  * @param      None
  * @return     HCLK frequency
  * @details    This function get HCLK frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetHCLKFreq(void)
{
    SystemCoreClockUpdate();
    return SystemCoreClock;
}

/**
  * @brief      Get PCLK0 frequency
  * @param      None
  * @return     PCLK0 frequency
  * @details    This function get PCLK0 frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetPCLK0Freq(void)
{
    uint32_t PCLK0Div;

    SystemCoreClockUpdate();
    PCLK0Div = (CLK->PCLKDIV & CLK_PCLKDIV_APB0DIV_Msk) >> CLK_PCLKDIV_APB0DIV_Pos;
    return (SystemCoreClock >> PCLK0Div);
}

/**
  * @brief      Get PCLK1 frequency
  * @param      None
  * @return     PCLK1 frequency
  * @details    This function get PCLK1 frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetPCLK1Freq(void)
{
    uint32_t PCLK1Div;

    SystemCoreClockUpdate();
    PCLK1Div = (CLK->PCLKDIV & CLK_PCLKDIV_APB1DIV_Msk) >> CLK_PCLKDIV_APB1DIV_Pos;
    return (SystemCoreClock >> PCLK1Div);
}

/**
  * @brief      Get PCLK2 frequency
  * @param      None
  * @return     PCLK2 frequency
  * @details    This function get PCLK2 frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetPCLK2Freq(void)
{
    uint32_t PCLK2Div;

    SystemCoreClockUpdate();
    PCLK2Div = (CLK->PCLKDIV & CLK_PCLKDIV_APB2DIV_Msk) >> CLK_PCLKDIV_APB2DIV_Pos;
    return (SystemCoreClock >> PCLK2Div);
}

/**
  * @brief      Get CPU frequency
  * @param      None
  * @return     CPU frequency
  * @details    This function get CPU frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetCPUFreq(void)
{
    SystemCoreClockUpdate();
    return SystemCoreClock;
}

/**
  * @brief      Set HCLK frequency
  * @param[in]  u32Hclk is HCLK frequency.
  *             It is not used in M2003C.
  * @return     HCLK frequency
  * @details    This function is used to set HCLK frequency by using HIRC. \n
  *             Power level is also set according to HCLK frequency. The frequency unit is Hz. \n
  *             The register write-protection function should be disabled before using this function.
  */
uint32_t CLK_SetCoreClock(uint32_t u32Hclk)
{
    /* The HCLK always is HIRC. */
    u32Hclk = __HIRC;

    /* Select HCLK clock source to HIRC,
       Select HCLK clock source divider as 1,
       and update system core clock
    */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1UL));

    /* Return actually HCLK frequency */
    return u32Hclk;
}


/**
  * @brief      Set HCLK clock source and HCLK clock divider
  * @param[in]  u32ClkSrc is HCLK clock source. Including :
  *             - \ref CLK_CLKSEL0_HCLKSEL_LIRC
  *             - \ref CLK_CLKSEL0_HCLKSEL_HIRC
  * @param[in]  u32ClkDiv is HCLK clock divider. Including :
  *             - \ref CLK_CLKDIV0_HCLK(x)
  * @return     None
  * @details    This function set HCLK clock source and HCLK clock divider. \n
  *             Power level and flash access cycle are also set according to HCLK operation frequency. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32ClkDiv)
{
    uint32_t u32HIRCSTB;

    /* Read HIRC clock source stable flag */
    u32HIRCSTB = CLK->STATUS & CLK_STATUS_HIRCSTB_Msk;

    /* Switch to HIRC for safe. Avoid HCLK too high when applying new divider. */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;

    /* Set Flash Access Cycle to 4 for safe */
    FMC->CYCCTL = (FMC->CYCCTL & (~FMC_CYCCTL_CYCLE_Msk)) | (4);

    /* Apply new Divider */
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | u32ClkDiv;

    /* Switch HCLK to new HCLK source */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | u32ClkSrc;

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Set Flash Access Cycle */
    if((SystemCoreClock >= FREQ_50MHZ) && SystemCoreClock < FREQ_75MHZ)
    {
        FMC->CYCCTL = (FMC->CYCCTL & (~FMC_CYCCTL_CYCLE_Msk)) | (3);
    }
    else if((SystemCoreClock >= FREQ_25MHZ) && SystemCoreClock < FREQ_50MHZ)
    {
        FMC->CYCCTL = (FMC->CYCCTL & (~FMC_CYCCTL_CYCLE_Msk)) | (2);
    }
    if(SystemCoreClock < FREQ_25MHZ)
    {
        FMC->CYCCTL = (FMC->CYCCTL & (~FMC_CYCCTL_CYCLE_Msk)) | (1);
    }

    /* Disable HIRC if HIRC is disabled before switching HCLK source */
    if(u32ClkSrc != CLK_CLKSEL0_HCLKSEL_HIRC)
    {
        if(u32HIRCSTB == 0UL)
        {
            CLK->PWRCTL &= ~CLK_PWRCTL_HIRCEN_Msk;
        }
    }
}

/**
  * @brief      This function set selected module clock source and module clock divider
  * @param[in]  u32ModuleIdx is module index.
  * @param[in]  u32ClkSrc is module clock source.
  * @param[in]  u32ClkDiv is module clock divider.
  * @return     None
  * @details    Valid parameter combinations listed in following table:
  *
  * |Module index        |Clock source                           |Divider                   |
  * | :----------------  | :------------------------------------ | :----------------------- |
  * |\ref ADC_MODULE     |\ref CLK_CLKSEL2_ADCSEL_HIRC           |\ref CLK_CLKDIV0_ADC(x)   |
  * |\ref ADC_MODULE     |\ref CLK_CLKSEL2_ADCSEL_PCLK2          |\ref CLK_CLKDIV0_ADC(x)   |
  * |\ref CLKO_MODULE    |\ref CLK_CLKSEL1_CLKOSEL_HCLK          | x                        |
  * |\ref CLKO_MODULE    |\ref CLK_CLKSEL1_CLKOSEL_HIRC          | x                        |
  * |\ref PWM0_MODULE    |\ref CLK_CLKSEL2_PWM0SEL_PCLK0         | x                        |
  * |\ref TMR0_MODULE    |\ref CLK_CLKSEL1_TMR0SEL_EXT_TRG       | x                        |
  * |\ref TMR0_MODULE    |\ref CLK_CLKSEL1_TMR0SEL_HIRC          | x                        |
  * |\ref TMR0_MODULE    |\ref CLK_CLKSEL1_TMR0SEL_LIRC          | x                        |
  * |\ref TMR0_MODULE    |\ref CLK_CLKSEL1_TMR0SEL_PCLK0         | x                        |
  * |\ref TMR1_MODULE    |\ref CLK_CLKSEL1_TMR1SEL_EXT_TRG       | x                        |
  * |\ref TMR1_MODULE    |\ref CLK_CLKSEL1_TMR1SEL_HIRC          | x                        |
  * |\ref TMR1_MODULE    |\ref CLK_CLKSEL1_TMR1SEL_LIRC          | x                        |
  * |\ref TMR1_MODULE    |\ref CLK_CLKSEL1_TMR1SEL_PCLK0         | x                        |
  * |\ref TMR2_MODULE    |\ref CLK_CLKSEL1_TMR2SEL_EXT_TRG       | x                        |
  * |\ref TMR2_MODULE    |\ref CLK_CLKSEL1_TMR2SEL_HIRC          | x                        |
  * |\ref TMR2_MODULE    |\ref CLK_CLKSEL1_TMR2SEL_LIRC          | x                        |
  * |\ref TMR2_MODULE    |\ref CLK_CLKSEL1_TMR2SEL_PCLK1         | x                        |
  * |\ref TMR3_MODULE    |\ref CLK_CLKSEL1_TMR3SEL_EXT_TRG       | x                        |
  * |\ref TMR3_MODULE    |\ref CLK_CLKSEL1_TMR3SEL_HIRC          | x                        |
  * |\ref TMR3_MODULE    |\ref CLK_CLKSEL1_TMR3SEL_LIRC          | x                        |
  * |\ref TMR3_MODULE    |\ref CLK_CLKSEL1_TMR3SEL_PCLK1         | x                        |
  * |\ref UART0_MODULE   |\ref CLK_CLKSEL2_UART0SEL_HIRC         |\ref CLK_CLKDIV0_UART0(x) |
  * |\ref UART0_MODULE   |\ref CLK_CLKSEL2_UART0SEL_PCLK0        |\ref CLK_CLKDIV0_UART0(x) |
  * |\ref UART1_MODULE   |\ref CLK_CLKSEL2_UART1SEL_HIRC         |\ref CLK_CLKDIV0_UART1(x) |
  * |\ref UART1_MODULE   |\ref CLK_CLKSEL2_UART1SEL_PCLK1        |\ref CLK_CLKDIV0_UART1(x) |
  * |\ref WDT_MODULE     |\ref CLK_CLKSEL1_WDTSEL_HCLK_DIV2048   | x                        |
  * |\ref WDT_MODULE     |\ref CLK_CLKSEL1_WDTSEL_LIRC           | x                        |
  * |\ref WWDT_MODULE    |\ref CLK_CLKSEL1_WWDTSEL_HCLK_DIV2048  | x                        |
  * |\ref WWDT_MODULE    |\ref CLK_CLKSEL1_WWDTSEL_LIRC          | x                        |
  */
void CLK_SetModuleClock(uint32_t u32ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv)
{
    uint32_t u32Sel = 0UL, u32Div = 0UL;
    uint32_t au32SelTbl[4] = {0x0UL, 0x4UL, 0x8UL, 0xCUL};
    uint32_t au32DivTbl[4] = {0x0UL, 0x4UL, 0x8UL, 0x10UL};

    if(MODULE_CLKDIV_Msk(u32ModuleIdx) != MODULE_NoMsk)
    {
        /* Get clock divider control register address */
        u32Div = (uint32_t)&CLK->CLKDIV0 + (au32DivTbl[MODULE_CLKDIV(u32ModuleIdx)]);
        /* Apply new divider */
        M32(u32Div) = (M32(u32Div) & (~(MODULE_CLKDIV_Msk(u32ModuleIdx) << MODULE_CLKDIV_Pos(u32ModuleIdx)))) | u32ClkDiv;
    }

    if(MODULE_CLKSEL_Msk(u32ModuleIdx) != MODULE_NoMsk)
    {
        /* Get clock select control register address */
        u32Sel = (uint32_t)&CLK->CLKSEL0 + (au32SelTbl[MODULE_CLKSEL(u32ModuleIdx)]);
        /* Set new clock selection setting */
        M32(u32Sel) = (M32(u32Sel) & (~(MODULE_CLKSEL_Msk(u32ModuleIdx) << MODULE_CLKSEL_Pos(u32ModuleIdx)))) | u32ClkSrc;
    }
}

/**
  * @brief      Set SysTick clock source
  * @param[in]  u32ClkSrc is module clock source. Including:
  *             - \ref CLK_CLKSEL0_STCLKSEL_HCLK_DIV2
  *             - \ref CLK_CLKSEL0_STCLKSEL_HIRC_DIV2
  * @return     None
  * @details    This function set SysTick clock source. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_SetSysTickClockSrc(uint32_t u32ClkSrc)
{
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_STCLKSEL_Msk) | u32ClkSrc;
}

/**
  * @brief      Enable clock source
  * @param[in]  u32ClkMask is clock source mask. Including :
  *             - \ref CLK_PWRCTL_HIRCEN_Msk
  *             - \ref CLK_PWRCTL_LIRCEN_Msk
  * @return     None
  * @details    This function enable clock source. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_EnableXtalRC(uint32_t u32ClkMask)
{
    CLK->PWRCTL |= u32ClkMask;
}

/**
  * @brief      Disable clock source
  * @param[in]  u32ClkMask is clock source mask. Including :
  *             - \ref CLK_PWRCTL_HIRCEN_Msk
  *             - \ref CLK_PWRCTL_LIRCEN_Msk
  * @return     None
  * @details    This function disable clock source. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_DisableXtalRC(uint32_t u32ClkMask)
{
    CLK->PWRCTL &= ~u32ClkMask;
}

/**
  * @brief      Enable module clock
  * @param[in]  u32ModuleIdx is module index. Including :
  *             - \ref ADC_MODULE
  *             - \ref CLKO_MODULE
  *             - \ref ECAP0_MODULE
  *             - \ref FMCIDLE_MODULE
  *             - \ref GPB_MODULE
  *             - \ref GPC_MODULE
  *             - \ref GPE_MODULE
  *             - \ref GPF_MODULE
  *             - \ref I2C0_MODULE
  *             - \ref ISP_MODULE
  *             - \ref PWM0_MODULE
  *             - \ref SRAM0_MODULE
  *             - \ref ST_MODULE
  *             - \ref TMR0_MODULE
  *             - \ref TMR1_MODULE
  *             - \ref TMR2_MODULE
  *             - \ref TMR3_MODULE
  *             - \ref UART0_MODULE
  *             - \ref UART1_MODULE
  *             - \ref USCI0_MODULE
  *             - \ref WDT_MODULE
  *             - \ref WWDT_MODULE
  * @return     None
  * @details    This function is used to enable module clock.
  */
void CLK_EnableModuleClock(uint32_t u32ModuleIdx)
{
    uint32_t u32TmpVal = 0UL, u32TmpAddr = 0UL;

    u32TmpVal = (1UL << MODULE_IP_EN_Pos(u32ModuleIdx));
    u32TmpAddr = (uint32_t)&CLK->AHBCLK;
    u32TmpAddr += ((MODULE_APBCLK(u32ModuleIdx) * 4UL));

    *(volatile uint32_t *)u32TmpAddr |= u32TmpVal;
}

/**
  * @brief      Disable module clock
  * @param[in]  u32ModuleIdx is module index
  *             - \ref ADC_MODULE
  *             - \ref CLKO_MODULE
  *             - \ref ECAP0_MODULE
  *             - \ref FMCIDLE_MODULE
  *             - \ref GPB_MODULE
  *             - \ref GPC_MODULE
  *             - \ref GPE_MODULE
  *             - \ref GPF_MODULE
  *             - \ref I2C0_MODULE
  *             - \ref ISP_MODULE
  *             - \ref PWM0_MODULE
  *             - \ref SRAM0_MODULE
  *             - \ref ST_MODULE
  *             - \ref TMR0_MODULE
  *             - \ref TMR1_MODULE
  *             - \ref TMR2_MODULE
  *             - \ref TMR3_MODULE
  *             - \ref UART0_MODULE
  *             - \ref UART1_MODULE
  *             - \ref USCI0_MODULE
  *             - \ref WDT_MODULE
  *             - \ref WWDT_MODULE
  * @return     None
  * @details    This function is used to disable module clock.
  */
void CLK_DisableModuleClock(uint32_t u32ModuleIdx)
{
    uint32_t u32TmpVal = 0UL, u32TmpAddr = 0UL;

    u32TmpVal = ~(1UL << MODULE_IP_EN_Pos(u32ModuleIdx));
    u32TmpAddr = (uint32_t)&CLK->AHBCLK;
    u32TmpAddr += ((MODULE_APBCLK(u32ModuleIdx) * 4UL));

    *(uint32_t *)u32TmpAddr &= u32TmpVal;
}

/**
  * @brief      This function check selected clock source status
  * @param[in]  u32ClkMask is selected clock source. Including :
  *             - \ref CLK_STATUS_HIRCSTB_Msk
  *             - \ref CLK_STATUS_LIRCSTB_Msk
  * @retval     0  clock is not stable
  * @retval     1  clock is stable
  * @details    To wait for clock ready by specified clock source stable flag or timeout (~500ms)
  * @note       This function sets g_CLK_i32ErrCode to CLK_TIMEOUT_ERR if clock source status is not stable
  */
uint32_t CLK_WaitClockReady(uint32_t u32ClkMask)
{
    uint32_t u32TimeOutCnt = SystemCoreClock / 2;
    uint32_t u32Ret = 1U;

    g_CLK_i32ErrCode = 0;
    while((CLK->STATUS & u32ClkMask) != u32ClkMask)
    {
        if(--u32TimeOutCnt == 0)
        {
            u32Ret = 0U;
            break;
        }
    }

    if(u32TimeOutCnt == 0)
        g_CLK_i32ErrCode = CLK_TIMEOUT_ERR;

    return u32Ret;
}

/**
  * @brief      Enable System Tick counter
  * @param[in]  u32ClkSrc is System Tick clock source. Including:
  *             - \ref CLK_CLKSEL0_STCLKSEL_HCLK_DIV2
  *             - \ref CLK_CLKSEL0_STCLKSEL_HIRC_DIV2
  *             - \ref CLK_CLKSEL0_STCLKSEL_HCLK
  * @param[in]  u32Count is System Tick reload value. It could be 0~0xFFFFFF.
  * @return     None
  * @details    This function set System Tick clock source, reload value, enable System Tick counter and interrupt. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_EnableSysTick(uint32_t u32ClkSrc, uint32_t u32Count)
{
    /* Set System Tick counter disabled */
    SysTick->CTRL = 0UL;

    /* Set System Tick clock source */
    if(u32ClkSrc == CLK_CLKSEL0_STCLKSEL_HCLK)
    {
        SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;
    }
    else
    {
        CLK->AHBCLK |= CLK_AHBCLK_STCKEN_Msk;
        CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_STCLKSEL_Msk) | u32ClkSrc;
    }

    /* Set System Tick reload value */
    SysTick->LOAD = u32Count;

    /* Clear System Tick current value and counter flag */
    SysTick->VAL = 0UL;

    /* Set System Tick interrupt enabled and counter enabled */
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

/**
  * @brief      Disable System Tick counter
  * @param      None
  * @return     None
  * @details    This function disable System Tick counter.
  */
void CLK_DisableSysTick(void)
{
    /* Set System Tick counter disabled */
    SysTick->CTRL = 0UL;
}

/**
  * @brief      Power-down mode selected
  * @param[in]  u32PDMode is power down mode index. Including :
  *             - \ref CLK_PMUCTL_PDMSEL_PD
  *             - \ref CLK_PMUCTL_PDMSEL_SPD
  *             - \ref CLK_PMUCTL_PDMSEL_DPD
  * @return     Setting success or not
  * @retval     0                   Success
  * @retval     CLK_TIMEOUT_ERR     Failed due to PMUCTL register is busy
  * @details    This function is used to set power-down mode.
  */
int32_t CLK_SetPowerDownMode(uint32_t u32PDMode)
{
    uint32_t u32TimeOutCnt = SystemCoreClock * 2;

    while(CLK->PMUCTL & CLK_PMUCTL_WRBUSY_Msk)
    {
        if(--u32TimeOutCnt == 0)
        {
            break;
        }
    }
    CLK->PMUCTL = (CLK->PMUCTL & (~CLK_PMUCTL_PDMSEL_Msk)) | (u32PDMode);

    if(u32TimeOutCnt == 0)
        return CLK_TIMEOUT_ERR;
    else
        return 0;
}

/**
 * @brief       Set Wake-up pin trigger type at Deep Power down mode
 * @param[in]   u32TriggerType Wake-up pin trigger type
 *              - \ref CLK_DPDWKPIN1_RISING
 *              - \ref CLK_DPDWKPIN1_FALLING
 *              - \ref CLK_DPDWKPIN1_BOTHEDGE
 *              - \ref CLK_DPDWKPIN2_RISING
 *              - \ref CLK_DPDWKPIN2_FALLING
 *              - \ref CLK_DPDWKPIN2_BOTHEDGE
 *              - \ref CLK_DPDWKPIN3_RISING
 *              - \ref CLK_DPDWKPIN3_FALLING
 *              - \ref CLK_DPDWKPIN3_BOTHEDGE
  * @return     Setting success or not
  * @retval     0                   Success
  * @retval     CLK_TIMEOUT_ERR     Failed due to PMUCTL register is busy
 * @details     This function is used to enable Wake-up pin trigger type.
 */
int32_t CLK_EnableDPDWKPin(uint32_t u32TriggerType)
{
    uint32_t u32TimeOutCnt = SystemCoreClock * 2;
    uint32_t u32Pin1, u32Pin2, u32Pin3;

    u32Pin1 = ((u32TriggerType) & CLK_PMUCTL_WKPINEN1_Msk);
    u32Pin2 = ((u32TriggerType) & CLK_PMUCTL_WKPINEN2_Msk);
    u32Pin3 = ((u32TriggerType) & CLK_PMUCTL_WKPINEN3_Msk);

    while(CLK->PMUCTL & CLK_PMUCTL_WRBUSY_Msk)
    {
        if(--u32TimeOutCnt == 0)
        {
            break;
        }
    }

    if(u32Pin1)
    {
        CLK->PMUCTL = (CLK->PMUCTL & ~(CLK_PMUCTL_WKPINEN1_Msk)) | u32TriggerType;
    }
    else if(u32Pin2)
    {
        CLK->PMUCTL = (CLK->PMUCTL & ~(CLK_PMUCTL_WKPINEN2_Msk)) | u32TriggerType;
    }
    else if(u32Pin3)
    {
        CLK->PMUCTL = (CLK->PMUCTL & ~(CLK_PMUCTL_WKPINEN3_Msk)) | u32TriggerType;
    }
    else
    {
        CLK->PMUCTL = (CLK->PMUCTL & ~(CLK_PMUCTL_WKPINEN1_Msk)) | u32TriggerType;
    }

    if(u32TimeOutCnt == 0)
        return CLK_TIMEOUT_ERR;
    else
        return 0;
}

/**
 * @brief       Get power manager wake up source
 * @param[in]   None
 * @return      None
 * @details     This function get power manager wake up source.
 */
uint32_t CLK_GetPMUWKSrc(void)
{
    return (CLK->PMUSTS);
}

/**
 * @brief       Set specified GPIO as wake up source at Standby Power-down mode
 * @param[in]   u32Port GPIO port. It could be 0~3.
 * @param[in]   u32Pin  The pin of specified GPIO port. It could be 0 ~ 15.
 * @param[in]   u32TriggerType Wake-up pin trigger type
 *              - \ref CLK_SPDWKPIN_RISING
 *              - \ref CLK_SPDWKPIN_FALLING
 * @param[in]   u32DebounceEn Standby Power-down mode wake-up pin de-bounce function
 *              - \ref CLK_SPDWKPIN_DEBOUNCEEN
 *              - \ref CLK_SPDWKPIN_DEBOUNCEDIS
 * @return      None
 * @details     This function is used to set specified GPIO as wake up source at Standby Power-down mode.
 */
void CLK_EnableSPDWKPin(uint32_t u32Port, uint32_t u32Pin, uint32_t u32TriggerType, uint32_t u32DebounceEn)
{
    uint32_t u32TmpAddr = 0UL;
    uint32_t u32TmpVal = 0UL;

    /* GPx Stand-by Power-down Wake-up Pin Select */
    u32TmpAddr = (uint32_t)&CLK->PBSWKCTL;
    u32TmpAddr += (0x4UL * u32Port);

    u32TmpVal = inpw((uint32_t *)u32TmpAddr);
    u32TmpVal = (u32TmpVal & ~(CLK_PBSWKCTL_WKPSEL_Msk | CLK_PBSWKCTL_PRWKEN_Msk | CLK_PBSWKCTL_PFWKEN_Msk | CLK_PBSWKCTL_DBEN_Msk | CLK_PBSWKCTL_WKEN_Msk)) |
                (u32Pin << CLK_PBSWKCTL_WKPSEL_Pos) | u32TriggerType | u32DebounceEn | CLK_SPDWKPIN_ENABLE;
    outpw((uint32_t *)u32TmpAddr, u32TmpVal);
}

/**
  * @brief      Get selected module clock source
  * @param[in]  u32ModuleIdx is module index.
  *             - \ref ADC_MODULE
  *             - \ref CLKO_MODULE
  *             - \ref PWM0_MODULE
  *             - \ref ST_MODULE
  *             - \ref TMR0_MODULE
  *             - \ref TMR1_MODULE
  *             - \ref TMR2_MODULE
  *             - \ref TMR3_MODULE
  *             - \ref UART0_MODULE
  *             - \ref UART1_MODULE
  *             - \ref WDT_MODULE
  *             - \ref WWDT_MODULE
  * @return     Selected module clock source setting
  * @details    This function get selected module clock source.
  */
//__NONSECURE_ENTRY_WEAK
uint32_t CLK_GetModuleClockSource(uint32_t u32ModuleIdx)
{
    uint32_t u32TmpVal = 0UL, u32TmpAddr = 0UL;
    uint32_t au32SelTbl[4] = {0x0UL, 0x4UL, 0x8UL, 0xCUL};

    if(MODULE_CLKSEL_Msk(u32ModuleIdx) != MODULE_NoMsk)
    {
        /* Get clock select control register address */
        u32TmpAddr = (uint32_t)&CLK->CLKSEL0 + (au32SelTbl[MODULE_CLKSEL(u32ModuleIdx)]);

        /* Get clock source selection setting */
        u32TmpVal = ((inpw((uint32_t *)u32TmpAddr) & (MODULE_CLKSEL_Msk(u32ModuleIdx) << MODULE_CLKSEL_Pos(u32ModuleIdx))) >> MODULE_CLKSEL_Pos(u32ModuleIdx));
    }

    return u32TmpVal;
}

/**
  * @brief      Get selected module clock divider number
  * @param[in]  u32ModuleIdx is module index.
  *             - \ref UART0_MODULE
  *             - \ref UART1_MODULE
  *             - \ref ADC_MODULE
  * @return     Selected module clock divider number setting
  * @details    This function get selected module clock divider number.
  */
//__NONSECURE_ENTRY_WEAK
uint32_t CLK_GetModuleClockDivider(uint32_t u32ModuleIdx)
{
    uint32_t u32TmpVal = 0UL, u32TmpAddr = 0UL;
    uint32_t au32DivTbl[4] = {0x0UL, 0x4UL, 0x8UL, 0x10UL};

    if(MODULE_CLKDIV_Msk(u32ModuleIdx) != MODULE_NoMsk)
    {
        /* Get clock divider control register address */
        u32TmpAddr = (uint32_t)&CLK->CLKDIV0 + (au32DivTbl[MODULE_CLKDIV(u32ModuleIdx)]);
        /* Get clock divider number setting */
        u32TmpVal = ((inpw((uint32_t *)u32TmpAddr) & (MODULE_CLKDIV_Msk(u32ModuleIdx) << MODULE_CLKDIV_Pos(u32ModuleIdx))) >> MODULE_CLKDIV_Pos(u32ModuleIdx));
    }

    return u32TmpVal;
}

/**@}*/ /* end of group CLK_EXPORTED_FUNCTIONS */

/**@}*/ /* end of group CLK_Driver */

/**@}*/ /* end of group Standard_Driver */
