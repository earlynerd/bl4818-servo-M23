/**************************************************************************//**
 * @file     sys.c
 * @version  V0.10
 * @brief    System Manager (SYS) driver source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup SYS_Driver SYS Driver
  @{
*/

int32_t g_SYS_i32ErrCode = 0;   /*!< SYS global error code */

/** @addtogroup SYS_EXPORTED_FUNCTIONS SYS Exported Functions
  @{
*/

/**
  * @brief      Clear reset source
  * @param[in]  u32Src is system reset source. Including :
  *             - \ref SYS_RSTSTS_CPULKRF_Msk
  *             - \ref SYS_RSTSTS_CPURF_Msk
  *             - \ref SYS_RSTSTS_SYSRF_Msk
  *             - \ref SYS_RSTSTS_BODRF_Msk
  *             - \ref SYS_RSTSTS_LVRF_Msk
  *             - \ref SYS_RSTSTS_WDTRF_Msk
  *             - \ref SYS_RSTSTS_PINRF_Msk
  *             - \ref SYS_RSTSTS_PORF_Msk
  * @return     None
  * @details    This function clear the selected system reset source.
  */
void SYS_ClearResetSrc(uint32_t u32Src)
{
    SYS->RSTSTS = u32Src;
}

/**
  * @brief      Get Brown-out detector output status
  * @param      None
  * @retval     0 System voltage is higher than BODVL setting or BODEN is 0.
  * @retval     1 System voltage is lower than BODVL setting.
  * @details    This function get Brown-out detector output status.
  */
uint32_t SYS_GetBODStatus(void)
{
    return ((SYS->BODCTL & SYS_BODCTL_BODOUT_Msk) >> SYS_BODCTL_BODOUT_Pos);
}

/**
  * @brief      Get reset status register value
  * @param      None
  * @return     Reset source
  * @details    This function get the system reset status register value.
  */
uint32_t SYS_GetResetSrc(void)
{
    return (SYS->RSTSTS);
}

/**
  * @brief      Check if register is locked nor not
  * @param      None
  * @retval     0 Write-protection function is disabled.
  *             1 Write-protection function is enabled.
  * @details    This function check register write-protection bit setting.
  */
uint32_t SYS_IsRegLocked(void)
{
    return (SYS->REGLCTL & 1UL) ? 0UL : 1UL;
}

/**
  * @brief      Get product ID
  * @param      None
  * @return     Product ID
  * @details    This function get product ID.
  */
uint32_t  SYS_ReadPDID(void)
{
    return SYS->PDID;
}

/**
  * @brief      Reset chip with chip reset
  * @param      None
  * @return     None
  * @details    This function reset chip with chip reset.
  *             The register write-protection function should be disabled before using this function.
  */
void SYS_ResetChip(void)
{
    SYS->IPRST0 |= SYS_IPRST0_CHIPRST_Msk;
}

/**
  * @brief      Reset chip with CPU reset
  * @param      None
  * @return     None
  * @details    This function reset CPU with CPU reset.
  *             The register write-protection function should be disabled before using this function.
  */
void SYS_ResetCPU(void)
{
    SYS->IPRST0 |= SYS_IPRST0_CPURST_Msk;
}

/**
  * @brief      Reset selected module
  * @param[in]  u32ModuleIndex is module index. Including :
  *             - \ref GPIO_RST
  *             - \ref TMR0_RST
  *             - \ref TMR1_RST
  *             - \ref TMR2_RST
  *             - \ref TMR3_RST
  *             - \ref I2C0_RST
  *             - \ref UART0_RST
  *             - \ref UART1_RST
  *             - \ref ADC_RST
  *             - \ref USCI0_RST
  *             - \ref PWM0_RST
  *             - \ref ECAP0_RST
  * @return     None
  * @details    This function reset selected module.
  */
void SYS_ResetModule(uint32_t u32ModuleIndex)
{
    uint32_t u32TmpVal = 0UL, u32TmpAddr = 0UL;

    /* Generate reset signal to the corresponding module */
    u32TmpVal = (1UL << (u32ModuleIndex & 0x00ffffffUL));
    u32TmpAddr = (uint32_t)&SYS->IPRST0 + ((u32ModuleIndex >> 24UL));
    *(uint32_t *)u32TmpAddr |= u32TmpVal;

    /* Release corresponding module from reset state */
    u32TmpVal = ~(1UL << (u32ModuleIndex & 0x00ffffffUL));
    *(uint32_t *)u32TmpAddr &= u32TmpVal;
}

/**
  * @brief      Enable and configure Brown-out detector function
  * @param[in]  i32Mode is reset or interrupt mode. Including :
  *             - \ref SYS_BODCTL_BOD_RST_EN
  *             - \ref SYS_BODCTL_BOD_INTERRUPT_EN
  * @param[in]  u32BODLevel is Brown-out voltage level. Including :
  *             - \ref SYS_BODCTL_BODVL_2_2V
  *             - \ref SYS_BODCTL_BODVL_2_7V
  *             - \ref SYS_BODCTL_BODVL_3_7V
  *             - \ref SYS_BODCTL_BODVL_4_4V
  * @return     Setting success or not
  * @retval     0                   Success
  * @retval     SYS_TIMEOUT_ERR     Failed due to BODCTL register is busy
  * @details    This function configure Brown-out detector reset or interrupt mode, enable Brown-out function and set Brown-out voltage level.
  *             The register write-protection function should be disabled before using this function.
  */
int32_t SYS_EnableBOD(int32_t i32Mode, uint32_t u32BODLevel)
{
    uint32_t u32TimeOutCnt = SystemCoreClock * 2;

    while(SYS->BODCTL & SYS_BODCTL_WRBUSY_Msk)
    {
        if(--u32TimeOutCnt == 0)
        {
            break;
        }
    }

    /* Enable Brown-out Detector function */
    /* Enable Brown-out interrupt or reset function */
    /* Select Brown-out Detector threshold voltage */
    SYS->BODCTL = (SYS->BODCTL & ~(SYS_BODCTL_BODRSTEN_Msk | SYS_BODCTL_BODVL_Msk)) |
                  ((uint32_t)i32Mode) | (u32BODLevel) | (SYS_BODCTL_BODEN_Msk);

    if(u32TimeOutCnt == 0)
        return SYS_TIMEOUT_ERR;
    else
        return 0;
}

/**
  * @brief      Disable Brown-out detector function
  * @param      None
  * @return     Setting success or not
  * @retval     0                   Success
  * @retval     SYS_TIMEOUT_ERR     Failed due to BODCTL register is busy
  * @details    This function disable Brown-out detector function.
  *             The register write-protection function should be disabled before using this function.
  */
int32_t SYS_DisableBOD(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock * 2;

    while(SYS->BODCTL & SYS_BODCTL_WRBUSY_Msk)
    {
        if(--u32TimeOutCnt == 0)
        {
            break;
        }
    }

    SYS->BODCTL &= ~SYS_BODCTL_BODEN_Msk;

    if(u32TimeOutCnt == 0)
        return SYS_TIMEOUT_ERR;
    else
        return 0;
}

/**@}*/ /* end of group SYS_EXPORTED_FUNCTIONS */

/**@}*/ /* end of group SYS_Driver */

/**@}*/ /* end of group Standard_Driver */
