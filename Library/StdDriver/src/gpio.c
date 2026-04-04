/**************************************************************************//**
 * @file     gpio.c
 * @version  V0.10
 * @brief    General Purpose I/O (GPIO) driver source file
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup GPIO_Driver GPIO Driver
  @{
*/

/** @addtogroup GPIO_EXPORTED_FUNCTIONS GPIO Exported Functions
  @{
*/

/**
 * @brief       Set GPIO operation mode
 * @param[in]   port        GPIO port. It could be PB, PC, or PF.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *                          It could be BIT0 ~ BIT5, BIT7 ~ BIT9, BIT11 ~ BIT15 for PB.
 *                          It could be BIT14 for PC.
 *                          It could be BIT0 ~ BIT1 for PF.
 * @param[in]   u32Mode     Operation mode. It could be
 *                          - \ref GPIO_MODE_INPUT
 *                          - \ref GPIO_MODE_OUTPUT
 *                          - \ref GPIO_MODE_OPEN_DRAIN
 *                          - \ref GPIO_MODE_QUASI
 * @return      None
 * @details     This function is used to set specified GPIO operation mode.
 */
void GPIO_SetMode(GPIO_T *port, uint32_t u32PinMask, uint32_t u32Mode)
{
    uint32_t u32Idx;

    for(u32Idx = 0ul; u32Idx < GPIO_PIN_MAX; u32Idx++)
    {
        if((u32PinMask & (1ul << u32Idx)) == (1ul << u32Idx))
        {
            port->MODE = (port->MODE & ~(0x3ul << (u32Idx << 1))) | (u32Mode << (u32Idx << 1));
        }
    }
}

/**
 * @brief       Enable GPIO interrupt
 * @param[in]   port        GPIO port. It could be PB, PC, PE, or PF.
 * @param[in]   u32Pin      The pin of specified GPIO port.
 *                          It could be 0 ~ 5, 7 ~ 9, 11 ~ 15 for PB.
 *                          It could be 14 for PC.
 *                          It could be 15 for PE.
 *                          It could be 0 ~ 1 for PF.
 * @param[in]   u32IntAttribs   The interrupt attribute of specified GPIO pin. It could be
 *                          - \ref GPIO_INT_RISING
 *                          - \ref GPIO_INT_FALLING
 *                          - \ref GPIO_INT_BOTH_EDGE
 *                          - \ref GPIO_INT_HIGH
 *                          - \ref GPIO_INT_LOW
 * @return      None
 * @details     This function is used to enable specified GPIO pin interrupt.
 */
void GPIO_EnableInt(GPIO_T *port, uint32_t u32Pin, uint32_t u32IntAttribs)
{
    /* Configure interrupt mode of specified pin */
    port->INTTYPE = (port->INTTYPE & ~(1ul << u32Pin)) | (((u32IntAttribs >> 24) & 0xFFUL) << u32Pin);

    /* Enable interrupt function of specified pin */
    port->INTEN = (port->INTEN & ~(0x00010001ul << u32Pin)) | ((u32IntAttribs & 0xFFFFFFUL) << u32Pin);
}

/**
 * @brief       Disable GPIO interrupt
 * @param[in]   port        GPIO port. It could be PB, PC, PE, or PF.
 * @param[in]   u32Pin      The pin of specified GPIO port.
 *                          It could be 0 ~ 5, 7 ~ 9, 11 ~ 15 for PB.
 *                          It could be 14 for PC.
 *                          It could be 15 for PE.
 *                          It could be 0 ~ 1 for PF.
 * @return      None
 * @details     This function is used to enable specified GPIO pin interrupt.
 */
void GPIO_DisableInt(GPIO_T *port, uint32_t u32Pin)
{
    /* Configure interrupt mode of specified pin */
    port->INTTYPE &= ~(1UL << u32Pin);

    /* Disable interrupt function of specified pin */
    port->INTEN &= ~((0x00010001UL) << u32Pin);
}

/**
 * @brief       Set GPIO slew rate control
 * @param[in]   port        GPIO port. It could be PB, PC, or PF.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *                          It could be BIT0 ~ BIT5, BIT7 ~ BIT9, BIT11 ~ BIT15 for PB.
 *                          It could be BIT14 for PC.
 *                          It could be BIT0 ~ BIT1 for PF.
 * @param[in]   u32Mode     Slew rate mode. It could be
 *                          - \ref GPIO_SLEWCTL_NORMAL
 *                          - \ref GPIO_SLEWCTL_HIGH
 *                          - \ref GPIO_SLEWCTL_FAST
 * @return      None
 * @details     This function is used to set specified GPIO operation mode.
 */
void GPIO_SetSlewCtl(GPIO_T *port, uint32_t u32PinMask, uint32_t u32Mode)
{
    uint32_t u32Idx;

    for(u32Idx = 0ul; u32Idx < GPIO_PIN_MAX; u32Idx++)
    {
        if(u32PinMask & (1ul << u32Idx))
        {
            port->SLEWCTL = (port->SLEWCTL & ~(0x3ul << (u32Idx << 1))) | (u32Mode << (u32Idx << 1));
        }
    }
}

/**
 * @brief       Set GPIO Pull-up and Pull-down control
 * @param[in]   port        GPIO port. It could be PB, PC, PE, or PF.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *                          It could be BIT0 ~ BIT5, BIT7 ~ BIT9, BIT11 ~ BIT15 for PB.
 *                          It could be BIT14 for PC.
 *                          It could be BIT15 for PE.
 *                          It could be BIT0 ~ BIT1 for PF.
 * @param[in]   u32Mode     The pin mode of specified GPIO pin. It could be
 *                          - \ref GPIO_PUSEL_DISABLE
 *                          - \ref GPIO_PUSEL_PULL_UP
 *                          - \ref GPIO_PUSEL_PULL_DOWN
 * @return      None
 * @details     Set the pin mode of specified GPIO pin.
 */
void GPIO_SetPullCtl(GPIO_T *port, uint32_t u32PinMask, uint32_t u32Mode)
{
    uint32_t u32Idx;

    for(u32Idx = 0ul; u32Idx < GPIO_PIN_MAX; u32Idx++)
    {
        if(u32PinMask & (1ul << u32Idx))
        {
            port->PUSEL = (port->PUSEL & ~(0x3ul << (u32Idx << 1))) | (u32Mode << (u32Idx << 1));
        }
    }
}

/**@}*/ /* end of group GPIO_EXPORTED_FUNCTIONS */

/**@}*/ /* end of group GPIO_Driver */

/**@}*/ /* end of group Standard_Driver */
