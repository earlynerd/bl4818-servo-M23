/**************************************************************************//**
 * @file     gpio.h
 * @version  V0.10
 * @brief    General Purpose I/O (GPIO) driver header file
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup GPIO_Driver GPIO Driver
  @{
*/

/** @addtogroup GPIO_EXPORTED_CONSTANTS GPIO Exported Constants
  @{
*/

#define GPIO_PIN_MAX            16UL /*!< Specify Maximum Pins of Each GPIO Port */


/*---------------------------------------------------------------------------------------------------------*/
/*  GPIO_MODE Constant Definitions                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#define GPIO_MODE_INPUT         0x0UL /*!< Input Mode */
#define GPIO_MODE_OUTPUT        0x1UL /*!< Output Mode */
#define GPIO_MODE_OPEN_DRAIN    0x2UL /*!< Open-Drain Mode */
#define GPIO_MODE_QUASI         0x3UL /*!< Quasi-bidirectional Mode */


/*---------------------------------------------------------------------------------------------------------*/
/*  GPIO Interrupt Type Constant Definitions                                                               */
/*---------------------------------------------------------------------------------------------------------*/
#define GPIO_INT_RISING         0x00010000UL /*!< Interrupt enable by Input Rising Edge */
#define GPIO_INT_FALLING        0x00000001UL /*!< Interrupt enable by Input Falling Edge */
#define GPIO_INT_BOTH_EDGE      0x00010001UL /*!< Interrupt enable by both Rising Edge and Falling Edge */
#define GPIO_INT_HIGH           0x01010000UL /*!< Interrupt enable by Level-High */
#define GPIO_INT_LOW            0x01000001UL /*!< Interrupt enable by Level-Level */


/*---------------------------------------------------------------------------------------------------------*/
/*  GPIO_INTTYPE Constant Definitions                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
#define GPIO_INTTYPE_EDGE           0UL /*!< GPIO_INTTYPE Setting for Edge Trigger Mode */
#define GPIO_INTTYPE_LEVEL          1UL /*!< GPIO_INTTYPE Setting for Edge Level Mode */


/*---------------------------------------------------------------------------------------------------------*/
/*  GPIO Slew Rate Type Constant Definitions                                                               */
/*---------------------------------------------------------------------------------------------------------*/
#define GPIO_SLEWCTL_NORMAL         0x0UL           /*!< GPIO slew setting for normal Mode */
#define GPIO_SLEWCTL_HIGH           0x1UL           /*!< GPIO slew setting for high Mode */
#define GPIO_SLEWCTL_FAST           0x2UL           /*!< GPIO slew setting for fast Mode */


/*---------------------------------------------------------------------------------------------------------*/
/*  GPIO Pull-up And Pull-down Type Constant Definitions                                                   */
/*---------------------------------------------------------------------------------------------------------*/
#define GPIO_PUSEL_DISABLE          0x0UL           /*!< GPIO PUSEL setting for Disable Mode */
#define GPIO_PUSEL_PULL_UP          0x1UL           /*!< GPIO PUSEL setting for Pull-up Mode */
#define GPIO_PUSEL_PULL_DOWN        0x2UL           /*!< GPIO PUSEL setting for Pull-down Mode */


/*---------------------------------------------------------------------------------------------------------*/
/*  GPIO_DBCTL Constant Definitions                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define GPIO_DBCTL_ICLK_OFF         (0x0UL<<GPIO_DBCTL_ICLKON_Pos)    /*!< GPIO_DBCTL setting for edge detection circuit is active only if IO pin corresponding GPIOx_IEN bit is set to 1 \hideinitializer */
#define GPIO_DBCTL_ICLK_ON          (0x1UL<<GPIO_DBCTL_ICLKON_Pos)    /*!< GPIO_DBCTL setting for all IO pins edge detection circuit is always active after reset     \hideinitializer */

#define GPIO_DBCTL_DBCLKSRC_HCLK    (0x0UL<<GPIO_DBCTL_DBCLKSRC_Pos)  /*!< GPIO_DBCTL setting for de-bounce counter clock source is the HCLK  \hideinitializer */
#define GPIO_DBCTL_DBCLKSRC_LIRC    (0x1UL<<GPIO_DBCTL_DBCLKSRC_Pos)  /*!< GPIO_DBCTL setting for de-bounce counter clock source is the LIRC  \hideinitializer */

#define GPIO_DBCTL_DBCLKSEL_1       (0x0UL<<GPIO_DBCTL_DBCLKSEL_Pos)  /*!< GPIO_DBCTL setting for sampling cycle = 1 clocks   \hideinitializer */
#define GPIO_DBCTL_DBCLKSEL_2       (0x1UL<<GPIO_DBCTL_DBCLKSEL_Pos)  /*!< GPIO_DBCTL setting for sampling cycle = 2 clocks   \hideinitializer */
#define GPIO_DBCTL_DBCLKSEL_4       (0x2UL<<GPIO_DBCTL_DBCLKSEL_Pos)  /*!< GPIO_DBCTL setting for sampling cycle = 4 clocks   \hideinitializer */
#define GPIO_DBCTL_DBCLKSEL_8       (0x3UL<<GPIO_DBCTL_DBCLKSEL_Pos)  /*!< GPIO_DBCTL setting for sampling cycle = 8 clocks   \hideinitializer */
#define GPIO_DBCTL_DBCLKSEL_16      (0x4UL<<GPIO_DBCTL_DBCLKSEL_Pos)  /*!< GPIO_DBCTL setting for sampling cycle = 16 clocks  \hideinitializer */
#define GPIO_DBCTL_DBCLKSEL_32      (0x5UL<<GPIO_DBCTL_DBCLKSEL_Pos)  /*!< GPIO_DBCTL setting for sampling cycle = 32 clocks  \hideinitializer */
#define GPIO_DBCTL_DBCLKSEL_64      (0x6UL<<GPIO_DBCTL_DBCLKSEL_Pos)  /*!< GPIO_DBCTL setting for sampling cycle = 64 clocks  \hideinitializer */
#define GPIO_DBCTL_DBCLKSEL_128     (0x7UL<<GPIO_DBCTL_DBCLKSEL_Pos)  /*!< GPIO_DBCTL setting for sampling cycle = 128 clocks \hideinitializer */
#define GPIO_DBCTL_DBCLKSEL_256     (0x8UL<<GPIO_DBCTL_DBCLKSEL_Pos)  /*!< GPIO_DBCTL setting for sampling cycle = 256 clocks \hideinitializer */
#define GPIO_DBCTL_DBCLKSEL_512     (0x9UL<<GPIO_DBCTL_DBCLKSEL_Pos)  /*!< GPIO_DBCTL setting for sampling cycle = 512 clocks \hideinitializer */
#define GPIO_DBCTL_DBCLKSEL_1024    (0xAUL<<GPIO_DBCTL_DBCLKSEL_Pos)  /*!< GPIO_DBCTL setting for sampling cycle = 1024 clocks    \hideinitializer */
#define GPIO_DBCTL_DBCLKSEL_2048    (0xBUL<<GPIO_DBCTL_DBCLKSEL_Pos)  /*!< GPIO_DBCTL setting for sampling cycle = 2048 clocks    \hideinitializer */
#define GPIO_DBCTL_DBCLKSEL_4096    (0xCUL<<GPIO_DBCTL_DBCLKSEL_Pos)  /*!< GPIO_DBCTL setting for sampling cycle = 4096 clocks    \hideinitializer */
#define GPIO_DBCTL_DBCLKSEL_8192    (0xDUL<<GPIO_DBCTL_DBCLKSEL_Pos)  /*!< GPIO_DBCTL setting for sampling cycle = 8192 clocks    \hideinitializer */
#define GPIO_DBCTL_DBCLKSEL_16384   (0xEUL<<GPIO_DBCTL_DBCLKSEL_Pos)  /*!< GPIO_DBCTL setting for sampling cycle = 16384 clocks   \hideinitializer */
#define GPIO_DBCTL_DBCLKSEL_32768   (0xFUL<<GPIO_DBCTL_DBCLKSEL_Pos)  /*!< GPIO_DBCTL setting for sampling cycle = 32768 clocks   \hideinitializer */


/** Define GPIO Pin Data Input/Output. It could be used to control each I/O pin by pin address mapping.
 *  Example 1:
 *
 *      PA0 = 1;
 *
 *  It is used to set PA.0 to high;
 *
 *  Example 2:
 *
 *      if (PA0)
 *          PA0 = 0;
 *
 *  If PA.0 pin status is high, then set PA.0 data output to low.
 */
#define GPIO_PIN_DATA(port, pin)    (*((volatile uint32_t *)((GPIO_PIN_DATA_BASE+(0x40*(port))) + ((pin)<<2)))) /*!< Specify GPIO Pin Data Input/Output \hideinitializer */
#define PB0             GPIO_PIN_DATA(1, 0 ) /*!< Specify PB.0  Pin Data Input/Output \hideinitializer */
#define PB1             GPIO_PIN_DATA(1, 1 ) /*!< Specify PB.1  Pin Data Input/Output \hideinitializer */
#define PB2             GPIO_PIN_DATA(1, 2 ) /*!< Specify PB.2  Pin Data Input/Output \hideinitializer */
#define PB3             GPIO_PIN_DATA(1, 3 ) /*!< Specify PB.3  Pin Data Input/Output \hideinitializer */
#define PB4             GPIO_PIN_DATA(1, 4 ) /*!< Specify PB.4  Pin Data Input/Output \hideinitializer */
#define PB5             GPIO_PIN_DATA(1, 5 ) /*!< Specify PB.5  Pin Data Input/Output \hideinitializer */
#define PB7             GPIO_PIN_DATA(1, 7 ) /*!< Specify PB.7  Pin Data Input/Output \hideinitializer */
#define PB8             GPIO_PIN_DATA(1, 8 ) /*!< Specify PB.8  Pin Data Input/Output \hideinitializer */
#define PB9             GPIO_PIN_DATA(1, 9 ) /*!< Specify PB.9  Pin Data Input/Output \hideinitializer */
#define PB11            GPIO_PIN_DATA(1, 11) /*!< Specify PB.11 Pin Data Input/Output \hideinitializer */
#define PB12            GPIO_PIN_DATA(1, 12) /*!< Specify PB.12 Pin Data Input/Output \hideinitializer */
#define PB13            GPIO_PIN_DATA(1, 13) /*!< Specify PB.13 Pin Data Input/Output \hideinitializer */
#define PB14            GPIO_PIN_DATA(1, 14) /*!< Specify PB.14 Pin Data Input/Output \hideinitializer */
#define PB15            GPIO_PIN_DATA(1, 15) /*!< Specify PB.15 Pin Data Input/Output \hideinitializer */

#define PC14            GPIO_PIN_DATA(2, 14) /*!< Specify PC.14 Pin Data Input/Output \hideinitializer */

#define PE15            GPIO_PIN_DATA(4, 15) /*!< Specify PE.15 Pin Data Input/Output \hideinitializer */

#define PF0             GPIO_PIN_DATA(5, 0 ) /*!< Specify PF.0  Pin Data Input/Output \hideinitializer */
#define PF1             GPIO_PIN_DATA(5, 1 ) /*!< Specify PF.1  Pin Data Input/Output \hideinitializer */


/**@}*/ /* end of group GPIO_EXPORTED_CONSTANTS */


/** @addtogroup GPIO_EXPORTED_FUNCTIONS GPIO Exported Functions
  @{
*/

/**
 * @brief       Clear GPIO Pin Interrupt Flag
 * @param[in]   port        GPIO port. It could be PB, PC, PE, or PF.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *                          It could be BIT0 ~ BIT5, BIT7 ~ BIT9, BIT11 ~ BIT15 for PB.
 *                          It could be BIT14 for PC.
 *                          It could be BIT15 for PE.
 *                          It could be BIT0 ~ BIT1 for PF.
 * @return      None
 * @details     Clear the interrupt status of specified GPIO pin.
 */
#define GPIO_CLR_INT_FLAG(port, u32PinMask)         ((port)->INTSRC = (u32PinMask))

/**
 * @brief       Disable Pin De-bounce Function
 * @param[in]   port        GPIO port. It could be PB, PC, PE, or PF.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *                          It could be BIT0 ~ BIT5, BIT7 ~ BIT9, BIT11 ~ BIT15 for PB.
 *                          It could be BIT14 for PC.
 *                          It could be BIT15 for PE.
 *                          It could be BIT0 ~ BIT1 for PF.
 * @return      None
 * @details     Disable the interrupt de-bounce function of specified GPIO pin.
 */
#define GPIO_DISABLE_DEBOUNCE(port, u32PinMask)     ((port)->DBEN &= ~(u32PinMask))

/**
 * @brief       Enable Pin De-bounce Function
 * @param[in]   port        GPIO port. It could be PB, PC, PE, or PF.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *                          It could be BIT0 ~ BIT5, BIT7 ~ BIT9, BIT11 ~ BIT15 for PB.
 *                          It could be BIT14 for PC.
 *                          It could be BIT15 for PE.
 *                          It could be BIT0 ~ BIT1 for PF.
 * @return      None
 * @details     Enable the interrupt de-bounce function of specified GPIO pin.
 */
#define GPIO_ENABLE_DEBOUNCE(port, u32PinMask)      ((port)->DBEN |= (u32PinMask))

/**
 * @brief       Disable I/O Digital Input Path
 * @param[in]   port        GPIO port. It could be PB, PC, or PF.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *                          It could be BIT0 ~ BIT5, BIT7 ~ BIT9, BIT11 ~ BIT15 for PB.
 *                          It could be BIT14 for PC.
 *                          It could be BIT0 ~ BIT1 for PF.
 * @return      None
 * @details     Disable I/O digital input path of specified GPIO pin.
 */
#define GPIO_DISABLE_DIGITAL_PATH(port, u32PinMask) ((port)->DINOFF |= ((u32PinMask)<<16))

/**
 * @brief       Enable I/O Digital Input Path
 * @param[in]   port        GPIO port. It could be PB, PC, or PF.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *                          It could be BIT0 ~ BIT5, BIT7 ~ BIT9, BIT11 ~ BIT15 for PB.
 *                          It could be BIT14 for PC.
 *                          It could be BIT0 ~ BIT1 for PF.
 * @return      None
 * @details     Enable I/O digital input path of specified GPIO pin.
 */
#define GPIO_ENABLE_DIGITAL_PATH(port, u32PinMask)  ((port)->DINOFF &= ~((u32PinMask)<<16))

/**
 * @brief       Disable I/O DOUT mask
 * @param[in]   port        GPIO port. It could be PB, PC, or PF.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *                          It could be BIT0 ~ BIT5, BIT7 ~ BIT9, BIT11 ~ BIT15 for PB.
 *                          It could be BIT14 for PC.
 *                          It could be BIT0 ~ BIT1 for PF.
 * @return      None
 * @details     Disable I/O DOUT mask of specified GPIO pin.
 */
#define GPIO_DISABLE_DOUT_MASK(port, u32PinMask)    ((port)->DATMSK &= ~(u32PinMask))

/**
 * @brief       Enable I/O DOUT mask
 * @param[in]   port        GPIO port. It could be PB, PC, or PF.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *                          It could be BIT0 ~ BIT5, BIT7 ~ BIT9, BIT11 ~ BIT15 for PB.
 *                          It could be BIT14 for PC.
 *                          It could be BIT0 ~ BIT1 for PF.
 * @return      None
 * @details     Enable I/O DOUT mask of specified GPIO pin.
 */
#define GPIO_ENABLE_DOUT_MASK(port, u32PinMask) ((port)->DATMSK |= (u32PinMask))

/**
 * @brief       Get GPIO Pin Interrupt Flag
 * @param[in]   port        GPIO port. It could be PB, PC, PE, or PF.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *                          It could be BIT0 ~ BIT5, BIT7 ~ BIT9, BIT11 ~ BIT15 for PB.
 *                          It could be BIT14 for PC.
 *                          It could be BIT15 for PE.
 *                          It could be BIT0 ~ BIT1 for PF.
 * @retval      0           No interrupt at specified GPIO pin
 * @retval      1           The specified GPIO pin generate an interrupt
 * @details     Get the interrupt status of specified GPIO pin.
 */
#define GPIO_GET_INT_FLAG(port, u32PinMask)     ((port)->INTSRC & (u32PinMask))

/**
 * @brief       Set De-bounce Sampling Cycle Time
 * @param[in]   port        GPIO port. It could be PB, PC, PE, or PF.
 * @param[in]   u32ClkSrc   The de-bounce counter clock source. It could be GPIO_DBCTL_DBCLKSRC_HCLK or GPIO_DBCTL_DBCLKSRC_LIRC.
 * @param[in]   u32ClkSel   The de-bounce sampling cycle selection. It could be
 *                            - \ref GPIO_DBCTL_DBCLKSEL_1
 *                            - \ref GPIO_DBCTL_DBCLKSEL_2
 *                            - \ref GPIO_DBCTL_DBCLKSEL_4
 *                            - \ref GPIO_DBCTL_DBCLKSEL_8
 *                            - \ref GPIO_DBCTL_DBCLKSEL_16
 *                            - \ref GPIO_DBCTL_DBCLKSEL_32
 *                            - \ref GPIO_DBCTL_DBCLKSEL_64
 *                            - \ref GPIO_DBCTL_DBCLKSEL_128
 *                            - \ref GPIO_DBCTL_DBCLKSEL_256
 *                            - \ref GPIO_DBCTL_DBCLKSEL_512
 *                            - \ref GPIO_DBCTL_DBCLKSEL_1024
 *                            - \ref GPIO_DBCTL_DBCLKSEL_2048
 *                            - \ref GPIO_DBCTL_DBCLKSEL_4096
 *                            - \ref GPIO_DBCTL_DBCLKSEL_8192
 *                            - \ref GPIO_DBCTL_DBCLKSEL_16384
 *                            - \ref GPIO_DBCTL_DBCLKSEL_32768
 * @return      None
 * @details     Set the interrupt de-bounce sampling cycle time based on the debounce counter clock source. \n
 *              Example: _GPIO_SET_DEBOUNCE_TIME(PA, GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_4). \n
 *              It's meaning the De-debounce counter clock source is internal 32 KHz and sampling cycle selection is 4. \n
 *              Then the target de-bounce sampling cycle time is (4)*(1/(32*1000)) s = 125 us,
 *              and system will sampling interrupt input once per 125 us.
 */
#define GPIO_SET_DEBOUNCE_TIME(port, u32ClkSrc, u32ClkSel)    ((port)->DBCTL = (GPIO_DBCTL_ICLKON_Msk | (u32ClkSrc) | (u32ClkSel)))

/**
  * @brief       Set GPIO Interrupt Clock on bit
  * @param[in]   port        GPIO port. It could be PB, PC, PE, or PF.
  * @return      None
  * @details     Set the I/O pins edge detection circuit always active after reset for specified port.
  * \hideinitializer
  */
#define GPIO_SET_DEBOUNCE_ICLKON(port)  ((port)->DBCTL |= GPIO_DBCTL_ICLKON_Msk)

/**
  * @brief       Clear GPIO Interrupt Clock on bit
  * @param[in]   port        GPIO port. It could be PB, PC, PE, or PF.
  * @return      None
  * @details     Set edge detection circuit active only if I/O pin edge interrupt enabled for specified port
  * \hideinitializer
  */
#define GPIO_CLR_DEBOUNCE_ICLKON(port)  ((port)->DBCTL &= ~(GPIO_DBCTL_ICLKON_Msk))

/**
 * @brief       Get GPIO Port IN Data
 * @param[in]   port        GPIO port. It could be PB, PC, PE, or PF.
 * @return      The specified port data
 * @details     Get the PIN register of specified GPIO port.
 */
#define GPIO_GET_IN_DATA(port)  ((port)->PIN)

/**
 * @brief       Set GPIO Port OUT Data
 * @param[in]   port        GPIO port. It could be PB, PC, or PF.
 * @param[in]   u32Data     GPIO port data.
 * @return      None
 * @details     Set the Data into specified GPIO port.
 */
#define GPIO_SET_OUT_DATA(port, u32Data)    ((port)->DOUT = (u32Data))

/**
 * @brief       Toggle Specified GPIO pin
 * @param[in]   u32Pin      Pxy
 * @return      None
 * @details     Toggle the specified GPIO pint.
 */
#define GPIO_TOGGLE(u32Pin) ((u32Pin) ^= 1)

/**
 * @brief       Enable External GPIO interrupt
 * @param[in]   port        GPIO port. It could be PB, PC, PE, or PF.
 * @param[in]   u32Pin      The pin of specified GPIO port.
 *                          It could be 0 ~ 5, 7 ~ 9, 11 ~ 15 for PB.
 *                          It could be 14 for PC.
 *                          It could be 15 for PE.
 *                          It could be 0 ~ 1 for PF.
 * @param[in]   u32IntAttribs   The interrupt attribute of specified GPIO pin. It could be
 *                              - \ref GPIO_INT_RISING
 *                              - \ref GPIO_INT_FALLING
 *                              - \ref GPIO_INT_BOTH_EDGE
 *                              - \ref GPIO_INT_HIGH
 *                              - \ref GPIO_INT_LOW
 * @return      None
 * @details     This function is used to enable specified GPIO pin interrupt.
 */
#define GPIO_EnableEINT     GPIO_EnableInt

/**
 * @brief       Disable External GPIO interrupt
 * @param[in]   port        GPIO port. It could be PB, PC, PE, or PF.
 * @param[in]   u32Pin      The pin of specified GPIO port.
 *                          It could be 0 ~ 5, 7 ~ 9, 11 ~ 15 for PB.
 *                          It could be 14 for PC.
 *                          It could be 15 for PE.
 *                          It could be 0 ~ 1 for PF.
 * @return      None
 * @details     This function is used to enable specified GPIO pin interrupt.
 */
#define GPIO_DisableEINT    GPIO_DisableInt


void GPIO_SetMode(GPIO_T *port, uint32_t u32PinMask, uint32_t u32Mode);
void GPIO_EnableInt(GPIO_T *port, uint32_t u32Pin, uint32_t u32IntAttribs);
void GPIO_DisableInt(GPIO_T *port, uint32_t u32Pin);
void GPIO_SetSlewCtl(GPIO_T *port, uint32_t u32PinMask, uint32_t u32Mode);
void GPIO_SetPullCtl(GPIO_T *port, uint32_t u32PinMask, uint32_t u32Mode);


/**@}*/ /* end of group GPIO_EXPORTED_FUNCTIONS */

/**@}*/ /* end of group GPIO_Driver */

/**@}*/ /* end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif /* __GPIO_H__ */
