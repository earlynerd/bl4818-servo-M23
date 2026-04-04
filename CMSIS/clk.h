/**************************************************************************//**
 * @file     clk.h
 * @version  V0.10
 * @brief    Clock Controller (CLK) driver header file
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __CLK_H__
#define __CLK_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup CLK_Driver CLK Driver
  @{
*/

/** @addtogroup CLK_EXPORTED_CONSTANTS CLK Exported Constants
  @{
*/

#define FREQ_2MHZ         (2000000UL)
#define FREQ_4MHZ         (4000000UL)
#define FREQ_8MHZ         (8000000UL)
#define FREQ_12MHZ       (12000000UL)
#define FREQ_24MHZ       (24000000UL)
#define FREQ_25MHZ       (25000000UL)
#define FREQ_48MHZ       (48000000UL)
#define FREQ_50MHZ       (50000000UL)
#define FREQ_64MHZ       (64000000UL)
#define FREQ_75MHZ       (75000000UL)
#define FREQ_84MHZ       (84000000UL)
#define FREQ_96MHZ       (96000000UL)
#define FREQ_144MHZ     (144000000UL)
#define FREQ_200MHZ     (200000000UL)


/*---------------------------------------------------------------------------------------------------------*/
/*  CLKSEL0 constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_CLKSEL0_HCLKSEL_LIRC        (0x03UL<<CLK_CLKSEL0_HCLKSEL_Pos)       /*!< Setting HCLK clock source as LIRC */
#define CLK_CLKSEL0_HCLKSEL_HIRC        (0x05UL<<CLK_CLKSEL0_HCLKSEL_Pos)       /*!< Setting HCLK clock source as HIRC */

#define CLK_CLKSEL0_STCLKSEL_HCLK_DIV2  (0x03UL<<CLK_CLKSEL0_STCLKSEL_Pos)      /*!< Setting SysTick clock source as HCLK/2 */
#define CLK_CLKSEL0_STCLKSEL_HIRC_DIV2  (0x04UL<<CLK_CLKSEL0_STCLKSEL_Pos)      /*!< Setting SysTick clock source as HIRC/2 */
#define CLK_CLKSEL0_STCLKSEL_HCLK       (0x01UL<<SysTick_CTRL_CLKSOURCE_Pos)    /*!< Setting SysTick clock source as HCLK */


/*---------------------------------------------------------------------------------------------------------*/
/*  CLKSEL1 constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_CLKSEL1_WDTSEL_HCLK_DIV2048  (0x2UL<<CLK_CLKSEL1_WDTSEL_Pos)  /*!< Setting WDT clock source as HCLK/2048 */
#define CLK_CLKSEL1_WDTSEL_LIRC          (0x3UL<<CLK_CLKSEL1_WDTSEL_Pos)  /*!< Setting WDT clock source as LIRC */

#define CLK_CLKSEL1_TMR0SEL_PCLK0        (0x2UL<<CLK_CLKSEL1_TMR0SEL_Pos) /*!< Setting Timer 0 clock source as PCLK0 */
#define CLK_CLKSEL1_TMR0SEL_EXT_TRG      (0x3UL<<CLK_CLKSEL1_TMR0SEL_Pos) /*!< Setting Timer 0 clock source as external trigger */
#define CLK_CLKSEL1_TMR0SEL_LIRC         (0x4UL<<CLK_CLKSEL1_TMR0SEL_Pos) /*!< Setting Timer 0 clock source as LIRC */
#define CLK_CLKSEL1_TMR0SEL_HIRC         (0x5UL<<CLK_CLKSEL1_TMR0SEL_Pos) /*!< Setting Timer 0 clock source as HIRC */

#define CLK_CLKSEL1_TMR1SEL_PCLK0        (0x2UL<<CLK_CLKSEL1_TMR1SEL_Pos) /*!< Setting Timer 1 clock source as PCLK0 */
#define CLK_CLKSEL1_TMR1SEL_EXT_TRG      (0x3UL<<CLK_CLKSEL1_TMR1SEL_Pos) /*!< Setting Timer 1 clock source as external trigger */
#define CLK_CLKSEL1_TMR1SEL_LIRC         (0x4UL<<CLK_CLKSEL1_TMR1SEL_Pos) /*!< Setting Timer 1 clock source as LIRC */
#define CLK_CLKSEL1_TMR1SEL_HIRC         (0x5UL<<CLK_CLKSEL1_TMR1SEL_Pos) /*!< Setting Timer 1 clock source as HIRC */

#define CLK_CLKSEL1_TMR2SEL_PCLK1        (0x2UL<<CLK_CLKSEL1_TMR2SEL_Pos) /*!< Setting Timer 2 clock source as PCLK1 */
#define CLK_CLKSEL1_TMR2SEL_EXT_TRG      (0x3UL<<CLK_CLKSEL1_TMR2SEL_Pos) /*!< Setting Timer 2 clock source as external trigger */
#define CLK_CLKSEL1_TMR2SEL_LIRC         (0x4UL<<CLK_CLKSEL1_TMR2SEL_Pos) /*!< Setting Timer 2 clock source as LIRC */
#define CLK_CLKSEL1_TMR2SEL_HIRC         (0x5UL<<CLK_CLKSEL1_TMR2SEL_Pos) /*!< Setting Timer 2 clock source as HIRC */

#define CLK_CLKSEL1_TMR3SEL_PCLK1        (0x2UL<<CLK_CLKSEL1_TMR3SEL_Pos) /*!< Setting Timer 3 clock source as PCLK1 */
#define CLK_CLKSEL1_TMR3SEL_EXT_TRG      (0x3UL<<CLK_CLKSEL1_TMR3SEL_Pos) /*!< Setting Timer 3 clock source as external trigger */
#define CLK_CLKSEL1_TMR3SEL_LIRC         (0x4UL<<CLK_CLKSEL1_TMR3SEL_Pos) /*!< Setting Timer 3 clock source as LIRC */
#define CLK_CLKSEL1_TMR3SEL_HIRC         (0x5UL<<CLK_CLKSEL1_TMR3SEL_Pos) /*!< Setting Timer 3 clock source as HIRC */

#define CLK_CLKSEL1_CLKOSEL_HCLK         (0x2UL<<CLK_CLKSEL1_CLKOSEL_Pos) /*!< Setting CLKO clock source as HCLK */
#define CLK_CLKSEL1_CLKOSEL_HIRC         (0x3UL<<CLK_CLKSEL1_CLKOSEL_Pos) /*!< Setting CLKO clock source as HIRC */

#define CLK_CLKSEL1_WWDTSEL_HCLK_DIV2048 (0x2UL<<CLK_CLKSEL1_WWDTSEL_Pos) /*!< Setting WWDT clock source as HCLK/2048 */
#define CLK_CLKSEL1_WWDTSEL_LIRC         (0x3UL<<CLK_CLKSEL1_WWDTSEL_Pos) /*!< Setting WWDT clock source as LIRC */


/*---------------------------------------------------------------------------------------------------------*/
/*  CLKSEL2 constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_CLKSEL2_PWM0SEL_PCLK0        (0x1UL<<CLK_CLKSEL2_PWM0SEL_Pos)  /*!< Setting PWM0 clock source as PCLK0 */

#define CLK_CLKSEL2_UART0SEL_HIRC        (0x3UL<<CLK_CLKSEL2_UART0SEL_Pos) /*!< Setting UART0 clock source as HIRC */
#define CLK_CLKSEL2_UART0SEL_PCLK0       (0x4UL<<CLK_CLKSEL2_UART0SEL_Pos) /*!< Setting UART0 clock source as PCLK0 */

#define CLK_CLKSEL2_UART1SEL_HIRC        (0x3UL<<CLK_CLKSEL2_UART1SEL_Pos) /*!< Setting UART1 clock source as HIRC */
#define CLK_CLKSEL2_UART1SEL_PCLK1       (0x4UL<<CLK_CLKSEL2_UART1SEL_Pos) /*!< Setting UART1 clock source as PCLK1 */

#define CLK_CLKSEL2_ADCSEL_PCLK2         (0x2UL<<CLK_CLKSEL2_ADCSEL_Pos)   /*!< Setting ADC clock source as PCLK2 */
#define CLK_CLKSEL2_ADCSEL_HIRC          (0x3UL<<CLK_CLKSEL2_ADCSEL_Pos)   /*!< Setting ADC clock source as HIRC */


/*---------------------------------------------------------------------------------------------------------*/
/*  CLKDIV0 constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_CLKDIV0_HCLK(x)     (((x)-1UL) << CLK_CLKDIV0_HCLKDIV_Pos)  /*!< CLKDIV0 Setting for HCLK clock divider. It could be 1~16 */
#define CLK_CLKDIV0_UART0(x)    (((x)-1UL) << CLK_CLKDIV0_UART0DIV_Pos) /*!< CLKDIV0 Setting for UART0 clock divider. It could be 1~16 */
#define CLK_CLKDIV0_UART1(x)    (((x)-1UL) << CLK_CLKDIV0_UART1DIV_Pos) /*!< CLKDIV0 Setting for UART1 clock divider. It could be 1~16 */
#define CLK_CLKDIV0_ADC(x)      (((x)-1UL) << CLK_CLKDIV0_ADCDIV_Pos)   /*!< CLKDIV0 Setting for ADC clock divider. It could be 1~256 */


/*---------------------------------------------------------------------------------------------------------*/
/*  PCLKDIV constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_PCLKDIV_APB0DIV_DIV1       (0x0UL<<CLK_PCLKDIV_APB0DIV_Pos)     /*!< PCLKDIV Setting for APB0 clock divider  1. \hideinitializer */
#define CLK_PCLKDIV_APB0DIV_DIV2       (0x1UL<<CLK_PCLKDIV_APB0DIV_Pos)     /*!< PCLKDIV Setting for APB0 clock divider  2. \hideinitializer */
#define CLK_PCLKDIV_APB0DIV_DIV4       (0x2UL<<CLK_PCLKDIV_APB0DIV_Pos)     /*!< PCLKDIV Setting for APB0 clock divider  4. \hideinitializer */
#define CLK_PCLKDIV_APB0DIV_DIV8       (0x3UL<<CLK_PCLKDIV_APB0DIV_Pos)     /*!< PCLKDIV Setting for APB0 clock divider  8. \hideinitializer */
#define CLK_PCLKDIV_APB0DIV_DIV16      (0x4UL<<CLK_PCLKDIV_APB0DIV_Pos)     /*!< PCLKDIV Setting for APB0 clock divider 16. \hideinitializer */
#define CLK_PCLKDIV_APB0DIV_DIV32      (0x5UL<<CLK_PCLKDIV_APB0DIV_Pos)     /*!< PCLKDIV Setting for APB0 clock divider 32. \hideinitializer */

#define CLK_PCLKDIV_APB1DIV_DIV1       (0x0UL<<CLK_PCLKDIV_APB1DIV_Pos)     /*!< PCLKDIV Setting for APB1 clock divider  1. \hideinitializer */
#define CLK_PCLKDIV_APB1DIV_DIV2       (0x1UL<<CLK_PCLKDIV_APB1DIV_Pos)     /*!< PCLKDIV Setting for APB1 clock divider  2. \hideinitializer */
#define CLK_PCLKDIV_APB1DIV_DIV4       (0x2UL<<CLK_PCLKDIV_APB1DIV_Pos)     /*!< PCLKDIV Setting for APB1 clock divider  4. \hideinitializer */
#define CLK_PCLKDIV_APB1DIV_DIV8       (0x3UL<<CLK_PCLKDIV_APB1DIV_Pos)     /*!< PCLKDIV Setting for APB1 clock divider  8. \hideinitializer */
#define CLK_PCLKDIV_APB1DIV_DIV16      (0x4UL<<CLK_PCLKDIV_APB1DIV_Pos)     /*!< PCLKDIV Setting for APB1 clock divider 16. \hideinitializer */
#define CLK_PCLKDIV_APB1DIV_DIV32      (0x5UL<<CLK_PCLKDIV_APB1DIV_Pos)     /*!< PCLKDIV Setting for APB1 clock divider 32. \hideinitializer */

#define CLK_PCLKDIV_APB2DIV_DIV1       (0x0UL<<CLK_PCLKDIV_APB2DIV_Pos)     /*!< PCLKDIV Setting for APB1 clock divider  1. \hideinitializer */
#define CLK_PCLKDIV_APB2DIV_DIV2       (0x1UL<<CLK_PCLKDIV_APB2DIV_Pos)     /*!< PCLKDIV Setting for APB1 clock divider  2. \hideinitializer */
#define CLK_PCLKDIV_APB2DIV_DIV4       (0x2UL<<CLK_PCLKDIV_APB2DIV_Pos)     /*!< PCLKDIV Setting for APB1 clock divider  4. \hideinitializer */
#define CLK_PCLKDIV_APB2DIV_DIV8       (0x3UL<<CLK_PCLKDIV_APB2DIV_Pos)     /*!< PCLKDIV Setting for APB1 clock divider  8. \hideinitializer */
#define CLK_PCLKDIV_APB2DIV_DIV16      (0x4UL<<CLK_PCLKDIV_APB2DIV_Pos)     /*!< PCLKDIV Setting for APB1 clock divider 16. \hideinitializer */
#define CLK_PCLKDIV_APB2DIV_DIV32      (0x5UL<<CLK_PCLKDIV_APB2DIV_Pos)     /*!< PCLKDIV Setting for APB1 clock divider 32. \hideinitializer */


/*---------------------------------------------------------------------------------------------------------*/
/*  MODULE constant definitions.                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
/* APBCLK(31:30)|CLKSEL(29:28)|CLKSEL_Msk(27:25) |CLKSEL_Pos(24:20)|CLKDIV(19:18)|CLKDIV_Msk(17:10)|CLKDIV_Pos(9:5)|IP_EN_Pos(4:0) */

#define MODULE_APBCLK(x)        (((x) >>30) & 0x3UL)    /*!< Calculate AHBCLK/APBCLK offset on MODULE index, 0x0:AHBCLK, 0x1:APBCLK0, 0x2:APBCLK1 */
#define MODULE_CLKSEL(x)        (((x) >>28) & 0x3UL)    /*!< Calculate CLKSEL offset on MODULE index, 0x0:CLKSEL0, 0x1:CLKSEL1, 0x2:CLKSEL2, 0x3:CLKSEL3 */
#define MODULE_CLKSEL_Msk(x)    (((x) >>25) & 0x7UL)    /*!< Calculate CLKSEL mask offset on MODULE index */
#define MODULE_CLKSEL_Pos(x)    (((x) >>20) & 0x1fUL)   /*!< Calculate CLKSEL position offset on MODULE index */
#define MODULE_CLKDIV(x)        (((x) >>18) & 0x3UL)    /*!< Calculate APBCLK CLKDIV on MODULE index, 0x0:CLKDIV0, 0x1:CLKDIV1, 0x4:CLKDIV4 */
#define MODULE_CLKDIV_Msk(x)    (((x) >>10) & 0xffUL)   /*!< Calculate CLKDIV mask offset on MODULE index */
#define MODULE_CLKDIV_Pos(x)    (((x) >>5 ) & 0x1fUL)   /*!< Calculate CLKDIV position offset on MODULE index */
#define MODULE_IP_EN_Pos(x)     (((x) >>0 ) & 0x1fUL)   /*!< Calculate APBCLK offset on MODULE index */
#define MODULE_NoMsk            0x0UL                   /*!< Not mask on MODULE index */
#define NA                      MODULE_NoMsk            /*!< Not Available */

#define MODULE_APBCLK_ENC(x)        (((x) & 0x03UL) << 30)   /*!< MODULE index, 0x0:AHBCLK, 0x1:APBCLK0, 0x2:APBCLK1 */
#define MODULE_CLKSEL_ENC(x)        (((x) & 0x03UL) << 28)   /*!< CLKSEL offset on MODULE index, 0x0:CLKSEL0, 0x1:CLKSEL1, 0x2:CLKSEL2, 0x3:CLKSEL3 */
#define MODULE_CLKSEL_Msk_ENC(x)    (((x) & 0x07UL) << 25)   /*!< CLKSEL mask offset on MODULE index */
#define MODULE_CLKSEL_Pos_ENC(x)    (((x) & 0x1fUL) << 20)   /*!< CLKSEL position offset on MODULE index */
#define MODULE_CLKDIV_ENC(x)        (((x) & 0x03UL) << 18)   /*!< APBCLK CLKDIV on MODULE index, 0x0:CLKDIV, 0x1:CLKDIV1, 0x4:CLKDIV4 */
#define MODULE_CLKDIV_Msk_ENC(x)    (((x) & 0xffUL) << 10)   /*!< CLKDIV mask offset on MODULE index */
#define MODULE_CLKDIV_Pos_ENC(x)    (((x) & 0x1fUL) <<  5)   /*!< CLKDIV position offset on MODULE index */
#define MODULE_IP_EN_Pos_ENC(x)     (((x) & 0x1fUL) <<  0)   /*!< AHBCLK/APBCLK offset on MODULE index */


/* AHBCLK */
#define FMCIDLE_MODULE (MODULE_APBCLK_ENC( 0UL)|MODULE_IP_EN_Pos_ENC((uint32_t)CLK_AHBCLK_FMCIDLE_Pos)|\
                        MODULE_CLKSEL_ENC(  NA)|MODULE_CLKSEL_Msk_ENC(NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                        MODULE_CLKDIV_ENC(  NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))    /*!< FMCIDLE Module */

#define ISP_MODULE     (MODULE_APBCLK_ENC( 0UL)|MODULE_IP_EN_Pos_ENC((uint32_t)CLK_AHBCLK_ISPCKEN_Pos)|\
                        MODULE_CLKSEL_ENC(  NA)|MODULE_CLKSEL_Msk_ENC(NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                        MODULE_CLKDIV_ENC(  NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))    /*!< ISP Module */

#define ST_MODULE      (MODULE_APBCLK_ENC( 0UL)|MODULE_IP_EN_Pos_ENC((uint32_t)CLK_AHBCLK_STCKEN_Pos)|\
                        MODULE_CLKSEL_ENC( 0UL)|MODULE_CLKSEL_Msk_ENC(7UL)|MODULE_CLKSEL_Pos_ENC(3UL)|\
                        MODULE_CLKDIV_ENC( NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))     /*!< SysTick Module */

#define SRAM0_MODULE   (MODULE_APBCLK_ENC( 0UL)|MODULE_IP_EN_Pos_ENC((uint32_t)CLK_AHBCLK_SRAM0CKEN_Pos)|\
                        MODULE_CLKSEL_ENC(  NA)|MODULE_CLKSEL_Msk_ENC(NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                        MODULE_CLKDIV_ENC(  NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))    /*!< SRAM0 Module */

#define GPB_MODULE     (MODULE_APBCLK_ENC( 0UL)|MODULE_IP_EN_Pos_ENC((uint32_t)CLK_AHBCLK_GPBCKEN_Pos)|\
                        MODULE_CLKSEL_ENC(  NA)|MODULE_CLKSEL_Msk_ENC(NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                        MODULE_CLKDIV_ENC(  NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))    /*!< GPB Module */

#define GPC_MODULE     (MODULE_APBCLK_ENC( 0UL)|MODULE_IP_EN_Pos_ENC((uint32_t)CLK_AHBCLK_GPCCKEN_Pos)|\
                        MODULE_CLKSEL_ENC(  NA)|MODULE_CLKSEL_Msk_ENC(NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                        MODULE_CLKDIV_ENC(  NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))    /*!< GPC Module */

#define GPE_MODULE     (MODULE_APBCLK_ENC( 0UL)|MODULE_IP_EN_Pos_ENC((uint32_t)CLK_AHBCLK_GPECKEN_Pos)|\
                        MODULE_CLKSEL_ENC(  NA)|MODULE_CLKSEL_Msk_ENC(NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                        MODULE_CLKDIV_ENC(  NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))    /*!< GPE Module */

#define GPF_MODULE     (MODULE_APBCLK_ENC( 0UL)|MODULE_IP_EN_Pos_ENC((uint32_t)CLK_AHBCLK_GPFCKEN_Pos)|\
                        MODULE_CLKSEL_ENC(  NA)|MODULE_CLKSEL_Msk_ENC(NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                        MODULE_CLKDIV_ENC(  NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))    /*!< GPF Module */

/* APBCLK0 */
#define WDT_MODULE     (MODULE_APBCLK_ENC( 1UL)|MODULE_IP_EN_Pos_ENC((uint32_t)CLK_APBCLK0_WDTCKEN_Pos)|\
                        MODULE_CLKSEL_ENC( 1UL)|MODULE_CLKSEL_Msk_ENC( 3UL)|MODULE_CLKSEL_Pos_ENC(0UL)|\
                        MODULE_CLKDIV_ENC(  NA)|MODULE_CLKDIV_Msk_ENC(  NA)|MODULE_CLKDIV_Pos_ENC(NA))  /*!< WDT Module */

#define WWDT_MODULE    (MODULE_APBCLK_ENC( 1UL)|MODULE_IP_EN_Pos_ENC((uint32_t)CLK_APBCLK0_WDTCKEN_Pos)|\
                        MODULE_CLKSEL_ENC( 1UL)|MODULE_CLKSEL_Msk_ENC( 3UL)|MODULE_CLKSEL_Pos_ENC(30UL)|\
                        MODULE_CLKDIV_ENC(  NA)|MODULE_CLKDIV_Msk_ENC(  NA)|MODULE_CLKDIV_Pos_ENC(NA))  /*!< WWDT Module */

#define TMR0_MODULE    (MODULE_APBCLK_ENC( 1UL)|MODULE_IP_EN_Pos_ENC((uint32_t)CLK_APBCLK0_TMR0CKEN_Pos)|\
                        MODULE_CLKSEL_ENC( 1UL)|MODULE_CLKSEL_Msk_ENC( 7UL)|MODULE_CLKSEL_Pos_ENC(8UL)|\
                        MODULE_CLKDIV_ENC(  NA)|MODULE_CLKDIV_Msk_ENC(  NA)|MODULE_CLKDIV_Pos_ENC(NA))  /*!< TMR0 Module */

#define TMR1_MODULE    (MODULE_APBCLK_ENC( 1UL)|MODULE_IP_EN_Pos_ENC((uint32_t)CLK_APBCLK0_TMR1CKEN_Pos) |\
                        MODULE_CLKSEL_ENC( 1UL)|MODULE_CLKSEL_Msk_ENC( 7UL)|MODULE_CLKSEL_Pos_ENC(12UL)|\
                        MODULE_CLKDIV_ENC(  NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))    /*!< TMR1 Module */

#define TMR2_MODULE    (MODULE_APBCLK_ENC( 1UL)|MODULE_IP_EN_Pos_ENC((uint32_t)CLK_APBCLK0_TMR2CKEN_Pos) |\
                        MODULE_CLKSEL_ENC( 1UL)|MODULE_CLKSEL_Msk_ENC( 7UL)|MODULE_CLKSEL_Pos_ENC(16UL)|\
                        MODULE_CLKDIV_ENC(  NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))    /*!< TMR2 Module */

#define TMR3_MODULE    (MODULE_APBCLK_ENC( 1UL)|MODULE_IP_EN_Pos_ENC((uint32_t)CLK_APBCLK0_TMR3CKEN_Pos) |\
                        MODULE_CLKSEL_ENC( 1UL)|MODULE_CLKSEL_Msk_ENC( 7UL)|MODULE_CLKSEL_Pos_ENC(20UL)|\
                        MODULE_CLKDIV_ENC(  NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))    /*!< TMR3 Module */

#define CLKO_MODULE    (MODULE_APBCLK_ENC( 1UL)|MODULE_IP_EN_Pos_ENC((uint32_t)CLK_APBCLK0_CLKOCKEN_Pos) |\
                        MODULE_CLKSEL_ENC( 1UL)|MODULE_CLKSEL_Msk_ENC(3UL)|MODULE_CLKSEL_Pos_ENC(28UL)|\
                        MODULE_CLKDIV_ENC(  NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))    /*!< CLKO Module */

#define I2C0_MODULE    (MODULE_APBCLK_ENC( 1UL)|MODULE_IP_EN_Pos_ENC((uint32_t)CLK_APBCLK0_I2C0CKEN_Pos) |\
                        MODULE_CLKSEL_ENC(  NA)|MODULE_CLKSEL_Msk_ENC(NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                        MODULE_CLKDIV_ENC(  NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))    /*!< I2C0 Module */

#define UART0_MODULE   (MODULE_APBCLK_ENC( 1UL)|MODULE_IP_EN_Pos_ENC((uint32_t)CLK_APBCLK0_UART0CKEN_Pos)|\
                        MODULE_CLKSEL_ENC( 2UL)|MODULE_CLKSEL_Msk_ENC(   7UL)|MODULE_CLKSEL_Pos_ENC(16UL)|\
                        MODULE_CLKDIV_ENC( 0UL)|MODULE_CLKDIV_Msk_ENC(0x0FUL)|MODULE_CLKDIV_Pos_ENC( 8UL))  /*!< UART0 Module */

#define UART1_MODULE   (MODULE_APBCLK_ENC( 1UL)|MODULE_IP_EN_Pos_ENC((uint32_t)CLK_APBCLK0_UART1CKEN_Pos)|\
                        MODULE_CLKSEL_ENC( 2UL)|MODULE_CLKSEL_Msk_ENC(   7UL)|MODULE_CLKSEL_Pos_ENC(20UL)|\
                        MODULE_CLKDIV_ENC( 0UL)|MODULE_CLKDIV_Msk_ENC(0x0FUL)|MODULE_CLKDIV_Pos_ENC(12UL))  /*!< UART1 Module */

#define ADC_MODULE     (MODULE_APBCLK_ENC( 1UL)|MODULE_IP_EN_Pos_ENC((uint32_t)CLK_APBCLK0_ADCCKEN_Pos)|\
                        MODULE_CLKSEL_ENC( 2UL)|MODULE_CLKSEL_Msk_ENC(   3UL)|MODULE_CLKSEL_Pos_ENC(24UL)|\
                        MODULE_CLKDIV_ENC( 0UL)|MODULE_CLKDIV_Msk_ENC(0xFFUL)|MODULE_CLKDIV_Pos_ENC(16UL))  /*!< ADC Module */

/* APBCLK1 */
#define USCI0_MODULE   (MODULE_APBCLK_ENC( 2UL)|MODULE_IP_EN_Pos_ENC((uint32_t)CLK_APBCLK1_USCI0CKEN_Pos)|\
                        MODULE_CLKSEL_ENC(  NA)|MODULE_CLKSEL_Msk_ENC(NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                        MODULE_CLKDIV_ENC(  NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))        /*!< USCI0 Module */

#define PWM0_MODULE    (MODULE_APBCLK_ENC( 2UL)|MODULE_IP_EN_Pos_ENC((uint32_t)CLK_APBCLK1_PWM0CKEN_Pos)|\
                        MODULE_CLKSEL_ENC( 2UL)|MODULE_CLKSEL_Msk_ENC(1UL)|MODULE_CLKSEL_Pos_ENC(0UL)|\
                        MODULE_CLKDIV_ENC(  NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))        /*!< PWM0 Module */

#define ECAP0_MODULE   (MODULE_APBCLK_ENC( 2UL)|MODULE_IP_EN_Pos_ENC((uint32_t)CLK_APBCLK1_ECAP0CKEN_Pos)|\
                        MODULE_CLKSEL_ENC(  NA)|MODULE_CLKSEL_Msk_ENC(NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                        MODULE_CLKDIV_ENC(  NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))        /*!< ECAP0 Module */

/*---------------------------------------------------------------------------------------------------------*/
/*  PDMSEL constant definitions.                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_PMUCTL_PDMSEL_PD        (0x0UL << CLK_PMUCTL_PDMSEL_Pos)    /*!< Select Power-down mode is Power-down mode */
#define CLK_PMUCTL_PDMSEL_SPD       (0x4UL << CLK_PMUCTL_PDMSEL_Pos)    /*!< Select Power-down mode is Standby Power-down mode */
#define CLK_PMUCTL_PDMSEL_DPD       (0x6UL << CLK_PMUCTL_PDMSEL_Pos)    /*!< Select Power-down mode is Deep Power-down mode */

/*---------------------------------------------------------------------------------------------------------*/
/*  WKTMRIS constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_PMUCTL_WKTMRIS_128      (0x0UL << CLK_PMUCTL_WKTMRIS_Pos)   /*!< Select Wake-up Timer Time-out Interval is 128 LIRC clocks (12.8 ms) */
#define CLK_PMUCTL_WKTMRIS_256      (0x1UL << CLK_PMUCTL_WKTMRIS_Pos)   /*!< Select Wake-up Timer Time-out Interval is 256 LIRC clocks (25.6 ms) */
#define CLK_PMUCTL_WKTMRIS_512      (0x2UL << CLK_PMUCTL_WKTMRIS_Pos)   /*!< Select Wake-up Timer Time-out Interval is 512 LIRC clocks (51.2 ms) */
#define CLK_PMUCTL_WKTMRIS_1024     (0x3UL << CLK_PMUCTL_WKTMRIS_Pos)   /*!< Select Wake-up Timer Time-out Interval is 1024 LIRC clocks (102.4ms) */
#define CLK_PMUCTL_WKTMRIS_4096     (0x4UL << CLK_PMUCTL_WKTMRIS_Pos)   /*!< Select Wake-up Timer Time-out Interval is 4096 LIRC clocks (409.6ms) */
#define CLK_PMUCTL_WKTMRIS_8192     (0x5UL << CLK_PMUCTL_WKTMRIS_Pos)   /*!< Select Wake-up Timer Time-out Interval is 8192 LIRC clocks (819.2ms) */
#define CLK_PMUCTL_WKTMRIS_16384    (0x6UL << CLK_PMUCTL_WKTMRIS_Pos)   /*!< Select Wake-up Timer Time-out Interval is 16384 LIRC clocks (1638.4ms) */
#define CLK_PMUCTL_WKTMRIS_65536    (0x7UL << CLK_PMUCTL_WKTMRIS_Pos)   /*!< Select Wake-up Timer Time-out Interval is 65536 LIRC clocks (6553.6ms) */

/*---------------------------------------------------------------------------------------------------------*/
/*  SWKDBCLKSEL constant definitions.                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_SWKDBCTL_SWKDBCLKSEL_1          (0x0UL << CLK_SWKDBCTL_SWKDBCLKSEL_Pos)     /*!< Select Standby Power-down Pin De-bounce Sampling Cycle is 1 clocks */
#define CLK_SWKDBCTL_SWKDBCLKSEL_2          (0x1UL << CLK_SWKDBCTL_SWKDBCLKSEL_Pos)     /*!< Select Standby Power-down Pin De-bounce Sampling Cycle is 2 clocks */
#define CLK_SWKDBCTL_SWKDBCLKSEL_4          (0x2UL << CLK_SWKDBCTL_SWKDBCLKSEL_Pos)     /*!< Select Standby Power-down Pin De-bounce Sampling Cycle is 4 clocks */
#define CLK_SWKDBCTL_SWKDBCLKSEL_8          (0x3UL << CLK_SWKDBCTL_SWKDBCLKSEL_Pos)     /*!< Select Standby Power-down Pin De-bounce Sampling Cycle is 8 clocks */
#define CLK_SWKDBCTL_SWKDBCLKSEL_16         (0x4UL << CLK_SWKDBCTL_SWKDBCLKSEL_Pos)     /*!< Select Standby Power-down Pin De-bounce Sampling Cycle is 16 clocks */
#define CLK_SWKDBCTL_SWKDBCLKSEL_32         (0x5UL << CLK_SWKDBCTL_SWKDBCLKSEL_Pos)     /*!< Select Standby Power-down Pin De-bounce Sampling Cycle is 32 clocks */
#define CLK_SWKDBCTL_SWKDBCLKSEL_64         (0x6UL << CLK_SWKDBCTL_SWKDBCLKSEL_Pos)     /*!< Select Standby Power-down Pin De-bounce Sampling Cycle is 64 clocks */
#define CLK_SWKDBCTL_SWKDBCLKSEL_128        (0x7UL << CLK_SWKDBCTL_SWKDBCLKSEL_Pos)     /*!< Select Standby Power-down Pin De-bounce Sampling Cycle is 128 clocks */
#define CLK_SWKDBCTL_SWKDBCLKSEL_256        (0x8UL << CLK_SWKDBCTL_SWKDBCLKSEL_Pos)     /*!< Select Standby Power-down Pin De-bounce Sampling Cycle is 256 clocks */
#define CLK_SWKDBCTL_SWKDBCLKSEL_2x256      (0x9UL << CLK_SWKDBCTL_SWKDBCLKSEL_Pos)     /*!< Select Standby Power-down Pin De-bounce Sampling Cycle is 2x256 clocks */
#define CLK_SWKDBCTL_SWKDBCLKSEL_4x256      (0xaUL << CLK_SWKDBCTL_SWKDBCLKSEL_Pos)     /*!< Select Standby Power-down Pin De-bounce Sampling Cycle is 4x256 clocks */
#define CLK_SWKDBCTL_SWKDBCLKSEL_8x256      (0xbUL << CLK_SWKDBCTL_SWKDBCLKSEL_Pos)     /*!< Select Standby Power-down Pin De-bounce Sampling Cycle is 8x256 clocks */
#define CLK_SWKDBCTL_SWKDBCLKSEL_16x256     (0xcUL << CLK_SWKDBCTL_SWKDBCLKSEL_Pos)     /*!< Select Standby Power-down Pin De-bounce Sampling Cycle is 16x256 clocks */
#define CLK_SWKDBCTL_SWKDBCLKSEL_32x256     (0xdUL << CLK_SWKDBCTL_SWKDBCLKSEL_Pos)     /*!< Select Standby Power-down Pin De-bounce Sampling Cycle is 32x256 clocks */
#define CLK_SWKDBCTL_SWKDBCLKSEL_64x256     (0xeUL << CLK_SWKDBCTL_SWKDBCLKSEL_Pos)     /*!< Select Standby Power-down Pin De-bounce Sampling Cycle is 64x256 clocks */
#define CLK_SWKDBCTL_SWKDBCLKSEL_128x256    (0xfUL << CLK_SWKDBCTL_SWKDBCLKSEL_Pos)     /*!< Select Standby Power-down Pin De-bounce Sampling Cycle is 128x256 clocks */

/*---------------------------------------------------------------------------------------------------------*/
/*  DPD Pin Rising/Falling Edge Wake-up Enable constant definitions.                                       */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_DPDWKPIN1_DISABLE    (0x0UL << CLK_PMUCTL_WKPINEN1_Pos)    /*!< Disable Wake-up pin1 (GPB.0) at Deep Power-down mode \hideinitializer */
#define CLK_DPDWKPIN1_RISING     (0x1UL << CLK_PMUCTL_WKPINEN1_Pos)    /*!< Enable Wake-up pin1 (GPB.0) rising edge at Deep Power-down mode \hideinitializer */
#define CLK_DPDWKPIN1_FALLING    (0x2UL << CLK_PMUCTL_WKPINEN1_Pos)    /*!< Enable Wake-up pin1 (GPB.0) falling edge at Deep Power-down mode \hideinitializer */
#define CLK_DPDWKPIN1_BOTHEDGE   (0x3UL << CLK_PMUCTL_WKPINEN1_Pos)    /*!< Enable Wake-up pin1 (GPB.0) both edge at Deep Power-down mode \hideinitializer */

#define CLK_DPDWKPIN2_DISABLE    (0x0UL << CLK_PMUCTL_WKPINEN2_Pos)    /*!< Disable Wake-up pin2 (GPB.2) at Deep Power-down mode \hideinitializer */
#define CLK_DPDWKPIN2_RISING     (0x1UL << CLK_PMUCTL_WKPINEN2_Pos)    /*!< Enable Wake-up pin2 (GPB.2) rising edge at Deep Power-down mode \hideinitializer */
#define CLK_DPDWKPIN2_FALLING    (0x2UL << CLK_PMUCTL_WKPINEN2_Pos)    /*!< Enable Wake-up pin2 (GPB.2) falling edge at Deep Power-down mode \hideinitializer */
#define CLK_DPDWKPIN2_BOTHEDGE   (0x3UL << CLK_PMUCTL_WKPINEN2_Pos)    /*!< Enable Wake-up pin2 (GPB.2) both edge at Deep Power-down mode \hideinitializer */

#define CLK_DPDWKPIN3_DISABLE    (0x0UL << CLK_PMUCTL_WKPINEN3_Pos)    /*!< Disable Wake-up pin3 (GPB.12) at Deep Power-down mode \hideinitializer */
#define CLK_DPDWKPIN3_RISING     (0x1UL << CLK_PMUCTL_WKPINEN3_Pos)    /*!< Enable Wake-up pin3 (GPB.12) rising edge at Deep Power-down mode \hideinitializer */
#define CLK_DPDWKPIN3_FALLING    (0x2UL << CLK_PMUCTL_WKPINEN3_Pos)    /*!< Enable Wake-up pin3 (GPB.12) falling edge at Deep Power-down mode \hideinitializer */
#define CLK_DPDWKPIN3_BOTHEDGE   (0x3UL << CLK_PMUCTL_WKPINEN3_Pos)    /*!< Enable Wake-up pin3 (GPB.12) both edge at Deep Power-down mode \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  SPD Pin Rising/Falling Edge Wake-up Enable constant definitions.                                       */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_SPDWKPIN_ENABLE         (0x1UL << CLK_PBSWKCTL_WKEN_Pos)    /*!< Enable Standby Power-down Pin Wake-up */
#define CLK_SPDWKPIN_RISING         (0x1UL << CLK_PBSWKCTL_PRWKEN_Pos)  /*!< Standby Power-down Wake-up on Standby Power-down Pin rising edge */
#define CLK_SPDWKPIN_FALLING        (0x1UL << CLK_PBSWKCTL_PFWKEN_Pos)  /*!< Standby Power-down Wake-up on Standby Power-down Pin falling edge */
#define CLK_SPDWKPIN_DEBOUNCEEN     (0x1UL << CLK_PBSWKCTL_DBEN_Pos)    /*!< Enable Standby power-down pin De-bounce function */
#define CLK_SPDWKPIN_DEBOUNCEDIS    (0x0UL << CLK_PBSWKCTL_DBEN_Pos)    /*!< Disable Standby power-down pin De-bounce function */

#define CLK_TIMEOUT_ERR             (-1)    /*!< Clock timeout error value \hideinitializer */

/**@}*/ /* end of group CLK_EXPORTED_CONSTANTS */

extern int32_t g_CLK_i32ErrCode;

/** @addtogroup CLK_EXPORTED_FUNCTIONS CLK Exported Functions
  @{
*/

/**
  * @brief      Disable Wake-up Timer
  * @param      None
  * @return     Setting success or not
  * @retval     0                   Success
  * @retval     CLK_TIMEOUT_ERR     Failed due to PMUCTL register is busy
  * @details    This macro disables Wake-up timer at Standby or Deep Power-down mode.
  */
__STATIC_INLINE int32_t CLK_DISABLE_WKTMR(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock * 2;

    while(CLK->PMUCTL & CLK_PMUCTL_WRBUSY_Msk)
    {
        if(--u32TimeOutCnt == 0)
        {
            break;
        }
    }
    CLK->PMUCTL &= ~CLK_PMUCTL_WKTMREN_Msk;

    if(u32TimeOutCnt == 0)
        return CLK_TIMEOUT_ERR;
    else
        return 0;
}

/**
  * @brief      Enable Wake-up Timer
  * @param      None
  * @return     Setting success or not
  * @retval     0                   Success
  * @retval     CLK_TIMEOUT_ERR     Failed due to PMUCTL register is busy
  * @details    This macro enables Wake-up timer at Standby or Deep Power-down mode.
  */
__STATIC_INLINE int32_t CLK_ENABLE_WKTMR(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock * 2;

    while(CLK->PMUCTL & CLK_PMUCTL_WRBUSY_Msk)
    {
        if(--u32TimeOutCnt == 0)
        {
            break;
        }
    }
    CLK->PMUCTL |= CLK_PMUCTL_WKTMREN_Msk;

    if(u32TimeOutCnt == 0)
        return CLK_TIMEOUT_ERR;
    else
        return 0;
}

/**
  * @brief      Disable DPD Mode Wake-up Pin 1
  * @param      None
  * @return     Setting success or not
  * @retval     0                   Success
  * @retval     CLK_TIMEOUT_ERR     Failed due to PMUCTL register is busy
  * @details    This macro disables Wake-up pin 1 (GPB.0) at Deep Power-down mode.
  */
__STATIC_INLINE int32_t CLK_DISABLE_DPDWKPIN1(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock * 2;

    while(CLK->PMUCTL & CLK_PMUCTL_WRBUSY_Msk)
    {
        if(--u32TimeOutCnt == 0)
        {
            break;
        }
    }
    CLK->PMUCTL &= ~CLK_PMUCTL_WKPINEN1_Msk;

    if(u32TimeOutCnt == 0)
        return CLK_TIMEOUT_ERR;
    else
        return 0;
}

/**
  * @brief      Disable DPD Mode Wake-up Pin 2
  * @param      None
  * @return     Setting success or not
  * @retval     0                   Success
  * @retval     CLK_TIMEOUT_ERR     Failed due to PMUCTL register is busy
  * @details    This macro disables Wake-up pin 2 (GPB.2) at Deep Power-down mode.
  */
__STATIC_INLINE int32_t CLK_DISABLE_DPDWKPIN2(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock * 2;

    while(CLK->PMUCTL & CLK_PMUCTL_WRBUSY_Msk)
    {
        if(--u32TimeOutCnt == 0)
        {
            break;
        }
    }
    CLK->PMUCTL &= ~CLK_PMUCTL_WKPINEN2_Msk;

    if(u32TimeOutCnt == 0)
        return CLK_TIMEOUT_ERR;
    else
        return 0;
}

/**
  * @brief      Disable DPD Mode Wake-up Pin 3
  * @param      None
  * @return     Setting success or not
  * @retval     0                   Success
  * @retval     CLK_TIMEOUT_ERR     Failed due to PMUCTL register is busy
  * @details    This macro disables Wake-up pin 3 (GPB.12) at Deep Power-down mode.
  */
__STATIC_INLINE int32_t CLK_DISABLE_DPDWKPIN3(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock * 2;

    while(CLK->PMUCTL & CLK_PMUCTL_WRBUSY_Msk)
    {
        if(--u32TimeOutCnt == 0)
        {
            break;
        }
    }
    CLK->PMUCTL &= ~CLK_PMUCTL_WKPINEN3_Msk;

    if(u32TimeOutCnt == 0)
        return CLK_TIMEOUT_ERR;
    else
        return 0;
}

/**
  * @brief      Set Wake-up Timer Time-out Interval
  * @param[in]  u32Interval  The Wake-up Timer Time-out Interval selection. It could be
  *                            - \ref CLK_PMUCTL_WKTMRIS_128
  *                            - \ref CLK_PMUCTL_WKTMRIS_256
  *                            - \ref CLK_PMUCTL_WKTMRIS_512
  *                            - \ref CLK_PMUCTL_WKTMRIS_1024
  *                            - \ref CLK_PMUCTL_WKTMRIS_4096
  *                            - \ref CLK_PMUCTL_WKTMRIS_8192
  *                            - \ref CLK_PMUCTL_WKTMRIS_16384
  *                            - \ref CLK_PMUCTL_WKTMRIS_65536
  * @return     Setting success or not
  * @retval     0                   Success
  * @retval     CLK_TIMEOUT_ERR     Failed due to PMUCTL register is busy
  * @details    This function set Wake-up Timer Time-out Interval.
  */
__STATIC_INLINE int32_t CLK_SET_WKTMR_INTERVAL(uint32_t u32Interval)
{
    uint32_t u32TimeOutCnt = SystemCoreClock * 2;

    while(CLK->PMUCTL & CLK_PMUCTL_WRBUSY_Msk)
    {
        if(--u32TimeOutCnt == 0)
        {
            break;
        }
    }
    CLK->PMUCTL = (CLK->PMUCTL & (~CLK_PMUCTL_WKTMRIS_Msk)) | (u32Interval);

    if(u32TimeOutCnt == 0)
        return CLK_TIMEOUT_ERR;
    else
        return 0;
}

/**
  * @brief      Set De-bounce Sampling Cycle Time
  * @param[in]  u32CycleSel  The de-bounce sampling cycle selection. It could be
  *                            - \ref CLK_SWKDBCTL_SWKDBCLKSEL_1
  *                            - \ref CLK_SWKDBCTL_SWKDBCLKSEL_2
  *                            - \ref CLK_SWKDBCTL_SWKDBCLKSEL_4
  *                            - \ref CLK_SWKDBCTL_SWKDBCLKSEL_8
  *                            - \ref CLK_SWKDBCTL_SWKDBCLKSEL_16
  *                            - \ref CLK_SWKDBCTL_SWKDBCLKSEL_32
  *                            - \ref CLK_SWKDBCTL_SWKDBCLKSEL_64
  *                            - \ref CLK_SWKDBCTL_SWKDBCLKSEL_128
  *                            - \ref CLK_SWKDBCTL_SWKDBCLKSEL_256
  *                            - \ref CLK_SWKDBCTL_SWKDBCLKSEL_2x256
  *                            - \ref CLK_SWKDBCTL_SWKDBCLKSEL_4x256
  *                            - \ref CLK_SWKDBCTL_SWKDBCLKSEL_8x256
  *                            - \ref CLK_SWKDBCTL_SWKDBCLKSEL_16x256
  *                            - \ref CLK_SWKDBCTL_SWKDBCLKSEL_32x256
  *                            - \ref CLK_SWKDBCTL_SWKDBCLKSEL_64x256
  *                            - \ref CLK_SWKDBCTL_SWKDBCLKSEL_128x256
  * @return     None
  * @details    This function set Set De-bounce Sampling Cycle Time for Standby Power-down pin wake-up.
  */
#define CLK_SET_SPDDEBOUNCETIME(u32CycleSel)    (CLK->SWKDBCTL = (u32CycleSel))

/*---------------------------------------------------------------------------------------------------------*/
/* static inline functions                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
/* Declare these inline functions here to avoid MISRA C 2004 rule 8.1 error */
__STATIC_INLINE int32_t CLK_SysTickDelay(uint32_t us);
__STATIC_INLINE int32_t CLK_SysTickLongDelay(uint32_t us);


/**
  * @brief      This function execute delay function.
  * @param[in]  us  Delay time. The Max value is (2^24-1) / CPU Clock(MHz). Ex:
  *                             96MHz => 174762us, 84MHz => 199728us,
  *                             64MHz => 262143us, 48MHz => 349525us ...
  * @return     Delay success or not
  * @retval     0                   Success, target delay time reached
  * @retval     CLK_TIMEOUT_ERR     Delay function execute failed due to SysTick stop working
  * @details    Use the SysTick to generate the delay time and the UNIT is in us.
  *             The SysTick clock source is from HCLK, i.e the same as system core clock.
  *             User can use SystemCoreClockUpdate() to calculate CyclesPerUs automatically before using this function.
  */
__STATIC_INLINE int32_t CLK_SysTickDelay(uint32_t us)
{
    /* The u32TimeOutCnt value must be greater than the max delay time of 1398ms if HCLK=12MHz */
    uint32_t u32TimeOutCnt = SystemCoreClock * 2;

    SysTick->LOAD = us * CyclesPerUs;
    SysTick->VAL  = (0x0UL);
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    /* Waiting for down-count to zero */
    while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0)
    {
        if(--u32TimeOutCnt == 0)
        {
            break;
        }
    }

    /* Disable SysTick counter */
    SysTick->CTRL = 0UL;

    if(u32TimeOutCnt == 0)
        return CLK_TIMEOUT_ERR;
    else
        return 0;
}

/**
  * @brief      This function execute long delay function.
  * @param[in]  us  Delay time.
  * @return     Delay success or not
  * @retval     0                   Success, target delay time reached
  * @retval     CLK_TIMEOUT_ERR     Delay function execute failed due to SysTick stop working
  * @details    Use the SysTick to generate the long delay time and the UNIT is in us.
  *             The SysTick clock source is from HCLK, i.e the same as system core clock.
  *             User can use SystemCoreClockUpdate() to calculate CyclesPerUs automatically before using this function.
  */
__STATIC_INLINE int32_t CLK_SysTickLongDelay(uint32_t us)
{
    /* The u32TimeOutCnt value must be greater than the max delay time of 1398ms if HCLK=12MHz */
    uint32_t u32TimeOutCnt = SystemCoreClock * 2;
    uint32_t u32Delay;

    /* It should <= 65536us for each delay loop */
    u32Delay = 65536UL;

    do
    {
        if(us > u32Delay)
        {
            us -= u32Delay;
        }
        else
        {
            u32Delay = us;
            us = 0UL;
        }

        SysTick->LOAD = u32Delay * CyclesPerUs;
        SysTick->VAL  = (0x0UL);
        SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

        /* Waiting for down-count to zero */
        while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0UL)
        {
            if(--u32TimeOutCnt == 0)
            {
                break;
            }
        }

        /* Disable SysTick counter */
        SysTick->CTRL = 0UL;

        if(u32TimeOutCnt == 0)
            return CLK_TIMEOUT_ERR;
        else
            return 0;

    }
    while(us > 0UL);
}


void     CLK_DisableCKO(void);
void     CLK_EnableCKO(uint32_t u32ClkSrc, uint32_t u32ClkDiv, uint32_t u32ClkDivBy1En);
void     CLK_PowerDown(void);
void     CLK_Idle(void);
uint32_t CLK_GetHCLKFreq(void);
uint32_t CLK_GetPCLK0Freq(void);
uint32_t CLK_GetPCLK1Freq(void);
uint32_t CLK_GetCPUFreq(void);
uint32_t CLK_SetCoreClock(uint32_t u32Hclk);
void     CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void     CLK_SetModuleClock(uint32_t u32ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void     CLK_SetSysTickClockSrc(uint32_t u32ClkSrc);
void     CLK_EnableXtalRC(uint32_t u32ClkMask);
void     CLK_DisableXtalRC(uint32_t u32ClkMask);
void     CLK_EnableModuleClock(uint32_t u32ModuleIdx);
void     CLK_DisableModuleClock(uint32_t u32ModuleIdx);
uint32_t CLK_WaitClockReady(uint32_t u32ClkMask);
void     CLK_EnableSysTick(uint32_t u32ClkSrc, uint32_t u32Count);
void     CLK_DisableSysTick(void);
int32_t  CLK_SetPowerDownMode(uint32_t u32PDMode);
int32_t  CLK_EnableDPDWKPin(uint32_t u32TriggerType);
uint32_t CLK_GetPMUWKSrc(void);
void     CLK_EnableSPDWKPin(uint32_t u32Port, uint32_t u32Pin, uint32_t u32TriggerType, uint32_t u32DebounceEn);
uint32_t CLK_GetModuleClockSource(uint32_t u32ModuleIdx);
uint32_t CLK_GetModuleClockDivider(uint32_t u32ModuleIdx);


/**@}*/ /* end of group CLK_EXPORTED_FUNCTIONS */

/**@}*/ /* end of group CLK_Driver */

/**@}*/ /* end of group Standard_Driver */


#ifdef __cplusplus
}
#endif

#endif /* __CLK_H__ */
