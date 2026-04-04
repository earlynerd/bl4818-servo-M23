/**************************************************************************//**
 * @file     clk_reg.h
 * @version  V1.00
 * @brief    CLK register definition header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __CLK_REG_H__
#define __CLK_REG_H__

#if defined ( __CC_ARM   )
#pragma anon_unions
#endif

/**
   @addtogroup REGISTER Control Register
   @{
*/

/**
   @addtogroup CLK System Clock Controller (CLK)
   Memory Mapped Structure for CLK Controller
   @{
*/

typedef struct
{
    /**
     * @var CLK_T::PWRCTL
     * Offset: 0x00  System Power-down Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2]     |HIRCEN    |HIRC Enable Bit (Write Protect)
     * |        |          |0 = 24 MHz internal high speed RC oscillator (HIRC) Disabled.
     * |        |          |1 = 24 MHz internal high speed RC oscillator (HIRC) Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: HIRC cannot be disabled and HIRCEN will read as 1 if HCLK clock source is selected from HIRC (clock source from HIRC).
     * |[3]     |LIRCEN    |LIRC Enable Bit (Write Protect)
     * |        |          |0 = 10 kHz internal low speed RC oscillator (LIRC) Disabled.
     * |        |          |1 = 10 kHz internal low speed RC oscillator (LIRC) Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: LIRC cannot be disabled and LIRCEN will read as 1 if HCLK clock source is selected from LIRC.
     * |        |          |Note 3: If CWDTEN(CONFIG0[31,4:3]) is set to 111, LIRC clock can be enabled or disabled by setting LIRCEN(CLK_PWRCTL[3]).
     * |        |          |If CWDTEN(CONFIG0[31,4:3]) is not set to 111, LIRC cannot be disabled in normal mode
     * |        |          |In Power-down mode, LIRC clock is controlled by LIRCEN(CLK_PWRCTL[3]) and CWDTPDEN(CONFIG0[30]) setting.
     * |[5]     |PDWKIEN   |Power-down Mode Wake-up Interrupt Enable Bit (Write Protect)
     * |        |          |0 = Power-down mode wake-up interrupt Disabled.
     * |        |          |1 = Power-down mode wake-up interrupt Enabled.
     * |        |          |Note 1: The interrupt will occur when both PDWKIF and PDWKIEN are high, after resume from power-down mode.
     * |        |          |Note 2: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[6]     |PDWKIF    |Power-down Mode Wake-up Interrupt Status
     * |        |          |Set by "Power-down wake-up event", it indicates that resume from "Power-down mode".
     * |        |          |The flag is set if the EINT0~5, GPIO, UART0~1, BOD, WDT, TIMER, I2C0, USCI0 and CLKD wake-up occurred.
     * |        |          |Note 1: Write 1 to clear the bit to 0.
     * |        |          |Note 2: This bit works only if PDWKIEN (CLK_PWRCTL[5]) set to 1.
     * |[7]     |PDEN      |System Power-down Enable (Write Protect)
     * |        |          |When this bit is set to 1, Power-down mode is enabled and the chip keeps active till the CPU sleep mode is also active and then the chip enters Power-down mode.
     * |        |          |When chip wakes up from Power-down mode, this bit is auto cleared.
     * |        |          |Users need to set this bit again for next Power-down.
     * |        |          |In Power-down mode, HIRC and system clock will be disabled and ignored the clock source selection.
     * |        |          |The clocks of peripheral are not controlled by Power-down mode, if the peripheral clock source is from LIRC.
     * |        |          |0 = Chip operating normally or chip in idle mode because of WFI command.
     * |        |          |1 = Chip waits CPU sleep command WFI and then enters Power-down mode.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var CLK_T::AHBCLK
     * Offset: 0x04  AHB Devices Clock Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |FMCIDLE   |Flash Memory Controller Clock Enable Bit in IDLE Mode
     * |        |          |0 = FMC clock Disabled when chip is under IDLE mode.
     * |        |          |1 = FMC clock Enabled when chip is under IDLE mode.
     * |[1]     |ISPCKEN   |Flash ISP Controller Clock Enable Bit
     * |        |          |0 = Flash ISP peripheral clock Disabled.
     * |        |          |1 = Flash ISP peripheral clock Enabled.
     * |[3]     |STCKEN    |System Tick Clock Enable Bit
     * |        |          |0 = External System tick clock Disabled.
     * |        |          |1 = External System tick clock Enabled.
     * |[8]     |SRAM0CKEN |SRAM Bank0 Controller Clock Enable Bit
     * |        |          |0 = SRAM bank0 clock Disabled.
     * |        |          |1 = SRAM bank0 clock Enabled.
     * |[17]    |GPBCKEN   |GPIOB Clock Enable Bit
     * |        |          |0 = GPIOB port clock Disabled.
     * |        |          |1 = GPIOB port clock Enabled.
     * |[18]    |GPCCKEN   |GPIOC Clock Enable Bit
     * |        |          |0 = GPIOC port clock Disabled.
     * |        |          |1 = GPIOC port clock Enabled.
     * |[20]    |GPECKEN   |GPIOE Clock Enable Bit
     * |        |          |0 = GPIOE port clock Disabled.
     * |        |          |1 = GPIOE port clock Enabled.
     * |[21]    |GPFCKEN   |GPIOF Clock Enable Bit
     * |        |          |0 = GPIOF port clock Disabled.
     * |        |          |1 = GPIOF port clock Enabled.
     * @var CLK_T::APBCLK0
     * Offset: 0x08  APB Devices Clock Enable Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WDTCKEN   |Watchdog Timer Clock Enable Bit (Write Protect)
     * |        |          |0 = Watchdog timer and Windows watchdog timer clock Disabled.
     * |        |          |1 = Watchdog timer and Windows watchdog timer clock Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[2]     |TMR0CKEN  |Timer0 Clock Enable Bit
     * |        |          |0 = Timer0 clock Disabled.
     * |        |          |1 = Timer0 clock Enabled.
     * |[3]     |TMR1CKEN  |Timer1 Clock Enable Bit
     * |        |          |0 = Timer1 clock Disabled.
     * |        |          |1 = Timer1 clock Enabled.
     * |[4]     |TMR2CKEN  |Timer2 Clock Enable Bit
     * |        |          |0 = Timer2 clock Disabled.
     * |        |          |1 = Timer2 clock Enabled.
     * |[5]     |TMR3CKEN  |Timer3 Clock Enable Bit
     * |        |          |0 = Timer3 clock Disabled.
     * |        |          |1 = Timer3 clock Enabled.
     * |[6]     |CLKOCKEN  |CLKO Clock Enable Bit
     * |        |          |0 = CLKO clock Disabled.
     * |        |          |1 = CLKO clock Enabled.
     * |[8]     |I2C0CKEN  |I2C0 Clock Enable Bit
     * |        |          |0 = I2C0 clock Disabled.
     * |        |          |1 = I2C0 clock Enabled.
     * |[16]    |UART0CKEN |UART0 Clock Enable Bit
     * |        |          |0 = UART0 clock Disabled.
     * |        |          |1 = UART0 clock Enabled.
     * |[17]    |UART1CKEN |UART1 Clock Enable Bit
     * |        |          |0 = UART1 clock Disabled.
     * |        |          |1 = UART1 clock Enabled.
     * |[28]    |ADCCKEN   |Analog-digital-converter Clock Enable Bit
     * |        |          |0 = ADC clock Disabled.
     * |        |          |1 = ADC clock Enabled.
     * @var CLK_T::APBCLK1
     * Offset: 0x0C  APB Devices Clock Enable Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[8]     |USCI0CKEN |USCI0 Clock Enable Bit
     * |        |          |0 = USCI0 clock Disabled.
     * |        |          |1 = USCI0 clock Enabled.
     * |[16]    |PWM0CKEN  |PWM0 Clock Enable Bit
     * |        |          |0 = PWM0 clock Disabled.
     * |        |          |1 = PWM0 clock Enabled.
     * |[26]    |ECAP0CKEN |ECAP0 Clock Enable Bit
     * |        |          |0 = ECAP0 clock Disabled.
     * |        |          |1 = ECAP0 clock Enabled.
     * @var CLK_T::CLKSEL0
     * Offset: 0x10  Clock Source Select Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |HCLKSEL   |HCLK Clock Source Selection (Write Protect)
     * |        |          |Before clock switching, the related clock sources (both pre-select and new-select) must be turned on.
     * |        |          |011 = Clock source from LIRC.
     * |        |          |101 = Clock source from HIRC.
     * |        |          |Others = Reserved.
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |[5:3]   |STCLKSEL  |SysTick Clock Source Selection (Write Protect)
     * |        |          |If SYST_CTRL[2]=0, SysTick uses listed clock source below.
     * |        |          |011 = Clock source from HCLK/2.
     * |        |          |100 = Clock source from HIRC/2.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: if SysTick clock source is not from HCLK (i.e. SYST_CTRL[2] = 0), SysTick need to enable EXSTCKEN(CLK_AHBCLK[4]) and clock frequency must less than or equal to HCLK/2.
     * |        |          |Note 2: These bits are write protected. Refer to the SYS_REGLCTL register.
     * @var CLK_T::CLKSEL1
     * Offset: 0x14  Clock Source Select Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |WDTSEL    |Watchdog Timer Clock Source Selection (Write Protect)
     * |        |          |10 = Clock source from HCLK/2048.
     * |        |          |11 = Clock source from 10 kHz internal low speed RC oscillator (LIRC).
     * |        |          |Others = Reserved.
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |[10:8]  |TMR0SEL   |TIMER0 Clock Source Selection
     * |        |          |010 = Clock source from PCLK0.
     * |        |          |011 = Clock source from external clock TM0 pin.
     * |        |          |100 = Clock source from 10 kHz internal low speed RC oscillator (LIRC).
     * |        |          |101 = Clock source from 24 MHz internal high speed RC oscillator (HIRC).
     * |        |          |Others = Reserved.
     * |[14:12] |TMR1SEL   |TIMER1 Clock Source Selection
     * |        |          |010 = Clock source from PCLK0.
     * |        |          |011 = Clock source from external clock TM1 pin.
     * |        |          |100 = Clock source from 10 kHz internal low speed RC oscillator (LIRC).
     * |        |          |101 = Clock source from 24 MHz internal high speed RC oscillator (HIRC).
     * |        |          |Others = Reserved.
     * |[18:16] |TMR2SEL   |TIMER2 Clock Source Selection
     * |        |          |010 = Clock source from PCLK1.
     * |        |          |011 = Clock source from external clock TM2 pin.
     * |        |          |100 = Clock source from 10 kHz internal low speed RC oscillator (LIRC).
     * |        |          |101 = Clock source from 24 MHz internal high speed RC oscillator (HIRC).
     * |        |          |Others = Reserved.
     * |[22:20] |TMR3SEL   |TIMER3 Clock Source Selection
     * |        |          |010 = Clock source from PCLK1.
     * |        |          |011 = Clock source from external clock TM3 pin.
     * |        |          |100 = Clock source from 10 kHz internal low speed RC oscillator (LIRC).
     * |        |          |101 = Clock source from 24 MHz internal high speed RC oscillator (HIRC).
     * |        |          |Others = Reserved.
     * |[29:28] |CLKOSEL   |Clock Output Clock Source Selection
     * |        |          |10 = Clock source from HCLK.
     * |        |          |11 = Clock source from 24 MHz internal high speed RC oscillator (HIRC).
     * |[31:30] |WWDTSEL   |Window Watchdog Timer Clock Source Selection (Write Protect)
     * |        |          |10 = Clock source from HCLK/2048.
     * |        |          |11 = Clock source from 10 kHz internal low speed RC oscillator (LIRC).
     * |        |          |Others = Reserved.
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     * @var CLK_T::CLKSEL2
     * Offset: 0x18  Clock Source Select Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PWM0SEL   |PWM0 Clock Source Selection (Read Only)
     * |        |          |The peripheral clock source of PWM0 is defined by PWM0SEL.
     * |        |          |0 = Reserved.
     * |        |          |1 = Clock source from PCLK0.
     * |[18:16] |UART0SEL  |UART0 Clock Source Selection
     * |        |          |011 = Clock source from 24 MHz internal high speed RC oscillator (HIRC).
     * |        |          |100 = Clock source from PCLK0.
     * |        |          |Others = Reserved.
     * |[22:20] |UART1SEL  |UART1 Clock Source Selection
     * |        |          |011 = Clock source from 24 MHz internal high speed RC oscillator (HIRC).
     * |        |          |100 = Clock source from PCLK1.
     * |        |          |Others = Reserved.
     * |[25:24] |ADCSEL    |ADC Clock Source Selection
     * |        |          |10 = Clock source from PCLK2.
     * |        |          |11 = Clock source from 24 MHz internal high speed RC oscillator (HIRC).
     * |        |          |Others = Reserved.
     * @var CLK_T::CLKDIV0
     * Offset: 0x20  Clock Divider Number Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |HCLKDIV   |HCLK Clock Divide Number From HCLK Clock Source
     * |        |          |HCLK clock frequency = (HCLK clock source frequency) / (HCLKDIV + 1).
     * |[11:8]  |UART0DIV  |UART0 Clock Divide Number From UART0 Clock Source
     * |        |          |UART0 clock frequency = (UART0 clock source frequency) / (UART0DIV + 1).
     * |[15:12] |UART1DIV  |UART1 Clock Divide Number From UART1 Clock Source
     * |        |          |UART1 clock frequency = (UART1 clock source frequency) / (UART1DIV + 1).
     * |[23:16] |ADCDIV    |ADC Clock Divide Number From ADC Clock Source
     * |        |          |ADC clock frequency = (ADC clock source frequency) / (ADCDIV + 1).
     * |        |          |Note: This field write ignore when CLK_APBCLK0's ADCCKEN = 1.
     * @var CLK_T::PCLKDIV
     * Offset: 0x34  APB Clock Divider Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |APB0DIV   |APB0 Clock Divider
     * |        |          |APB0 clock can be divided from HCLK.
     * |        |          |000 = PCLK0 frequency is HCLK.
     * |        |          |001 = PCLK0 frequency is 1/2 HCLK.
     * |        |          |010 = PCLK0 frequency is 1/4 HCLK.
     * |        |          |011 = PCLK0 frequency is 1/8 HCLK.
     * |        |          |100 = PCLK0 frequency is 1/16 HCLK.
     * |        |          |101 = PCLK0 frequency is 1/32 HCLK.
     * |        |          |Others = Reserved.
     * |[6:4]   |APB1DIV   |APB1 Clock Divider
     * |        |          |APB1 clock can be divided from HCLK.
     * |        |          |000 = PCLK1 frequency is HCLK.
     * |        |          |001 = PCLK1 frequency is 1/2 HCLK.
     * |        |          |010 = PCLK1 frequency is 1/4 HCLK.
     * |        |          |011 = PCLK1 frequency is 1/8 HCLK.
     * |        |          |100 = PCLK1 frequency is 1/16 HCLK.
     * |        |          |101 = PCLK1 frequency is 1/32 HCLK.
     * |        |          |Others = Reserved.
     * |[10:8]  |APB2DIV   |APB2 Clock Divider
     * |        |          |APB2 clock can be divided from HCLK.
     * |        |          |000 = PCLK2 frequency is HCLK.
     * |        |          |001 = PCLK2 frequency is 1/2 HCLK.
     * |        |          |010 = PCLK2 frequency is 1/4 HCLK.
     * |        |          |011 = PCLK2 frequency is 1/8 HCLK.
     * |        |          |100 = PCLK2 frequency is 1/16 HCLK.
     * |        |          |101 = PCLK2 frequency is 1/32 HCLK.
     * |        |          |Others = Reserved.
     * @var CLK_T::STATUS
     * Offset: 0x50  Clock Status Monitor Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3]     |LIRCSTB   |LIRC Clock Source Stable Flag (Read Only)
     * |        |          |0 = 10 kHz internal low speed RC oscillator (LIRC) clock is not stable or disabled.
     * |        |          |1 = 10 kHz internal low speed RC oscillator (LIRC) clock is stable and enabled.
     * |[4]     |HIRCSTB   |HIRC Clock Source Stable Flag (Read Only)
     * |        |          |0 = 24 MHz internal high speed RC oscillator (HIRC) clock is not stable or disabled.
     * |        |          |1 = 24 MHz internal high speed RC oscillator (HIRC) clock is stable and enabled.
     * |[7]     |CLKSFAIL  |Clock Switching Fail Flag (Read Only)
     * |        |          |This bit is updated when software switches system clock source.
     * |        |          |If switch target clock is stable, this bit will be set to 0.
     * |        |          |If switch target clock is not stable, this bit will be set to 1.
     * |        |          |0 = Clock switching success.
     * |        |          |1 = Clock switching failure.
     * |        |          |Note: This bit is read only.
     * |        |          |After selected clock source is stable, hardware will switch system clock to selected clock automatically, and CLKSFAIL will be cleared automatically by hardware.
     * @var CLK_T::CLKOCTL
     * Offset: 0x60  Clock Output Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |FREQSEL   |Clock Output Frequency Selection
     * |        |          |The formula of output frequency is Fout = Fin/2^(N+1).
     * |        |          |Fin is the input clock frequency.
     * |        |          |Fout is the frequency of divider output clock.
     * |        |          |N is the 4-bit value of FREQSEL[3:0].
     * |[4]     |CLKOEN    |Clock Output Enable Bit
     * |        |          |0 = Clock Output function Disabled.
     * |        |          |1 = Clock Output function Enabled.
     * |[5]     |DIV1EN    |Clock Output Divide One Enable Bit
     * |        |          |0 = Clock Output will output clock with source frequency divided by FREQSEL.
     * |        |          |1 = Clock Output will output clock with source frequency.
     * |[6]     |CLK1HZEN  |CLK1HZEN
     * |        |          |Note: This bits has to set to 0.
     * @var CLK_T::PMUCTL
     * Offset: 0x90  Power Manager Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |PDMSEL    |Power-down Mode Selection (Write Protect)
     * |        |          |These bits control chip power-down mode grade selection when CPU executes WFI/WFE instruction.
     * |        |          |000 = Power-down mode is selected (PD).
     * |        |          |001 = Reserved.
     * |        |          |010 = Reserved.
     * |        |          |011 = Reserved.
     * |        |          |100 = Reserved.
     * |        |          |101 = Reserved.
     * |        |          |110 = Reserved.
     * |        |          |111 = Reserved.
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |[7]     |WRBUSY    |Write Busy Flag (Read Only)
     * |        |          |If CLK_PMUCTL be written, this bit be asserted automatic by hardware, and be de-asserted when write procedure finish.
     * |        |          |0 = CLK_PMUCTL write ready.
     * |        |          |1 = CLK_PMUCTL write ignore.
     * |[8]     |WKTMREN   |Wake-up Timer Enable Bit (Write Protect)
     * |        |          |0 = Wake-up timer disable at Power-down mode.
     * |        |          |1 = Wake-up timer enabled at Power-down mode.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[11:9]  |WKTMRIS   |Wake-up Timer Time-out Interval Select Bits (Write Protect)
     * |        |          |These bits control wake-up timer time-out interval when chip under Power-down mode.
     * |        |          |000 = Time-out interval is 128 LIRC clocks (12.8ms).
     * |        |          |001 = Time-out interval is 256 LIRC clocks (25.6ms).
     * |        |          |010 = Time-out interval is 512 LIRC clocks (51.2ms).
     * |        |          |011 = Time-out interval is 1024 LIRC clocks (102.4ms).
     * |        |          |100 = Time-out interval is 4096 LIRC clocks (409.6ms).
     * |        |          |101 = Time-out interval is 8192 LIRC clocks (819.2ms).
     * |        |          |110 = Time-out interval is 16384 LIRC clocks (1638.4ms).
     * |        |          |111 = Time-out interval is 65536 LIRC clocks (6553.6ms).
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |[25:24] |WKPINEN1  |Wake-up Pin 1 Enable Bits (Write Protect)
     * |        |          |This is control register for GPB.0 to wake-up pin.
     * |        |          |00 = Wake-up pin disable at Power-down mode.
     * |        |          |01 = Wake-up pin rising edge enabled at Power-down mode.
     * |        |          |10 = Wake-up pin falling edge enabled at Power-down mode.
     * |        |          |11 = Wake-up pin both edge enabled at Power-down mode.
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |[27:26] |WKPINEN2  |Wake-up Pin 2 Enable Bits (Write Protect)
     * |        |          |This is control register for GPB.2 to wake-up pin.
     * |        |          |00 = Wake-up pin disable at Power-down mode.
     * |        |          |01 = Wake-up pin rising edge enabled at Power-down mode.
     * |        |          |10 = Wake-up pin falling edge enabled at Power-down mode.
     * |        |          |11 = Wake-up pin both edge enabled at Power-down mode.
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |[29:28] |WKPINEN3  |Wake-up Pin 3 Enable Bits (Write Protect)
     * |        |          |This is control register for GPB.12 to wake-up pin.
     * |        |          |00 = Wake-up pin disable at Power-down mode.
     * |        |          |01 = Wake-up pin rising edge enabled at Power-down mode.
     * |        |          |10 = Wake-up pin falling edge enabled at Power-down mode.
     * |        |          |11 = Wake-up pin both edge enabled at Power-down mode.
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     * @var CLK_T::PMUSTS
     * Offset: 0x94  Power Manager Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |TMRWK     |Timer Wake-up Flag (Read Only)
     * |        |          |This flag indicates that wake-up of chip from Power-down mode was requested by wakeup timer time-out. 
     * |[3]     |PINWK1    |Pin 1 Wake-up Flag (Read Only)
     * |        |          |This flag indicates that wake-up of chip from Power-down mode uEE52as requested by a transition of the Wake-up pin (GPB.0).
     * |[4]     |PINWK2    |Pin 2 Wake-up Flag (Read Only)
     * |        |          |This flag indicates that wake-up of chip from Power-down mode uEE52as requested by a transition of the Wake-up pin (GPB.2).
     * |[5]     |PINWK3    |Pin 3 Wake-up Flag (Read Only)
     * |        |          |This flag indicates that wake-up of chip from Power-down mode uEE52as requested by a transition of the Wake-up pin (GPB.12).
     * |[9]     |GPBWK     |GPB Wake-up Flag (Read Only)
     * |        |          |This flag indicates that wake-up of chip from Power-down mode uEE52as requested by a transition of selected one GPB group pins.
     * |[10]    |GPCWK     |GPC Wake-up Flag (Read Only)
     * |        |          |This flag indicates that wake-up of chip from Power-down mode uEE52as requested by a transition of selected one GPC group pins.
     * |[12]    |LVRWK     |LVR Wake-up Flag (Read Only)
     * |        |          |This flag indicates that wakeup of device from Power-down mode uEE52as requested with a LVR happened. 
     * |[13]    |BODWK     |BOD Wake-up Flag (Read Only)
     * |        |          |This flag indicates that wakeup of device from Power-down mode uEE52as requested with a BOD happened.
     * |[31]    |CLRWK     |Clear Wake-up Flag
     * |        |          |0 = No clear.
     * |        |          |1 = Clear all of wake-up flag.
     * |        |          |Note: This bit is auto cleared by hardware.
     * @var CLK_T::SWKDBCTL
     * Offset: 0x9C  Power-down Wake-up De-bounce Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |SWKDBCLKSEL|Power-down Wake-up De-bounce Sampling Cycle Selection
     * |        |          |0000 = Sample wake-up input once per 1 clocks.
     * |        |          |0001 = Sample wake-up input once per 2 clocks.
     * |        |          |0010 = Sample wake-up input once per 4 clocks.
     * |        |          |0011 = Sample wake-up input once per 8 clocks.
     * |        |          |0100 = Sample wake-up input once per 16 clocks.
     * |        |          |0101 = Sample wake-up input once per 32 clocks.
     * |        |          |0110 = Sample wake-up input once per 64 clocks.
     * |        |          |0111 = Sample wake-up input once per 128 clocks.
     * |        |          |1000 = Sample wake-up input once per 256 clocks.
     * |        |          |1001 = Sample wake-up input once per 2*256 clocks.
     * |        |          |1010 = Sample wake-up input once per 4*256 clocks.
     * |        |          |1011 = Sample wake-up input once per 8*256 clocks.
     * |        |          |1100 = Sample wake-up input once per 16*256 clocks.
     * |        |          |1101 = Sample wake-up input once per 32*256 clocks.
     * |        |          |1110 = Sample wake-up input once per 64*256 clocks.
     * |        |          |1111 = Sample wake-up input once per 128*256 clocks.
     * |        |          |Note: De-bounce counter clock source is the 10kHz internal low speed RC oscillator (LIRC).
     * @var CLK_T::PBSWKCTL
     * Offset: 0xA4  GPB Power-down Wake-up Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WKEN      |Power-down Pin Wake-up Enable Bit
     * |        |          |0 = GPB group pin wake-up function Disabled.
     * |        |          |1 = GPB group pin wake-up function Enabled.
     * |[1]     |PRWKEN    |Pin Rising Edge Wake-up Enable Bit
     * |        |          |0 = GPB group pin rising edge wake-up function Disabled.
     * |        |          |1 = GPB group pin rising edge wake-up function Enabled.
     * |[2]     |PFWKEN    |Pin Falling Edge Wake-up Enable Bit
     * |        |          |0 = GPB group pin falling edge wake-up function Disabled.
     * |        |          |1 = GPB group pin falling edge wake-up function Enabled.
     * |[7:4]   |WKPSEL    |GPB Standby Power-down Wake-up Pin Select
     * |        |          |0000 = GPB.0 wake-up function Enabled.
     * |        |          |0001 = GPB.1 wake-up function Enabled.
     * |        |          |0010 = GPB.2 wake-up function Enabled.
     * |        |          |0011 = GPB.3 wake-up function Enabled.
     * |        |          |0100 = GPB.4 wake-up function Enabled.
     * |        |          |0101 = GPB.5 wake-up function Enabled.
     * |        |          |0111 = GPB.7 wake-up function Enabled.
     * |        |          |1000 = GPB.8 wake-up function Enabled.
     * |        |          |1001 = GPB.9 wake-up function Enabled.
     * |        |          |1011 = GPB.11 wake-up function Enabled.
     * |        |          |1100 = GPB.12 wake-up function Enabled.
     * |        |          |1101 = GPB.13 wake-up function Enabled.
     * |        |          |1110 = GPB.14 wake-up function Enabled.
     * |        |          |1111 = GPB.15 wake-up function Enabled.
     * |        |          |Others are reserved.
     * |[8]     |DBEN      |GPB Input Signal De-bounce Enable Bit
     * |        |          |The DBEN bit is used to enable the de-bounce function for each corresponding I/O
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger the wakeup.
     * |        |          |The de-bounce clock source is the 10 kHz internal low speed RC oscillator (LIRC).
     * |        |          |0 = power-down wake-up pin De-bounce function Disabled.
     * |        |          |1 = power-down wake-up pin De-bounce function Enabled.
     * |        |          |Note: The de-bounce function is valid only for edge triggered.
     * @var CLK_T::PCSWKCTL
     * Offset: 0xA8  GPC Power-down Wake-up Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WKEN      |Standby Power-down Pin Wake-up Enable Bit
     * |        |          |0 = GPC group pin wake-up function Disabled.
     * |        |          |1 = GPC group pin wake-up function Enabled.
     * |[1]     |PRWKEN    |Pin Rising Edge Wake-up Enable Bit
     * |        |          |0 = GPC group pin rising edge wake-up function Disabled.
     * |        |          |1 = GPC group pin rising edge wake-up function Enabled.
     * |[2]     |PFWKEN    |Pin Falling Edge Wake-up Enable Bit
     * |        |          |0 = GPC group pin falling edge wake-up function Disabled.
     * |        |          |1 = GPC group pin falling edge wake-up function Enabled.
     * |[7:4]   |WKPSEL    |GPC Power-down Wake-up Pin Select
     * |        |          |1110 = GPC.14 wake-up function Enabled.
     * |        |          |Others are reserved
     * |[8]     |DBEN      |GPC Input Signal De-bounce Enable Bit
     * |        |          |The DBEN bit is used to enable the de-bounce function for each corresponding I/O
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger the wakeup.
     * |        |          |The de-bounce clock source is the 10 kHz internal low speed RC oscillator (LIRC).
     * |        |          |0 = power-down wake-up pin De-bounce function Disabled.
     * |        |          |1 = power-down wake-up pin De-bounce function Enabled.
     * |        |          |The de-bounce function is valid only for edge triggered.
     * @var CLK_T::IOPDCTL
     * Offset: 0xB0  GPIO Power-down Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |IOHR      |GPIO Hold Release
     * |        |          |When GPIO enter power-down mode, all I/O status are hold to keep normal operating status.
     * |        |          |After chip was waked up from power-down mode, the I/O are still keep hold status until user set this bit to release I/O hold status.
     * |        |          |Note: This bit is auto cleared by hardware.
     */

    __IO uint32_t PWRCTL;                /*!< [0x0000] System Power-down Control Register                               */
    __IO uint32_t AHBCLK;                /*!< [0x0004] AHB Devices Clock Enable Register                                */
    __IO uint32_t APBCLK0;               /*!< [0x0008] APB Devices Clock Enable Register 0                              */
    __IO uint32_t APBCLK1;               /*!< [0x000c] APB Devices Clock Enable Register 1                              */
    __IO uint32_t CLKSEL0;               /*!< [0x0010] Clock Source Select Register 0                                   */
    __IO uint32_t CLKSEL1;               /*!< [0x0014] Clock Source Select Register 1                                   */
    __IO uint32_t CLKSEL2;               /*!< [0x0018] Clock Source Select Register 2                                   */
    __I  uint32_t RESERVE0[1];
    __IO uint32_t CLKDIV0;               /*!< [0x0020] Clock Divider Number Register 0                                  */
    __I  uint32_t RESERVE1[4];
    __IO uint32_t PCLKDIV;               /*!< [0x0034] APB Clock Divider Register                                       */
    __I  uint32_t RESERVE2[6];
    __I  uint32_t STATUS;                /*!< [0x0050] Clock Status Monitor Register                                    */
    __I  uint32_t RESERVE3[3];
    __IO uint32_t CLKOCTL;               /*!< [0x0060] Clock Output Control Register                                    */
    __I  uint32_t RESERVE4[11];
    __IO uint32_t PMUCTL;                /*!< [0x0090] Power Manager Control Register                                   */
    __IO uint32_t PMUSTS;                /*!< [0x0094] Power Manager Status Register                                    */
    __I  uint32_t RESERVE5[1];
    __IO uint32_t SWKDBCTL;              /*!< [0x009c] Standby Power-down Wake-up De-bounce Control Register            */
    __I  uint32_t RESERVE6[1];
    __IO uint32_t PBSWKCTL;              /*!< [0x00a4] GPB Standby Power-down Wake-up Control Register                  */
    __IO uint32_t PCSWKCTL;              /*!< [0x00a8] GPC Standby Power-down Wake-up Control Register                  */
    __I  uint32_t RESERVE7[1];
    __IO uint32_t IOPDCTL;               /*!< [0x00b0] GPIO Standby Power-down Control Register                         */

} CLK_T;

/**
    @addtogroup CLK_CONST CLK Bit Field Definition
    Constant Definitions for CLK Controller
  @{
*/

#define CLK_PWRCTL_HIRCEN_Pos            (2)                                               /*!< CLK_T::PWRCTL: HIRCEN Position         */
#define CLK_PWRCTL_HIRCEN_Msk            (0x1ul << CLK_PWRCTL_HIRCEN_Pos)                  /*!< CLK_T::PWRCTL: HIRCEN Mask             */

#define CLK_PWRCTL_LIRCEN_Pos            (3)                                               /*!< CLK_T::PWRCTL: LIRCEN Position         */
#define CLK_PWRCTL_LIRCEN_Msk            (0x1ul << CLK_PWRCTL_LIRCEN_Pos)                  /*!< CLK_T::PWRCTL: LIRCEN Mask             */

#define CLK_PWRCTL_PDWKIEN_Pos           (5)                                               /*!< CLK_T::PWRCTL: PDWKIEN Position        */
#define CLK_PWRCTL_PDWKIEN_Msk           (0x1ul << CLK_PWRCTL_PDWKIEN_Pos)                 /*!< CLK_T::PWRCTL: PDWKIEN Mask            */

#define CLK_PWRCTL_PDWKIF_Pos            (6)                                               /*!< CLK_T::PWRCTL: PDWKIF Position         */
#define CLK_PWRCTL_PDWKIF_Msk            (0x1ul << CLK_PWRCTL_PDWKIF_Pos)                  /*!< CLK_T::PWRCTL: PDWKIF Mask             */

#define CLK_PWRCTL_PDEN_Pos              (7)                                               /*!< CLK_T::PWRCTL: PDEN Position           */
#define CLK_PWRCTL_PDEN_Msk              (0x1ul << CLK_PWRCTL_PDEN_Pos)                    /*!< CLK_T::PWRCTL: PDEN Mask               */

#define CLK_AHBCLK_FMCIDLE_Pos           (0)                                               /*!< CLK_T::AHBCLK: FMCIDLE Position        */
#define CLK_AHBCLK_FMCIDLE_Msk           (0x1ul << CLK_AHBCLK_FMCIDLE_Pos)                 /*!< CLK_T::AHBCLK: FMCIDLE Mask            */

#define CLK_AHBCLK_ISPCKEN_Pos           (1)                                               /*!< CLK_T::AHBCLK: ISPCKEN Position        */
#define CLK_AHBCLK_ISPCKEN_Msk           (0x1ul << CLK_AHBCLK_ISPCKEN_Pos)                 /*!< CLK_T::AHBCLK: ISPCKEN Mask            */

#define CLK_AHBCLK_STCKEN_Pos            (3)                                               /*!< CLK_T::AHBCLK: STCKEN Position         */
#define CLK_AHBCLK_STCKEN_Msk            (0x1ul << CLK_AHBCLK_STCKEN_Pos)                  /*!< CLK_T::AHBCLK: STCKEN Mask             */

#define CLK_AHBCLK_SRAM0CKEN_Pos         (8)                                               /*!< CLK_T::AHBCLK: SRAM0CKEN Position      */
#define CLK_AHBCLK_SRAM0CKEN_Msk         (0x1ul << CLK_AHBCLK_SRAM0CKEN_Pos)               /*!< CLK_T::AHBCLK: SRAM0CKEN Mask          */

#define CLK_AHBCLK_GPBCKEN_Pos           (17)                                              /*!< CLK_T::AHBCLK: GPBCKEN Position        */
#define CLK_AHBCLK_GPBCKEN_Msk           (0x1ul << CLK_AHBCLK_GPBCKEN_Pos)                 /*!< CLK_T::AHBCLK: GPBCKEN Mask            */

#define CLK_AHBCLK_GPCCKEN_Pos           (18)                                              /*!< CLK_T::AHBCLK: GPCCKEN Position        */
#define CLK_AHBCLK_GPCCKEN_Msk           (0x1ul << CLK_AHBCLK_GPCCKEN_Pos)                 /*!< CLK_T::AHBCLK: GPCCKEN Mask            */

#define CLK_AHBCLK_GPECKEN_Pos           (20)                                              /*!< CLK_T::AHBCLK: GPECKEN Position        */
#define CLK_AHBCLK_GPECKEN_Msk           (0x1ul << CLK_AHBCLK_GPECKEN_Pos)                 /*!< CLK_T::AHBCLK: GPECKEN Mask            */

#define CLK_AHBCLK_GPFCKEN_Pos           (21)                                              /*!< CLK_T::AHBCLK: GPFCKEN Position        */
#define CLK_AHBCLK_GPFCKEN_Msk           (0x1ul << CLK_AHBCLK_GPFCKEN_Pos)                 /*!< CLK_T::AHBCLK: GPFCKEN Mask            */

#define CLK_APBCLK0_WDTCKEN_Pos          (0)                                               /*!< CLK_T::APBCLK0: WDTCKEN Position       */
#define CLK_APBCLK0_WDTCKEN_Msk          (0x1ul << CLK_APBCLK0_WDTCKEN_Pos)                /*!< CLK_T::APBCLK0: WDTCKEN Mask           */

#define CLK_APBCLK0_TMR0CKEN_Pos         (2)                                               /*!< CLK_T::APBCLK0: TMR0CKEN Position      */
#define CLK_APBCLK0_TMR0CKEN_Msk         (0x1ul << CLK_APBCLK0_TMR0CKEN_Pos)               /*!< CLK_T::APBCLK0: TMR0CKEN Mask          */

#define CLK_APBCLK0_TMR1CKEN_Pos         (3)                                               /*!< CLK_T::APBCLK0: TMR1CKEN Position      */
#define CLK_APBCLK0_TMR1CKEN_Msk         (0x1ul << CLK_APBCLK0_TMR1CKEN_Pos)               /*!< CLK_T::APBCLK0: TMR1CKEN Mask          */

#define CLK_APBCLK0_TMR2CKEN_Pos         (4)                                               /*!< CLK_T::APBCLK0: TMR2CKEN Position      */
#define CLK_APBCLK0_TMR2CKEN_Msk         (0x1ul << CLK_APBCLK0_TMR2CKEN_Pos)               /*!< CLK_T::APBCLK0: TMR2CKEN Mask          */

#define CLK_APBCLK0_TMR3CKEN_Pos         (5)                                               /*!< CLK_T::APBCLK0: TMR3CKEN Position      */
#define CLK_APBCLK0_TMR3CKEN_Msk         (0x1ul << CLK_APBCLK0_TMR3CKEN_Pos)               /*!< CLK_T::APBCLK0: TMR3CKEN Mask          */

#define CLK_APBCLK0_CLKOCKEN_Pos         (6)                                               /*!< CLK_T::APBCLK0: CLKOCKEN Position      */
#define CLK_APBCLK0_CLKOCKEN_Msk         (0x1ul << CLK_APBCLK0_CLKOCKEN_Pos)               /*!< CLK_T::APBCLK0: CLKOCKEN Mask          */

#define CLK_APBCLK0_I2C0CKEN_Pos         (8)                                               /*!< CLK_T::APBCLK0: I2C0CKEN Position      */
#define CLK_APBCLK0_I2C0CKEN_Msk         (0x1ul << CLK_APBCLK0_I2C0CKEN_Pos)               /*!< CLK_T::APBCLK0: I2C0CKEN Mask          */

#define CLK_APBCLK0_UART0CKEN_Pos        (16)                                              /*!< CLK_T::APBCLK0: UART0CKEN Position     */
#define CLK_APBCLK0_UART0CKEN_Msk        (0x1ul << CLK_APBCLK0_UART0CKEN_Pos)              /*!< CLK_T::APBCLK0: UART0CKEN Mask         */

#define CLK_APBCLK0_UART1CKEN_Pos        (17)                                              /*!< CLK_T::APBCLK0: UART1CKEN Position     */
#define CLK_APBCLK0_UART1CKEN_Msk        (0x1ul << CLK_APBCLK0_UART1CKEN_Pos)              /*!< CLK_T::APBCLK0: UART1CKEN Mask         */

#define CLK_APBCLK0_ADCCKEN_Pos          (28)                                              /*!< CLK_T::APBCLK0: ADCCKEN Position       */
#define CLK_APBCLK0_ADCCKEN_Msk          (0x1ul << CLK_APBCLK0_ADCCKEN_Pos)                /*!< CLK_T::APBCLK0: ADCCKEN Mask           */

#define CLK_APBCLK1_USCI0CKEN_Pos        (8)                                               /*!< CLK_T::APBCLK1: USCI0CKEN Position     */
#define CLK_APBCLK1_USCI0CKEN_Msk        (0x1ul << CLK_APBCLK1_USCI0CKEN_Pos)              /*!< CLK_T::APBCLK1: USCI0CKEN Mask         */

#define CLK_APBCLK1_PWM0CKEN_Pos         (16)                                              /*!< CLK_T::APBCLK1: PWM0CKEN Position      */
#define CLK_APBCLK1_PWM0CKEN_Msk         (0x1ul << CLK_APBCLK1_PWM0CKEN_Pos)               /*!< CLK_T::APBCLK1: PWM0CKEN Mask          */

#define CLK_APBCLK1_ECAP0CKEN_Pos        (26)                                              /*!< CLK_T::APBCLK1: ECAP0CKEN Position     */
#define CLK_APBCLK1_ECAP0CKEN_Msk        (0x1ul << CLK_APBCLK1_ECAP0CKEN_Pos)              /*!< CLK_T::APBCLK1: ECAP0CKEN Mask         */

#define CLK_CLKSEL0_HCLKSEL_Pos          (0)                                               /*!< CLK_T::CLKSEL0: HCLKSEL Position       */
#define CLK_CLKSEL0_HCLKSEL_Msk          (0x7ul << CLK_CLKSEL0_HCLKSEL_Pos)                /*!< CLK_T::CLKSEL0: HCLKSEL Mask           */

#define CLK_CLKSEL0_STCLKSEL_Pos         (3)                                               /*!< CLK_T::CLKSEL0: STCLKSEL Position      */
#define CLK_CLKSEL0_STCLKSEL_Msk         (0x7ul << CLK_CLKSEL0_STCLKSEL_Pos)               /*!< CLK_T::CLKSEL0: STCLKSEL Mask          */

#define CLK_CLKSEL1_WDTSEL_Pos           (0)                                               /*!< CLK_T::CLKSEL1: WDTSEL Position        */
#define CLK_CLKSEL1_WDTSEL_Msk           (0x3ul << CLK_CLKSEL1_WDTSEL_Pos)                 /*!< CLK_T::CLKSEL1: WDTSEL Mask            */

#define CLK_CLKSEL1_TMR0SEL_Pos          (8)                                               /*!< CLK_T::CLKSEL1: TMR0SEL Position       */
#define CLK_CLKSEL1_TMR0SEL_Msk          (0x7ul << CLK_CLKSEL1_TMR0SEL_Pos)                /*!< CLK_T::CLKSEL1: TMR0SEL Mask           */

#define CLK_CLKSEL1_TMR1SEL_Pos          (12)                                              /*!< CLK_T::CLKSEL1: TMR1SEL Position       */
#define CLK_CLKSEL1_TMR1SEL_Msk          (0x7ul << CLK_CLKSEL1_TMR1SEL_Pos)                /*!< CLK_T::CLKSEL1: TMR1SEL Mask           */

#define CLK_CLKSEL1_TMR2SEL_Pos          (16)                                              /*!< CLK_T::CLKSEL1: TMR2SEL Position       */
#define CLK_CLKSEL1_TMR2SEL_Msk          (0x7ul << CLK_CLKSEL1_TMR2SEL_Pos)                /*!< CLK_T::CLKSEL1: TMR2SEL Mask           */

#define CLK_CLKSEL1_TMR3SEL_Pos          (20)                                              /*!< CLK_T::CLKSEL1: TMR3SEL Position       */
#define CLK_CLKSEL1_TMR3SEL_Msk          (0x7ul << CLK_CLKSEL1_TMR3SEL_Pos)                /*!< CLK_T::CLKSEL1: TMR3SEL Mask           */

#define CLK_CLKSEL1_CLKOSEL_Pos          (28)                                              /*!< CLK_T::CLKSEL1: CLKOSEL Position       */
#define CLK_CLKSEL1_CLKOSEL_Msk          (0x3ul << CLK_CLKSEL1_CLKOSEL_Pos)                /*!< CLK_T::CLKSEL1: CLKOSEL Mask           */

#define CLK_CLKSEL1_WWDTSEL_Pos          (30)                                              /*!< CLK_T::CLKSEL1: WWDTSEL Position       */
#define CLK_CLKSEL1_WWDTSEL_Msk          (0x3ul << CLK_CLKSEL1_WWDTSEL_Pos)                /*!< CLK_T::CLKSEL1: WWDTSEL Mask           */

#define CLK_CLKSEL2_PWM0SEL_Pos          (0)                                               /*!< CLK_T::CLKSEL2: PWM0SEL Position       */
#define CLK_CLKSEL2_PWM0SEL_Msk          (0x1ul << CLK_CLKSEL2_PWM0SEL_Pos)                /*!< CLK_T::CLKSEL2: PWM0SEL Mask           */

#define CLK_CLKSEL2_UART0SEL_Pos         (16)                                              /*!< CLK_T::CLKSEL2: UART0SEL Position      */
#define CLK_CLKSEL2_UART0SEL_Msk         (0x7ul << CLK_CLKSEL2_UART0SEL_Pos)               /*!< CLK_T::CLKSEL2: UART0SEL Mask          */

#define CLK_CLKSEL2_UART1SEL_Pos         (20)                                              /*!< CLK_T::CLKSEL2: UART1SEL Position      */
#define CLK_CLKSEL2_UART1SEL_Msk         (0x7ul << CLK_CLKSEL2_UART1SEL_Pos)               /*!< CLK_T::CLKSEL2: UART1SEL Mask          */

#define CLK_CLKSEL2_ADCSEL_Pos           (24)                                              /*!< CLK_T::CLKSEL2: ADCSEL Position        */
#define CLK_CLKSEL2_ADCSEL_Msk           (0x3ul << CLK_CLKSEL2_ADCSEL_Pos)                 /*!< CLK_T::CLKSEL2: ADCSEL Mask            */

#define CLK_CLKDIV0_HCLKDIV_Pos          (0)                                               /*!< CLK_T::CLKDIV0: HCLKDIV Position       */
#define CLK_CLKDIV0_HCLKDIV_Msk          (0xful << CLK_CLKDIV0_HCLKDIV_Pos)                /*!< CLK_T::CLKDIV0: HCLKDIV Mask           */

#define CLK_CLKDIV0_UART0DIV_Pos         (8)                                               /*!< CLK_T::CLKDIV0: UART0DIV Position      */
#define CLK_CLKDIV0_UART0DIV_Msk         (0xful << CLK_CLKDIV0_UART0DIV_Pos)               /*!< CLK_T::CLKDIV0: UART0DIV Mask          */

#define CLK_CLKDIV0_UART1DIV_Pos         (12)                                              /*!< CLK_T::CLKDIV0: UART1DIV Position      */
#define CLK_CLKDIV0_UART1DIV_Msk         (0xful << CLK_CLKDIV0_UART1DIV_Pos)               /*!< CLK_T::CLKDIV0: UART1DIV Mask          */

#define CLK_CLKDIV0_ADCDIV_Pos           (16)                                              /*!< CLK_T::CLKDIV0: ADCDIV Position        */
#define CLK_CLKDIV0_ADCDIV_Msk           (0xfful << CLK_CLKDIV0_ADCDIV_Pos)                /*!< CLK_T::CLKDIV0: ADCDIV Mask            */

#define CLK_PCLKDIV_APB0DIV_Pos          (0)                                               /*!< CLK_T::PCLKDIV: APB0DIV Position       */
#define CLK_PCLKDIV_APB0DIV_Msk          (0x7ul << CLK_PCLKDIV_APB0DIV_Pos)                /*!< CLK_T::PCLKDIV: APB0DIV Mask           */

#define CLK_PCLKDIV_APB1DIV_Pos          (4)                                               /*!< CLK_T::PCLKDIV: APB1DIV Position       */
#define CLK_PCLKDIV_APB1DIV_Msk          (0x7ul << CLK_PCLKDIV_APB1DIV_Pos)                /*!< CLK_T::PCLKDIV: APB1DIV Mask           */

#define CLK_PCLKDIV_APB2DIV_Pos          (8)                                               /*!< CLK_T::PCLKDIV: APB2DIV Position       */
#define CLK_PCLKDIV_APB2DIV_Msk          (0x7ul << CLK_PCLKDIV_APB2DIV_Pos)                /*!< CLK_T::PCLKDIV: APB2DIV Mask           */

#define CLK_STATUS_LIRCSTB_Pos           (3)                                               /*!< CLK_T::STATUS: LIRCSTB Position        */
#define CLK_STATUS_LIRCSTB_Msk           (0x1ul << CLK_STATUS_LIRCSTB_Pos)                 /*!< CLK_T::STATUS: LIRCSTB Mask            */

#define CLK_STATUS_HIRCSTB_Pos           (4)                                               /*!< CLK_T::STATUS: HIRCSTB Position        */
#define CLK_STATUS_HIRCSTB_Msk           (0x1ul << CLK_STATUS_HIRCSTB_Pos)                 /*!< CLK_T::STATUS: HIRCSTB Mask            */

#define CLK_STATUS_CLKSFAIL_Pos          (7)                                               /*!< CLK_T::STATUS: CLKSFAIL Position       */
#define CLK_STATUS_CLKSFAIL_Msk          (0x1ul << CLK_STATUS_CLKSFAIL_Pos)                /*!< CLK_T::STATUS: CLKSFAIL Mask           */

#define CLK_CLKOCTL_FREQSEL_Pos          (0)                                               /*!< CLK_T::CLKOCTL: FREQSEL Position       */
#define CLK_CLKOCTL_FREQSEL_Msk          (0xful << CLK_CLKOCTL_FREQSEL_Pos)                /*!< CLK_T::CLKOCTL: FREQSEL Mask           */

#define CLK_CLKOCTL_CLKOEN_Pos           (4)                                               /*!< CLK_T::CLKOCTL: CLKOEN Position        */
#define CLK_CLKOCTL_CLKOEN_Msk           (0x1ul << CLK_CLKOCTL_CLKOEN_Pos)                 /*!< CLK_T::CLKOCTL: CLKOEN Mask            */

#define CLK_CLKOCTL_DIV1EN_Pos           (5)                                               /*!< CLK_T::CLKOCTL: DIV1EN Position        */
#define CLK_CLKOCTL_DIV1EN_Msk           (0x1ul << CLK_CLKOCTL_DIV1EN_Pos)                 /*!< CLK_T::CLKOCTL: DIV1EN Mask            */

#define CLK_CLKOCTL_CLK1HZEN_Pos         (6)                                               /*!< CLK_T::CLKOCTL: CLK1HZEN Position      */
#define CLK_CLKOCTL_CLK1HZEN_Msk         (0x1ul << CLK_CLKOCTL_CLK1HZEN_Pos)               /*!< CLK_T::CLKOCTL: CLK1HZEN Mask          */

#define CLK_PMUCTL_PDMSEL_Pos            (0)                                               /*!< CLK_T::PMUCTL: PDMSEL Position         */
#define CLK_PMUCTL_PDMSEL_Msk            (0x7ul << CLK_PMUCTL_PDMSEL_Pos)                  /*!< CLK_T::PMUCTL: PDMSEL Mask             */

#define CLK_PMUCTL_WRBUSY_Pos            (7)                                               /*!< CLK_T::PMUCTL: WRBUSY Position         */
#define CLK_PMUCTL_WRBUSY_Msk            (0x1ul << CLK_PMUCTL_WRBUSY_Pos)                  /*!< CLK_T::PMUCTL: WRBUSY Mask             */

#define CLK_PMUCTL_WKTMREN_Pos           (8)                                               /*!< CLK_T::PMUCTL: WKTMREN Position        */
#define CLK_PMUCTL_WKTMREN_Msk           (0x1ul << CLK_PMUCTL_WKTMREN_Pos)                 /*!< CLK_T::PMUCTL: WKTMREN Mask            */

#define CLK_PMUCTL_WKTMRIS_Pos           (9)                                               /*!< CLK_T::PMUCTL: WKTMRIS Position        */
#define CLK_PMUCTL_WKTMRIS_Msk           (0x7ul << CLK_PMUCTL_WKTMRIS_Pos)                 /*!< CLK_T::PMUCTL: WKTMRIS Mask            */

#define CLK_PMUCTL_WKPINEN1_Pos          (24)                                              /*!< CLK_T::PMUCTL: WKPINEN1 Position       */
#define CLK_PMUCTL_WKPINEN1_Msk          (0x3ul << CLK_PMUCTL_WKPINEN1_Pos)                /*!< CLK_T::PMUCTL: WKPINEN1 Mask           */

#define CLK_PMUCTL_WKPINEN2_Pos          (26)                                              /*!< CLK_T::PMUCTL: WKPINEN2 Position       */
#define CLK_PMUCTL_WKPINEN2_Msk          (0x3ul << CLK_PMUCTL_WKPINEN2_Pos)                /*!< CLK_T::PMUCTL: WKPINEN2 Mask           */

#define CLK_PMUCTL_WKPINEN3_Pos          (28)                                              /*!< CLK_T::PMUCTL: WKPINEN3 Position       */
#define CLK_PMUCTL_WKPINEN3_Msk          (0x3ul << CLK_PMUCTL_WKPINEN3_Pos)                /*!< CLK_T::PMUCTL: WKPINEN3 Mask           */

#define CLK_PMUSTS_TMRWK_Pos             (1)                                               /*!< CLK_T::PMUSTS: TMRWK Position          */
#define CLK_PMUSTS_TMRWK_Msk             (0x1ul << CLK_PMUSTS_TMRWK_Pos)                   /*!< CLK_T::PMUSTS: TMRWK Mask              */

#define CLK_PMUSTS_PINWK1_Pos            (3)                                               /*!< CLK_T::PMUSTS: PINWK1 Position         */
#define CLK_PMUSTS_PINWK1_Msk            (0x1ul << CLK_PMUSTS_PINWK1_Pos)                  /*!< CLK_T::PMUSTS: PINWK1 Mask             */

#define CLK_PMUSTS_PINWK2_Pos            (4)                                               /*!< CLK_T::PMUSTS: PINWK2 Position         */
#define CLK_PMUSTS_PINWK2_Msk            (0x1ul << CLK_PMUSTS_PINWK2_Pos)                  /*!< CLK_T::PMUSTS: PINWK2 Mask             */

#define CLK_PMUSTS_PINWK3_Pos            (5)                                               /*!< CLK_T::PMUSTS: PINWK3 Position         */
#define CLK_PMUSTS_PINWK3_Msk            (0x1ul << CLK_PMUSTS_PINWK3_Pos)                  /*!< CLK_T::PMUSTS: PINWK3 Mask             */

#define CLK_PMUSTS_GPBWK_Pos             (9)                                               /*!< CLK_T::PMUSTS: GPBWK Position          */
#define CLK_PMUSTS_GPBWK_Msk             (0x1ul << CLK_PMUSTS_GPBWK_Pos)                   /*!< CLK_T::PMUSTS: GPBWK Mask              */

#define CLK_PMUSTS_GPCWK_Pos             (10)                                              /*!< CLK_T::PMUSTS: GPCWK Position          */
#define CLK_PMUSTS_GPCWK_Msk             (0x1ul << CLK_PMUSTS_GPCWK_Pos)                   /*!< CLK_T::PMUSTS: GPCWK Mask              */

#define CLK_PMUSTS_LVRWK_Pos             (12)                                              /*!< CLK_T::PMUSTS: LVRWK Position          */
#define CLK_PMUSTS_LVRWK_Msk             (0x1ul << CLK_PMUSTS_LVRWK_Pos)                   /*!< CLK_T::PMUSTS: LVRWK Mask              */

#define CLK_PMUSTS_BODWK_Pos             (13)                                              /*!< CLK_T::PMUSTS: BODWK Position          */
#define CLK_PMUSTS_BODWK_Msk             (0x1ul << CLK_PMUSTS_BODWK_Pos)                   /*!< CLK_T::PMUSTS: BODWK Mask              */

#define CLK_PMUSTS_CLRWK_Pos             (31)                                              /*!< CLK_T::PMUSTS: CLRWK Position          */
#define CLK_PMUSTS_CLRWK_Msk             (0x1ul << CLK_PMUSTS_CLRWK_Pos)                   /*!< CLK_T::PMUSTS: CLRWK Mask              */

#define CLK_SWKDBCTL_SWKDBCLKSEL_Pos     (0)                                               /*!< CLK_T::SWKDBCTL: SWKDBCLKSEL Position  */
#define CLK_SWKDBCTL_SWKDBCLKSEL_Msk     (0xful << CLK_SWKDBCTL_SWKDBCLKSEL_Pos)           /*!< CLK_T::SWKDBCTL: SWKDBCLKSEL Mask      */

#define CLK_PBSWKCTL_WKEN_Pos            (0)                                               /*!< CLK_T::PBSWKCTL: WKEN Position         */
#define CLK_PBSWKCTL_WKEN_Msk            (0x1ul << CLK_PBSWKCTL_WKEN_Pos)                  /*!< CLK_T::PBSWKCTL: WKEN Mask             */

#define CLK_PBSWKCTL_PRWKEN_Pos          (1)                                               /*!< CLK_T::PBSWKCTL: PRWKEN Position       */
#define CLK_PBSWKCTL_PRWKEN_Msk          (0x1ul << CLK_PBSWKCTL_PRWKEN_Pos)                /*!< CLK_T::PBSWKCTL: PRWKEN Mask           */

#define CLK_PBSWKCTL_PFWKEN_Pos          (2)                                               /*!< CLK_T::PBSWKCTL: PFWKEN Position       */
#define CLK_PBSWKCTL_PFWKEN_Msk          (0x1ul << CLK_PBSWKCTL_PFWKEN_Pos)                /*!< CLK_T::PBSWKCTL: PFWKEN Mask           */

#define CLK_PBSWKCTL_WKPSEL_Pos          (4)                                               /*!< CLK_T::PBSWKCTL: WKPSEL Position       */
#define CLK_PBSWKCTL_WKPSEL_Msk          (0xful << CLK_PBSWKCTL_WKPSEL_Pos)                /*!< CLK_T::PBSWKCTL: WKPSEL Mask           */

#define CLK_PBSWKCTL_DBEN_Pos            (8)                                               /*!< CLK_T::PBSWKCTL: DBEN Position         */
#define CLK_PBSWKCTL_DBEN_Msk            (0x1ul << CLK_PBSWKCTL_DBEN_Pos)                  /*!< CLK_T::PBSWKCTL: DBEN Mask             */

#define CLK_PCSWKCTL_WKEN_Pos            (0)                                               /*!< CLK_T::PCSWKCTL: WKEN Position         */
#define CLK_PCSWKCTL_WKEN_Msk            (0x1ul << CLK_PCSWKCTL_WKEN_Pos)                  /*!< CLK_T::PCSWKCTL: WKEN Mask             */

#define CLK_PCSWKCTL_PRWKEN_Pos          (1)                                               /*!< CLK_T::PCSWKCTL: PRWKEN Position       */
#define CLK_PCSWKCTL_PRWKEN_Msk          (0x1ul << CLK_PCSWKCTL_PRWKEN_Pos)                /*!< CLK_T::PCSWKCTL: PRWKEN Mask           */

#define CLK_PCSWKCTL_PFWKEN_Pos          (2)                                               /*!< CLK_T::PCSWKCTL: PFWKEN Position       */
#define CLK_PCSWKCTL_PFWKEN_Msk          (0x1ul << CLK_PCSWKCTL_PFWKEN_Pos)                /*!< CLK_T::PCSWKCTL: PFWKEN Mask           */

#define CLK_PCSWKCTL_WKPSEL_Pos          (4)                                               /*!< CLK_T::PCSWKCTL: WKPSEL Position       */
#define CLK_PCSWKCTL_WKPSEL_Msk          (0xful << CLK_PCSWKCTL_WKPSEL_Pos)                /*!< CLK_T::PCSWKCTL: WKPSEL Mask           */

#define CLK_PCSWKCTL_DBEN_Pos            (8)                                               /*!< CLK_T::PCSWKCTL: DBEN Position         */
#define CLK_PCSWKCTL_DBEN_Msk            (0x1ul << CLK_PCSWKCTL_DBEN_Pos)                  /*!< CLK_T::PCSWKCTL: DBEN Mask             */

#define CLK_IOPDCTL_IOHR_Pos             (0)                                               /*!< CLK_T::IOPDCTL: IOHR Position          */
#define CLK_IOPDCTL_IOHR_Msk             (0x1ul << CLK_IOPDCTL_IOHR_Pos)                   /*!< CLK_T::IOPDCTL: IOHR Mask              */

/**@}*/ /* CLK_CONST */
/**@}*/ /* end of CLK register group */
/**@}*/ /* end of REGISTER group */

#endif /* __CLK_REG_H__ */
