#include <stdint.h>
#include "M2003.h"
#include "bl_internal.h"

#define BL_RX_SIZE 128u
#define BL_RX_MASK (BL_RX_SIZE - 1u)

static volatile uint8_t rx_buffer[BL_RX_SIZE];
static volatile uint8_t rx_head;
static volatile uint8_t rx_tail;
static volatile uint8_t rx_overflow;
static volatile uint8_t echo_enabled;

void UART1_IRQHandler(void)
{
    uint32_t status = UART1->INTSTS;

    if ((status & UART_INTSTS_RLSINT_Msk) != 0u) {
        UART1->FIFOSTS = UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk |
                         UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_ADDRDETF_Msk;
    }
    if ((status & UART_INTSTS_BUFERRINT_Msk) != 0u) {
        UART1->FIFOSTS = UART_FIFOSTS_RXOVIF_Msk | UART_FIFOSTS_TXOVIF_Msk;
        rx_overflow = 1u;
    }
    if ((status & (UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk)) != 0u) {
        while ((UART1->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0u) {
            uint8_t value = (uint8_t)UART1->DAT;
            uint8_t next;

            if (echo_enabled != 0u) {
                while ((UART1->FIFOSTS & UART_FIFOSTS_TXFULL_Msk) != 0u) {
                }
                UART1->DAT = value;
            }

            next = (uint8_t)((rx_head + 1u) & BL_RX_MASK);
            if (next == rx_tail) {
                rx_overflow = 1u;
            } else {
                rx_buffer[rx_head] = value;
                rx_head = next;
            }
        }
    }
}

void bl_uart_init(void)
{
    CLK->APBCLK0 |= CLK_APBCLK0_UART1CKEN_Msk;
    CLK->CLKSEL2 = (CLK->CLKSEL2 & ~CLK_CLKSEL2_UART1SEL_Msk) |
                   CLK_CLKSEL2_UART1SEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_UART1DIV_Msk) |
                   CLK_CLKDIV0_UART1(1);

    SYS->IPRST1 |= SYS_IPRST1_UART1RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART1RST_Msk;
    SYS->GPF_MFPL = (SYS->GPF_MFPL &
                     ~(SYS_GPF_MFPL_PF0MFP_Msk | SYS_GPF_MFPL_PF1MFP_Msk)) |
                    SYS_GPF_MFPL_PF0MFP_UART1_TXD |
                    SYS_GPF_MFPL_PF1MFP_UART1_RXD;
    SYS->GPF_MFOS &= ~(SYS_GPF_MFOS_PF0MFOS_Msk |
                       SYS_GPF_MFOS_PF1MFOS_Msk);

    /* Do not inherit application GPIO state across a software reset. */
    PF->MODE = (PF->MODE &
                ~(GPIO_MODE_MODE0_Msk | GPIO_MODE_MODE1_Msk)) |
               (GPIO_MODE_OUTPUT << GPIO_MODE_MODE0_Pos);
    PF->PUSEL &= ~(GPIO_PUSEL_PUSEL0_Msk | GPIO_PUSEL_PUSEL1_Msk);
    PF->DINOFF &= ~GPIO_DINOFF_DINOFF1_Msk;
    PF->SMTEN &= ~GPIO_SMTEN_SMTEN1_Msk;

    UART1->FUNCSEL = UART_FUNCSEL_UART;
    UART1->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
    UART1->BAUD = UART_BAUD_MODE2 |
                  UART_BAUD_MODE2_DIVIDER(24000000UL, BL_UART_BAUD);
    UART1->FIFO = UART_FIFO_RFITL_1BYTE | UART_FIFO_RXRST_Msk |
                  UART_FIFO_TXRST_Msk;

    rx_head = 0u;
    rx_tail = 0u;
    rx_overflow = 0u;
    echo_enabled = 1u;

    UART1->INTEN = UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk |
                   UART_INTEN_RLSIEN_Msk | UART_INTEN_BUFERRIEN_Msk;
    NVIC_SetPriority(UART1_IRQn, 3u);
    NVIC_EnableIRQ(UART1_IRQn);
}

void bl_uart_echo_enable(void)
{
    echo_enabled = 1u;
}

void bl_uart_echo_disable(void)
{
    echo_enabled = 0u;
}

void bl_uart_disable(void)
{
    NVIC_DisableIRQ(UART1_IRQn);
    UART1->INTEN = 0u;
}

uint8_t bl_uart_available(void)
{
    return (rx_head != rx_tail) ? 1u : 0u;
}

uint8_t bl_uart_getc(void)
{
    uint8_t value;

    while (rx_head == rx_tail) {
    }
    value = rx_buffer[rx_tail];
    rx_tail = (uint8_t)((rx_tail + 1u) & BL_RX_MASK);
    return value;
}

uint8_t bl_uart_overflowed(void)
{
    return rx_overflow;
}

void bl_uart_clear_overflow(void)
{
    rx_tail = rx_head;
    rx_overflow = 0u;
}

void bl_uart_putc(uint8_t value)
{
    while ((UART1->FIFOSTS & UART_FIFOSTS_TXFULL_Msk) != 0u) {
    }
    UART1->DAT = value;
}

void bl_uart_write(const uint8_t *data, uint32_t length)
{
    while (length-- != 0u)
        bl_uart_putc(*data++);
}

void bl_uart_flush(void)
{
    while ((UART1->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk) == 0u) {
    }
}
