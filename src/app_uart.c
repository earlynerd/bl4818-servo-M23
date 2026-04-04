/*
 * UART1 Driver for Nuvoton M2003 (Ring Protocol Interface)
 *
 * This ports the interrupt-driven ring buffer logic from the MS51.
 */

#include <stdint.h>
#include "m2003_config.h"
#include "M2003.h"

#define UART_RX_BUF_SIZE 128
#define UART_TX_BUF_SIZE 128

static volatile uint8_t rx_buf[UART_RX_BUF_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;

static volatile uint8_t tx_buf[UART_TX_BUF_SIZE];
static volatile uint16_t tx_head = 0;
static volatile uint16_t tx_tail = 0;
static volatile uint8_t tx_running = 0;

void UART1_IRQHandler(void)
{
    uint32_t status = UART1->INTSTS;

    /* ── Receive Interrupt ─────────────────────────────────────────── */
    if (status & (UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk)) {
        while (!(UART1->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk)) {
            uint8_t c = (uint8_t)UART1->DAT;
            uint16_t next = (rx_head + 1) % UART_RX_BUF_SIZE;
            if (next != rx_tail) {
                rx_buf[rx_head] = c;
                rx_head = next;
            }
        }
    }

    /* ── Transmit Interrupt ────────────────────────────────────────── */
    if (status & UART_INTSTS_THREINT_Msk) {
        if (tx_head != tx_tail) {
            /* Fill the 16-byte hardware FIFO */
            while (!(UART1->FIFOSTS & UART_FIFOSTS_TXFULL_Msk) && (tx_head != tx_tail)) {
                UART1->DAT = tx_buf[tx_tail];
                tx_tail = (tx_tail + 1) % UART_TX_BUF_SIZE;
            }
        } else {
            /* Buffer empty, stop TX interrupt */
            UART1->INTEN &= ~UART_INTEN_THREIEN_Msk;
            tx_running = 0;
        }
    }
}

void uart_init(uint32_t baud)
{
    /* 1. Enable UART1 Clock and Pins */
    CLK->APBCLK0 |= CLK_APBCLK0_UART1CKEN_Msk;
    CLK->CLKSEL2 = (CLK->CLKSEL2 & ~CLK_CLKSEL2_UART1SEL_Msk) | CLK_CLKSEL2_UART1SEL_HIRC;

    /* Route UART1 to PF.0 (TX) and PF.1 (RX) to avoid PWM pin conflicts */
    SYS->GPF_MFPL = (SYS->GPF_MFPL & ~SYS_GPF_MFPL_PF0MFP_Msk) | SYS_GPF_MFPL_PF0MFP_UART1_TXD;
    SYS->GPF_MFPL = (SYS->GPF_MFPL & ~SYS_GPF_MFPL_PF1MFP_Msk) | SYS_GPF_MFPL_PF1MFP_UART1_RXD;

    /* 2. Configure UART Line Settings (8N1) */
    UART1->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;

    /* 3. Set Baud Rate */
    /* MODE2 calculation: Baud = UART_CLK / (BAUD_DIV + 2) */
    UART1->BAUD = UART_BAUD_BAUDM1_Msk | UART_BAUD_BAUDM0_Msk | (((24000000UL) / baud) - 2);

    /* 4. Reset FIFOs and Set Thresholds */
    UART1->FIFO = UART_FIFO_RFITL_8BYTES;
    UART1->FIFO |= (UART_FIFO_RXRST_Msk | UART_FIFO_TXRST_Msk);

    /* 5. Enable Interrupts */
    UART1->INTEN |= (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
    NVIC_EnableIRQ(UART1_IRQn);
}

void uart_putc(uint8_t c)
{
    uint16_t next = (tx_head + 1) % UART_TX_BUF_SIZE;

    // Wait for space in buffer
    while (next == tx_tail);

    __disable_irq();
    tx_buf[tx_head] = c;
    tx_head = next;

    if (!tx_running) {
        tx_running = 1;
        UART1->INTEN |= UART_INTEN_THREIEN_Msk;
    }
    __enable_irq();
}

void uart_puts(const char *str)
{
    while (*str) {
        uart_putc((uint8_t)*str++);
    }
}

uint8_t uart_getc(void)
{
    uint8_t c = 0;
    if (rx_head != rx_tail) {
        c = rx_buf[rx_tail];
        rx_tail = (rx_tail + 1) % UART_RX_BUF_SIZE;
    }
    return c;
}

uint8_t uart_available(void)
{
    return (rx_head != rx_tail);
}

void uart_rx_flush(void)
{
    __disable_irq();
    rx_head = rx_tail;
    UART1->FIFO |= UART_FIFO_RXRST_Msk;
    __enable_irq();
}
