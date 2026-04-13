/*
 * UART1 Driver for Nuvoton M2003 (Ring Protocol Interface)
 */

#include <stdint.h>
#include "m2003_config.h"
#include "M2003.h"
#include "app_uart.h"

#define UART_RX_BUF_SIZE 256
#define UART_TX_BUF_SIZE 256

static volatile uint8_t rx_buf[UART_RX_BUF_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;
static volatile uint8_t rx_overflow = 0;

static volatile uint8_t tx_buf[UART_TX_BUF_SIZE];
static volatile uint16_t tx_head = 0;
static volatile uint16_t tx_tail = 0;
static volatile uint8_t tx_running = 0;

static volatile uint8_t echo_enabled = 0;

static uint32_t irq_save(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void irq_restore(uint32_t primask)
{
    if ((primask & 1u) == 0u) {
        __enable_irq();
    }
}

void UART1_IRQHandler(void)
{
    uint32_t status = UART1->INTSTS;

    if (status & UART_INTSTS_RLSINT_Msk) {
        UART1->FIFOSTS = UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk;
        UART1->FIFOSTS = UART_FIFOSTS_ADDRDETF_Msk;
    }

    if (status & UART_INTSTS_BUFERRINT_Msk) {
        rx_overflow = 1u;
        UART1->FIFOSTS = UART_FIFOSTS_RXOVIF_Msk | UART_FIFOSTS_TXOVIF_Msk;
    }

    /* ── Receive Interrupt ─────────────────────────────────────────── */
    if (status & (UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk)) {
        while (!(UART1->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk)) {
            uint8_t c = (uint8_t)UART1->DAT;

            /* Cut-through: echo RX to TX at ISR speed */
            if (echo_enabled) {
                while (UART1->FIFOSTS & UART_FIFOSTS_TXFULL_Msk) { }
                UART1->DAT = c;
            }

            uint16_t next = (rx_head + 1) % UART_RX_BUF_SIZE;
            if (next != rx_tail) {
                rx_buf[rx_head] = c;
                rx_head = next;
            } else {
                rx_overflow = 1u;
            }
        }
    }

    /* ── Transmit Interrupt ────────────────────────────────────────── */
    if (status & UART_INTSTS_THREINT_Msk) {
        if (tx_head != tx_tail) {
            while (!(UART1->FIFOSTS & UART_FIFOSTS_TXFULL_Msk) && (tx_head != tx_tail)) {
                UART1->DAT = tx_buf[tx_tail];
                tx_tail = (tx_tail + 1) % UART_TX_BUF_SIZE;
            }
        } else {
            UART1->INTEN &= ~UART_INTEN_THREIEN_Msk;
            tx_running = 0;
        }
    }

}

void uart_init(uint32_t baud)
{
    CLK->APBCLK0 |= CLK_APBCLK0_UART1CKEN_Msk;
    CLK->CLKSEL2 = (CLK->CLKSEL2 & ~CLK_CLKSEL2_UART1SEL_Msk) | CLK_CLKSEL2_UART1SEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_UART1DIV_Msk) | CLK_CLKDIV0_UART1(1);
    SYS_ResetModule(UART1_RST);

    SYS->GPF_MFPL = (SYS->GPF_MFPL & ~SYS_GPF_MFPL_PF0MFP_Msk) | SYS_GPF_MFPL_PF0MFP_UART1_TXD;
    SYS->GPF_MFPL = (SYS->GPF_MFPL & ~SYS_GPF_MFPL_PF1MFP_Msk) | SYS_GPF_MFPL_PF1MFP_UART1_RXD;
    SYS->GPF_MFOS &= ~(SYS_GPF_MFOS_PF0MFOS_Msk | SYS_GPF_MFOS_PF1MFOS_Msk);

    GPIO_SetMode(PF, BIT0, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PF, BIT1, GPIO_MODE_INPUT);
    PF->PUSEL &= ~(GPIO_PUSEL_PUSEL0_Msk | GPIO_PUSEL_PUSEL1_Msk);
    GPIO_DISABLE_DOUT_MASK(PF, BIT0 | BIT1);
    GPIO_ENABLE_DIGITAL_PATH(PF, BIT1);
    GPIO_DISABLE_DEBOUNCE(PF, BIT1);
    PF->SMTEN &= ~GPIO_SMTEN_SMTEN1_Msk;

    UART_Open(UART1, baud);
    UART_SetLine_Config(UART1, baud, UART_WORD_LEN_8, UART_PARITY_NONE, UART_STOP_BIT_1);
    rx_head = 0u;
    rx_tail = 0u;
    tx_head = 0u;
    tx_tail = 0u;
    tx_running = 0u;
    rx_overflow = 0u;
    echo_enabled = 0u;

    UART1->FIFO = (UART1->FIFO & ~UART_FIFO_RFITL_Msk) | UART_FIFO_RFITL_1BYTE;
    UART1->FIFO |= (UART_FIFO_RXRST_Msk | UART_FIFO_TXRST_Msk);

    UART1->INTEN |= (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk |
                     UART_INTEN_RLSIEN_Msk | UART_INTEN_BUFERRIEN_Msk);
    NVIC_SetPriority(UART1_IRQn, 2u);
    NVIC_EnableIRQ(UART1_IRQn);
}

void uart_putc(uint8_t c)
{
    uint16_t next = (tx_head + 1) % UART_TX_BUF_SIZE;
    uint32_t primask;

    while (next == tx_tail);

    primask = irq_save();
    tx_buf[tx_head] = c;
    tx_head = next;

    if (!tx_running) {
        tx_running = 1;
        UART1->DAT = tx_buf[tx_tail];
        tx_tail = (tx_tail + 1) % UART_TX_BUF_SIZE;
        UART1->INTEN |= UART_INTEN_THREIEN_Msk;
    }
    irq_restore(primask);
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
    return (rx_head != rx_tail) ? 1u : 0u;
}

void uart_echo_enable(void)
{
    uint32_t primask = irq_save();
    echo_enabled = 1u;
    irq_restore(primask);
}

void uart_echo_disable(void)
{
    uint32_t primask = irq_save();
    echo_enabled = 0u;
    irq_restore(primask);
}

uint8_t uart_tx_busy(void)
{
    return (tx_running || (tx_head != tx_tail)) ? 1u : 0u;
}

void uart_tx_flush(void)
{
    while (uart_tx_busy());
}

void uart_rx_flush(void)
{
    uint32_t primask = irq_save();
    rx_head = rx_tail;
    rx_overflow = 0u;
    UART1->FIFO |= UART_FIFO_RXRST_Msk;
    irq_restore(primask);
}

uint8_t uart_rx_overflowed(void)
{
    return rx_overflow;
}

void uart_rx_clear_overflow(void)
{
    uint32_t primask = irq_save();
    rx_overflow = 0u;
    irq_restore(primask);
}
