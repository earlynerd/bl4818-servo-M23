/*
 * UART1 Driver
 */
#ifndef UART_H
#define UART_H

#include <stdint.h>

void uart_init(uint32_t baudrate);
void uart_putc(uint8_t c);
void uart_puts(const char *s);
uint8_t uart_available(void);
uint8_t uart_getc(void);
void uart_rx_flush(void);

#endif /* UART_H */
