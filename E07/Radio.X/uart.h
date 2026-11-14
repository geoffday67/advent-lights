/*
 * uart.h - EUSART1: log output on RC1 (38400 RX build, 9600 MODEM build);
 * MODEM build also receives commands on RC2.
 */
#ifndef UART_H
#define UART_H

#include <stdint.h>

void uart_init(void);
void uart_putc(char c);
void uart_puts(const char *s);
void uart_puthex(uint8_t v);
void uart_putdec(int16_t v);
void uart_flush(void);

#if defined(ROLE_MODEM)
void    uart_rx_flush(void);
int16_t uart_getc_timeout(uint16_t ms);
#endif

#endif
