/*
 * uart.h - EUSART1 transmit-only debug output, 38400 8N1 on RC1.
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

#endif
