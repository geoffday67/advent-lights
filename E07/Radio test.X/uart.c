/*
 * uart.c - EUSART1 transmit-only debug output.
 */

#include "board.h"
#include "uart.h"

void uart_init(void) {
    // 38400 baud, 8N1, transmit only. Asynchronous, BRGH = 1, BRG16 = 1:
    // baud = FOSC / (4 * (n + 1)) = 4 MHz / (4 * 26) = 38462 (+0.16 %).
    // At 4 MHz the next standard rate up, 57600, is +2.12 % (DS Table 24-2),
    // too close to the limit once HFINTOSC tolerance is added.
    BAUD1CON = 0b00001000;              // BRG16 = 1, SCKP = 0 (TX idles high)
    SP1BRGH  = 0;
    SP1BRGL  = 25;
    TX1STA   = 0b00100100;              // TXEN = 1, SYNC = 0, BRGH = 1
    RC1STA   = 0b10000000;              // SPEN = 1 (receiver left disabled)
}

void uart_putc(char c) {
    while (!PIR1bits.TX1IF);            // Wait for TX1REG to be empty
    TX1REG = c;
}

void uart_puts(const char *s) {
    while (*s) {
        uart_putc(*s++);
    }
}

void uart_puthex(uint8_t v) {
    static const char hex[] = "0123456789ABCDEF";
    uart_puts("0x");
    uart_putc(hex[v >> 4]);
    uart_putc(hex[v & 0x0F]);
}


void uart_putdec(int16_t v) {
    char buf[6];
    uint8_t i = 0;
    uint16_t u;

    if (v < 0) {
        uart_putc('-');
        u = (uint16_t)(-v);
    } else {
        u = (uint16_t)v;
    }
    do {
        buf[i++] = (char)('0' + u % 10);
        u /= 10;
    } while (u);
    while (i) {
        uart_putc(buf[--i]);
    }
}

// Wait until the last character has completely left the shift register.
// Needed before SLEEP, which stops the clock mid-character otherwise.
void uart_flush(void) {
    while (!TX1STAbits.TRMT);
}
