/*
 * uart.c - EUSART1: log output on RC1 (TP1); in the MODEM build also command
 * input on RC2.
 */

#include "board.h"
#include "uart.h"

void uart_init(void) {
    // 8N1, asynchronous, BRGH = 1, BRG16 = 1: baud = FOSC / (4 * (n + 1)).
    // RX: 38400 (n = 25, +0.16 %). MODEM: 9600 (n = 103, +0.16 %), a rate the
    // controller's SoftwareSerial sends reliably. At 4 MHz, 57600 would be +2.12 %
    // (DS Table 24-2), too close to the limit once HFINTOSC tolerance is added.
    BAUD1CON = 0b00001000;              // BRG16 = 1, SCKP = 0 (TX idles high)
    SP1BRGH  = 0;
    SP1BRGL  = UART_SPBRG;
    TX1STA   = 0b00100100;              // TXEN = 1, SYNC = 0, BRGH = 1
#if defined(ROLE_MODEM)
    RC1STA   = 0b10010000;              // SPEN = 1, CREN = 1 (receiver on)
#else
    RC1STA   = 0b10000000;              // SPEN = 1 (receiver left disabled)
#endif
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

#if defined(ROLE_MODEM)
// Discard anything in the receive FIFO and clear an overrun.
void uart_rx_flush(void) {
    if (RC1STAbits.OERR) {
        RC1STAbits.CREN = 0;            // Clearing CREN clears the overrun
        RC1STAbits.CREN = 1;
    }
    while (PIR1bits.RC1IF) {
        (void)RC1REG;
    }
}

// Wait up to 'ms' for a byte. Returns the byte, or -1 on timeout. Bytes with a
// framing error are discarded.
int16_t uart_getc_timeout(uint16_t ms) {
    for (uint16_t i = 0; i < ms; i++) {
        for (uint8_t j = 0; j < 100; j++) {     // ~1 ms in 10 us steps
            if (RC1STAbits.OERR) {
                RC1STAbits.CREN = 0;
                RC1STAbits.CREN = 1;
            }
            if (PIR1bits.RC1IF) {
                uint8_t ferr = RC1STAbits.FERR;   // Read FERR before RC1REG
                uint8_t c = RC1REG;
                if (!ferr) {
                    return c;
                }
            }
            __delay_us(10);
        }
    }
    return -1;
}
#endif
