#include <xc.h>
#include "serial.h"

#define TX_BUFFER_SIZE 64

static volatile char     tx_buf[TX_BUFFER_SIZE];
static volatile uint8_t  tx_head = 0;   // next write position
static volatile uint8_t  tx_tail = 0;   // next read position

void serial_init(void) {
    // Route TX1 output to RA5 via PPS (TX1/CK1 = 0x05).
    RA5PPS = 0x05;

    // 9600 baud at 1MHz: BRGH=1, BRG16=1, SPBRG=25
    // Actual = 1,000,000 / (4 * 26) = 9615 baud (0.16% error)
    SP1BRGH = 0;
    SP1BRGL = 25;

    BAUD1CON = 0b00001000;  // BRG16=1
    TX1STA   = 0b00100100;  // TXEN=1, BRGH=1, SYNC=0 (async)
    RC1STA   = 0b10000000;  // SPEN=1

    // Leave TX1IE disabled until data is queued.
    PIE1bits.TX1IE  = 0;
    INTCONbits.PEIE = 1;    // Enable peripheral interrupts.
    INTCONbits.GIE  = 1;    // Enable global interrupts.
}

void serial_print(const char *s) {
    while (*s) {
        uint8_t next = (tx_head + 1) % TX_BUFFER_SIZE;
        while (next == tx_tail);            // Block if buffer is full.
        tx_buf[tx_head] = *s++;
        tx_head = next;
    }
    PIE1bits.TX1IE = 1;                     // Kick the ISR to start sending.
}

void serial_flush(void) {
    while (tx_head != tx_tail);   // Wait for ring buffer to drain via ISR.
    while (!TX1STAbits.TRMT);     // Wait for shift register to finish.
}

// Call from the ISR when TX1IF is set.
// TX1IF is cleared automatically by writing to TX1REG.
void serial_tx_isr(void) {
    if (PIR1bits.TX1IF && PIE1bits.TX1IE) {
        if (tx_tail != tx_head) {
            TX1REG = tx_buf[tx_tail];
            tx_tail = (tx_tail + 1) % TX_BUFFER_SIZE;
        } else {
            PIE1bits.TX1IE = 0;             // Buffer empty, disable interrupt.
        }
    }
}
