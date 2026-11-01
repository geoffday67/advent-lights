/*
 * tx_main.c - Transmitter test: send a long-preamble 'GD' packet on each button press.
 * Built in the TX configuration only.
 */

#include "board.h"
#include "uart.h"
#include "radio.h"

// Preamble must outlast the receiver's WOR polling interval (t_EVENT0) plus its
// wake-up and settling time. 300 ms = 1.2 x a 250 ms poll interval.
#define PREAMBLE_MS 300

static void radio_send_test(void) {
    static const uint8_t packet[] = { 2, 'G', 'D' };   // Length byte, then payload

    radio_strobe(CC1101_SIDLE);
    radio_wait_state(STATE_IDLE);
    radio_strobe(CC1101_SFTX);              // Empty TX FIFO

    // STX with the FIFO empty: after calibration the modulator sends preamble
    // and keeps sending it until the first byte is written to the TX FIFO
    // (datasheet Section 15.2).
    radio_strobe(CC1101_STX);
    if (!radio_wait_state(STATE_TX)) {
        uart_puts("TX: did not enter TX\r\n");
        radio_strobe(CC1101_SIDLE);
        return;
    }

    // PLL lock check: FSCAL1 = 0x3F after calibration means no lock (Section 22.1).
    if (radio_read_reg(CC1101_FSCAL1) == 0x3F) {
        uart_puts("TX: PLL not locked\r\n");
        radio_strobe(CC1101_SIDLE);
        return;
    }

    __delay_ms(PREAMBLE_MS);

    // Length byte then payload in one burst. Writing the first byte ends the
    // preamble; the remaining bytes follow well before the 4 sync bytes are sent.
    radio_write_burst(CC1101_TXFIFO, packet, sizeof packet);

    // MCSM1.TXOFF_MODE = IDLE (default), so the chip returns to IDLE after the packet.
    if (radio_wait_state(STATE_IDLE)) {
        uart_puts("TX: sent GD\r\n");
    } else {
        uart_puts("TX: did not finish (underflow or timeout)\r\n");
        radio_strobe(CC1101_SIDLE);
        radio_strobe(CC1101_SFTX);
    }
}

#define DEBOUNCE_MS 30                 // Input must be stable this long to be accepted

void main(void) {
    clock_init();
    pins_init();
    uart_init();
    spi_init();

    radio_check();

    uart_puts("CC1101 config: ");
    uint8_t errors = radio_configure();
    if (errors == 0) {
        uart_puts("OK\r\n");
    } else {
        uart_puts("readback errors\r\n");
    }

    uint8_t pressed = 0;                // Debounced state, 1 = pressed
    uint8_t count = 0;                  // ms the raw input has differed from the debounced state

    while (1) {
        __delay_ms(1);

        uint8_t raw = (BUTTON == 0);    // Active low
        if (raw == pressed) {
            count = 0;                  // Agrees with debounced state; any bounce restarts the count
        } else if (++count >= DEBOUNCE_MS) {
            count = 0;
            pressed = raw;
            if (pressed) {
                radio_send_test();      // Once per press, on the debounced press edge
            }
        }
    }
}
