/*
 * modem_main.c - Radio modem for the controller. Built in the MODEM configuration only.
 *
 * The controller sends frames over a one-way serial link into RC2 (J3 pin 1, through
 * R3), 9600 8N1. Each valid frame's payload is transmitted with a long preamble so the
 * wake-on-radio receivers catch it. Progress is logged on RC1 (TP1) at 9600.
 *
 * Frame:   'S', len (1..MAX_PAYLOAD), payload[len], checksum
 *          checksum = (len + sum of payload bytes) & 0xFF
 *
 * Power: between frames the CC1101 is in SLEEP (SPWD) and the PIC in Sleep. The PIC
 * wakes on the EUSART's auto-wake-up on break (datasheet Section 24.3.3): with WUE set,
 * the falling edge of a start bit sets RC1IF and wakes the PIC; WUE clears itself on
 * the next rising edge, and the receiver then takes the following bytes normally.
 * The wake character is consumed and must be all zeros, so the controller sends
 * 0x00 and then the frame straight after it, with no gap needed.
 */

#include "board.h"
#include "uart.h"
#include "radio.h"
#include "selftest.h"

// Preamble must outlast the receivers' WOR polling interval (t_EVENT0 = 250 ms) plus
// their wake-up and settling time.
#define PREAMBLE_MS     300

// Test: 1 = ignore the serial input and send the 'GD' test packet every
// TEST_PERIOD_MS, logging each send on TP1. 0 = normal modem.
#define MODEM_TEST_SEND 0
#define TEST_PERIOD_MS  3000    // 300 ms preamble every 3 s = 10 %, the 433 MHz duty-cycle limit

// Debug: 1 = put the CC1101 in SLEEP (SPWD) between sends (normal, lowest power).
// 0 = leave it in IDLE, to check whether waking from SLEEP is the problem.
#define MODEM_SLEEP_RADIO 1

#define FRAME_START     'S'
#define MAX_PAYLOAD     32
#define WAKE_END_MS     5       // The 0x00 wake byte is low for 9 bit times (0.94 ms at 9600)
#define FIRST_BYTE_MS   50      // Time allowed after the wake byte for the frame to start
#define NEXT_BYTE_MS    5       // Gap allowed between bytes within a frame

// Transmit 'len' payload bytes as one variable-length packet with a long preamble.
static uint8_t radio_send(const uint8_t *payload, uint8_t len) {
    static uint8_t buf[MAX_PAYLOAD + 1];

    radio_strobe(CC1101_SIDLE);             // Also wakes the radio from SLEEP (CSn low)
    radio_wait_state(STATE_IDLE);
    radio_strobe(CC1101_SFTX);              // Empty TX FIFO

    // STX with the FIFO empty: after calibration the modulator sends preamble and keeps
    // sending it until the first byte is written to the TX FIFO (datasheet Section 15.2).
    radio_strobe(CC1101_STX);
    if (!radio_wait_state(STATE_TX)) {
        // Diagnostics: chip status byte, MARCSTATE (0x13 = TX, 0x16 = TX FIFO
        // underflow, 0x01 = IDLE), and PKTCTRL0, which reads 0x45 (its reset value)
        // instead of 0x05 if the radio has been reset, e.g. by a supply dip.
        uint8_t status = radio_strobe(CC1101_SNOP);
        uart_puts("MODEM: did not enter TX, status ");
        uart_puthex(status);
        uart_puts(" MARCSTATE ");
        uart_puthex(radio_read_status(CC1101_MARCSTATE) & 0x1F);
        uart_puts(" PKTCTRL0 ");
        uart_puthex(radio_read_reg(CC1101_PKTCTRL0));
        // CSn should be high here (LATC3 = 1). The pin reading shows whether it
        // actually gets there: 0 means something is holding the line low.
        uart_puts(" CS pin ");
        uart_putc(PORTCbits.RC3 ? '1' : '0');
        uart_puts("\r\n");
        radio_strobe(CC1101_SIDLE);
        return 0;
    }

    // PLL lock check: FSCAL1 = 0x3F after calibration means no lock (Section 22.1).
    if (radio_read_reg(CC1101_FSCAL1) == 0x3F) {
        uart_puts("MODEM: PLL not locked\r\n");
        radio_strobe(CC1101_SIDLE);
        return 0;
    }

    __delay_ms(PREAMBLE_MS);

    // Length byte then payload in one burst. Writing the first byte ends the preamble.
    buf[0] = len;
    for (uint8_t i = 0; i < len; i++) {
        buf[i + 1] = payload[i];
    }
    radio_write_burst(CC1101_TXFIFO, buf, len + 1);

    // MCSM1.TXOFF_MODE = IDLE (default), so the chip returns to IDLE after the packet.
    if (radio_wait_state(STATE_IDLE)) {
        return 1;
    }
    uart_puts("MODEM: TX did not finish (underflow or timeout)\r\n");
    radio_strobe(CC1101_SIDLE);
    radio_strobe(CC1101_SFTX);
    return 0;
}

// The command line is low but no wake byte ended: the controller is off or holding
// the line low. Sleep until the line goes high, woken by its rising edge through
// interrupt-on-change, with the receiver off so framing errors cannot keep waking
// the PIC. If the line rises between the pin check and SLEEP, the IOC flag is
// already set and SLEEP returns at once.
static void wait_line_high(void) {
    uart_puts("MODEM: command line held low, waiting\r\n");
    uart_flush();

    RC1STAbits.CREN = 0;
    while (PIR1bits.RC1IF) {
        (void)RC1REG;
    }
    CMD_IOC_POS = 1;
    CMD_IOC_FLAG = 0;
    PIE0bits.IOCIE = 1;
    while (!CMD_PIN) {
        SLEEP();
        NOP();
        CMD_IOC_FLAG = 0;
    }
    PIE0bits.IOCIE = 0;
    CMD_IOC_POS = 0;
    CMD_IOC_FLAG = 0;
    RC1STAbits.CREN = 1;

    uart_puts("MODEM: command line high\r\n");
}

// Sleep until the controller's wake byte. Returns 1 once the wake byte has ended and
// the receiver is ready for the frame, 0 if there is nothing to read.
static uint8_t sleep_until_wake(void) {
    if (!CMD_PIN) {
        wait_line_high();               // WUE needs an idle-high line to see a falling edge
    }

    uart_flush();                       // Let the log finish before the clock stops
    uart_rx_flush();                    // RC1IF must be clear, or SLEEP returns at once
    while (!BAUD1CONbits.RCIDL);        // Don't arm WUE mid-character (Section 24.3.3.1)
    BAUD1CONbits.WUE = 1;
    SLEEP();
    NOP();
    (void)RC1REG;                       // Clears RC1IF from the wake event; contents discarded

    // WUE clears on the rising edge at the end of the wake byte.
    for (uint16_t i = 0; i < WAKE_END_MS * 100; i++) {
        if (!BAUD1CONbits.WUE) {
            return 1;
        }
        __delay_us(10);
    }

    BAUD1CONbits.WUE = 0;
    if (!CMD_PIN) {
        wait_line_high();               // Still low after 5 ms: not a wake byte
    }
    return 0;                           // Otherwise woken without a break: nothing to read
}

// Read one frame. Returns the payload length, or 0 if no valid frame arrived.
static uint8_t read_frame(uint8_t *payload) {
    int16_t c;

    // Skip anything before the start byte until the frame starts or time runs out.
    do {
        c = uart_getc_timeout(FIRST_BYTE_MS);
        if (c < 0) {
            return 0;
        }
    } while (c != FRAME_START);

    c = uart_getc_timeout(NEXT_BYTE_MS);
    if (c < 1 || c > MAX_PAYLOAD) {
        uart_puts("MODEM: bad length\r\n");
        return 0;
    }
    uint8_t len = (uint8_t)c;
    uint8_t sum = len;

    for (uint8_t i = 0; i < len; i++) {
        c = uart_getc_timeout(NEXT_BYTE_MS);
        if (c < 0) {
            uart_puts("MODEM: frame timeout\r\n");
            return 0;
        }
        payload[i] = (uint8_t)c;
        sum += (uint8_t)c;
    }

    c = uart_getc_timeout(NEXT_BYTE_MS);
    if (c < 0 || (uint8_t)c != sum) {
        uart_puts("MODEM: bad checksum\r\n");
        return 0;
    }
    return len;
}

void main(void) {
    static uint8_t payload[MAX_PAYLOAD];

    clock_init();
    selftest_pins();                    // Before pins_init(): drives PORTC pins directly
    pins_init();
    uart_init();
    spi_init();

    selftest_run();
    radio_check();

    uart_puts("CC1101 config: ");
    if (radio_configure() == 0) {
        uart_puts("OK\r\n");
    } else {
        uart_puts("readback errors\r\n");
    }

    // Radio to SLEEP until there is something to send. Registers are kept, apart from
    // TEST0-2 (left at their reset values here) and PATABLE entries other than [0].
    radio_strobe(CC1101_SIDLE);
    radio_wait_state(STATE_IDLE);
#if MODEM_SLEEP_RADIO
    radio_strobe(CC1101_SPWD);
#endif

    // EUSART receive interrupt wakes the PIC from Sleep (auto-wake-up on break). With
    // GIE clear there is no ISR: execution just carries on after SLEEP. PEIE is set
    // because PIE1 sources need it (datasheet PIE1 note); with GIE clear it does
    // nothing else.
    INTCONbits.PEIE = 1;
    PIE1bits.RC1IE = 1;

#if MODEM_TEST_SEND
    {
        static const uint8_t test_payload[] = { 'G', 'D' };
        uint16_t count = 0;

        uart_puts("MODEM: test mode, sending GD every 3 s\r\n");
        while (1) {
            uint8_t ok = radio_send(test_payload, sizeof test_payload);
#if MODEM_SLEEP_RADIO
            radio_strobe(CC1101_SPWD);
#endif
            uart_puts(ok ? "MODEM: test sent GD #" : "MODEM: test send failed #");
            uart_putdec((int16_t)++count);
            uart_puts("\r\n");
            __delay_ms(TEST_PERIOD_MS);
        }
    }
#endif

    uart_puts("MODEM: ready\r\n");

    while (1) {
        if (!sleep_until_wake()) {
            continue;
        }

        uint8_t len = read_frame(payload);
        if (len == 0) {
            continue;
        }

        uint8_t ok = radio_send(payload, len);
#if MODEM_SLEEP_RADIO
        radio_strobe(CC1101_SPWD);          // Back to SLEEP when CSn goes high
#endif

        uart_puts(ok ? "MODEM: sent" : "MODEM: send failed");
        for (uint8_t i = 0; i < len; i++) {
            uart_putc(' ');
            uart_puthex(payload[i]);
        }
        uart_puts("\r\n");
    }
}
