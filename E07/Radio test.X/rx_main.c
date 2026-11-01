/*
 * rx_main.c - Receiver test: wake-on-radio, log each packet's contents and RSSI.
 * Built in the RX configuration only.
 *
 * The CC1101 polls on its own (WOR) while the PIC sleeps. GDO2 (IOCFG2 = 0x06)
 * goes high on sync word and low at the end of the packet; that falling edge on
 * RC3 wakes the PIC through interrupt-on-change.
 */

#include "board.h"
#include "uart.h"
#include "radio.h"

// WOR poll interval. t_EVENT0 = 750 / f_xosc * EVENT0 * 2^(5 * WOR_RES)
// (AN047 Equation 1). With WOR_RES = 0: 750 / 26 MHz * 8667 = 250.0 ms.
// Must stay below the transmitter's PREAMBLE_MS (300 ms) less the wake-up time.
#define EVENT0  8667

// Debug: 1 = plain continuous RX (no WOR, no RX timeout) to separate radio and
// packet problems from WOR timing problems. 0 = normal wake-on-radio.
#define RX_CONTINUOUS 0

// MCSM2 in WOR mode. Normal value 0x1D. Debug values to bisect a WOR problem:
//   0x07  RX_TIME = 7: first WOR wake stays in RX until a packet (tests that WOR wakes at all)
//   0x0D  RX_TIME_QUAL only: stay in RX if PQT reached (tests PQT, bypasses carrier sense)
//   0x1D  RX_TIME_RSSI + RX_TIME_QUAL: normal low-power setting
#define WOR_MCSM2 0x1D

// WORCTRL.EVENT1: delay from Event 0 (crystal start) to Event 1 (RX strobe).
// 3 = 346 us, 4 = 462 us, 5 = 692 us, 6 = 923 us, 7 = 1.385 ms (AN047 Table 1).
// CC1101 errata: tEvent1 must exceed max crystal start-up + PO_TIMEOUT, otherwise
// the radio can enter an illegal state and miss Event 1 (never enters RX).
#define WOR_EVENT1 3

// Debug: route an internal WOR signal to GDO2 instead of the packet signal (0x06).
// Each falling edge wakes the PIC, which logs "wake with empty FIFO".
// Settings >= 0x20 keep working while the radio is in SLEEP (datasheet Section 26).
//   0     off (normal)
//   0x24  WOR_EVNT0: one wake per poll, about 4 per second
//   0x26  CLK_256: RC oscillator / 128, about 270 Hz; floods the log if the RC oscillator runs
#define DEBUG_GDO2_CFG 0

// Debug: at startup, with the radio in IDLE (no WOR), check the GDO2 pin and the
// RC oscillator directly by polling RC3, and print the RC calibration results.
#define DEBUG_RC_CHECK 0

// Receiver-only settings, written on top of the shared configuration.
static const uint8_t rx_config[][2] = {
    // PQT = 2 (threshold 8), CRC_AUTOFLUSH = 0, APPEND_STATUS = 1, no address check.
    { CC1101_PKTCTRL1, 0x44 },
    // RX_TIME_RSSI = 1: leave RX at once if no carrier.
    // RX_TIME_QUAL = 1: at timeout, stay in RX if sync word found or PQT reached.
    // RX_TIME = 5: timeout = EVENT0 * 0.1127 us = 0.98 ms (AN047 Table 6).
#if RX_CONTINUOUS
    { CC1101_MCSM2,    0x07 },          // RX_TIME = 7: no timeout, stay in RX until a packet arrives
#else
    { CC1101_MCSM2,    WOR_MCSM2 },
#endif
    // FS_AUTOCAL = 11: calibrate the PLL every 4th automatic RX -> IDLE transition
    // instead of on every IDLE -> RX. Calibration (~0.77 ms at ~8.7 mA) was about
    // 60 % of each WOR wake's charge. PO_TIMEOUT = 10 as before.
    { CC1101_MCSM0,    0x38 },
    { CC1101_WOREVT1,  (uint8_t)(EVENT0 >> 8) },
    { CC1101_WOREVT0,  (uint8_t)(EVENT0 & 0xFF) },
    // RC_PD = 0 (RC oscillator on), EVENT1 = WOR_EVENT1, RC_CAL = 1, WOR_RES = 0.
    { CC1101_WORCTRL,  (uint8_t)((WOR_EVENT1 << 4) | 0x08) },
#if DEBUG_GDO2_CFG
    { CC1101_IOCFG2,   DEBUG_GDO2_CFG },
#endif
};

static uint8_t rx_configure(void) {
    uint8_t errors = 0;
    for (uint8_t i = 0; i < sizeof rx_config / sizeof rx_config[0]; i++) {
        radio_write_reg(rx_config[i][0], rx_config[i][1]);
    }
    for (uint8_t i = 0; i < sizeof rx_config / sizeof rx_config[0]; i++) {
        uint8_t v = radio_read_reg(rx_config[i][0]);
        if (v != rx_config[i][1]) {
            errors++;
            uart_puts("  reg ");
            uart_puthex(rx_config[i][0]);
            uart_puts(" wrote ");
            uart_puthex(rx_config[i][1]);
            uart_puts(" read ");
            uart_puthex(v);
            uart_puts("\r\n");
        }
    }
    return errors;
}

// Put the radio back into WOR with an empty RX FIFO.
static void rx_arm(void) {
    radio_strobe(CC1101_SIDLE);
    radio_wait_state(STATE_IDLE);
    radio_strobe(CC1101_SFRX);
    IOCCFbits.IOCCF3 = 0;
#if RX_CONTINUOUS
    radio_strobe(CC1101_SRX);
#else
    radio_strobe(CC1101_SWORRST);
    radio_strobe(CC1101_SWOR);          // Radio sleeps once CSn goes high
#endif
}

// RXBYTES can be misread if the radio updates it during the SPI read, so read
// until two reads agree (CC1101 datasheet Section 10.3).
static uint8_t rx_bytes(void) {
    uint8_t a, b = radio_read_status(CC1101_RXBYTES);
    do {
        a = b;
        b = radio_read_status(CC1101_RXBYTES);
    } while (a != b);
    return a;
}

static void rx_read_packet(void) {
    static uint8_t buf[64];

    uint8_t n = rx_bytes();
    if (n & 0x80) {
        uart_puts("RX: FIFO overflow\r\n");
        return;
    }
    n &= 0x7F;
    if (n == 0) {
        uart_puts("RX: wake with empty FIFO\r\n");
        return;
    }

    radio_read_burst(CC1101_RXFIFO, buf, n);

    // FIFO holds: length, payload[length], RSSI, LQI | CRC_OK << 7
    uint8_t len = buf[0];
    if ((uint16_t)len + 3 != n) {
        uart_puts("RX: length ");
        uart_putdec(len);
        uart_puts(" but ");
        uart_putdec(n);
        uart_puts(" bytes in FIFO\r\n");
        return;
    }
    uint8_t rssi_raw = buf[len + 1];
    uint8_t lqi_crc  = buf[len + 2];

    uart_puts("RX: '");
    for (uint8_t i = 1; i <= len; i++) {
        char c = (char)buf[i];
        uart_putc((c >= 0x20 && c < 0x7F) ? c : '.');
    }
    uart_puts("'");

    // RSSI is two's complement in 0.5 dB steps; RSSI_offset = 74 dB at 433 MHz
    // (datasheet Section 17.3, Table 31).
    int16_t rssi_dbm = (int16_t)(int8_t)rssi_raw / 2 - 74;
    uart_puts(" RSSI ");
    uart_putdec(rssi_dbm);
    uart_puts(" dBm LQI ");
    uart_putdec(lqi_crc & 0x7F);
    uart_puts((lqi_crc & 0x80) ? " CRC OK\r\n" : " CRC FAIL\r\n");
}

#if DEBUG_RC_CHECK
#define CC1101_RCCTRL1_STATUS 0x3C
#define CC1101_RCCTRL0_STATUS 0x3D

static void rc_check(void) {
    // GDO2 driven to fixed levels: HW to 0 (0x2F), and inverted for HW to 1 (0x6F).
    radio_write_reg(CC1101_IOCFG2, 0x2F);
    __delay_us(100);
    uint8_t lo = RADIO_GDO2;
    radio_write_reg(CC1101_IOCFG2, 0x6F);
    __delay_us(100);
    uint8_t hi = RADIO_GDO2;
    uart_puts("DBG: GDO2 forced 0 reads ");
    uart_putdec(lo);
    uart_puts(", forced 1 reads ");
    uart_putdec(hi);
    uart_puts("\r\n");

    // CLK_256 on GDO2: RC oscillator / 128, about 270 Hz from a 26 MHz crystal.
    // Radio is in IDLE with the crystal running, so RC calibration can proceed.
    radio_write_reg(CC1101_IOCFG2, 0x26);
    __delay_ms(10);
    uint16_t edges = 0;
    uint8_t prev = RADIO_GDO2;
    for (uint16_t i = 0; i < 10000; i++) {      // 10000 x (100 us + loop overhead): over 1 s
        uint8_t now = RADIO_GDO2;
        if (now && !prev) {
            edges++;
        }
        prev = now;
        __delay_us(100);
    }
    uart_puts("DBG: CLK_256 rising edges in >1 s: ");
    uart_putdec((int16_t)edges);
    uart_puts(" (270 per second; loop takes over 1 s, so expect more than 270)\r\n");

    uart_puts("DBG: RCCTRL1_STATUS ");
    uart_puthex(radio_read_status(CC1101_RCCTRL1_STATUS));
    uart_puts(" RCCTRL0_STATUS ");
    uart_puthex(radio_read_status(CC1101_RCCTRL0_STATUS));
    uart_puts(" WORCTRL ");
    uart_puthex(radio_read_reg(CC1101_WORCTRL));
    uart_puts("\r\n");

    radio_write_reg(CC1101_IOCFG2, DEBUG_GDO2_CFG ? DEBUG_GDO2_CFG : 0x06);
}
#endif

void main(void) {
    clock_init();
    pins_init();
    uart_init();
    spi_init();

    radio_check();

    uart_puts("CC1101 config: ");
    // Shared settings first, then the receiver's on top. Two statements, because
    // C does not define the evaluation order of the operands of '+'.
    uint8_t errors = radio_configure();
    errors += rx_configure();
    if (errors == 0) {
        uart_puts("OK\r\n");
    } else {
        uart_puts("readback errors\r\n");
    }

#if DEBUG_RC_CHECK
    rc_check();
#endif

    // With FS_AUTOCAL = 11 the radio only calibrates after RX, so calibrate once
    // now; otherwise the first wakes would use the uncalibrated preset FSCAL values.
    radio_strobe(CC1101_SCAL);          // Allowed from IDLE without manual-cal mode
    if (!radio_wait_state(STATE_IDLE)) {
        uart_puts("RX: calibration did not finish\r\n");
    }

    // GDO2 falling edge on RC3 = end of packet. With GIE clear there is no ISR:
    // the interrupt just wakes the PIC from SLEEP and execution carries on.
    IOCCPbits.IOCCP3 = 0;
    IOCCNbits.IOCCN3 = 1;
    IOCCFbits.IOCCF3 = 0;
    PIE0bits.IOCIE = 1;

    uart_puts(RX_CONTINUOUS ? "RX: listening (continuous)\r\n" : "RX: listening (WOR)\r\n");
    rx_arm();

    while (1) {
        uart_flush();
        SLEEP();
        NOP();

        if (IOCCFbits.IOCCF3) {
            IOCCFbits.IOCCF3 = 0;
            rx_read_packet();
            rx_arm();
        }
    }
}
