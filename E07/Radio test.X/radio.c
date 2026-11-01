/*
 * radio.c - SPI (MSSP1) and CC1101 access, reset, ID check and configuration.
 */

#include "board.h"
#include "uart.h"
#include "radio.h"

// ---------------------------------------------------------------------------
// SPI (MSSP1, host) and CC1101 low-level access
// ---------------------------------------------------------------------------

void spi_init(void) {
    // SPI mode 0 for the CC1101: clock idles low, data sampled on the rising edge.
    SSP1STAT = 0b01000000;              // SMP = 0, CKE = 1 (transmit on active-to-idle edge)
    SSP1CON1 = 0b00100000;              // SSPEN = 1, CKP = 0, SSPM = 0000: host, FOSC/4 = 1 MHz
}

uint8_t spi_xfer(uint8_t out) {
    SSP1BUF = out;
    while (!SSP1STATbits.BF);           // Wait for the byte to finish
    return SSP1BUF;                     // Reading clears BF
}

// Wait for SO to go low (CHIP_RDYn) with CSn already low. Returns 0 on timeout.
uint8_t radio_wait_ready(void) {
    for (uint16_t i = 0; i < 1000; i++) {   // 1000 x 10 us = 10 ms
        if (RADIO_SO == 0) {
            return 1;
        }
        __delay_us(10);
    }
    return 0;
}

// Manual reset, CC1101 datasheet Section 19.1.2. Returns 0 if the chip never
// signals ready. The datasheet also asks for SCLK = 1 during the CSn strobe; that
// only matters if pin control mode (MCSM0.PIN_CTRL_EN) is enabled, which it is not
// after power-on and is never set by this code, so SCLK is left at its idle low.
uint8_t radio_reset(void) {
    RADIO_CS = 0;                       // Strobe CSn low/high
    __delay_us(10);
    RADIO_CS = 1;
    __delay_us(50);                     // Hold high for at least 40 us

    RADIO_CS = 0;
    if (!radio_wait_ready()) {          // Crystal running, chip ready
        RADIO_CS = 1;
        return 0;
    }
    spi_xfer(CC1101_SRES);
    uint8_t ok = radio_wait_ready();    // SO goes low again when reset is complete
    RADIO_CS = 1;
    return ok;
}

// Read one status register (0x30-0x3D). These need the burst bit set, otherwise
// the same address is a command strobe.
uint8_t radio_read_status(uint8_t addr) {
    RADIO_CS = 0;
    radio_wait_ready();
    spi_xfer(addr | CC1101_READ | CC1101_BURST);
    uint8_t v = spi_xfer(0x00);
    RADIO_CS = 1;
    return v;
}

void radio_check(void) {
    uart_puts("\r\nCC1101: ");
    if (!radio_reset()) {
        uart_puts("no response (SO did not go low)\r\n");
        return;
    }
    uint8_t partnum = radio_read_status(CC1101_PARTNUM);
    uint8_t version = radio_read_status(CC1101_VERSION);

    uart_puts("PARTNUM ");
    uart_puthex(partnum);
    uart_puts(" VERSION ");
    uart_puthex(version);

    // PARTNUM is 0x00 on the CC1101, which a stuck-low MISO would also give, so
    // VERSION is what proves the link: 0x00 or 0xFF means MISO is stuck.
    if (partnum == 0x00 && version != 0x00 && version != 0xFF) {
        uart_puts(" OK\r\n");
    } else {
        uart_puts(" unexpected\r\n");
    }
}

// ---------------------------------------------------------------------------
// CC1101 register access and configuration
// ---------------------------------------------------------------------------

void radio_write_reg(uint8_t addr, uint8_t value) {
    RADIO_CS = 0;
    radio_wait_ready();
    spi_xfer(addr);
    spi_xfer(value);
    RADIO_CS = 1;
}

uint8_t radio_read_reg(uint8_t addr) {
    RADIO_CS = 0;
    radio_wait_ready();
    spi_xfer(addr | CC1101_READ);
    uint8_t v = spi_xfer(0x00);
    RADIO_CS = 1;
    return v;
}

// Send a command strobe; returns the chip status byte.
uint8_t radio_strobe(uint8_t cmd) {
    RADIO_CS = 0;
    radio_wait_ready();
    uint8_t status = spi_xfer(cmd);
    RADIO_CS = 1;
    return status;
}

// 433.92 MHz, GFSK, 250 kBaud, 127 kHz deviation, 541 kHz RX filter.
// Values from TI SmartRF Studio 7 (v2.31) preset "Data rate: 250 kBaud, Dev.: 127 kHz,
// Mod.: GFSK, RX BW: 540 kHz, Optimized for sensitivity" (cc1101/settings/ts_setting16.xml),
// with the changes marked. That preset is for 868 MHz; the frequency, TEST0 and packet
// settings are adjusted here. Registers not listed stay at their post-SRES defaults.
static const uint8_t radio_config[][2] = {
    { CC1101_IOCFG2,   0x06 },  // Changed: GDO2 asserts on sync word, deasserts at end of packet
    { CC1101_IOCFG0,   0x2E },  // GDO0 high-Z (not connected); stops the default clock output
    { CC1101_PKTLEN,   0x3D },  // Changed: max 61-byte payload (fits FIFO with length + 2 status bytes)
    { CC1101_PKTCTRL0, 0x05 },  // Changed: variable length (first byte), CRC on, no whitening
    { CC1101_FSCTRL1,  0x0C },  // IF = 26 MHz / 1024 * 12 = 304.7 kHz
    { CC1101_FREQ2,    0x10 },  // Changed: FREQ = 0x10B071 -> 433.9198 MHz
    { CC1101_FREQ1,    0xB0 },
    { CC1101_FREQ0,    0x71 },
    { CC1101_MDMCFG4,  0x2D },  // RX BW = 26 MHz / (8 * (4 + 2)) = 541.7 kHz; DRATE_E = 13
    { CC1101_MDMCFG3,  0x3B },  // DRATE_M = 59 -> 249.94 kBaud
    { CC1101_MDMCFG2,  0x13 },  // GFSK, 30/32 sync bits, DC filter on
    { CC1101_DEVIATN,  0x62 },  // Deviation = 126.95 kHz
    { CC1101_MCSM0,    0x18 },  // Calibrate on IDLE -> RX/TX
    { CC1101_FOCCFG,   0x1D },
    { CC1101_BSCFG,    0x1C },
    { CC1101_AGCCTRL2, 0xC7 },
    { CC1101_AGCCTRL1, 0x00 },
    { CC1101_AGCCTRL0, 0xB0 },
    { CC1101_WORCTRL,  0xFB },
    { CC1101_FREND1,   0xB6 },
    { CC1101_FREND0,   0x10 },  // PA_POWER = 0: PATABLE[0] only, no ramping
    { CC1101_FSCAL3,   0xEA },
    { CC1101_FSCAL2,   0x2A },
    { CC1101_FSCAL1,   0x00 },
    { CC1101_FSCAL0,   0x1F },
    { CC1101_TEST0,    0x0B },  // Changed from 0x09: VCO selection calibration enabled (reset value)
};

#define PA_0DBM  0x60               // SmartRF Studio PA table, CC1101 434 MHz, 0 dBm (non-ramping)

// Write the configuration and read it back. Returns the number of mismatches.
uint8_t radio_configure(void) {
    uint8_t errors = 0;
    for (uint8_t i = 0; i < sizeof radio_config / sizeof radio_config[0]; i++) {
        radio_write_reg(radio_config[i][0], radio_config[i][1]);
    }
    radio_write_reg(CC1101_PATABLE, PA_0DBM);   // Single write goes to PATABLE[0]

    for (uint8_t i = 0; i < sizeof radio_config / sizeof radio_config[0]; i++) {
        uint8_t v = radio_read_reg(radio_config[i][0]);
        if (v != radio_config[i][1]) {
            errors++;
            uart_puts("  reg ");
            uart_puthex(radio_config[i][0]);
            uart_puts(" wrote ");
            uart_puthex(radio_config[i][1]);
            uart_puts(" read ");
            uart_puthex(v);
            uart_puts("\r\n");
        }
    }
    return errors;
}

// Wait for the chip status STATE to equal 'state'. Returns 0 on timeout (~10 ms).
uint8_t radio_wait_state(uint8_t state) {
    for (uint16_t i = 0; i < 1000; i++) {
        uint8_t s = radio_strobe(CC1101_SNOP) & STATE_MASK;
        if (s == state) {
            return 1;
        }
        if (s == STATE_TXFIFO_UFLOW) {
            return 0;
        }
        __delay_us(10);
    }
    return 0;
}

// Burst-write 'n' bytes starting at 'addr' (e.g. CC1101_TXFIFO).
void radio_write_burst(uint8_t addr, const uint8_t *data, uint8_t n) {
    RADIO_CS = 0;
    radio_wait_ready();
    spi_xfer(addr | CC1101_BURST);
    while (n--) {
        spi_xfer(*data++);
    }
    RADIO_CS = 1;
}

// Burst-read 'n' bytes starting at 'addr' (e.g. CC1101_RXFIFO).
void radio_read_burst(uint8_t addr, uint8_t *data, uint8_t n) {
    RADIO_CS = 0;
    radio_wait_ready();
    spi_xfer(addr | CC1101_READ | CC1101_BURST);
    while (n--) {
        *data++ = spi_xfer(0x00);
    }
    RADIO_CS = 1;
}
