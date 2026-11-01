#include <xc.h>
#include <string.h>

#include "radio.h"

#define _XTAL_FREQ 1000000

const int READ_MODE = 0x80;
const int BURST_MODE = 0x40;

const int IOCFG2 = 0x00;
const int IOCFG1 = 0x01;
const int IOCFG0 = 0x02;
const int FIFOTHR = 0x03;
const int SYNC1 = 0x04;
const int SYNC0 = 0x05;
const int PKTLEN = 0x06;
const int PKTCTRL1 = 0x07;
const int PKTCTRL0 = 0x08;
const int ADDR = 0x09;
const int CHANNR = 0x0A;
const int FSCTRL1 = 0x0B;
const int FSCTRL0 = 0x0C;
const int FREQ2 = 0x0D;
const int FREQ1 = 0x0E;
const int FREQ0 = 0x0F;
const int MDMCFG4 = 0x10;
const int MDMCFG3 = 0x11;
const int MDMCFG2 = 0x12;
const int MDMCFG1 = 0x13;
const int MDMCFG0 = 0x14;
const int DEVIATN = 0x15;
const int MCSM2 = 0x16;
const int MCSM1 = 0x17;
const int MCSM0 = 0x18;
const int FOCCFG = 0x19;
const int BSCFG = 0x1A;
const int AGCCTRL2 = 0x1B;
const int AGCCTRL1 = 0x1C;
const int AGCCTRL0 = 0x1D;
const int WOREVT1 = 0x1E;
const int WOREVT0 = 0x1F;
const int WORCTRL = 0x20;
const int FREND1 = 0x21;
const int FREND0 = 0x22;
const int FSCAL3 = 0x23;
const int FSCAL2 = 0x24;
const int FSCAL1 = 0x25;
const int FSCAL0 = 0x26;
const int RCCTRL1 = 0x27;
const int RCCTRL0 = 0x28;
const int FSTEST = 0x29;
const int PTEST = 0x2A;
const int AGCTEST = 0x2B;
const int TEST2 = 0x2C;
const int TEST1 = 0x2D;
const int TEST0 = 0x2E;

const int SRESET = 0x30;
const int VERSION = 0x31;
const int SCAL = 0x33;
const int SRX = 0x34;
const int STX = 0x35;
const int SIDLE = 0x36;
const int SWOR = 0x38;
const int SPWD = 0x39;
const int SFRX = 0x3A;
const int SFTX = 0x3B;
const int SNOP = 0x3D;

const int PATABLE = 0x3E;
const int FIFO = 0x3F;

const int RCCTRL1_STATUS = 0x3C;
const int RCCTRL0_STATUS = 0x3D;

// Uncomment to use hardware MSSP SPI instead of bit-banged SPI.
#define USE_HW_SPI

#define LATCH2(P) LAT ## P
#define LATCH(P) LATCH2(P)

#define PORT2(P) PORT ## P
#define PORT(P) PORT2(P)

#define READ_MISO (PORT(MISO_PORT) & MISO_BIT)

#define MOSI_HIGH (LATCH(MOSI_PORT) |= MOSI_BIT)
#define MOSI_LOW (LATCH(MOSI_PORT) &= ~MOSI_BIT)

#define CLOCK_HIGH (LATCH(CLOCK_PORT) |= CLOCK_BIT)
#define CLOCK_LOW (LATCH(CLOCK_PORT) &= ~CLOCK_BIT)

#define CS_HIGH (LATCH(CS_PORT) |= CS_BIT)
#define CS_LOW (LATCH(CS_PORT) &= ~CS_BIT)

void activateCS() {
    // Assert CS and wait for MISO low.
    CS_LOW;
    while (READ_MISO);
}

void deactivateCS() {
    // De-assert CS.
    CS_HIGH;
    _delay(16); // ~64us at 1MHz (was 500 at 32MHz)
}

#ifdef USE_HW_SPI

static void spi_hw_init(void) {
    // Route MSSP SPI signals to the radio pins via PPS.
    // SCK1 output -> RC1 (CLOCK_BIT), input feedback also from RC1.
    RC1PPS = 0x07; // SCL1/SCK1 output on RC1.
    SSP1CLKPPS = 0x11; // SCK1 input from RC1 (PORTC=010, pin1=001).
    // SDO1 output -> RC0 (MOSI_BIT).
    RC0PPS = 0x08; // SDA1/SDO1 output on RC0.
    // SDI1 input <- RA2 (MISO_BIT).
    SSP1DATPPS = 0x02; // SDI1 input from RA2 (PORTA=000, pin2=010).

    // SPI Mode 0 (CC1101: CPOL=0, CPHA=0): CKP=0, CKE=1, SMP=0.
    SSP1STAT = 0b01000000; // CKE=1.
    // SPI Master, Fosc/4 = 250kHz at 1MHz, CKP=0, SSPEN=1.
    SSP1CON1 = 0b00100000; // SSPEN=1, SSPM[3:0]=0000.
}

uint8_t sendByte(uint8_t value) {
    SSP1BUF = value;
    while (!SSP1STATbits.BF); // Wait for transfer complete.
    return SSP1BUF; // Reading BUF also clears BF.
}

#else

uint8_t sendByte(uint8_t value) {
    int n;
    uint8_t received;

    received = 0;

    // Send byte by shifting one bit at a time onto MOSI while raising CLK for each bit.
    for (n = 0; n < 8; n++) {
        if (value & 0x80) {
            MOSI_HIGH;
        } else {
            MOSI_LOW;
        }

        // Delay to allow the data to settle before signalling to device to sample it on leading clock edge.
        _delay(1); // ~4us data setup at 1MHz (was 10 at 32MHz)

        CLOCK_HIGH;

        // While CLK is high, shift the next bit into the received data.
        received <<= 1;
        if (READ_MISO) {
            received |= 0x01;
        }

        CLOCK_LOW;

        value <<= 1;
    }

    return received;
}

#endif

// A strobe is a single byte command with a header byte only. It returns the status byte received while sending the strobe.

uint8_t writeStrobe(uint8_t strobe) {
    uint8_t result;

    activateCS();
    result = sendByte(strobe);
    deactivateCS();

    return result;
}

uint8_t getStatus() {
    return writeStrobe(SNOP);
}

void writeRegister(uint8_t target, uint8_t value) {
    activateCS();
    sendByte(target);
    sendByte(value);
    deactivateCS();
}

uint8_t readStatusRegister(uint8_t target) {
    // Read a register by setting the BURST and READ bits.
    uint8_t result;

    activateCS();
    sendByte(target | READ_MODE | BURST_MODE);
    result = sendByte(0);
    deactivateCS();

    return result;
}

void writeBurst(uint8_t target, const void *pvalues, int count) {
    int n;
    uint8_t *ptx = (uint8_t*) pvalues;

    activateCS();
    sendByte(target | BURST_MODE);
    for (n = 0; n < count; n++) {
        sendByte(ptx[n]);
    }
    deactivateCS();
}

static void readBurst(uint8_t target, uint8_t *pdata, int count) {
    int n;
    activateCS();
    sendByte(target | READ_MODE | BURST_MODE);
    for (n = 0; n < count; n++) {
        pdata[n] = sendByte(0);
    }
    deactivateCS();
}

void reset() {
    writeStrobe(SRESET);
}

void powerDown() {
    writeStrobe(SPWD);
}

void enableRx() {
    writeStrobe(SRX);
}

void enableWor() {
    // SWOR is only valid from IDLE - ensure we're there first.
    writeStrobe(SIDLE);
    writeStrobe(SWOR);
}

int receiveValid(void *ppacket) {
    // Read the full packet from the RX FIFO:
    // [ADDR][D0][D1][D2][RSSI][CRC_OK|LQI]
    uint8_t buf[6];
    readBurst(FIFO, buf, 6);

    if (!(buf[5] & 0x80)) {
        // CRC failed - packet already read out of FIFO, just discard it.
        return 0;
    }

    // Copy the 3 data bytes (skip address byte at buf[0]).
    memcpy(ppacket, buf + 1, 3);
    return 1;
}

void initialise() {
#ifdef USE_HW_SPI
    spi_hw_init();
#endif
    writeRegister(IOCFG2, 0x01); // GDO2 assert on FIFO Rx threshold met or FIFO Rx threshold exceeded.
    writeRegister(IOCFG0, 0x01); // GDO0 asserts high when packet received into FIFO.
    writeRegister(SYNC1, 'G');
    writeRegister(SYNC0, 'D');
    writeRegister(PKTLEN, 4); // Address byte + 3 data bytes.
    writeRegister(PKTCTRL1, 0b10000101); // PQT = 4, append status, address check.
    writeRegister(PKTCTRL0, 0b00000100); // No data whitening, CRC, fixed packet length.
    writeRegister(ADDR, 0x01);
    writeRegister(FSCTRL1, 0x06);

    writeRegister(FREQ2, 0x10);
    writeRegister(FREQ1, 0xB0);
    writeRegister(FREQ0, 0x71);

    // 250000 baud
    writeRegister(MDMCFG4, 0x2D);
    writeRegister(MDMCFG3, 0x3B);

    writeRegister(MDMCFG2, 0b00010010); // GFSK, no Manchester, 16/16 sync word.
    writeRegister(MDMCFG1, 0b00100010); // No FEC/interleave, 4 preamble bytes minimum.
    writeRegister(DEVIATN, 0x62);
    writeRegister(MCSM2, 0b00011101); // RX_TIME=5: 391us listen window at 100ms EVENT0, 0.39% duty cycle.
    writeRegister(MCSM1, 0b00110000); // Return to IDLE after receiving a packet or after transmission.
    writeRegister(MCSM0, 0b00001000); // No auto-calibration.
    writeRegister(FOCCFG, 0x16);
    writeRegister(AGCCTRL2, 0x07);

    // WOR: ~100ms sleep period (for future use).
    writeRegister(WOREVT1, 0x0D);
    writeRegister(WOREVT0, 0x8B);

    writeRegister(FREND0, 0x10);
    writeRegister(FSCAL3, 0xE9);
    writeRegister(FSCAL2, 0x2A);
    writeRegister(FSCAL1, 0x00);
    writeRegister(FSCAL0, 0x1F);
    writeRegister(TEST2, 0x81);
    writeRegister(TEST1, 0x35);
    writeRegister(TEST0, 0x09);

    uint8_t table[] = {0xC0}; // Max output power
    writeBurst(PATABLE, table, 1);

    // One-time RC oscillator calibration (AN047).
    // Enable calibration, wait for one cycle (~2ms), read back the result and
    // write it to RCCTRL so it is retained during SLEEP, then stop calibrating.
    writeRegister(WORCTRL, 0b00011000); // RC_PD=0, EVENT1=1, RC_CAL=1
    __delay_ms(3);
    writeRegister(RCCTRL1, readStatusRegister(RCCTRL1_STATUS));
    writeRegister(RCCTRL0, readStatusRegister(RCCTRL0_STATUS));
    writeRegister(WORCTRL, 0b00010000); // RC_CAL=0: freeze result.

    // Calibrate the frequency synthesizer.
    writeStrobe(SCAL);
    __delay_ms(3);
}

void initialiseAfterPowerDown() {
    writeRegister(TEST2, 0x81);
    writeRegister(TEST1, 0x35);
    writeRegister(TEST0, 0x09);
}

uint8_t radio_version() {
    return readStatusRegister(VERSION);
}
