#include "cc1101.h"

#include <Arduino.h>

void CC1101::activateCS() {
  // Assert CS and wait for MISO low.
  gpio_set_level((gpio_num_t)cs_pin, LOW);

  while (gpio_get_level((gpio_num_t)miso_pin));
}

void CC1101::deactivateCS() {
  // De-assert CS.
  gpio_set_level((gpio_num_t)cs_pin, HIGH);
}

uint8_t CC1101::sendByte(uint8_t value) {
  int n, m;
  uint8_t received;

  received = 0;

  // Send byte by shifting one bit at a time onto MOSI while raising CLK for each bit.
  for (n = 0; n < 8; n++) {
    if (value & 0x80) {
      gpio_set_level((gpio_num_t)mosi_pin, HIGH);
    } else {
      gpio_set_level((gpio_num_t)mosi_pin, LOW);
    }

    // Delay to allow the data to settle before signalling to device to sample it on leading clock edge.
    //_delay(1);  // ~4us data setup at 1MHz (was 10 at 32MHz)

    // Raise clock.
    gpio_set_level((gpio_num_t)clock_pin, HIGH);

    // While CLK is high, shift the next bit into the received data.
    received <<= 1;
    if (gpio_get_level((gpio_num_t)miso_pin)) {
      received |= 0x01;
    }

    // Lower clock.
    // LATC &= ~0b00000010; for distance
    gpio_set_level((gpio_num_t)clock_pin, LOW);

    value <<= 1;
  }

  return received;
}

uint8_t CC1101::writeStrobe(uint8_t strobe) {
  // A strobe is a single byte command with a header byte only. It returns the status byte received while sending the strobe.
  uint8_t result;

  activateCS();
  result = sendByte(strobe);
  deactivateCS();

  return result;
}

uint8_t CC1101::getStatus() {
    return writeStrobe(SNOP);
}

void CC1101::writeRegister(uint8_t target, uint8_t value) {
    activateCS();
    sendByte(target);
    sendByte(value);
    deactivateCS();
}

uint8_t CC1101::readStatusRegister(uint8_t target) {
    // Read a register by setting the BURST and READ bits.
    uint8_t result;

    activateCS();
    sendByte(target | READ_MODE | BURST_MODE);
    result = sendByte(0);
    deactivateCS();

    return result;
}

void CC1101::writeBurst(uint8_t target, const void *pvalues, int count) {
    int n;
    uint8_t *ptx = (uint8_t*) pvalues;

    activateCS();
    sendByte(target | BURST_MODE);
    for (n = 0; n < count; n++) {
        sendByte(ptx[n]);
    }
    deactivateCS();
}

void CC1101::readBurst(uint8_t target, uint8_t* pdata, int count) {
    int n;

    activateCS();
    sendByte(target | READ_MODE | BURST_MODE);
    for (n = 0; n < count; n++) {
        pdata[n] = sendByte(0);
    }
    deactivateCS();
}

void CC1101::reset() {
    // Per datasheet section 19.1.2: set SCLK=1, SI=0 first
    gpio_set_level((gpio_num_t)clock_pin, HIGH);
    gpio_set_level((gpio_num_t)mosi_pin, LOW);
    // Strobe CSn low/high, wait ≥40µs
    gpio_set_level((gpio_num_t)cs_pin, LOW);
    delayMicroseconds(5);
    gpio_set_level((gpio_num_t)cs_pin, HIGH);
    delayMicroseconds(40);

    writeStrobe(SRESET);
    delayMicroseconds(100);
}

void CC1101::powerDown() {
    writeStrobe(SPWD);
}

void CC1101::begin(int mosi, int miso, int clk, int cs) {
  this->cs_pin = cs;
  gpio_reset_pin((gpio_num_t)cs);
  gpio_set_direction((gpio_num_t)cs, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t)cs, HIGH);

  this->mosi_pin = mosi;
  gpio_reset_pin((gpio_num_t)mosi);
  gpio_set_direction((gpio_num_t)mosi, GPIO_MODE_OUTPUT);

  this->miso_pin = miso;
  gpio_reset_pin((gpio_num_t)miso);
  gpio_set_direction((gpio_num_t)miso, GPIO_MODE_INPUT);

  this->clock_pin = clk;
  gpio_reset_pin((gpio_num_t)clk);
  gpio_set_direction((gpio_num_t)clk, GPIO_MODE_OUTPUT);
}

void CC1101::initialise() {
  writeRegister(IOCFG2, 0x01);   // GDO2 assert on FIFO Rx threshold met or FIFO Rx threshold exceeded.
  writeRegister(IOCFG0, 0x01);   // GDO0 = packet received.
  writeRegister(SYNC1, 'G');
  writeRegister(SYNC0, 'D');
  writeRegister(PKTLEN, 4);   // Address byte + 3 data bytes.
  writeRegister(PKTCTRL1, 0b10000101);  // PQT = 4, append status, address check.
  writeRegister(PKTCTRL0, 0b00000100);  // No data whitening, CRC, fixed packet length.
  writeRegister(ADDR, 0x01);
  writeRegister(FSCTRL1, 0x06);

  writeRegister(FREQ2, 0x10);
  writeRegister(FREQ1, 0xB0);
  writeRegister(FREQ0, 0x71);

  // 250000 baud
  writeRegister(MDMCFG4, 0x2D);
  writeRegister(MDMCFG3, 0x3B);

  writeRegister(MDMCFG2, 0b00010010);  // GFSK, no Manchester, 16/16 sync word.
  writeRegister(MDMCFG1, 0b00100010);  // No FEC/interleave, 4 preamble bytes minimum.
  writeRegister(DEVIATN, 0x62);
  writeRegister(MCSM2, 0b00011101);  // RX_TIME=5: 391µs listen window at 100ms EVENT0, 0.39% duty cycle
  writeRegister(MCSM1, 0b00111100);  // Stay in Rx after receiving a packet, Tx -> IDLE after transmission.
  writeRegister(MCSM0, 0b00001000);  // No auto-calibration.
  writeRegister(FOCCFG, 0x16);
  writeRegister(AGCCTRL2, 0x07);
  
  // WOR: ~100ms sleep period.
  writeRegister(WOREVT1, 0x0D);
  writeRegister(WOREVT0, 0x8B);
  
  // WOR: ~200ms sleep period.
  // writeRegister(WOREVT1, 0x1B);
  // writeRegister(WOREVT0, 0x16);
  
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
  writeRegister(WORCTRL, 0b00011000);  // RC_PD=0, EVENT1=1, RC_CAL=1
  delay(3);
  writeRegister(RCCTRL1, readStatusRegister(RCCTRL1_STATUS));
  writeRegister(RCCTRL0, readStatusRegister(RCCTRL0_STATUS));
  writeRegister(WORCTRL, 0b00010000);  // RC_CAL=0: freeze result

  // Calibrate the frequency synthesizer.
  writeStrobe(SCAL);
  delay(3);
}

void CC1101::idle() {
  writeStrobe(SIDLE);
}

void CC1101::enableRx() {
  writeStrobe(SRX);
}

void CC1101::enableWor() {
  writeStrobe(SWOR);
}

void CC1101::enableTx() {
  writeStrobe(STX);
}

int CC1101::transmitFifoCount() {
  return readStatusRegister(TXBYTES);
}

int CC1101::receiveAvailable() {
  return readStatusRegister(RXBYTES);
}

uint8_t CC1101::getVersion() {
  return readStatusRegister(VERSION);
}

void CC1101::flushRx() {
  writeStrobe(SFRX);
}

void CC1101::loadTx(const void *pdata, uint8_t length, uint8_t address) {
    int n;
    uint8_t packet[64];

    memset(packet, 0, 64);
    packet[0] = address;
    memcpy(packet + 1, pdata, length);

    writeBurst(FIFO, packet, length + 1);
}

void CC1101::fetchRx (void *pdata, uint8_t length) {
  readBurst(FIFO, (uint8_t*)pdata, length);
}
