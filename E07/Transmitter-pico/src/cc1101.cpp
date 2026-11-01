#include "cc1101.h"

#include <Arduino.h>

void CC1101::activateCS() {
  digitalWrite(cs_pin, LOW);
  while (digitalRead(miso_pin));
}

void CC1101::deactivateCS() {
  digitalWrite(cs_pin, HIGH);
}

uint8_t CC1101::sendByte(uint8_t value) {
  uint8_t received = 0;

  for (int n = 0; n < 8; n++) {
    if (value & 0x80)
      digitalWrite(mosi_pin, HIGH);
    else
      digitalWrite(mosi_pin, LOW);

    digitalWrite(clk_pin, HIGH);
    received = (received << 1) | (digitalRead(miso_pin));
    digitalWrite(clk_pin, LOW);

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

void CC1101::writeBurst(uint8_t target, const void* pvalues, int count) {
  uint8_t* ptx = (uint8_t*)pvalues;

  activateCS();
  sendByte(target | BURST_MODE);
  for (int n = 0; n < count; n++) {
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
  // Per datasheet section 19.1.2: set SCLK=1, SI=0 first.
  digitalWrite(clk_pin, HIGH);
  digitalWrite(mosi_pin, LOW);

  // Strobe CSn low/high, wait ≥40µs
  digitalWrite(cs_pin, LOW);
  delayMicroseconds(5);
  digitalWrite(cs_pin, HIGH);
  delayMicroseconds(40);

  writeStrobe(SRESET);
  delayMicroseconds(100);
}

void CC1101::powerDown() {
  writeStrobe(SPWD);
}

void CC1101::begin(int mosi, int miso, int clk, int cs) {
  mosi_pin = mosi;
  miso_pin = miso;
  clk_pin = clk;
  cs_pin = cs;

  pinMode(mosi_pin, OUTPUT);
  pinMode(miso_pin, INPUT);
  pinMode(clk_pin, OUTPUT);
  pinMode(cs_pin, OUTPUT);
}

void CC1101::initialise() {
  //
  // Rf settings for CC1101
  //
  writeRegister(IOCFG2, 0x01);  // GDO2 assert on FIFO Rx threshold met or FIFO Rx threshold exceeded.
  writeRegister(IOCFG0, 0x01);  // GDO0 Output Pin Configuration
  writeRegister(SYNC1, 'G');
  writeRegister(SYNC0, 'D');
  writeRegister(PKTLEN, 4);             // Address byte + 3 data bytes.
  writeRegister(PKTCTRL1, 0b10000101);  // PQT = 4, append status, address check.
  writeRegister(PKTCTRL0, 0x05);        // Packet Automation Control
  writeRegister(ADDR, 0x01);
  writeRegister(FSCTRL1, 0x06);  // Frequency Synthesizer Control
  writeRegister(FREQ2, 0x10);    // Frequency Control Word, High Byte
  writeRegister(FREQ1, 0xA7);    // Frequency Control Word, Middle Byte
  writeRegister(FREQ0, 0x62);    // Frequency Control Word, Low Byte
  writeRegister(MDMCFG4, 0xFD);  // Modem Configuration
  writeRegister(MDMCFG3, 0x3B);  // Modem Configuration
  writeRegister(MDMCFG2, 0x13);  // Modem Configuration
  writeRegister(DEVIATN, 0x15);  // Modem Deviation Setting
  writeRegister(MCSM0, 0x18);    // Main Radio Control State Machine Configuration
  writeRegister(FOCCFG, 0x16);   // Frequency Offset Compensation Configuration
  writeRegister(WORCTRL, 0xFB);  // Wake On Radio Control
  writeRegister(FSCAL3, 0xEA);   // Frequency Synthesizer Calibration
  writeRegister(FSCAL2, 0x2A);   // Frequency Synthesizer Calibration
  writeRegister(FSCAL1, 0x00);   // Frequency Synthesizer Calibration
  writeRegister(FSCAL0, 0x1F);   // Frequency Synthesizer Calibration
  writeRegister(TEST2, 0x81);    // Various Test Settings
  writeRegister(TEST1, 0x35);    // Various Test Settings
  writeRegister(TEST0, 0x09);    // Various Test Settings
  writeRegister(PATABLE, 0x34);

  //   writeRegister(IOCFG2, 0x01);  // GDO2 assert on FIFO Rx threshold met or FIFO Rx threshold exceeded.
  //   writeRegister(IOCFG0, 0x01);  // GDO0 = packet received.
  //   writeRegister(SYNC1, 'G');
  //   writeRegister(SYNC0, 'D');
  //   writeRegister(PKTLEN, 4);             // Address byte + 3 data bytes.
  //   writeRegister(PKTCTRL1, 0b10000101);  // PQT = 4, append status, address check.
  //   writeRegister(PKTCTRL0, 0b00000100);  // No data whitening, CRC, fixed packet length.
  //   writeRegister(ADDR, 0x01);
  //   writeRegister(FSCTRL1, 0x06);

  //   writeRegister(FREQ2, 0x10);
  //   writeRegister(FREQ1, 0xB0);
  //   writeRegister(FREQ0, 0x71);

  //   // 250000 baud
  //   writeRegister(MDMCFG4, 0x2D);
  //   writeRegister(MDMCFG3, 0x3B);

  //   writeRegister(MDMCFG2, 0b00010010);  // GFSK, no Manchester, 16/16 sync word.
  //   writeRegister(MDMCFG1, 0b00100010);  // No FEC/interleave, 4 preamble bytes minimum.
  //   writeRegister(DEVIATN, 0x62);
  //   writeRegister(MCSM2, 0b00011101);  // RX_TIME=5: 391µs listen window at 100ms EVENT0, 0.39% duty cycle
  //   writeRegister(MCSM1, 0b00111100);  // Stay in Rx after receiving a packet, Tx -> IDLE after transmission.
  //   writeRegister(MCSM0, 0b00001000);  // No auto-calibration.
  //   writeRegister(FOCCFG, 0x16);
  //   writeRegister(AGCCTRL2, 0x07);

  //   // WOR: ~100ms sleep period.
  //   writeRegister(WOREVT1, 0x0D);
  //   writeRegister(WOREVT0, 0x8B);

  //   // WOR: ~200ms sleep period.
  //   // writeRegister(WOREVT1, 0x1B);
  //   // writeRegister(WOREVT0, 0x16);

  //   //writeRegister(FREND0, 0x10);
  //   writeRegister(FSCAL3, 0xE9);
  //   writeRegister(FSCAL2, 0x2A);
  //   writeRegister(FSCAL1, 0x00);
  //   writeRegister(FSCAL0, 0x1F);
  //   writeRegister(TEST2, 0x81);
  //   writeRegister(TEST1, 0x35);
  //   writeRegister(TEST0, 0x09);

  //   /*
  //   The power ramping at the start and at the end
  // of a packet can be turned off by setting
  // FREND0.PA_POWER=0 and then program the
  // desired output power to index 0 in the
  // PATABLE.

  // PATABLE must be programmed in
  // burst mode if you want to write to other entries
  // than PATABLE[0].
  //   */

  //   writeRegister(PATABLE, 0x34);
  //   //uint8_t table[] = {0x34};  // 0xC0 = max output power
  //   //writeBurst(PATABLE, table, 1);

  //   // One-time RC oscillator calibration (AN047).
  //   // Enable calibration, wait for one cycle (~2ms), read back the result and
  //   // write it to RCCTRL so it is retained during SLEEP, then stop calibrating.
  //   writeRegister(WORCTRL, 0b00011000);  // RC_PD=0, EVENT1=1, RC_CAL=1
  //   delay(3);
  //   writeRegister(RCCTRL1, readStatusRegister(RCCTRL1_STATUS));
  //   writeRegister(RCCTRL0, readStatusRegister(RCCTRL0_STATUS));
  //   writeRegister(WORCTRL, 0b00010000);  // RC_CAL=0: freeze result

  //   // Calibrate the frequency synthesizer.
  //   writeStrobe(SCAL);
  //   delay(3);
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

void CC1101::flushTx() {
  writeStrobe(SFTX);
}

void CC1101::loadTx(const void* pdata, uint8_t length, uint8_t address) {
  int n;
  uint8_t packet[64];

  memset(packet, 0, 64);
  packet[0] = address;
  memcpy(packet + 1, pdata, length);

  writeBurst(FIFO, packet, length + 1);
}

void CC1101::fetchRx(void* pdata, uint8_t length) {
  readBurst(FIFO, (uint8_t*)pdata, length);
}
