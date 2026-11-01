#pragma once

#include "Arduino.h"

class CC1101 {
 private:
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

  const int VERSION = 0x31;
  const int TXBYTES = 0x3A;
  const int RXBYTES = 0x3B;
  const int RCCTRL1_STATUS = 0x3C;
  const int RCCTRL0_STATUS = 0x3D;

  int mosi_pin, miso_pin, clock_pin, cs_pin;

  void activateCS();
  void deactivateCS();
  uint8_t sendByte(uint8_t);
  uint8_t writeStrobe(uint8_t);
  void writeRegister(uint8_t target, uint8_t value);
  uint8_t readStatusRegister(uint8_t target);
  void writeBurst(uint8_t target, const void* pvalues, int count);
  void readBurst(uint8_t target, uint8_t* pdata, int count);

 public:
  void begin(int mosi, int miso, int clk, int cs);
  void initialise();
  void idle();
  void reset();
  void powerDown();
  void enableTx();
  void enableRx();
  void enableWor();
  void flushRx();
  int transmitFifoCount();
  int receiveAvailable();
  uint8_t getVersion();
  void loadTx(const void *pdata, uint8_t length, uint8_t address);
  void fetchRx (void *pdata, uint8_t length);
  uint8_t getStatus();
};