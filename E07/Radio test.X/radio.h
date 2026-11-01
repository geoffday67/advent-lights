/*
 * radio.h - SPI (MSSP1) and CC1101 access.
 */
#ifndef RADIO_H
#define RADIO_H

#include <stdint.h>

// SPI header bits
#define CC1101_READ         0x80
#define CC1101_BURST        0x40

// Configuration registers
#define CC1101_IOCFG2       0x00
#define CC1101_IOCFG0       0x02
#define CC1101_PKTLEN       0x06
#define CC1101_PKTCTRL1     0x07
#define CC1101_PKTCTRL0     0x08
#define CC1101_FSCTRL1      0x0B
#define CC1101_FREQ2        0x0D
#define CC1101_FREQ1        0x0E
#define CC1101_FREQ0        0x0F
#define CC1101_MDMCFG4      0x10
#define CC1101_MDMCFG3      0x11
#define CC1101_MDMCFG2      0x12
#define CC1101_DEVIATN      0x15
#define CC1101_MCSM2        0x16
#define CC1101_MCSM0        0x18
#define CC1101_FOCCFG       0x19
#define CC1101_BSCFG        0x1A
#define CC1101_AGCCTRL2     0x1B
#define CC1101_AGCCTRL1     0x1C
#define CC1101_AGCCTRL0     0x1D
#define CC1101_WOREVT1      0x1E
#define CC1101_WOREVT0      0x1F
#define CC1101_WORCTRL      0x20
#define CC1101_FREND1       0x21
#define CC1101_FREND0       0x22
#define CC1101_FSCAL3       0x23
#define CC1101_FSCAL2       0x24
#define CC1101_FSCAL1       0x25
#define CC1101_FSCAL0       0x26
#define CC1101_TEST0        0x2E

// Command strobes
#define CC1101_SRES         0x30
#define CC1101_SCAL         0x33
#define CC1101_SRX          0x34
#define CC1101_STX          0x35
#define CC1101_SIDLE        0x36
#define CC1101_SWOR         0x38
#define CC1101_SFRX         0x3A
#define CC1101_SFTX         0x3B
#define CC1101_SWORRST      0x3C
#define CC1101_SNOP         0x3D

// Status registers (read with the burst bit set)
#define CC1101_PARTNUM      0x30
#define CC1101_VERSION      0x31
#define CC1101_RXBYTES      0x3B        // Bit 7 = RX FIFO overflow, bits 6:0 = byte count

// Multi-byte registers
#define CC1101_PATABLE      0x3E
#define CC1101_TXFIFO       0x3F        // Write access
#define CC1101_RXFIFO       0x3F        // Read access

// Chip status byte STATE[2:0] (bits 6:4), datasheet Table 23
#define STATE_MASK          0x70
#define STATE_IDLE          0x00
#define STATE_TX            0x20
#define STATE_TXFIFO_UFLOW  0x70

void    spi_init(void);
uint8_t spi_xfer(uint8_t out);
uint8_t radio_wait_ready(void);
uint8_t radio_reset(void);
uint8_t radio_read_status(uint8_t addr);
void    radio_check(void);
void    radio_write_reg(uint8_t addr, uint8_t value);
uint8_t radio_read_reg(uint8_t addr);
uint8_t radio_strobe(uint8_t cmd);
uint8_t radio_configure(void);
uint8_t radio_wait_state(uint8_t state);
void    radio_write_burst(uint8_t addr, const uint8_t *data, uint8_t n);
void    radio_read_burst(uint8_t addr, uint8_t *data, uint8_t n);

#endif
