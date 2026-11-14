/*
 * board.h - Clock, pins and pin access for the receiver PCB (advent_rx v1).
 *
 * Two build configurations, both on the same PCB:
 *   ROLE_RX     tree receiver: wake-on-radio, LED on RC2 (J3).
 *   ROLE_MODEM  radio modem for the controller: commands in on RC2 (J3 pin 1),
 *               transmitted with a long preamble. Wakes on EUSART auto-wake-up.
 */
#ifndef BOARD_H
#define BOARD_H

#define _XTAL_FREQ 4000000     // Must match OSCFRQ set in clock_init() (HFINTOSC 4 MHz)

#include <xc.h>
#include <stdint.h>

#if !defined(ROLE_RX) && !defined(ROLE_MODEM)
#error "Define ROLE_RX or ROLE_MODEM"
#endif

// Radio (same on both roles)
#define RADIO_CS        LATCbits.LATC3      // CC1101 CSn, active low
#define RADIO_SO        PORTAbits.RA4       // CC1101 SO/MISO, read directly to see CHIP_RDYn
#define RADIO_GDO2      PORTAbits.RA5       // CC1101 GDO2
#define GDO2_IOC_POS    IOCAPbits.IOCAP5    // Interrupt-on-change enables and flag for GDO2
#define GDO2_IOC_NEG    IOCANbits.IOCAN5
#define GDO2_IOC_FLAG   IOCAFbits.IOCAF5

#if defined(ROLE_RX)
#define LED             LATCbits.LATC2      // High = LED on (through R3)
#else
#define CMD_PIN         PORTCbits.RC2       // Command input level (idle high)
#define CMD_IOC_POS     IOCCPbits.IOCCP2    // Rising edge: only used to wait for a line held low
#define CMD_IOC_FLAG    IOCCFbits.IOCCF2
#endif

// EUSART1 baud rate generator (BRG16 = 1, BRGH = 1): baud = 4 MHz / (4 * (n + 1)).
#if defined(ROLE_MODEM)
#define UART_SPBRG      103                 // 9615 baud (+0.16 %): commands in, log out
#else
#define UART_SPBRG      25                  // 38462 baud (+0.16 %): log out
#endif

void clock_init(void);
void pins_init(void);

#endif
