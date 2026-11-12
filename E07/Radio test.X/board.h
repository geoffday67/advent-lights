/*
 * board.h - Clock, pins and pin access.
 *
 * ROLE_RX: receiver PCB (advent_rx v1).
 * ROLE_TX: stripboard test transmitter.
 * The two boards wire the PIC to the radio differently; see board.c for the maps.
 */
#ifndef BOARD_H
#define BOARD_H

#define _XTAL_FREQ 4000000     // Must match OSCFRQ set in clock_init() (HFINTOSC 4 MHz)

#include <xc.h>
#include <stdint.h>

#if defined(ROLE_RX)
// Receiver PCB
#define RADIO_CS        LATCbits.LATC3      // CC1101 CSn, active low
#define RADIO_SO        PORTAbits.RA4       // CC1101 SO/MISO, read directly to see CHIP_RDYn
#define RADIO_GDO2      PORTAbits.RA5       // CC1101 GDO2
#define GDO2_IOC_POS    IOCAPbits.IOCAP5    // Interrupt-on-change enables and flag for GDO2
#define GDO2_IOC_NEG    IOCANbits.IOCAN5
#define GDO2_IOC_FLAG   IOCAFbits.IOCAF5
#define LED             LATCbits.LATC2      // High = LED on (through R3)
#elif defined(ROLE_TX)
// Stripboard test transmitter
#define RADIO_CS        LATAbits.LATA5      // CC1101 CSn, active low
#define RADIO_SO        PORTCbits.RC5       // CC1101 SO/MISO, read directly to see CHIP_RDYn
#define RADIO_GDO2      PORTCbits.RC3       // CC1101 GDO2
#define BUTTON          PORTCbits.RC2       // Low when pressed
#else
#error "Define ROLE_RX or ROLE_TX"
#endif

void clock_init(void);
void pins_init(void);

#endif
