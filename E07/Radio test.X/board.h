/*
 * board.h - Clock, pins and pin access for the radio test board.
 */
#ifndef BOARD_H
#define BOARD_H

#define _XTAL_FREQ 4000000     // Must match OSCFRQ set in clock_init() (HFINTOSC 4 MHz)

#include <xc.h>
#include <stdint.h>

// Pin access
#define RADIO_CS    LATAbits.LATA5      // CC1101 CSn, active low
#define RADIO_SO    PORTCbits.RC5       // CC1101 SO/MISO, read directly to see CHIP_RDYn
#define RADIO_GDO2  PORTCbits.RC3       // CC1101 GDO2
#define BUTTON      PORTCbits.RC2       // Low when pressed

void clock_init(void);
void pins_init(void);

#endif
