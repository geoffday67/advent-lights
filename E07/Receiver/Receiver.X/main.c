/*
 * File:   main.c
 * Author: geoff
 *
 * Created on April 24, 2026, 8:23 PM
 */

// PIC16F15225 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FEXTOSC = OFF
#pragma config RSTOSC = HFINTOSC_1MHZ// Power-up Default Value for COSC bits (HFINTOSC (1 MHz))
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; I/O function on RA4)
#pragma config VDDAR = HI       // VDD Range Analog Calibration Selection bit (Internal analog systems are calibrated for operation between VDD = 2.3V - 5.5V)

// CONFIG2
#pragma config MCLRE = EXTMCLR  // Master Clear Enable bit (If LVP = 0, MCLR pin is MCLR; If LVP = 1, RA3 pin function is MCLR)
#pragma config PWRTS = PWRT_OFF // Power-up Timer Selection bits (PWRT is disabled)
#pragma config WDTE = OFF       // WDT Operating Mode bits (WDT disabled; SEN is ignored)
#pragma config BOREN = OFF      // Brown-out Reset Enable bits (Brown-out Reset disabled)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection bit (Brown-out Reset Voltage (VBOR) set to 1.9V)
#pragma config PPS1WAY = ON     // PPSLOCKED One-Way Set Enable bit (The PPSLOCKED bit can be set once after an unlocking sequence is executed; once PPSLOCKED is set, all future changes to PPS registers are prevented)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a reset)

// CONFIG3

// CONFIG4
#pragma config BBSIZE = BB512   // Boot Block Size Selection bits (512 words boot block size)
#pragma config BBEN = OFF       // Boot Block Enable bit (Boot Block is disabled)
#pragma config SAFEN = OFF      // SAF Enable bit (SAF is disabled)
#pragma config WRTAPP = OFF     // Application Block Write Protection bit (Application Block is not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block is not write-protected)
#pragma config WRTC = OFF       // Configuration Registers Write Protection bit (Configuration Registers are not write-protected)
#pragma config WRTSAF = OFF     // Storage Area Flash (SAF) Write Protection bit (SAF is not write-protected)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/Vpp pin function is MCLR. MCLRE Configuration bit is ignored.)

// CONFIG5
#pragma config CP = OFF         // User Program Flash Memory Code Protection bit (User Program Flash Memory code protection is disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#define DAY_NUMBER 2

#define _XTAL_FREQ 1000000

#include <xc.h>
#include <stdio.h>

#include "radio.h"
#include "serial.h"

char s[32];

volatile uint8_t packet_ready = 0;

void __interrupt() isr(void) {
    // IOC interrupt: GDO0 (RC3) went high - packet received into FIFO.
    if (IOCCFbits.IOCCF3) {
        IOCCFbits.IOCCF3 = 0;
        packet_ready = 1;
    }
    serial_tx_isr();
}

void handleDay(const char *pday) {
    int d = (pday[0] - '0') * 10 + (pday[1] - '0');

    if (d >= DAY_NUMBER) {
        LATCbits.LATC4 = 1;
    } else {
        LATCbits.LATC4 = 0;
    }
}

void main(void) {
    ANSELA = 0x00;
    TRISA = 0b00000100; // RA2 (MISO) input; RA5 driven by EUSART via PPS.
    PORTA = 0b00000000;

    ANSELC = 0x00;
    TRISC = 0b00001000; // RC3 (GDO0) input; RC0/RC1/RC2 SPI/CS outputs.
    PORTC = 0b00000100; // CS idle high.

    serial_init();
    serial_print("Starting\r\n");

    initialise();
    sprintf(s, "CC1101 version 0x%02X\r\n", radio_version());
    serial_print(s);

    // GDO0 is on RC3, configured in CC1101 to assert high when a packet is
    // in the RX FIFO. Trigger an interrupt on the rising edge.
    IOCCPbits.IOCCP3 = 1; // Rising edge.
    IOCCFbits.IOCCF3 = 0; // Clear any stale flag before enabling.
    PIE0bits.IOCIE = 1; // Enable IOC interrupt (GIE/PEIE set by serial_init).

    enableWor();
    serial_print("Listening\r\n");

    char data[3];

    while (1) {
        serial_flush();
        SLEEP();
        if (packet_ready) {
            packet_ready = 0;
            // GDO0 stays high while the FIFO is non-empty, so drain all
            // queued packets before returning to WOR sleep.
            while (PORTCbits.RC3) {
                if (receiveValid(data)) {
                    sprintf(s, "RX via WOR: %c%c%c\r\n", data[0], data[1], data[2]);
                    serial_print(s);

                    if (data[0] == 'D') {
                        handleDay(data + 1);
                    }
                }
            }
            enableWor();
        }
    }
}
