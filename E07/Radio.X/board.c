/*
 * board.c - Configuration bits, clock and pin setup.
 */

// CONFIG1
#pragma config FEXTOSC = OFF    // External Oscillator Mode Selection bits (Oscillator not enabled; RA5 free for I/O)
#pragma config RSTOSC = HFINTOSC_32MHZ // Power-up Default Value for COSC bits
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; I/O function on RA4)
#pragma config VDDAR = LO       // VDD Range Analog Calibration Selection bit

// CONFIG2
#pragma config MCLRE = EXTMCLR  // Master Clear Enable bit (If LVP = 0, MCLR pin is MCLR; If LVP = 1, RA3 pin function is MCLR)
#pragma config PWRTS = PWRT_OFF // Power-up Timer Selection bits (PWRT is disabled)
#pragma config WDTE = OFF
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

/*
 * PIC16F15225, PDIP-14 (top view)
 *
 *                 +----U----+
 *           VDD --| 1    14 |-- VSS
 *           RA5 --| 2    13 |-- RA0/ICSPDAT
 *           RA4 --| 3    12 |-- RA1/ICSPCLK
 * /MCLR/VPP/RA3 --| 4    11 |-- RA2
 *           RC5 --| 5    10 |-- RC0
 *           RC4 --| 6     9 |-- RC1
 *           RC3 --| 7     8 |-- RC2
 *                 +---------+
 */

/*
 * Receiver PCB pin map
 *   RA5 - GDO2 (IOC wake in RX)
 *   RA4 - MISO
 *   RC5 - MOSI
 *   RC4 - SCK
 *   RC3 - CS
 *   RC2 - RX: LED output (high = on).  MODEM: command input, EUSART RX (J3 pin 1, via R3)
 *   RC1 - UART TX log (TP1)
 *   RC0 - Unused
 *   RA2 - Unused
 *
 * Unused pins are driven low as outputs to avoid floating-input current in sleep.
 */

#include "board.h"

void clock_init(void) {
    // RSTOSC = HFINTOSC_32MHZ starts with OSCFRQ = 32 MHz; drop to 4 MHz.
    OSCFRQ = 0b010;                     // FRQ = 010: 4 MHz (DS Section 11.3.4)
    while (!OSCSTATbits.HFOR);          // Wait for HFINTOSC to lock at the new frequency
}

void pins_init(void) {
    // Analog select: all digital except the ICSP pins. RA0/RA1 are left analog
    // so their digital input buffers are off when the programmer is unplugged.
    ANSELA = 0b00000011;                // RA0, RA1 analog; RA2-RA5 digital
    ANSELC = 0b00000000;                // All PORTC digital

    // Output latches before enabling drivers, so nothing glitches.
    LATA   = 0b00000000;                // RA2 low
    LATC   = 0b00001010;                // RC3 (CS) high = radio deselected; RC1 (UART TX) high = idle;
                                        // RC4 (SCK) low = idle for CKP=0; RC5 (MOSI), RC2, RC0 low

    // Directions (1 = input).
    TRISA  = 0b00111011;                // RA0, RA1 (ICSP), RA3 (MCLR), RA4 (MISO), RA5 (GDO2) inputs; RA2 output
#if defined(ROLE_MODEM)
    TRISC  = 0b00000100;                // RC2 command input; rest outputs
#else
    TRISC  = 0b00000000;                // All PORTC outputs (RC2 = LED)
#endif

    // Weak pull-ups.
    WPUA   = 0b00010000;                // RA4 MISO (CC1101 SO is high-Z while CSn is high)
#if defined(ROLE_MODEM)
    WPUC   = 0b00000100;                // RC2: holds the command line at UART idle (high) if unplugged
#else
    WPUC   = 0b00000000;
#endif

    // Peripheral Pin Select. PPSLOCKED is clear after reset, so no unlock is needed.
    RC4PPS     = 0x07;                  // RC4 <- SCK1 output
    SSP1CLKPPS = 0x14;                  // SCK1 input <- RC4 (must match the output pin in host mode)
    RC5PPS     = 0x08;                  // RC5 <- SDO1 (MOSI)
    SSP1DATPPS = 0x04;                  // SDI1 (MISO) <- RA4
    RC1PPS     = 0x05;                  // RC1 <- TX1
#if defined(ROLE_MODEM)
    RX1PPS     = 0x12;                  // RX1 input <- RC2
#endif

    // Lock PPS. GIE is still clear here, as the sequence requires.
    // With PPS1WAY = ON this lasts until the next reset.
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 1;
}
