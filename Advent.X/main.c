
// PIC16F15225 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FEXTOSC = OFF    // External Oscillator Mode Selection bits (EC (external clock) 16 MHz and above)
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

#include <xc.h>

#include <stdio.h>
#include <stdlib.h>

#define IR_PIN ((uint8_t) 0x10)
#define LED_PIN ((uint8_t) 0x04)

#define STATE_IDLE 0
#define STATE_SEQUENCE 1
#define STATE_COMPLETE 2

#define COMMAND_DAY 1
#define COMMAND_OFF 2

volatile uint8_t state;
volatile int sequence[12];
volatile uint8_t bit_number;

void __interrupt() pulseISR() {
    uint8_t width;

    // Input goes low when an IR pulse is detected.
    if (PORTA & IR_PIN) {
        // IR pulse ended, read the counter and calculate the pulse width.
        width = TMR0L;

        switch (state) {
            case STATE_IDLE:
                if (width > 72 && width < 82) {
                    bit_number = 0;
                    state = STATE_SEQUENCE;
                }
                break;
            case STATE_SEQUENCE:
                if (width > 72 && width < 82) {
                    // Start the sequence again if we get a sync pulse during a sequence.
                    bit_number = 0;
                } else if (width > 33 && width < 43) {
                    sequence[bit_number++] = 1;
                } else if (width > 14 && width < 24) {
                    sequence[bit_number++] = 0;
                }
                if (bit_number > 7) {
                    state = STATE_COMPLETE;
                }
                break;
            case STATE_COMPLETE:
                // Waiting for another process to handle the current sequence.
                break;
        }
    } else {
        // IR pulse detected, start the counter.
        TMR0L = 0x00;
    }

    IOCAF = 0x00;
}

int getID() {
    int id;

    // Enable low 5 bits of port C for input with pull-ups, read value, disable pull-ups.
    TRISC = 0x1F;
    ANSELC = 0xE0;
    WPUC = 0x1F;

    id = (PORTC & 0x1F) ^ 0x1F;

    WPUC = 0x00;

    return id;
}

void handleOn(int data) {
    if (getID() <= data) {
        PORTA = LED_PIN;
    } else {
        PORTA = 0x00;
    }
}

void handleOff() {
    PORTA = 0x00;
}

int main(int argc, char** argv) {
    int command, data;

    state = STATE_IDLE;

    TRISA = IR_PIN;
    ANSELA = ~IR_PIN;
    WPUA = 0x00;
    PORTA = 0x00;

    // Enable timer 0 at 31KHz and 256 prescalar, so that IR pulses are a sensible number of ticks long.
    T0CON0 = 0b10000000;
    T0CON1 = 0b10010000;

    // Enable interrupts on both edges of IR pin.
    IOCAN = IR_PIN;
    IOCAP = IR_PIN;

    PIE0 = 0b00010000;
    INTCON = 0b10000000;

    while (1) {
        while (state != STATE_COMPLETE) {
            SLEEP();
        }

        command = (4 * sequence[0]) + (2 * sequence[1]) + sequence[2];
        data = (16 * sequence[3]) + (8 * sequence[4]) + (4 * sequence[5]) + (2 * sequence[6]) + sequence[7];

        switch (command) {
            case COMMAND_DAY:
                handleOn(data);
                break;
            case COMMAND_OFF:
                handleOff();
                break;
        }

        state = STATE_IDLE;
    }
}