/*
 * selftest.c - Startup check of the PIC16F15225's solder joints (both builds).
 *
 * What proves each pin (14-pin package):
 *   1 VDD, 4 MCLR, 12 ICSPCLK, 13 ICSPDAT, 14 VSS   Programming with verify.
 *   9 RC1 (TX, TP1)                                 The log appearing.
 *   8 RC2 (LED / J3)     RX: the LED flash at the end of the test (one long = pass,
 *                        ten short = fail). MODEM: the first command from the controller.
 *   3 RA4 SO, 5 RC5 SI, 6 RC4 SCK                   Write and read back all 256 values.
 *   7 RC3 CSn            A burst write must end when CSn goes high, and GDO1 is
 *                        only visible on SO while CSn is high.
 *   2 RA5 GDO2           GDO2 forced to 0 and to 1.
 *
 * Bridges between adjacent pins:
 *   5-6, 6-7, 7-8, 8-9, 9-10   Pin-pair test, before pins_init().
 *   1-2, 2-3                   GDO1 and GDO2 forced to opposite levels.
 *   3-4, 4-5                   Pin 4 is MCLR (LVP = ON), so a bridge resets the PIC each
 *                              time SO or SI goes low: the log restarts over and over
 *                              with reset cause MCLR.
 *   10-11, 11-12               Not tested: RC0 and RA2 are unused outputs held low, and
 *                              the PIC is held in reset while being programmed, so a
 *                              bridge there does no harm.
 *   12-13, 13-14               Programming fails.
 */

#include "board.h"
#include "uart.h"
#include "radio.h"
#include "selftest.h"

// GDOx_CFG = 0x2F: hardwired to 0; with GDOx_INV (bit 6) set, hardwired to 1
// (CC1101 datasheet Table 41).
#define GDO_HW0     0x2F
#define GDO_INV     0x40

#define RA4_BIT     (1u << 4)           // SO / GDO1
#define RA5_BIT     (1u << 5)           // GDO2

// Adjacent PORTC pins. The 'drive' pin is pulled low while the 'sense' pin is read
// with only its weak pull-up on. RC1 (UART TX) is never driven, so the serial
// adapter sees no break. RC2 is never sensed: in the RX build the LED and R3 hold
// it below the input-high threshold.
static const uint8_t pair_drive[] = { 5, 4, 2, 2, 0 };     // RCn
static const uint8_t pair_sense[] = { 4, 3, 3, 1, 1 };
#define NPAIRS      (sizeof pair_drive / sizeof pair_drive[0])
#define SENSE_MASK  ((1u << 4) | (1u << 3) | (1u << 1))

#define PIN_OF_RC(n)    (10 - (n))      // RC0 = pin 10 ... RC5 = pin 5

static uint8_t reset_cause;             // PCON0 as found at startup
static uint8_t pins_low;                // PORTC bits that read low with only the pull-up on
static uint8_t pairs_bridged;           // Bit i set: pair i bridged
static uint8_t failures;

void selftest_pins(void) {
    reset_cause = PCON0;
    PCON0 = 0x1F;                       // STKOVF = STKUNF = 0; RWDT, RMCLR, RI, POR, BOR = 1

    // Reset state: all pins inputs, analog. Make PORTC digital inputs with pull-ups.
    ANSELC = 0x00;
    LATC   = 0x00;                      // A pin made an output drives low
    WPUC   = 0x3F;
    TRISC  = 0x3F;
    __delay_us(100);

    pins_low = (uint8_t)(~PORTC & SENSE_MASK);
    pairs_bridged = 0;
    for (uint8_t i = 0; i < NPAIRS; i++) {
        uint8_t drive = (uint8_t)(1u << pair_drive[i]);
        uint8_t sense = (uint8_t)(1u << pair_sense[i]);
        if (pins_low & sense) {
            continue;                   // Already reported as low
        }
        TRISC &= (uint8_t)~drive;
        __delay_us(20);
        if (!(PORTC & sense)) {
            pairs_bridged |= (uint8_t)(1u << i);
        }
        TRISC |= drive;
        __delay_us(20);
    }

    WPUC = 0x00;                        // pins_init() sets the working directions and pull-ups
}

static void fail(const char *msg) {
    uart_puts("  FAIL ");
    uart_puts(msg);
    uart_puts("\r\n");
    failures++;
}

static void put_reset_cause(void) {
    uart_puts("Reset cause: ");
    if (!(reset_cause & 0x02)) {
        uart_puts("power-on");
    } else if (!(reset_cause & 0x08)) {
        uart_puts("MCLR (pin 4)");
    } else if (reset_cause & 0xC0) {
        uart_puts("stack over/underflow");
    } else if (!(reset_cause & 0x04)) {
        uart_puts("RESET instruction");
    } else if (!(reset_cause & 0x10)) {
        uart_puts("watchdog");
    } else {
        uart_puts("other");
    }
    uart_puts(" (PCON0 ");
    uart_puthex(reset_cause);
    uart_puts(")\r\n");
}

static void radio_tests(void) {
    if (!radio_reset()) {
        fail("radio: SO did not go low with CSn low (pin 7 RC3 CSn, pin 3 RA4 SO)");
        return;
    }

    // 1. SPI data path: every value written to FREQ1 (all 8 bits R/W) and read back.
    uint16_t bad = 0;
    for (uint16_t v = 0; v < 256; v++) {
        radio_write_reg(CC1101_FREQ1, (uint8_t)v);
        if (radio_read_reg(CC1101_FREQ1) != (uint8_t)v) {
            bad++;
        }
    }
    if (bad) {
        uart_puts("  FAIL SPI: ");
        uart_putdec((int16_t)bad);
        uart_puts(" of 256 values read back wrong (pin 3 RA4 SO, pin 5 RC5 SI, pin 6 RC4 SCK)\r\n");
        failures++;
        return;                         // The tests below need a working SPI link
    }

    // 2. CSn: a burst write only ends when CSn goes high. If CSn stays low at the
    //    radio, the single write and reads that follow are taken as more burst data.
    //    Single accesses work without CSn going high, so test 1 does not catch this.
    static const uint8_t pattern[2] = { 0xA5, 0x5A };
    radio_write_burst(CC1101_FREQ1, pattern, 2);        // FREQ1, FREQ0
    radio_write_reg(CC1101_PKTLEN, 0xC3);
    uint8_t f1 = radio_read_reg(CC1101_FREQ1);
    uint8_t f0 = radio_read_reg(CC1101_FREQ0);
    uint8_t pl = radio_read_reg(CC1101_PKTLEN);
    if (f1 != 0xA5 || f0 != 0x5A || pl != 0xC3) {
        fail("CSn: burst write did not end, so CSn is not going high at the radio (pin 7 RC3)");
    }

    // 3. GDO1 and GDO2 forced to opposite levels, read with CSn high. SO carries
    //    GDO1 only while CSn is high (datasheet Section 11.2), so with test 1 passed
    //    a wrong GDO1 level means CSn is not going high. The RA5 pull-up makes an
    //    open GDO2 joint read 1.
    WPUAbits.WPUA5 = 1;
    radio_write_reg(CC1101_IOCFG1, GDO_HW0);
    radio_write_reg(CC1101_IOCFG2, GDO_HW0 | GDO_INV);
    __delay_us(20);
    uint8_t a = PORTA;                  // Expect RA4 = 0, RA5 = 1
    radio_write_reg(CC1101_IOCFG1, GDO_HW0 | GDO_INV);
    radio_write_reg(CC1101_IOCFG2, GDO_HW0);
    __delay_us(20);
    uint8_t b = PORTA;                  // Expect RA4 = 1, RA5 = 0
    WPUAbits.WPUA5 = 0;

    uint8_t so_ok   = !(a & RA4_BIT) &&  (b & RA4_BIT);
    uint8_t gdo2_ok =  (a & RA5_BIT) && !(b & RA5_BIT);
    uint8_t same_a  = ((a & RA4_BIT) != 0) == ((a & RA5_BIT) != 0);
    uint8_t same_b  = ((b & RA4_BIT) != 0) == ((b & RA5_BIT) != 0);
    if (!so_ok && !gdo2_ok && same_a && same_b) {
        fail("pins 2 and 3 bridged (RA5 GDO2 and RA4 SO always read the same)");
    } else {
        if (!gdo2_ok) {
            fail("GDO2 not seen on pin 2 (RA5): open joint, or bridged to pin 1 (VDD)");
        }
        if (!so_ok) {
            fail("GDO1 not seen on SO with CSn high: CSn not going high at the radio (pin 7 RC3)");
        }
    }

    radio_reset();                      // Back to reset values before the real configuration
}

uint8_t selftest_run(void) {
    failures = 0;

    put_reset_cause();
    uart_puts("Self-test:\r\n");

    for (uint8_t n = 0; n < 6; n++) {
        if (pins_low & (1u << n)) {
            uart_puts("  FAIL pin ");
            uart_putdec(PIN_OF_RC(n));
            uart_puts(" (RC");
            uart_putdec(n);
            uart_puts(") reads low with only its pull-up: shorted to GND\r\n");
            failures++;
        }
    }
    for (uint8_t i = 0; i < NPAIRS; i++) {
        if (pairs_bridged & (1u << i)) {
            uart_puts("  FAIL pins ");
            uart_putdec(PIN_OF_RC(pair_drive[i]));
            uart_puts(" and ");
            uart_putdec(PIN_OF_RC(pair_sense[i]));
            uart_puts(" bridged (RC");
            uart_putdec(pair_drive[i]);
            uart_puts(", RC");
            uart_putdec(pair_sense[i]);
            uart_puts(")\r\n");
            failures++;
        }
    }

    radio_tests();

    if (failures == 0) {
        uart_puts("  PASS\r\n");
    } else {
        uart_puts("  ");
        uart_putdec(failures);
        uart_puts(" failure(s)\r\n");
    }

#if defined(ROLE_RX)
    // Pin 8 check by eye: one 200 ms flash = pass, ten short flashes = fail.
    if (failures == 0) {
        LED = 1;
        __delay_ms(200);
        LED = 0;
    } else {
        for (uint8_t i = 0; i < 10; i++) {
            LED = 1;
            __delay_ms(50);
            LED = 0;
            __delay_ms(50);
        }
    }
#endif

    return failures;
}
