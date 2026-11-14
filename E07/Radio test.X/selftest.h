/*
 * selftest.h - Startup check of the PIC16F15225's solder joints (both builds).
 */
#ifndef SELFTEST_H
#define SELFTEST_H

#include <stdint.h>

void    selftest_pins(void);    // Call straight after clock_init(), before pins_init()
uint8_t selftest_run(void);     // Call after uart_init() and spi_init(); returns the failure count

#endif
