/*
 * File:   serial.h
 * Author: geoff
 *
 * Interrupt-driven UART TX on RA5 via PPS.
 * 9600 baud, 8N1, at 1MHz HFINTOSC.
 */

#ifndef SERIAL_H
#define SERIAL_H

#ifdef __cplusplus
extern "C" {
#endif

void serial_init(void);
void serial_print(const char *s);
void serial_flush(void);
void serial_tx_isr(void);

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_H */
