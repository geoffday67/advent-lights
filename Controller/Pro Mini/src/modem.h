/*
 * modem.h - Command link from the Pro Mini to the radio modem PCB (PIC MODEM build).
 *
 * One way, 9600 8N1 on D9 (AltSoftSerial TX) -> modem J3 pin 1; GND -> J3 pin 2.
 * Each message is a wake byte 0x00, then straight after it a frame:
 *   'S', len (1..MODEM_MAX_PAYLOAD), payload[len], checksum = (len + sum of payload) & 0xFF
 * The modem consumes the 0x00 to wake from Sleep (EUSART auto-wake-up on break), then
 * transmits the payload with a 300ms preamble. It is not listening while it transmits,
 * so messages are spaced at least MODEM_MIN_GAP_MS apart.
 */
#ifndef MODEM_H
#define MODEM_H

#include <Arduino.h>

#define MODEM_TX_PIN        9       // AltSoftSerial's fixed TX pin on the ATmega328
#define MODEM_RX_PIN        8       // AltSoftSerial's fixed RX pin; unused and unconnected
#define MODEM_BAUD          9600
#define MODEM_MAX_PAYLOAD   32
#define MODEM_MIN_GAP_MS    400     // 300ms preamble + the modem's log line on TP1

void modem_begin(void);
bool modem_send(const uint8_t *payload, uint8_t len);

#endif
