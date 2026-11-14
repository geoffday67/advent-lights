/*
 * modem.cpp - Command link from the Pro Mini to the radio modem PCB. See modem.h.
 */
#include "modem.h"
#include <AltSoftSerial.h>

// AltSoftSerial times each bit with Timer1 output compare instead of a delay loop with
// interrupts off, so the encoder interrupts keep running while a message goes out.
// On the ATmega328 its pins are fixed: TX = D9, RX = D8 (input capture, unused here).
// Timer1 is taken, so D10 can't do PWM.
static AltSoftSerial modem_port;
static uint32_t last_send_ms;
static bool sent_before;

void modem_begin(void) {
    modem_port.begin(MODEM_BAUD);
    pinMode(MODEM_RX_PIN, INPUT_PULLUP);    // RX unused and unconnected: hold it idle-high
}

// True if a message sent now would go straight out, without modem_send() waiting
// for MODEM_MIN_GAP_MS since the previous one.
bool modem_ready(void) {
    return !sent_before || millis() - last_send_ms >= MODEM_MIN_GAP_MS;
}

// Send one message. Returns false if the length is out of range. Waits if the
// previous message was less than MODEM_MIN_GAP_MS ago. Bytes are queued and sent by
// Timer1 interrupts; the function returns once they are all out (flush).
bool modem_send(const uint8_t *payload, uint8_t len) {
    if (len < 1 || len > MODEM_MAX_PAYLOAD) {
        return false;
    }

    if (sent_before) {
        uint32_t since = millis() - last_send_ms;
        if (since < MODEM_MIN_GAP_MS) {
            delay(MODEM_MIN_GAP_MS - since);
        }
    }

    uint8_t sum = len;
    modem_port.write((uint8_t)0x00);    // Wake byte: must be all zeros
    modem_port.write((uint8_t)'S');
    modem_port.write(len);
    for (uint8_t i = 0; i < len; i++) {
        modem_port.write(payload[i]);
        sum += payload[i];
    }
    modem_port.write(sum);
    modem_port.flush();                 // Wait until the last byte has gone

    last_send_ms = millis();
    sent_before = true;
    return true;
}
