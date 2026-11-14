/*
 * modem.cpp - Command link from the Pro Mini to the radio modem PCB. See modem.h.
 */
#include "modem.h"
#include <SoftwareSerial.h>

static SoftwareSerial modem_port(MODEM_RX_PIN, MODEM_TX_PIN);
static uint32_t last_send_ms;
static bool sent_before;

void modem_begin(void) {
    modem_port.begin(MODEM_BAUD);   // TX pin set high (idle) before it becomes an output
    modem_port.stopListening();     // Receive is never used: turn off the RX pin-change interrupt
}

// Send one message. Returns false if the length is out of range. Waits if the
// previous message was less than MODEM_MIN_GAP_MS ago. Interrupts are off during
// each byte (about 1.04ms at 9600), as in any SoftwareSerial write.
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

    last_send_ms = millis();
    sent_before = true;
    return true;
}
