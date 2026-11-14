/*
 * Advent lights controller - Pro Mini 3.3V / 8MHz.
 * Modem link test: send the 'GD' test packet to the modem PCB every SEND_PERIOD_MS.
 * On D9 each message is 00 53 02 47 44 8D (wake, 'S', len, 'G', 'D', checksum).
 * The receiver PCBs toggle their LED on each packet they get.
 *
 * Pins (planned): D0/D1 USB serial log, D2/D3 encoder A/B, D4 encoder switch,
 * A4/A5 I2C display, D9 modem command output, D8 unused SoftwareSerial RX.
 */
#include <Arduino.h>
#include "modem.h"

// Test rate.
static const uint32_t SEND_PERIOD_MS = 2000;

void setup() {
    Serial.begin(38400);
    Serial.println(F("\r\nController: modem link test"));
    modem_begin();
}

void loop() {
    static const uint8_t payload[] = { 'G', 'D' };
    static uint16_t count = 0;

    modem_send(payload, sizeof payload);
    Serial.print(F("MODEM: sent GD #"));
    Serial.println(++count);
    delay(SEND_PERIOD_MS);
}
