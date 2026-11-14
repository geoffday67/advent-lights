/*
 * Advent lights controller - Pro Mini 3.3V / 8MHz.
 *
 * Operation (trees don't follow the encoder; a command is sent only on a press):
 *   - Asleep: ATmega328P in power-down, OLED off. A press of the encoder button wakes it.
 *   - Wake: the last confirmed day is restored and shown. The waking press is ignored.
 *   - Turn the encoder to choose a day (DAY_MIN..DAY_MAX, stops at the ends).
 *   - Short press: confirm. The day is saved (EEPROM) and sent to the trees; then sleep.
 *   - Long press (BUTTON_LONG_MS): send day 0 = all trees off; then sleep. The confirmed
 *     day is kept, so the next wake shows it again and one press turns the trees back on.
 *   - No activity for IDLE_TIMEOUT_MS: sleep without sending; the choice is discarded.
 *
 * Tree command (modem payload): 'D', day. Day 0 = all off.
 *
 * Pins: D0/D1 USB serial log, D2/D3 encoder A/B, D4 encoder switch,
 * A4/A5 I2C display, D9 modem command output, D8 unused AltSoftSerial RX.
 */
#include <Arduino.h>
#include <EEPROM.h>
#include <avr/sleep.h>
#include "modem.h"
#include "display.h"
#include "encoder.h"

// Day number range. 0 is reserved for "all trees off"; trees are numbered 1-25.
#define DAY_MIN             0
#define DAY_MAX             25
#define DAY_DEFAULT         1       // Used if the EEPROM holds no valid day

#define CMD_DAY             'D'     // Payload: CMD_DAY, day
#define DAY_ALL_OFF         0

#define IDLE_TIMEOUT_MS     30000UL // Awake with no turn or press for this long -> sleep
#define EEPROM_DAY_ADDR     0

// The day being chosen (changed by the encoder, shown on the display) and the
// last day confirmed with a short press (sent to the trees, kept in EEPROM).
uint8_t current_day;
uint8_t confirmed_day;

static uint32_t last_activity_ms;

static void show_day(void) {
    display_show_number(current_day);
    Serial.print(F("DAY: "));
    Serial.println(current_day);
}

// Apply a number of encoder detents to current_day, clamped to DAY_MIN..DAY_MAX.
// Returns true if it changed.
static bool day_adjust(int8_t steps) {
    int16_t d = (int16_t)current_day + steps;
    if (d < DAY_MIN) {
        d = DAY_MIN;
    } else if (d > DAY_MAX) {
        d = DAY_MAX;
    }
    if (d == current_day) {
        return false;
    }
    current_day = (uint8_t)d;
    return true;
}

static void send_day(uint8_t d) {
    uint8_t payload[2] = { CMD_DAY, d };
    modem_send(payload, sizeof payload);
    Serial.print(F("SEND: day "));
    Serial.println(d);
}

// Pin-change interrupt on D4 (PCINT20) exists only to wake from power-down.
ISR(PCINT2_vect) {
}

// Power down until the encoder button is pressed. INT0/INT1 edge interrupts need the
// I/O clock, so turning the encoder does not wake it; only the D4 pin change does.
static void sleep_until_press(void) {
    Serial.println(F("SLEEP"));
    Serial.flush();
    display_power(false);
    button_wait_release();              // A held button would wake it again at once

    uint8_t adcsra = ADCSRA;
    ADCSRA &= (uint8_t)~_BV(ADEN);      // ADC off for the lowest power-down current

    do {
        PCIFR = _BV(PCIF2);             // Clear any stale pin-change flag
        PCMSK2 |= _BV(PCINT20);         // D4
        PCICR |= _BV(PCIE2);

        set_sleep_mode(SLEEP_MODE_PWR_DOWN);
        cli();
        sleep_enable();
        sleep_bod_disable();            // BOD off while asleep (ATmega328P), back on at wake
        sei();
        sleep_cpu();
        sleep_disable();

        PCICR &= (uint8_t)~_BV(PCIE2);
        PCMSK2 &= (uint8_t)~_BV(PCINT20);
        delay(BUTTON_DEBOUNCE_MS);
    } while (!button_is_down());        // Woken by noise or a bounce: sleep again

    ADCSRA = adcsra;
}

// Restore the confirmed day and show it; the press that woke us is not a confirm.
static void wake_up(void) {
    encoder_reset();
    button_suppress();
    current_day = confirmed_day;
    display_power(true);
    Serial.println(F("WAKE"));
    show_day();
    last_activity_ms = millis();
}

void setup() {
    Serial.begin(38400);
    Serial.println(F("\r\nController"));
    modem_begin();
    encoder_begin();
    display_begin();

    confirmed_day = EEPROM.read(EEPROM_DAY_ADDR);
    if (confirmed_day > DAY_MAX) {      // Erased EEPROM reads 0xFF
        confirmed_day = DAY_DEFAULT;
    }

    // Start as if just woken, so the stored day is shown and can be confirmed.
    wake_up();
}

void loop() {
    // Detents turned while the display was being redrawn are kept by the encoder
    // module and picked up on the next pass, so fast turns aren't lost.
    int8_t steps = encoder_take();
    if (steps != 0) {
        last_activity_ms = millis();
        if (day_adjust(steps)) {
            show_day();
        }
    }

    switch (button_event()) {
    case BUTTON_SHORT:
        confirmed_day = current_day;
        EEPROM.update(EEPROM_DAY_ADDR, confirmed_day);  // Writes only if changed
        send_day(confirmed_day);
        sleep_until_press();
        wake_up();
        return;

    case BUTTON_LONG:
        send_day(DAY_ALL_OFF);
        sleep_until_press();
        wake_up();
        return;

    case BUTTON_NONE:
        break;
    }

    if (millis() - last_activity_ms >= IDLE_TIMEOUT_MS) {
        Serial.println(F("IDLE: no change sent"));
        sleep_until_press();
        wake_up();
    }
}
