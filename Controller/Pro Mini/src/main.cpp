/*
 * Advent lights controller - Pro Mini 3.3V / 8MHz.
 *
 * Two user interfaces, chosen with UI_FOLLOW at build time. Common to both:
 *   - Asleep: ATmega328P in power-down, OLED off. A press of the encoder button wakes it.
 *   - Wake: the stored day is restored and shown. Releasing the waking press does
 *     nothing more; holding it for BUTTON_LONG_MS is a long press (all off, sleep), so
 *     the trees can be turned off from sleep with one long press.
 *   - Turn the encoder to change the day (DAY_MIN..DAY_MAX, stops at the ends).
 *   - Long press (BUTTON_LONG_MS): send day 0 = all trees off, then sleep.
 *
 * UI_FOLLOW 1 - trees follow the encoder:
 *   - Each change is sent as soon as the modem can take it (at most one message per
 *     MODEM_MIN_GAP_MS); if the day changes again meanwhile, only the latest is sent.
 *   - FOLLOW_IDLE_MS with no turn: the latest day is sent if it hasn't been, and the
 *     controller sleeps.
 *   - Short press: send the shown day (even if unchanged, e.g. to turn the trees back on
 *     after a long press) and sleep at once.
 *   - Every sleep (idle, short or long press) stores the shown day in EEPROM, so the
 *     next wake shows it. The EEPROM byte is only written if the day changed.
 *
 * UI_FOLLOW 0 - confirm with a press:
 *   - Nothing is sent while turning. Short press: store and send the day, then sleep.
 *   - CONFIRM_IDLE_MS with no turn or press: sleep without sending; the choice is dropped.
 *   - Long press keeps the stored (last confirmed) day.
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

#define UI_FOLLOW           1

// Day number range. 0 is reserved for "all trees off"; trees are numbered 1-25.
#define DAY_MIN             0
#define DAY_MAX             25
#define DAY_DEFAULT         1       // Used if the EEPROM holds no valid day

#define CMD_DAY             'D'     // Payload: CMD_DAY, day
#define DAY_ALL_OFF         0
#define DAY_NONE            0xFF    // last_sent_day: nothing sent since waking

#define FOLLOW_IDLE_MS      5000UL  // UI_FOLLOW: no turn for this long -> store and sleep
#define CONFIRM_IDLE_MS     30000UL // Confirm UI: no turn or press -> sleep without sending
#define EEPROM_DAY_ADDR     0

// current_day: shown on the display and changed by the encoder.
// stored_day: last day stored in EEPROM, shown again at the next wake.
uint8_t current_day;
uint8_t stored_day;

static uint8_t  last_sent_day;
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

// Send a day command. modem_send() waits if the previous message was too recent.
static void send_day(uint8_t d) {
    uint8_t payload[2] = { CMD_DAY, d };
    modem_send(payload, sizeof payload);
    last_sent_day = d;
    Serial.print(F("SEND: day "));
    Serial.println(d);
}

static void store_day(void) {
    stored_day = current_day;
    EEPROM.update(EEPROM_DAY_ADDR, stored_day);     // Writes only if changed
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

// Restore the stored day and show it. The press that woke us only wakes, unless it is
// held for BUTTON_LONG_MS, which counts as a long press (all off) straight from sleep.
static void wake_up(void) {
    encoder_reset();
    button_wake_press();
    current_day = stored_day;
    last_sent_day = DAY_NONE;
    display_power(true);
    Serial.println(F("WAKE"));
    show_day();
    last_activity_ms = millis();
}

// UI_FOLLOW: the shown day is stored every time the controller goes to sleep,
// whatever sent it there (idle, short press or long press), so the next wake shows it.
// The confirm UI only stores on a short press; anything unconfirmed is dropped.
static void sleep_then_wake(void) {
#if UI_FOLLOW
    store_day();
#endif
    sleep_until_press();
    wake_up();
}

void setup() {
    Serial.begin(38400);
    Serial.println(F("\r\nController"));
    modem_begin();
    encoder_begin();
    display_begin();

    stored_day = EEPROM.read(EEPROM_DAY_ADDR);
    if (stored_day > DAY_MAX) {         // Erased EEPROM reads 0xFF
        stored_day = DAY_DEFAULT;
    }

    // Start as if just woken, so the stored day is shown.
    wake_up();
}

void loop() {
    // Detents turned while the display was being redrawn or a message was going out
    // are kept by the encoder module and picked up on the next pass.
    int8_t steps = encoder_take();
    if (steps != 0) {
        last_activity_ms = millis();
        if (day_adjust(steps)) {
            show_day();
        }
    }

    ButtonEvent button = button_event();

    if (button == BUTTON_LONG) {
        send_day(DAY_ALL_OFF);
        sleep_then_wake();
        return;
    }

#if UI_FOLLOW
    // A changed day goes out as soon as the modem can take it. Only the stored day at
    // wake counts as "already on the trees" if nothing has been sent since; the first
    // change after waking is always sent.
    bool changed = current_day != (last_sent_day == DAY_NONE ? stored_day : last_sent_day);
    if (changed && modem_ready()) {
        send_day(current_day);
    }

    if (button == BUTTON_SHORT) {
        send_day(current_day);          // Even if unchanged: re-sends after a long press
        sleep_then_wake();
        return;
    }

    if (millis() - last_activity_ms >= FOLLOW_IDLE_MS) {
        if (current_day != (last_sent_day == DAY_NONE ? stored_day : last_sent_day)) {
            send_day(current_day);      // Waits for the modem gap if needed
        }
        Serial.println(F("IDLE"));
        sleep_then_wake();
    }
#else
    if (button == BUTTON_SHORT) {
        store_day();
        send_day(current_day);
        sleep_then_wake();
        return;
    }

    if (button != BUTTON_NONE || button_is_down()) {
        last_activity_ms = millis();
    }
    if (millis() - last_activity_ms >= CONFIRM_IDLE_MS) {
        Serial.println(F("IDLE: no change sent"));
        sleep_then_wake();
    }
#endif
}
