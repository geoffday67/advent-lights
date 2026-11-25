/*
 * Advent lights controller - Pro Mini 3.3V / 8MHz.
 *
 * Two user interfaces, chosen with UI_FOLLOW at build time. Common to both:
 *   - Asleep: ATmega328P in power-down, OLED off. A press of the encoder button wakes it.
 *   - Wake: the stored day is restored, shown, and sent to the trees once the waking
 *     press is released, so trees turned off by a long press come back to it. Holding
 *     the waking press for BUTTON_LONG_MS is a long press instead (all off, sleep).
 *   - Every command is sent SEND_REPEATS times (currently 1), MODEM_MIN_GAP_MS apart.
 *     A newer command replaces copies of an older one not yet sent.
 *   - Turn the encoder to change the day (DAY_MIN..DAY_MAX, stops at the ends).
 *   - Long press (BUTTON_LONG_MS): send day 0 = all trees off, then sleep.
 *
 * UI_FOLLOW 1 - trees follow the encoder:
 *   - Each change is queued at once and sent as the modem allows (one message per
 *     MODEM_MIN_GAP_MS); if the day changes again meanwhile, only the latest is sent.
 *   - FOLLOW_IDLE_MS with no turn: any remaining copies are sent and the controller
 *     sleeps.
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
#define DAY_MIN             1
#define DAY_MAX             24
#define DAY_DEFAULT         1       // Used if the EEPROM holds no valid day

#define CMD_DAY             'D'     // Payload: CMD_DAY, day
#define DAY_ALL_OFF         0

// Copies of each command, sent MODEM_MIN_GAP_MS apart. Was 3 for trees at the edge of
// range; set to 1 on 22 Nov 2026 after the trees' WOR change (MCSM2 0x0D) fixed range.
// Raise it again if trees miss commands: repeating is harmless, a tree just sets its
// LED to the same state again.
#define SEND_REPEATS        1

#define FOLLOW_IDLE_MS      5000UL  // UI_FOLLOW: no turn for this long -> store and sleep
#define CONFIRM_IDLE_MS     30000UL // Confirm UI: no turn or press -> sleep without sending
#define EEPROM_DAY_ADDR     0

// current_day: shown on the display and changed by the encoder.
// stored_day: last day stored in EEPROM, shown again at the next wake.
uint8_t current_day;
uint8_t stored_day;

static uint32_t last_activity_ms;

// Command being sent and the copies still to go. A new command replaces any copies
// of the previous one that haven't gone yet.
static uint8_t tx_day;
static uint8_t tx_left;

// Set at wake: send the shown day once the waking press is released (unless it turns
// into a long press, which sends all-off instead).
static bool wake_send_pending;

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

// Queue a day command for SEND_REPEATS copies.
static void queue_day(uint8_t d) {
    tx_day = d;
    tx_left = SEND_REPEATS;
}

// Send one copy of the queued command. modem_send() waits if the previous message
// was less than MODEM_MIN_GAP_MS ago.
static void send_copy(void) {
    uint8_t payload[2] = { CMD_DAY, tx_day };
    modem_send(payload, sizeof payload);
    tx_left--;
    Serial.print(F("SEND: day "));
    Serial.print(tx_day);
    Serial.print(F(" ("));
    Serial.print(SEND_REPEATS - tx_left);
    Serial.print(F("/"));
    Serial.print(SEND_REPEATS);
    Serial.println(F(")"));
}

// Non-blocking: send the next copy only if the modem can take it now.
static void service_tx(void) {
    if (tx_left && modem_ready()) {
        send_copy();
    }
}

// Blocking: send all remaining copies (about 0.4s each), e.g. before sleeping.
static void flush_tx(void) {
    while (tx_left) {
        send_copy();
    }
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
    encoder_sleep();                    // No pull-up current through a closed contact

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

    encoder_wake();
    ADCSRA = adcsra;
}

// Restore the stored day and show it. The press that woke us only wakes, unless it is
// held for BUTTON_LONG_MS, which counts as a long press (all off) straight from sleep.
static void wake_up(void) {
    encoder_reset();
    button_wake_press();
    current_day = stored_day;
    tx_left = 0;
    wake_send_pending = true;
    display_power(true);
    Serial.println(F("WAKE"));
    show_day();
    last_activity_ms = millis();
}

// UI_FOLLOW: the shown day is stored every time the controller goes to sleep,
// whatever sent it there (idle, short press or long press), so the next wake shows it.
// The confirm UI only stores on a short press; anything unconfirmed is dropped.
// All queued copies are sent before sleeping.
static void sleep_then_wake(void) {
    flush_tx();
#if UI_FOLLOW
    store_day();
#endif
    sleep_until_press();
    wake_up();
}

// Reset cause, for diagnosing unexpected restarts. MCUSR (ATmega328P MCU Status Register)
// keeps its flags until cleared, unless the bootloader clears it first.
//
// RESET_LOG 1: each boot's MCUSR is also saved in an 8-entry ring in EEPROM (bytes
// RESET_LOG_ADDR..+7, next-slot index at RESET_LOG_ADDR + 8), and the whole history is
// printed at start-up, oldest first. So restarts that happen with the serial adapter
// unplugged can be read later. Connecting the adapter resets the Mini through DTR,
// so the newest entry is then normally "reset pin". Two EEPROM writes per boot.
#define RESET_LOG           0
#define RESET_LOG_ADDR      1
#define RESET_LOG_SIZE      8

static void print_cause(uint8_t mcusr) {
    Serial.print(F("0x"));
    Serial.print(mcusr, HEX);
    Serial.print(F(":"));
    if (mcusr & _BV(PORF))  Serial.print(F(" power-on"));
    if (mcusr & _BV(EXTRF)) Serial.print(F(" reset-pin"));
    if (mcusr & _BV(BORF))  Serial.print(F(" brown-out"));
    if (mcusr & _BV(WDRF))  Serial.print(F(" watchdog"));
    if (mcusr == 0)         Serial.print(F(" none (cleared by bootloader, or a jump to address 0)"));
    Serial.println();
}

static void print_reset_cause(void) {
    uint8_t mcusr = MCUSR;
    MCUSR = 0;

#if RESET_LOG
    uint8_t next = EEPROM.read(RESET_LOG_ADDR + RESET_LOG_SIZE);
    if (next >= RESET_LOG_SIZE) {               // Erased EEPROM: start a fresh log
        next = 0;
        for (uint8_t i = 0; i < RESET_LOG_SIZE; i++) {
            EEPROM.update(RESET_LOG_ADDR + i, 0xFF);
        }
    }
    EEPROM.update(RESET_LOG_ADDR + next, mcusr);
    next = (uint8_t)((next + 1) % RESET_LOG_SIZE);
    EEPROM.update(RESET_LOG_ADDR + RESET_LOG_SIZE, next);

    Serial.println(F("Reset log, oldest first:"));
    for (uint8_t i = 0; i < RESET_LOG_SIZE; i++) {
        uint8_t v = EEPROM.read(RESET_LOG_ADDR + (next + i) % RESET_LOG_SIZE);
        if (v == 0xFF) {
            continue;                           // Unused slot
        }
        Serial.print(F("  "));
        print_cause(v);
    }
#endif

    Serial.print(F("Reset cause (this boot) "));
    print_cause(mcusr);
}

void setup() {
    Serial.begin(38400);
    Serial.println(F("\r\nController"));
    print_reset_cause();
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
#if UI_FOLLOW
            queue_day(current_day);     // Trees follow; replaces any older pending copies
#endif
        }
    }

    ButtonEvent button = button_event();

    if (button == BUTTON_LONG) {
        wake_send_pending = false;
        queue_day(DAY_ALL_OFF);
        sleep_then_wake();
        return;
    }

    // Wake command: once the waking press is released (not a long press), send the shown
    // day, so trees left all off by a long press go back to it without choosing again.
    if (wake_send_pending && !button_is_down()) {
        wake_send_pending = false;
        queue_day(current_day);
    }

    service_tx();

#if UI_FOLLOW
    if (button == BUTTON_SHORT) {
        queue_day(current_day);         // Even if unchanged: re-sends the shown day
        sleep_then_wake();
        return;
    }

    if (millis() - last_activity_ms >= FOLLOW_IDLE_MS) {
        Serial.println(F("IDLE"));
        sleep_then_wake();              // Sends any remaining copies first
    }
#else
    if (button == BUTTON_SHORT) {
        store_day();
        queue_day(current_day);
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
