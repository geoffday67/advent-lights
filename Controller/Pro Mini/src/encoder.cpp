/*
 * encoder.cpp - Rotary encoder with push switch. See encoder.h.
 *
 * Both encoder pins interrupt on every change. Each interrupt reads A and B together
 * from PIND and looks up the transition from the previous state: +1 or -1 for a valid
 * single-bit change, 0 for no change or a two-bit jump (a transition missed, e.g. while
 * interrupts are held off).
 *
 * Detents are counted at the rest state, not by dividing a running total. The
 * transitions since the encoder last left a rest state are summed; when it arrives at
 * a rest state the sum decides: at least half a detent forward = +1, at least half
 * back = -1, otherwise 0 (bounce, or a partial turn that came back). The sum then
 * restarts, so a missed transition can't leave the count out of step with the clicks.
 */
#include "encoder.h"

#define REST_AB         0x03        // A = B = 1: contacts open at a detent
#define HALF_DETENT     (ENCODER_STEPS_PER_DETENT / 2)

static volatile uint16_t raw;       // All transitions, wraps; for testing only
static volatile uint8_t  prev_ab;
static volatile int8_t   since_rest;
static volatile int8_t   detents;   // Not yet taken by encoder_take()

// Index = previous AB << 2 | new AB, with A in bit 0 and B in bit 1.
static const int8_t transition[16] = {
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0,
};

static inline bool is_rest(uint8_t ab) {
#if ENCODER_STEPS_PER_DETENT == 2
    return ab == REST_AB || ab == 0x00;
#else
    return ab == REST_AB;
#endif
}

static void encoder_isr(void) {
    uint8_t ab = (PIND >> 2) & 0x03;    // D2 = PD2 = A, D3 = PD3 = B
    int8_t t = transition[(prev_ab << 2) | ab];
    prev_ab = ab;
    raw += (uint16_t)t;
    since_rest += t;

    if (is_rest(ab)) {
        if (since_rest >= HALF_DETENT) {
            detents++;
        } else if (since_rest <= -HALF_DETENT) {
            detents--;
        }
        since_rest = 0;
    }
}

void encoder_begin(void) {
    pinMode(ENCODER_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_B_PIN, INPUT_PULLUP);
    pinMode(ENCODER_SW_PIN, INPUT_PULLUP);
    prev_ab = (PIND >> 2) & 0x03;
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoder_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), encoder_isr, CHANGE);
}

uint16_t encoder_raw(void) {
    noInterrupts();
    uint16_t r = raw;
    interrupts();
    return r;
}

uint8_t encoder_ab(void) {
    return (PIND >> 2) & 0x03;
}

int8_t encoder_take(void) {
    noInterrupts();
    int8_t d = detents;
    detents = 0;
    interrupts();
#if ENCODER_REVERSE
    return (int8_t)-d;
#else
    return d;
#endif
}

// Before power-down: remove the A/B pull-ups so no current can flow through a closed
// contact, whatever position the knob is left in or turned to while asleep (each closed
// contact otherwise draws about 100uA through the internal pull-up at 3.3V).
// With the pull-ups off, an open contact leaves its pin floating. That is safe only
// because INT0/INT1 are disabled here as well: in power-down the input buffers are
// clamped (the SLEEP signal, ATmega328P "Digital Input Enable and Sleep Modes"), but
// not on pins enabled as external interrupts. D2/D3 are not in PCMSK2 either, so their
// buffers are clamped and a floating level draws no current.
// Relies on the encoder board's own A/B pull-up resistors being removed (see encoder.h).
void encoder_sleep(void) {
    EIMSK &= (uint8_t)~(_BV(INT0) | _BV(INT1));
    pinMode(ENCODER_A_PIN, INPUT);
    pinMode(ENCODER_B_PIN, INPUT);
}

// After wake: pull-ups back on, let them settle, clear any edge flags set while the
// interrupts were off, re-enable INT0/INT1 and resync, so the mode change and any
// turning while asleep don't count as clicks.
void encoder_wake(void) {
    pinMode(ENCODER_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_B_PIN, INPUT_PULLUP);
    delayMicroseconds(100);
    EIFR = _BV(INTF0) | _BV(INTF1);
    EIMSK |= _BV(INT0) | _BV(INT1);
    encoder_reset();
}

void encoder_reset(void) {
    noInterrupts();
    prev_ab = (PIND >> 2) & 0x03;
    since_rest = 0;
    detents = 0;
    interrupts();
}

// ---------------------------------------------------------------------------
// Push switch: debounced, short and long presses
// ---------------------------------------------------------------------------

static bool     sw_stable_closed;
static bool     sw_last_reading;
static bool     sw_used;            // Current press already reported as long
static bool     sw_short_ok = true; // false for the waking press: its release is not a short press
static uint32_t sw_changed_ms;
static uint32_t sw_pressed_ms;

bool button_is_down(void) {
    return digitalRead(ENCODER_SW_PIN) == LOW;
}

ButtonEvent button_event(void) {
    uint32_t now = millis();
    bool closed = button_is_down();

    if (closed != sw_last_reading) {
        sw_last_reading = closed;
        sw_changed_ms = now;
    }

    if (closed != sw_stable_closed && now - sw_changed_ms >= BUTTON_DEBOUNCE_MS) {
        sw_stable_closed = closed;
        if (closed) {
            sw_pressed_ms = now;
            sw_used = false;
            sw_short_ok = true;
        } else if (!sw_used && sw_short_ok) {
            return BUTTON_SHORT;
        }
    }

    if (sw_stable_closed && !sw_used && now - sw_pressed_ms >= BUTTON_LONG_MS) {
        sw_used = true;
        return BUTTON_LONG;
    }
    return BUTTON_NONE;
}

// Take over the press that woke the controller: treat the switch as pressed from now.
// Releasing it gives no event (waking is all a short press does), but holding it for
// BUTTON_LONG_MS still gives BUTTON_LONG, so a long press works straight from sleep.
void button_wake_press(void) {
    uint32_t now = millis();
    sw_stable_closed = true;
    sw_last_reading = true;
    sw_used = false;
    sw_short_ok = false;
    sw_changed_ms = now;
    sw_pressed_ms = now;
}

void button_wait_release(void) {
    uint32_t open_since = millis();
    while (millis() - open_since < BUTTON_DEBOUNCE_MS) {
        if (button_is_down()) {
            open_since = millis();
        }
    }
    sw_stable_closed = false;
    sw_last_reading = false;
}
