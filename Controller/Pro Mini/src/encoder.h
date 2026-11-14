/*
 * encoder.h - Rotary encoder with push switch.
 * A = D2 (INT0), B = D3 (INT1), switch = D4, common to GND. Inputs use INPUT_PULLUP.
 */
#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

#define ENCODER_A_PIN           2
#define ENCODER_B_PIN           3
#define ENCODER_SW_PIN          4

// Quadrature transitions per detent: 4 for the common EC11 / KY-040 type, which rests
// with both contacts open (A = B = 1 with the pull-ups) at every detent. Set 2 for the
// type that rests alternately at A = B = 1 and A = B = 0.
#define ENCODER_STEPS_PER_DETENT 4

#define BUTTON_DEBOUNCE_MS      20
#define BUTTON_LONG_MS          1000    // Held this long = long press

enum ButtonEvent : uint8_t {
    BUTTON_NONE,
    BUTTON_SHORT,       // Released before BUTTON_LONG_MS
    BUTTON_LONG,        // Reported as soon as BUTTON_LONG_MS is reached; release then ignored
};

void        encoder_begin(void);
void        encoder_reset(void);            // Resync to the current pin state, drop pending detents
int8_t      encoder_take(void);             // Detents turned since the last call (+ / -)
uint16_t    encoder_raw(void);              // Raw transition count, for testing
uint8_t     encoder_ab(void);               // Current A (bit 0) and B (bit 1), for testing

ButtonEvent button_event(void);             // Call often; returns each event once
void        button_suppress(void);          // Treat the current press as used: no event for it
bool        button_is_down(void);           // Raw switch state (closed = true)
void        button_wait_release(void);      // Block until open and stable for BUTTON_DEBOUNCE_MS

#endif
