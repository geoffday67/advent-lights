/*
 * display_test.cpp - Stand-alone test of the I2C OLED on A4 (SDA) / A5 (SCL).
 * Built only by the [env:display_test] environment (the normal environment excludes this
 * file). Needs only the Mini, the OLED and the USB adapter.
 *
 * Serial log at 38400:
 *   1. SDA/SCL idle levels, before the I2C peripheral is enabled. Both should read 1
 *      (the OLED module's pull-ups). 0 = shorted to GND, missing pull-ups, or the
 *      module unpowered.
 *   2. I2C bus scan, 0x08-0x77. An SSD1306 answers at 0x3C (or 0x3D).
 *   3. If found: the panel is driven with a repeating sequence, 1s each:
 *      all pixels on, all off, a 1-pixel frame round the edge, then "88".
 */
#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>

static U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);
static uint8_t found_addr;

static void check_lines(void) {
    pinMode(SDA, INPUT);                // No internal pull-ups: shows the external ones
    pinMode(SCL, INPUT);
    delay(5);
    Serial.print(F("Idle levels (no internal pull-ups): SDA "));
    Serial.print(digitalRead(SDA));
    Serial.print(F(", SCL "));
    Serial.println(digitalRead(SCL));
}

static void scan(void) {
    Wire.begin();
    Wire.setWireTimeout(25000, true);   // 25ms: a stuck bus reports a timeout, not a hang
    Serial.println(F("I2C scan:"));
    uint8_t count = 0;
    for (uint8_t a = 0x08; a <= 0x77; a++) {
        Wire.beginTransmission(a);
        uint8_t err = Wire.endTransmission();
        if (err == 0) {
            Serial.print(F("  device at 0x"));
            Serial.println(a, HEX);
            count++;
            if (a == 0x3C || a == 0x3D) {
                found_addr = a;
            }
        } else if (err == 5) {
            Serial.println(F("  bus timeout: SDA or SCL held low"));
            return;
        }
    }
    if (count == 0) {
        Serial.println(F("  nothing answered: check VCC/GND to the OLED, SDA=A4, SCL=A5 (not swapped)"));
    }
}

static void start_display(void) {
    oled.setI2CAddress(found_addr * 2);         // U8g2 takes the 8-bit (shifted) address
    oled.begin();
    Serial.print(F("SSD1306 at 0x"));
    Serial.print(found_addr, HEX);
    Serial.println(F(": running pattern"));
}

void setup() {
    Serial.begin(38400);
    Serial.println(F("\r\nDisplay test"));
    check_lines();
    scan();

    if (found_addr) {
        start_display();
    } else {
        Serial.println(F("No SSD1306 found; scan repeats every 2s"));
    }
}

static void draw_step(uint8_t step) {
    oled.firstPage();
    do {
        switch (step) {
        case 0:
            oled.drawBox(0, 0, 128, 32);
            break;
        case 1:
            break;                              // All off
        case 2:
            oled.drawFrame(0, 0, 128, 32);
            break;
        default:
            oled.setFont(u8g2_font_logisoso32_tn);
            oled.drawStr(45, 32, "89");
            break;
        }
    } while (oled.nextPage());
}

void loop() {
    static const char *const names[] = { "all on", "all off", "frame", "88" };
    static uint8_t step = 0;

    if (!found_addr) {
        delay(2000);
        scan();
        if (found_addr) {
            start_display();
        }
        return;
    }

    draw_step(step);
    Serial.print(F("  "));
    Serial.println(names[step]);
    step = (uint8_t)((step + 1) % 4);
    delay(1000);
}
