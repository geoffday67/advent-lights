#include <Arduino.h>
#include <avr/sleep.h>
#include <rotary.h>
#include <U8g2lib.h>

#include "cc1101.h"

#define SLEEP_TIMEOUT_MS 5000

#define RADIO_MOSI 3
#define RADIO_MISO 2
#define RADIO_CLK 4
#define RADIO_CS 5

#define ENCODER_CLK 7
#define ENCODER_DT 6
#define ENCODER_BUTTON ??

CC1101 cc1101;
Rotary encoder(ENCODER_CLK, ENCODER_DT);
U8G2_SSD1306_128X64_NONAME_2_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);
int day;
unsigned long lastActivityMs;

ISR(PCINT2_vect) {
  // wakes the CPU; encoder is processed in loop() after wake
}

void updateDisplay() {
  char buf[3];
  sprintf(buf, "%02d", day);
  display.firstPage();
  do {
    display.drawStr(0, 63, buf);
  } while (display.nextPage());
}

void sendDay(int day) {
  char buf[3];
  sprintf(buf, "D%02d", day);
  cc1101.idle();
  cc1101.enableTx();
  delay(120);
  cc1101.loadTx(buf, 3, 1);
}

void setDay (int newDay) {
  day = newDay;
  updateDisplay();
}

void goToSleep() {
  display.setPowerSave(1);

  PCMSK2 |= (1 << PCINT22) | (1 << PCINT23);  // pins 6 and 7
  PCICR  |= (1 << PCIE2);

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_cpu();
  sleep_disable();

  PCICR  &= ~(1 << PCIE2);

  display.setPowerSave(0);
  updateDisplay();
  lastActivityMs = millis();
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting");

  cc1101.begin(RADIO_MOSI, RADIO_MISO, RADIO_CLK, RADIO_CS);
  cc1101.reset();
  cc1101.initialise();
  Serial.println("CC1101 initialized");

  encoder.begin();
  Serial.println("Encoder initialized");

  day = 0;
  Serial.println("Day initialized");

  display.setBusClock(400000);
  display.begin();
  display.setFont(u8g2_font_profont29_tr);
  updateDisplay();
  lastActivityMs = millis();
  Serial.println("Display initialized");
}


void loop() {
  cc1101.idle();
  cc1101.enableTx();
  delay(120);
  cc1101.loadTx("D99", 3, 1);

  delay(1000);
}

// void loop() {
//   int n = encoder.getDirection();
//   if (n != 0) {
//     int newDay = constrain(day + n, 0, 26);
//     if (newDay != day) {
//       setDay(newDay);
//       sendDay(newDay);
//     }
//     lastActivityMs = millis();
//   }

//   // if (millis() - lastActivityMs > SLEEP_TIMEOUT_MS) {
//   //   goToSleep();
//   // }
// }
