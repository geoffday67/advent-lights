#include <Arduino.h>
#include <EEPROM.h>
#include <debouncer.h>
#include <rotary.h>

#include "output.h"

#define IR_PIN 13
#define OFF_PIN 6

#define ENCODER_CLK 11
#define ENCODER_DT 10
#define ENCODER_BUTTON 9

#define COMMAND_DAY 1
#define COMMAND_OFF 2

uint8_t day;
Rotary encoder(ENCODER_CLK, ENCODER_DT);

void setDay(uint8_t new_day) {
  day = new_day;
  EEPROM.write(0, day);

  Serial.print("Day = ");
  Serial.println(day);
}

void xmitSync() {
  digitalWrite(IR_PIN, HIGH);
  delayMicroseconds(2400);
  digitalWrite(IR_PIN, LOW);
  delayMicroseconds(600);
}

void xmitZero() {
  digitalWrite(IR_PIN, HIGH);
  delayMicroseconds(600);
  digitalWrite(IR_PIN, LOW);
  delayMicroseconds(600);
}

void xmitOne() {
  digitalWrite(IR_PIN, HIGH);
  delayMicroseconds(1200);
  digitalWrite(IR_PIN, LOW);
  delayMicroseconds(600);
}

void xmitCommand(uint8_t command, uint8_t data) {
  int n, m;

  uint8_t send = (command << 5) | (data & 0x1F);

  xmitSync();
  for (m = 0; m < 3; m++) {
    for (n = 0; n < 8; n++) {
      if (send & 0x80) {
        xmitOne();
      } else {
        xmitZero();
      }
      send <<= 1;
    }

    delay(45);
  }
}

void xmitCommand(uint8_t command) {
  xmitCommand(command, 0);
}

int getEncoder() {
  return digitalRead(ENCODER_BUTTON) == 0;
}

void encoderPressed(int value) {
  if (value) {
    Serial.println("Sending day");
    xmitCommand(COMMAND_DAY, day);
  }
}

Debouncer encoderDebouncer(getEncoder, encoderPressed, 100);

int getOff() {
  return digitalRead(OFF_PIN) == 0;
}

void offChanged(int value) {
  if (value) {
    Serial.println("All off");
    xmitCommand(COMMAND_OFF);
  }
}

Debouncer offDebouncer(getOff, offChanged, 100);

void showDay() {
  char s[16];

  if (day <= 9) {
    Output.setFont(NORMAL);
    Output.drawCentre(day + '0');
  } else {
    Output.setFont(SMALL);
    sprintf(s, "%d", day);
    Output.drawText(s);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Starting");

  Output.begin();
  Output.setFont(NORMAL);
  Serial.println("Output initialised");

  day = EEPROM.read(0);
  if (day == 256) {
    setDay(1);
  }
  showDay();
  Serial.println("Day initialised");

  encoder.begin();
  Serial.println("Encoder initialised");

  pinMode(IR_PIN, OUTPUT);
  digitalWrite(IR_PIN, LOW);
  Serial.println("IR initialised");

  pinMode(OFF_PIN, INPUT_PULLUP);
  Serial.println("Off initialised");
}

void loop() {
  int direction;

  encoderDebouncer.loop();
  offDebouncer.loop();

  switch ((direction = encoder.getDirection())) {
    case 1:
      if (day < 24) {
        setDay(day + 1);
        showDay();
      }
      break;

    case -1:
      if (day > 1) {
        setDay(day - 1);
        showDay();
      }
      break;
  }

  return;
}