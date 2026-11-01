#include <Arduino.h>
#include "cc1101.h"

#define RADIO_MOSI 7
#define RADIO_MISO 2
#define RADIO_CLK 6
#define RADIO_CS 10

CC1101 cc1101;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting");

  cc1101.begin(RADIO_MOSI, RADIO_MISO, RADIO_CLK, RADIO_CS);
  cc1101.reset();
  cc1101.initialise();
  Serial.println("CC1101 initialized");
}

void loop() {
  cc1101.idle();
  cc1101.flushTx();
  cc1101.enableTx();
  delay(120);
  cc1101.loadTx("D99", 3, 1);

  delay(1000);
}
