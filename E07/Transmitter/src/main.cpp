#include <Arduino.h>

#include "cc1101.h"

#define RADIO_MOSI 23
#define RADIO_MISO 19
#define RADIO_CLK 18
#define RADIO_CS 4

CC1101 cc1101;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting");

  cc1101.begin(RADIO_MOSI, RADIO_MISO, RADIO_CLK, RADIO_CS);
  cc1101.reset();
  cc1101.initialise();
  Serial.println("CC1101 initialized");

  // Enter WOR mode — GDO0 will pulse every ~90ms on each WOR_EVNT0 wake.
  //cc1101.enableWor();
  //Serial.println("WOR enabled");
}

void loop() {
  cc1101.enableTx();
  delay(120); // 120ms of preamble to ensure that listeners get it since they wake to check every 100ms.
  cc1101.loadTx("25", 2);

  delay(1000);
}