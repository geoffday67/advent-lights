#include <Arduino.h>

#include "cc1101.h"

#define RADIO_MOSI 23
#define RADIO_MISO 19
#define RADIO_CLK 18
#define RADIO_CS 4
#define RADIO_PACKET 16  // We'll get an interrupt here when there's a packet available.

#define WAIT_ALL pdTRUE
#define WAIT_ONE pdFALSE
#define CLEAR_ON_EXIT pdTRUE
#define NO_CLEAR pdFALSE

#define RADIO_RECEIVED 0x04

CC1101 cc1101;
EventGroupHandle_t eventGroup;

#define ARDUINO_STACK 8192
#define ARDUINO_PRIORITY 1
#define ARDUINO_CORE 1

TaskHandle_t startTask(TaskFunction_t pcode, const char* const pname, void* const pparameters = 0) {
  TaskHandle_t handle;
  xTaskCreatePinnedToCore(
      pcode,
      pname,
      ARDUINO_STACK,
      pparameters,
      ARDUINO_PRIORITY,
      &handle,
      ARDUINO_CORE);
  return handle;
}

char s[64];

void radioTask(void* pparameters) {
  while (1) {
    xEventGroupWaitBits(eventGroup, RADIO_RECEIVED, CLEAR_ON_EXIT, WAIT_ALL, portMAX_DELAY);

    if (cc1101.receiveAvailable() >= 6) {  // 4 data bytes (1st is address) + 2 status bytes
      cc1101.fetchRx(s, 6);

      if (s[5] & 0x80) {
        s[4] = 0;
        Serial.printf("Received: [%s]\n", s + 1);
      } else {
        Serial.println("CRC error");
      }
    }

    cc1101.enableWor();
  }
}

void ARDUINO_ISR_ATTR handlePacket() {
  BaseType_t woken = pdFALSE;

  xEventGroupSetBitsFromISR(eventGroup, RADIO_RECEIVED, &woken);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting");

  eventGroup = xEventGroupCreate();
  startTask(radioTask, "radioTask");
  Serial.println("Tasks created");

  // We get a HIGH on GDO0 from the CC1101 when there's a packet available.
  // So set an interrupt on the connected pin.
  pinMode(RADIO_PACKET, INPUT);
  attachInterrupt(RADIO_PACKET, handlePacket, HIGH);

  cc1101.begin(RADIO_MOSI, RADIO_MISO, RADIO_CLK, RADIO_CS);
  cc1101.reset();
  cc1101.initialise();
  Serial.printf("CC1101 initialized, version 0x%02X\n", cc1101.getVersion());

  cc1101.enableWor();
  Serial.println("WOR enabled");
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}