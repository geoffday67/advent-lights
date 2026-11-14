/*
 * display.h - 0.91in 128x32 I2C OLED (SSD1306), U8g2 in page-buffer mode.
 * Hardware I2C: A4 = SDA, A5 = SCL.
 */
#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>

void display_begin(void);
void display_show_number(uint8_t n);    // 0..99, centred, full display height
void display_power(bool on);            // false = panel off (SSD1306 sleep), contents kept

#endif
