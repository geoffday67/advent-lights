#pragma once

#include <U8g2lib.h>

enum OutputFont {
  SMALL, NORMAL, ICON
};

class OutputClass {
 private:
  const int Clock = 4;
  const int ChipSelect = 2;
  const int DataIn = 3;

  U8G2_MAX7219_8X8_1_4W_SW_SPI *pDisplay;
  OutputFont font;

 public:
  OutputClass();

  void begin();
  void drawText(char*);
  void drawCentre(char);
  void flush();
  void clear();
  void blank();
  void restore();
  void setFont(OutputFont);
};

extern OutputClass Output;