#include "output.h"

#include "font/small.h"

OutputClass::OutputClass() {
}

void OutputClass::begin() {
  pDisplay = new U8G2_MAX7219_8X8_1_4W_SW_SPI(U8G2_R0, Clock, DataIn, ChipSelect, U8X8_PIN_NONE);
  pDisplay->begin();
  pDisplay->setContrast(255);
  pDisplay->setDisplayRotation(U8G2_R2);
  setFont(NORMAL);
}

void OutputClass::drawCentre(char c) {
  int x;

  switch (font) {
    case NORMAL:
      x = 1;
      break;
    default:
      x = 0;
      break;
  }
  pDisplay->clearBuffer();
  pDisplay->drawGlyph(x, 8, c);
  pDisplay->sendBuffer();
}

void OutputClass::drawText(char *ptext) {
  pDisplay->clearBuffer();
  pDisplay->drawStr(0, 8, ptext);
  pDisplay->sendBuffer();
}

void OutputClass::flush() {
  pDisplay->sendBuffer();
}

void OutputClass::clear() {
  pDisplay->clearBuffer();
}

void OutputClass::blank() {
  pDisplay->setPowerSave(true);
}

void OutputClass::restore() {
  pDisplay->setPowerSave(false);
}

void OutputClass::setFont(OutputFont font) {
  this->font = font;

  switch (font) {
    case SMALL:
      pDisplay->setFont(small);
      break;
    case NORMAL:
      pDisplay->setFont(u8g2_font_bitcasual_tf);
      break;
    case ICON:
      pDisplay->setFont(u8g2_font_open_iconic_embedded_1x_t);
      break;
      
  }
}

OutputClass Output;