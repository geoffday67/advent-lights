/*
 * display.cpp - 0.91in 128x32 I2C OLED (SSD1306). See display.h.
 */
#include "display.h"
#include <U8g2lib.h>
#include <Wire.h>

// Page-buffer mode (_1_): 128 bytes of RAM instead of 512 for the full frame buffer.
static U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

// Logisoso 32, digits only: each digit glyph is 16x32 pixels, 20 pixels advance,
// sitting on the baseline, so the digits fill all 32 rows of the display.
// u8g2_font_fub30_tn is a bolder alternative: 20x31 glyphs, 23 pixels advance.
static const uint8_t *const NUMBER_FONT = u8g2_font_logisoso32_tn;

// Christmas tree, 25 pixels wide and 32 high, centred on column cx: three stacked
// filled triangles and a 5x5 trunk.
static void draw_tree(int16_t cx) {
    oled.drawTriangle(cx, 0,  cx - 7,  11, cx + 7,  11);
    oled.drawTriangle(cx, 6,  cx - 10, 19, cx + 10, 19);
    oled.drawTriangle(cx, 13, cx - 12, 27, cx + 12, 27);
    oled.drawBox(cx - 2, 27, 5, 5);
}

// Tree centres, symmetric about the ink of a centred two-digit number (columns 47-82),
// leaving a 16-pixel gap on each side. Fixed, so they don't move with the number.
#define TREE_LEFT_CX    18
#define TREE_RIGHT_CX   111

void display_begin(void) {
    oled.begin();
}

void display_show_number(uint8_t n) {
    char text[4];
    snprintf(text, sizeof text, "%u", n);

    oled.setFont(NUMBER_FONT);
    int16_t x = (oled.getDisplayWidth() - oled.getStrWidth(text)) / 2;

    oled.firstPage();
    do {
        oled.drawStr(x, 32, text);      // y is the baseline: digits occupy rows 0..31
        draw_tree(TREE_LEFT_CX);
        draw_tree(TREE_RIGHT_CX);
    } while (oled.nextPage());
}

void display_power(bool on) {
    oled.setPowerSave(on ? 0 : 1);
}
