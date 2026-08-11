#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

void setup(void) {
  u8g2.begin();
}

uint8_t counter = 0;

void loop(void) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_profont12_mf);
  u8g2.drawStr(5, 15, "Screen Test");  
  u8g2.setCursor(5, 35);
  u8g2.print("Counter: ");
  u8g2.print(counter);
  u8g2.sendBuffer();
  counter += 1;
  delay(500);
}
