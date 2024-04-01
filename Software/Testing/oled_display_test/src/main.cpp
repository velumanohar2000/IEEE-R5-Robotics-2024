#include <Wire.h>               // Only needed for Arduino 1.6.5 and earlier
#include "SSD1306Wire.h"        // legacy: #include "SSD1306.h"

SSD1306Wire display(0x3c);   // ADDRESS, SDA, SCL  -  SDA and SCL usually populate 

void setup() {
  Serial.begin(115200);
  Wire.begin(2, 3);

  // Initialising the UI will init the display too.
  display.init();
  display.flipScreenVertically();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
}

void loop() {
  // clear the display
  display.clear();
  display.drawString(0, 0, "Bit Bangers");
  display.drawString(0, 16, "Hello world again");
  display.display();
}