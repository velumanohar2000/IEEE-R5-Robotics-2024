#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void setup() {
  Wire.begin(9, 8);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
}

void displayText(String text, char station) {
  display.clearDisplay();
  display.setTextSize(3);
  display.setTextColor(WHITE);
  display.setCursor(55, 6);
  display.printf("%c",station);
  display.setCursor(13, 36);
  display.printf("%s", text);
  display.display();
}

void loop() {
  displayText("(1, 0)", 'A');
}