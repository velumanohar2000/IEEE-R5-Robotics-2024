#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_TCS34725.h"

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X);

void setup(void) {
  Serial.begin(115200);

  Wire.begin(7,6);

  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop(void) {
  uint16_t r, g, b, c, colorTemp, lux;

  tcs.getRawData(&r, &g, &b, &c);

  Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.println(b, DEC);

  uint16_t lowest;

  lowest = min(min(r, g), b);

  printf("Lowest: %d\n", lowest);

  r -= lowest;
  g -= lowest;
  b -= lowest;

  Serial.printf("New RGB: %d %d %d\n", r, g, b);

  neopixelWrite(RGB_BUILTIN, r, g, b);
}