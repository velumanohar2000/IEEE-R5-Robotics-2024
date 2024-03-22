#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <SPI.h>

#define RED_PIN 9
#define GREEN_PIN 10
#define BLUE_PIN 11



Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void setup()
{
  Serial.begin(115200);
  // ledcSetup(0, 5000, 8); // channel 0, 5000 Hz, 8-bit resolution
  // ledcSetup(1, 5000, 8); // channel 1, 5000 Hz, 8-bit resolution
  // ledcSetup(2, 5000, 8); // channel 2, 5000 Hz, 8-bit resolution

  // ledcAttachPin(RED_PIN, 0);   // attach RED_PIN to channel 0
  // ledcAttachPin(GREEN_PIN, 1); // attach GREEN_PIN to channel 1
  // ledcAttachPin(BLUE_PIN, 2);  // attach BLUE_PIN to channel 2
  
  //Serial.println("Color View Test!");

  if (tcs.begin()) {
    //Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }
}

void loop()
{
  uint16_t r, g, b, c, colorTemp, lux;
  tcs.getRawData(&r, &g, &b, &c);
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b,c);
  lux = tcs.calculateLux(r,g,b);

  Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
  Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
  Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
  Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
  Serial.println(" ");

  float rx, gx, bx;
  tcs.getRGB(&rx, &gx, &bx);

  neopixelWrite(LED_BUILTIN, rx, 0, 0);
  neopixelWrite(LED_BUILTIN, 0, gx, 0);
  neopixelWrite(LED_BUILTIN, 0, 0, bx);

}

// /*!
//  * @file  colorview.ino
//  * @brief Gets the ambient light color
//  * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
//  * @license     The MIT License (MIT)
//  * @author      PengKaixing(kaixing.peng@dfrobot.com)
//  * @version     V1.0.0
//  * @date        2022-03-16
//  * @url         https://github.com/DFRobot/DFRobot_TCS34725
//  */

// #include <Arduino.h>
// #include "DFRobot_TCS34725.h"

// DFRobot_TCS34725 tcs = DFRobot_TCS34725(&Wire, TCS34725_ADDRESS,TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
// void setup() 
// {
//   Serial.begin(115200);
//   Serial.println("Color View Test!");

//   while(!tcs.begin())
//   {
//     Serial.println("No TCS34725 found ... check your connections");
//     delay(1000);
//   }
// }

// void loop() {
//   uint16_t clear, red, green, blue;
//   tcs.getRGBC(&red, &green, &blue, &clear);
//   // turn off LED
//   tcs.lock();  
//   Serial.print("C:\t"); Serial.print(clear);
//   Serial.print("\tR:\t"); Serial.print(red);
//   Serial.print("\tG:\t"); Serial.print(green);
//   Serial.print("\tB:\t"); Serial.print(blue);
//   Serial.println("\t");
  
//   // Figure out some basic hex code for visualization
//   uint32_t sum = clear;
//   float r, g, b;
//   r = red; r /= sum;
//   g = green; g /= sum;
//   b = blue; b /= sum;
//   r *= 256; g *= 256; b *= 256;
//   Serial.print("\t");
//   Serial.print((int)r, HEX); Serial.print((int)g, HEX); Serial.print((int)b, HEX);
//   Serial.println();
// }