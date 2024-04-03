/*
 * Bit Bangers TCS34725 Color Detection Test for Round 2
 * github.com/Bit-Bangers-UTA/Senior-Design
 *
 * Authors:
 * Rolando Rosales
 * 
 * Comments:
 * Requires adafruit/Adafruit TCS34725@^1.4.4 library
*/

// Libraries ------------------------------------------------------------------

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_TCS34725.h"

// Defines --------------------------------------------------------------------

// I2C Pins
#define SDA 14
#define SCL 15

// Color states
#define RED 1
#define ORANGE 2
#define YELLOW 3
#define GREEN 4
#define GRAY 5
#define BLACK 6
#define PURPLE 7
#define BLUE 8

// Variables & Constants ------------------------------------------------------

// Ambient color values
const uint16_t r_amb = 10;
const uint16_t g_amb = 15;
const uint16_t b_amb = 15;

// Structures & Classes -------------------------------------------------------

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);

// Functions ------------------------------------------------------------------

void setup(void) {
  Serial.begin(115200);

  Wire.begin(SDA, SCL);

  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop(void) {
  uint16_t r_raw, g_raw, b_raw, c;
  uint8_t start_color = 0;

  tcs.getRawData(&r_raw, &g_raw, &b_raw, &c);

  Serial.println("Raw:");
  Serial.print("R: "); Serial.print(r_raw, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g_raw, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.println(b_raw, DEC);

  int16_t r = r_raw - r_amb;
  int16_t g = g_raw - g_amb;
  int16_t b = b_raw - b_amb;

  if (r < 0) r = 0;
  if (g < 0) g = 0;
  if (b < 0) b = 0;

  Serial.println("Raw - amb:");
  Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.println(b, DEC);

  uint16_t lowest;

  lowest = min(min(r, g), b);

  r -= lowest;
  g -= lowest;
  b -= lowest;

  Serial.println("Calibrated:");
  Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.println(b, DEC);

  if (r > 60)
  {
    if (g < 30)
    {
      start_color = RED;
    }
    else if ((r - g) < 30)
    {
      start_color = YELLOW;
    }
    else if (g > 20)
    {
      start_color = ORANGE;
    }
  }
  else if (b > 30)
  {
    if (r < 5)
    {
      start_color = BLUE;
    }
    else
    {
      start_color = PURPLE;
    }
  }
  else if (g > 30)
  {
    start_color = GREEN;
  }
  else if (r < 5 && g < 5 && b < 5)
  {
    start_color = BLACK;
  }
  else
  {
    start_color = GRAY;
  }
  

  Serial.print("Color is: ");
  switch (start_color)
  {
    case RED:
      Serial.println("Red");
      break;
    case ORANGE:
      Serial.println("Orange");
      break;
    case YELLOW:
      Serial.println("Yellow");
      break;
    case GREEN:
      Serial.println("Green");
      break;
    case GRAY:  
      Serial.println("Gray");
      break;
    case BLACK: 
      Serial.println("Black");
      break;
    case PURPLE:  
      Serial.println("Purple");
      break;
    case BLUE:  
      Serial.println("Blue");
      break;
    default: 
      Serial.println("Unknown");
      break;
  }

  // neopixelWrite(RGB_BUILTIN, r, g, b);
  Serial.println();
}