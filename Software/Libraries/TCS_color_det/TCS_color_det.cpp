/*
 * Bit Bangers TCS34725 Color Detection Library
 * github.com/Bit-Bangers-UTA/Senior-Design
 *
 * Authors:
 * Rolando Rosales
 * 
 * Comments:
 * Use these to debug:
 * #define TCS_PRINT_DEBUG
 * #define TCS_ENABLE_LED
 * Libraries:
 * Requires adafruit/Adafruit TCS34725@^1.4.4 library
*/

// Libraries ------------------------------------------------------------------

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_TCS34725.h"

// Defines --------------------------------------------------------------------

// Preprocessor Directives
#define TCS_PRINT_DEBUG
#define TCS_ENABLE_LED

// Variables & Constants ------------------------------------------------------

// Ambient color values
uint16_t r_amb = 0;
uint16_t g_amb = 0;
uint16_t b_amb = 0;

// Structures & Classes -------------------------------------------------------

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);

// Functions ------------------------------------------------------------------

void initTCS(uint16_t r, uint16_t g, uint16_t b) {
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }

  r_amb = r;
  g_amb = g;
  b_amb = b;
}

uint8_t getColorCode(void) {
  uint16_t r_raw, g_raw, b_raw, c;
  uint8_t color = 0;

  tcs.getRawData(&r_raw, &g_raw, &b_raw, &c); // gets raw color values

  #ifdef TCS_PRINT_DEBUG
    Serial.println("Raw:");
    Serial.print("R: "); Serial.print(r_raw, DEC); Serial.print(" ");
    Serial.print("G: "); Serial.print(g_raw, DEC); Serial.print(" ");
    Serial.print("B: "); Serial.println(b_raw, DEC);
  #endif

  // adjusts color values depending on ambient light
  int16_t r = r_raw - r_amb;
  int16_t g = g_raw - g_amb;
  int16_t b = b_raw - b_amb;

  // if any number is negative, it will float to 0
  if (r < 0) r = 0;
  if (g < 0) g = 0;
  if (b < 0) b = 0;

  #ifdef TCS_PRINT_DEBUG
    Serial.println("Raw - Ambient:");
    Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
    Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
    Serial.print("B: "); Serial.println(b, DEC);
  #endif

  // will filter out the lowest value from the RGB values
  uint16_t lowest = min(min(r, g), b);

  r -= lowest;
  g -= lowest;
  b -= lowest;

  #ifdef TCS_PRINT_DEBUG
    Serial.println("Calibrated:");
    Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
    Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
    Serial.print("B: "); Serial.println(b, DEC);
  #endif

  if (r > 60) // checks warm colors
  {
    if (g < 20) // if the green value is too low, its red
    {
      color = RED;
    }
    else if ((r - g) < 50) // y is bright. if r and g are close its yellow
    {
      color = YELLOW;
    }
    else
    {
      color = ORANGE; // if g is kinda high, but not close to r its orange
    }
  }
  else if (b > 30) // checks blue colors
  {
    if (g - r)
      color = PURPLE;
    else
      color = BLUE;
    /**
    if (r < 5) // if there's barely any red, its blue
    {
      color = BLUE;
    }
    else
    {
      color = PURPLE; // if there's noticably some red, its purple
    }
    */
  }
  else if (g > 25) // looks for green
  {
    color = GREEN;
  }
  else if (r < 13 && g < 13 && b < 13)
  {
    color = BLACK; // if too dark, its black
  }
  else
  {
    color = GRAY; // anything else is just gray
  }

  #ifdef TCS_PRINT_DEBUG
    Serial.print("Color is: ");
    switch (color)
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
    Serial.println();
  #endif

  #ifdef TCS_ENABLE_LED
    neopixelWrite(RGB_BUILTIN, r, g, b); // commented out because of stm32
  #endif

  return color;
}