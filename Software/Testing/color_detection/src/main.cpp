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
const uint16_t r_amb = 14;
const uint16_t g_amb = 21;
const uint16_t b_amb = 19;

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

  tcs.getRawData(&r_raw, &g_raw, &b_raw, &c); // gets raw color values

  Serial.println("Raw:");
  Serial.print("R: "); Serial.print(r_raw, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g_raw, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.println(b_raw, DEC);

  // adjusts color values depending on ambient light
  int16_t r = r_raw - r_amb;
  int16_t g = g_raw - g_amb;
  int16_t b = b_raw - b_amb;

  // if any number is negative, it will float to 0
  if (r < 0) r = 0;
  if (g < 0) g = 0;
  if (b < 0) b = 0;

  Serial.println("Raw - Ambient:");
  Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.println(b, DEC);

  // will filter out the lowest value from the RGB values
  uint16_t lowest = min(min(r, g), b);

  r -= lowest;
  g -= lowest;
  b -= lowest;

  Serial.println("Calibrated:");
  Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.println(b, DEC);

  if (r > 60) // checks warm colors
  {
    if (g < 20) // if the green value is too low, its red
    {
      start_color = RED;
    }
    else if ((r - g) < 70) // y is bright. if r and g are close its yellow
    {
      start_color = YELLOW;
    }
    else
    {
      start_color = ORANGE; // if g is kinda high, but not close to r its orange
    }
  }
  else if (b > 30) // checks blue colors
  {
    if (r < 5) // if there's barely any red, its blue
    {
      start_color = BLUE;
    }
    else
    {
      start_color = PURPLE; // if there's noticably some red, its purple
    }
  }
  else if (g > 25) // looks for green
  {
    start_color = GREEN;
  }
  else if (r < 13 && g < 13 && b < 13)
  {
    start_color = BLACK; // if too dark, its black
  }
  else
  {
    start_color = GRAY; // anything else is just gray
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

  // neopixelWrite(RGB_BUILTIN, r, g, b); // commented out because of stm32
  Serial.println();
}