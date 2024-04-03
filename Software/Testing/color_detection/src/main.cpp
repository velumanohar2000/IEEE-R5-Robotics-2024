/*
 * Bit Bangers TCS34725 Color Detection Test for Round 2
 * github.com/Bit-Bangers-UTA/Senior-Design
 *
 * Authors:
 * Rolando Rosales
*/

// Libraries ------------------------------------------------------------------

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "TCS_color_det.h"

// Defines --------------------------------------------------------------------

// Preprocessor Directives
#define TCS_PRINT_DEBUG
#define TCS_ENABLE_LED

// I2C Pins
#define SDA 35
#define SCL 36

// Ambient color values
#define R_AMB 17
#define G_AMB 23
#define B_AMB 18

// Variables & Constants ------------------------------------------------------

// Structures & Classes -------------------------------------------------------

// Functions ------------------------------------------------------------------

void setup(void) {
  Serial.begin(115200);

  Wire.begin(SDA, SCL);

  initTCS(R_AMB, G_AMB, B_AMB);
}

void loop(void) {
  uint8_t start_color = getColorCode();

  /*
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
  */

  Serial.println();
}