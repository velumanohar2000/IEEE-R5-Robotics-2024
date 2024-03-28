/*
 * Bit Bangers OPT3101 Whisker Library Test
 * github.com/Bit-Bangers-UTA/Senior-Design
 *
 * Authors:
 * Rolando Rosales
*/

// Libraries ------------------------------------------------------------------

#include <Arduino.h>
#include <Wire.h>
#include "OPT3101_whisker.h"

// Defines --------------------------------------------------------------------

// Variables & Constants ------------------------------------------------------

// Structures & Classes -------------------------------------------------------

// Functions ------------------------------------------------------------------

void setup()
{
  Serial.begin(115200);
  Wire.begin(9, 8);

  initOPT3101();
}

void loop()
{
  Serial.println(getWhiskerDistanceCm());
}