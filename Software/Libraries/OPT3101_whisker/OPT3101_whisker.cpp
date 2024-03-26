/*
 * Bit Bangers OPT3101 Whisker Library
 * github.com/Bit-Bangers-UTA/Senior-Design
 *
 * Authors:
 * Rolando Rosales
 * 
 * Comments:
 * Configured for 60 FoV, getting new values at roughly 29-31 Hz
*/

// Libraries ------------------------------------------------------------------

#include <Arduino.h>
#include <OPT3101.h>
#include <Wire.h>
#include "OPT3101_whisker.h"

// Defines --------------------------------------------------------------------

// Variables & Constants ------------------------------------------------------

// Structures & Classes -------------------------------------------------------

OPT3101 Whisker;

// Functions ------------------------------------------------------------------

void initOPT3101(void)
{
  Whisker.init();
  if (Whisker.getLastError())
  {
    Serial.print(F("Failed to initialize OPT3101: error "));
    Serial.println(Whisker.getLastError());
    while (1) {}
  }

  // This tells the OPT3101 how many readings to average
  // together when it takes a sample.  Each reading takes
  // 0.25 ms, so getting 256 takes about 32 ms.
  // The library adds an extra 6% margin of error, making
  // it 34 ms.  You can specify any power of 2 between
  // 1 and 4096.
  Whisker.setFrameTiming(128);

  // 1 means to use TX1, the middle channel.
  Whisker.setChannel(1);

  // Adaptive means to automatically choose high or low brightness.
  // Other options you can use here are High and Low.
  Whisker.setBrightness(OPT3101Brightness::Adaptive);
}

int16_t getWhiskerDistanceCm(void)
{
  Whisker.sample();
  return Whisker.distanceMillimeters / 10;
}