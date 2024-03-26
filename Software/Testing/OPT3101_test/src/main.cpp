// This example shows basic usage of the OPT3101 library.

#include <Arduino.h>
#include <Wire.h>
#include "OPT3101_whisker.h"

void setup()
{
  Serial.begin(115200);
  Wire.begin(6, 7);

  initOPT3101();
}

void loop()
{
  Serial.println(getWhiskerDistanceCm());
}