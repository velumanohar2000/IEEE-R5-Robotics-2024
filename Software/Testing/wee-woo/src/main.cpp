/*
 * Bit Bangers wee woo
 * github.com/Bit-Bangers-UTA/Senior-Design
 * 
 * Authors:
 * Rolando Rosales
 * 
 * I just made this to test:
 * - the RGB LED
 * - the esp-prog for upload, debug, and monitor
*/

// Libraries ------------------------------------------------------------------

#include <Arduino.h>

// Defines --------------------------------------------------------------------

// LED
#define BRIGHTNESS 2 // can go from 0 to 255

// Variables & Constants ------------------------------------------------------

// Space counter for soundOfDa
int count = 0;
bool flip = false;

// Structures & Classes -------------------------------------------------------

// None

// Functions ------------------------------------------------------------------

void soundOfDa()
{
  if (count == 40)
    flip = true;
  if (count == 0)
    flip = false;

  Serial.println("Wee woo");

  for (int i = 0; i <= count; i++)
    Serial.print(" ");

  flip == true ? count-- : count++;
}

void setup()
{
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  soundOfDa();

  neopixelWrite(RGB_BUILTIN, BRIGHTNESS, 0, 0);
  delay(100);

  neopixelWrite(RGB_BUILTIN, 0, 0, BRIGHTNESS); 
  delay(100);
}