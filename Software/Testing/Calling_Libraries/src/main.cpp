/*
 * Bit Bangers Library Calling Test
 * github.com/Bit-Bangers-UTA/Senior-Design
 *
 * Authors:
 * Rolando Rosales
 * 
 * Comments:
 * All you need to add in your 'platformio.ino' is...
 * 
 * [platformio]
 * lib_deps = ./../../Libraries/
 * 
 * ... and it'll pull libraries from that folder
*/

// Libraries ------------------------------------------------------------------

#include <Arduino.h>
#include "Test_Library.h"

// Defines --------------------------------------------------------------------

// Variables & Constants ------------------------------------------------------

// Structures & Classes -------------------------------------------------------

// Functions ------------------------------------------------------------------

void setup() {
  Serial.begin(9600);
}

void loop() {
  helloWorld();
  delay(1000);
}