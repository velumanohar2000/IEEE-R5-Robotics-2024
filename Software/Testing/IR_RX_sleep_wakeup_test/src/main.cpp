/*
 * Bit Bangers IR "Kill Switch" Receiver Sleep Test
 * github.com/Bit-Bangers-UTA/Senior-Design
 *
 * Authors: 
 * Rolando Rosales
 *
 * Hardware setup:
 * IR:
 * IR receiver(s) data pin connected to GPIO 17
 * Remote:
 * The TCL - Roku TV remote is used for testing
 * Netflix button for kill
 * Hulu button for wake
 * - or -
 * Custom ESP32C3 remote with push button (see IR Kill SW TX project)
 * 
 * Comments:
 * Deep sleep state machine:
 * B) Deep sleep
 * A) Wake up (EXT0 or normal startup)
 * B) Check for correct code
 *   1) If wake code is EXT0, check IR code
 *     a) If IR code is lit, make sleep flag false
 *     b) Else, enter deep sleep
 *   2) If wake from normal startup, enter deep sleep
 * PlatformIO Libraries:
 * Requires crankyoldgit/IRremoteESP8266@^2.8.6 library
*/

// Libraries ------------------------------------------------------------------

#include <Arduino.h>
#include "IR_switch.h"

// Defines --------------------------------------------------------------------

// Constants & Variables ------------------------------------------------------

// IR
const uint32_t lit_code = 0x574309F6; // Amazon button
const uint32_t unalive_code = 0x5743D32C; // Netflix button
gpio_num_t ir_pin = GPIO_NUM_11; // pin number of IR receiver(s)

// Structures & Classes -------------------------------------------------------

// Functions ------------------------------------------------------------------

void sleepHandler()
{
  // turn off peripherals here
  digitalWrite(LED_BUILTIN, LOW);
  Serial.end();
}

void setup()
{
  Serial.begin(115200);

  initIR(lit_code, unalive_code);
  if (!wokeFromIR())
  {
    timeToSleep();
  }

  pinMode(LED_BUILTIN, OUTPUT);

  // Regular setup goes here
}

 void loop()
{
  if (sleepCodeReceived())
  {
    sleepHandler(); // needs to be added in main.cpp
    timeToSleep();
  }

  digitalWrite(LED_BUILTIN, HIGH);

 // Regular loop goes here
}