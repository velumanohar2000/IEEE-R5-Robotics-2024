/*
 * Bit Bangers IR "Kill Switch" Library
 * github.com/Bit-Bangers-UTA/Senior-Design
 *
 * Authors: 
 * Rolando Rosales
 *
 * Hardware setup:
 * IR:
 * IR receiver(s) data pin connected to a GPIO pin
 * Remote:
 * Roku TV remote is used for testing
 * Netflix button for kill
 * Hulu button for wake
 * 
 * Comments:
 * For example code, see IR_RX_sleep_wakeup_test project
 * Deep sleep state machine:
 * A) Deep sleep
 * B) Wake up (EXT0 or normal startup)
 * C) Check for correct wake code
 *   1) If wake code is EXT0, check IR code
 *     a) If IR code is lit, return woke from IR true
 *     b) Else, return false and enter sleep in setup()
 *   2) If wake from normal startup, enter deep sleep
 * D) Wait for sleep code in loop(), then enter sleep
 * PlatformIO Libraries:
 * Requires crankyoldgit/IRremoteESP8266@^2.8.6 library
*/

#ifndef IR_SWITCH_H_
#define IR_SWITCH_H_

// Libraries ------------------------------------------------------------------

#include <Arduino.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>

// Defines --------------------------------------------------------------------

// Constants & Variables ------------------------------------------------------

extern gpio_num_t ir_pin; // GPIO_NUM_X, this must initialized in main.cpp

// Structures & Classes -------------------------------------------------------

// Functions ------------------------------------------------------------------

void initIR(uint32_t lit, uint32_t unalive);
bool wokeFromIR();
bool sleepCodeReceived();
uint8_t getDestination();
void timeToSleep();

#endif