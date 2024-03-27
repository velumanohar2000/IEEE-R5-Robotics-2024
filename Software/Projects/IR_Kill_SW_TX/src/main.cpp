/*
 * Bit Bangers IR "Kill Switch" Transmitter Library
 * github.com/Bit-Bangers-UTA/Senior-Design
 *
 * Authors: 
 * Rolando Rosales
 *
 * Hardware setup:
 * IR LED between Vcc and NPN transistor C
 * 470 Ohm resistor between GPIO 2 and NPN transistor B
 * GND connected to NPN transistor E
 * Push button between Vcc and GPIO 1
 * 
 * Requires crankyoldgit/IRremoteESP8266@^2.8.6 library
 */

// Libraries ------------------------------------------------------------------

#include <Arduino.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>

// Defines --------------------------------------------------------------------

#define PUSH_BUTTON 4 // Push button connected to pin 3
#define IR_TX_PIN 3 // IR LED connected to pin 2

// Variables & Constants ------------------------------------------------------

// IR control
const uint32_t kill_code = 0xAF83F03B; // Code to unalive the robot

// Debounce
bool lastButtonState = LOW; // Previous state of the button
bool reading = LOW; // Current reading from the button
uint8_t buttonState = 0; // Current state of the button
uint8_t debounceDelay = 10; // Debounce delay in milliseconds
uint32_t lastDebounceTime = 0; // Last time the button's state changed

// Structures & Classes -------------------------------------------------------

IRsend irsend(IR_TX_PIN); // IRsend object to send IR codes

// Functions ------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  pinMode(PUSH_BUTTON, INPUT_PULLDOWN); // Needed as connected to Vcc

  irsend.begin(); // Initialize the IRsend object

  pinMode(LED_BUILTIN, OUTPUT); // Set the LED pin as an output
  digitalWrite(LED_BUILTIN, LOW); // Turn off the LED
}

void loop() {
  bool reading = digitalRead(PUSH_BUTTON);
  
  // Check if the button state has changed
  if (reading != lastButtonState) {
    lastDebounceTime = millis(); // Reset the debouncing timer
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // If state has not changed for longer than delay, consider it stable
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == HIGH) {
        irsend.sendNEC(kill_code, 32); // Send the unalive code

        Serial.print("Sent IR code: ");
        Serial.println(kill_code, HEX); // Print sent code in hex
      }
    }
  }

  lastButtonState = reading; // Save reading
}
