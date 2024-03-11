/*
 * Bit Bangers IR "Kill Switch" Receiver Library
 * github.com/Bit-Bangers-UTA/Senior-Design
 *
 * Authors: 
 * Rolando Rosales
 *
 * Hardware setup:
 * IR receiver data pin connected to GPIO 3
 * 
 * Requires crankyoldgit/IRremoteESP8266@^2.8.6 library
 */

// Libraries ------------------------------------------------------------------

#include <Arduino.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>

// Defines --------------------------------------------------------------------

#define IR_RX_PIN 3

// Variables ------------------------------------------------------------------

decode_results results;
const uint32_t kill_code = 0xAF83F03B;
bool LED_STATE = false;

// Structures & Classes -------------------------------------------------------

IRrecv irrecv(IR_RX_PIN);

// Functions ------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  
  irrecv.enableIRIn(); // Start the receiver

  pinMode(LED_BUILTIN, OUTPUT); // Set the LED pin as an output
  digitalWrite(LED_BUILTIN, LOW); // Turn off in case it is on
}

void loop() {
  if (irrecv.decode(&results)) {
    Serial.print("Received IR code: ");
    Serial.println(results.value, HEX); // Print received code in hex

    if (results.value == kill_code) { // Toggles LED if code is unalive code
      LED_STATE = !LED_STATE;
      digitalWrite(LED_BUILTIN, LED_STATE ? HIGH : LOW);
    }

    irrecv.resume(); // Receive the next value
  }
}