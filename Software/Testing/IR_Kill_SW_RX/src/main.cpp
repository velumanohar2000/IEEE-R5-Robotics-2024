/*
 * Bit Bangers IR "Kill Switch" Receiver Test
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
// #define IR_ISR_PIN 9

// Variables & Constants ------------------------------------------------------

const uint8_t unalive_code = 0xF;
bool LED_STATE = false;
bool wakeup_ir = false;

// Structures & Classes -------------------------------------------------------

IRrecv irrecv(IR_RX_PIN);
decode_results results;

// Functions ------------------------------------------------------------------

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

void setup() {
  Serial.begin(115200);
  irrecv.enableIRIn(); // Start the receiver

  esp_sleep_enable_ext0_wakeup(IR_RX_PIN, 1); //1 = High, 0 = Low

  Serial.println("Zzz...");
  delay(1000);
  esp_deep_sleep_start();

}

void loop() {
}