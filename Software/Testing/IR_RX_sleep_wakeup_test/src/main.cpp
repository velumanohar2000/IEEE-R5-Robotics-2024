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
#include <IRremoteESP8266.h>
#include <IRrecv.h>

// Defines --------------------------------------------------------------------

#define IR_RX_PIN GPIO_NUM_14

// Constants & Variables ------------------------------------------------------

// IR
const uint32_t unalive_code = 0x5743D32C; // Netflix button
const uint32_t lit_code = 0x4732DD25; // Amazon button
const uint8_t custom_code = 0xF; // For custom remote, will likely not use

// Deep Sleep
bool go_sleep = true;
bool check_ir_wake = false;
esp_sleep_wakeup_cause_t wake_code;

// Misc
bool led_state = false;

// Structures & Classes -------------------------------------------------------

// IR
decode_results results;
IRrecv irrecv(IR_RX_PIN);

// Functions ------------------------------------------------------------------

// On startup, we will check the code of what woke up the esp32
esp_sleep_wakeup_cause_t getWakeCode()
{
  esp_sleep_wakeup_cause_t code = esp_sleep_get_wakeup_cause();

  switch(code)
  {
    case ESP_SLEEP_WAKEUP_EXT0: // Caused by GPIO 17
      Serial.println("Wakeup caused by external signal using RTC_IO");
      check_ir_wake = true;
      break;
    default: // Woke up some other wake (typically normally)
      Serial.printf("Wakeup was not caused by deep sleep: %d\n",code);
      break;
  }

  return code;
}

// If woken up by EXT0, check the IR code
void checkIRWake()
{
  if (irrecv.decode(&results)) // Decodes the IR code
  {
    if (results.value == lit_code)  // Checks if IR code is wake code
    {
      Serial.println("IR code is lit");
      go_sleep = false; 
    }
    else
    {
      Serial.print("Wrong code: "); // Print incorrect code
      Serial.println(results.value, HEX);
    }
    irrecv.resume(); // Will start looking for next value
  }
  else // Will run if there was no IR code to decode
  {
    Serial.println("No IR signal detected");
  }
}

// Checks to see if IR sleep code has been sent
void checkIRSleep()
{
  if (irrecv.decode(&results)) // Decodes the IR code
  {
    if (results.value == unalive_code)  // Checks if IR code is wake code
    {
      Serial.println("Time to sleep");
      Serial.flush(); // Waits until all serial data is finished

      esp_sleep_enable_ext0_wakeup(IR_RX_PIN, 0); // Wake when IR is low (pressed)
      esp_deep_sleep_start();
    }
    else
    {
      Serial.print("Wrong code: "); // Print incorrect code
      Serial.println(results.value, HEX);
    }
    irrecv.resume(); // Will start looking for next value
  }
}

void setup()
{
  Serial.begin(115200);

  irrecv.enableIRIn(); // Enables the IR receiver

  digitalWrite(LED_BUILTIN, LOW);

  delay(40);
digitalWrite(LED_BUILTIN, LOW);
  pinMode(IR_RX_PIN, INPUT); // Sets GPIO 17 as input

  wake_code = getWakeCode();

  if (check_ir_wake) // Checks IR code to see if sleep needed
  {
    checkIRWake();
  }
  if (go_sleep) // Runs if sleep is needed (bad IR code or normal wake)
  {
    Serial.println("Zzz...");
    Serial.flush(); 

    esp_sleep_enable_ext0_wakeup(IR_RX_PIN, 0); // Wake when IR is low (pressed)
    esp_deep_sleep_start();
  }
}

void loop()
{
  checkIRSleep();
  delay(40); // There needs to be some time between IR samples
  neopixelWrite(LED_BUILTIN, 4, 4, 4);
}