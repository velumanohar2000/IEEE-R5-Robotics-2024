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

// Libraries ------------------------------------------------------------------

#include <Arduino.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>

// Defines --------------------------------------------------------------------

// Preprocessor directives
// #define IR_PRINT_DEBUG

// Constants & Variables ------------------------------------------------------

// IR
uint32_t lit_code = 0x0; // 0x4732DD25; // Amazon button
uint32_t unalive_code = 0x0; // 0x5743D32C; // Netflix button
extern gpio_num_t ir_pin; // GPIO_NUM_X, this must initialized in main.cpp

// Deep Sleep
bool check_ir_wake = false;
bool woke_from_ir = false;
bool go_sleep = false;

// Structures & Classes -------------------------------------------------------

// IR
decode_results results;
IRrecv irrecv(ir_pin);

// Functions ------------------------------------------------------------------

void initIR(uint32_t lit, uint32_t unalive)
{
  lit_code = lit;
  unalive_code = unalive;

  irrecv.enableIRIn(); // Enables the IR receiver
  pinMode(ir_pin, INPUT);
  delay(40);
}

bool wokeFromIR()
{
  esp_sleep_wakeup_cause_t code = esp_sleep_get_wakeup_cause();

  woke_from_ir = false;

  switch(code)
  {
    case ESP_SLEEP_WAKEUP_EXT0: // Caused by IR GPIO
      #ifdef IR_PRINT_DEBUG
        Serial.println("Wakeup caused by external signal using RTC_IO");
      #endif
      check_ir_wake = true;
      break;
    default: // Woke up some other wake (typically normally)
      #ifdef IR_PRINT_DEBUG
        Serial.printf("Wakeup was not caused by deep sleep: %d\n",code);
      #endif
      check_ir_wake = false;
      break;
  }

  Serial.println(check_ir_wake ? "true" : "false");

  if (check_ir_wake)
  {
    Serial.println("entered");
    if (irrecv.decode(&results)) // Decodes the IR code
    {
        if (results.value == lit_code)  // Checks if IR code is wake code
        {
          #ifdef IR_PRINT_DEBUG
            Serial.println("IR code is lit");
          #endif
          woke_from_ir = true;
        }
        else
        {
          #ifdef IR_PRINT_DEBUG
            Serial.print("Wrong code: "); // Print incorrect code
            Serial.println(results.value, HEX);
          #endif
          woke_from_ir = false;
        }
        irrecv.resume(); // Will start looking for next value
    }
    else // Will run if there was no IR code to decode
    {
      #ifdef IR_PRINT_DEBUG
        Serial.println("No IR signal detected");
      #endif
      woke_from_ir = false;
    }
  }

  return woke_from_ir;
}

bool sleepCodeReceived()
{
  if (irrecv.decode(&results)) // Decodes the IR code
  {
    if (results.value == unalive_code)  // Checks if IR code is wake code
    {
      #ifdef IR_PRINT_DEBUG
        Serial.println("Time to sleep");
        Serial.flush(); // Waits until all serial data is finished
      #endif
      go_sleep = true;
    }
    else // Will run if there was no IR code to decode
    {
      #ifdef IR_PRINT_DEBUG
        Serial.print("Wrong code: "); // Print incorrect code
        Serial.println(results.value, HEX);
      #endif
      go_sleep = false;
    }
    irrecv.resume(); // Will start looking for next value
  }

  return go_sleep;
}

void timeToSleep()
{
  #ifdef IR_PRINT_DEBUG
    Serial.println("Zzz...");  
  #endif   
  Serial.flush(); // Waits until all serial data is finished
  esp_sleep_enable_ext0_wakeup(ir_pin, 0); // Wake when IR is low (pressed)
  esp_deep_sleep_start(); // Go to sleep
}

// Example of how main.cpp would look

/* 
 * void setup()
 * {
 *   initIR();
 *   if (wokeFromIR());
 *   {
 *     timeToSleep();
 *   }
 * 
 *   // Regular setup goes here
 * }
 *
 * void loop()
 * {
 *   if (sleepCodeReceived())
 *   {
 *     sleepHandler(); // needs to be added in main.cpp
 *     timeToSleep();
 *   }
 *
 *  // Regular loop goes here
 * }
*/

// Original IR sleep test code

/*
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
*/