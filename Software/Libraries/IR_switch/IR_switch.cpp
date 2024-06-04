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
#define IR_PRINT_DEBUG

// IR Timing
#define IR_SAMPLE_TIME 100;

// Constants & Variables ------------------------------------------------------

// IR
uint32_t lit_code = 0x0; // 0x4732DD25; // Amazon button
uint32_t unalive_code = 0x0; // 0x5743D32C; // Netflix button

#define STATION_A_CODE 0x400E40BFL // 1
#define STATION_B_CODE 0x400EC03FL // 2
#define STATION_C_CODE 0x400E20DFL // 3
#define STATION_D_CODE 0x400EA05FL // 4
#define STATION_E_CODE 0x400E609FL // 5
#define STATION_F_CODE 0x400EE01FL // 6
#define STATION_G_CODE 0x400E10EFL // 7
#define STATION_H_CODE 0x400E906FL // 8
#define PREDETERMINED_PATH 0x400ED02F // 0 

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
  delay(100);
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

  uint8_t i = 0;

  for (i = 0; i < 20; i++)
  {
    if (check_ir_wake)
    {
      if (irrecv.decode(&results)) // Decodes the IR code
      {
          if (results.value == lit_code)  // Checks if IR code is wake code
          {
            #ifdef IR_PRINT_DEBUG
              Serial.println("IR code is lit");
            #endif
            i = 20;
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
      delay(50);
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

uint8_t getDestination()
{
  uint8_t retVal = 0;
  if (irrecv.decode(&results)) // Decodes the IR code
  {
    switch (results.value)
    {
    case STATION_A_CODE:
      Serial.println("Station A");
      retVal = 1;
      break;
    case STATION_B_CODE:
      Serial.println("Station B");
      retVal = 2;
      break;
    case STATION_C_CODE:
      Serial.println("Station C");
      retVal = 3;
      break;
    case STATION_D_CODE:
      Serial.println("Station D");
      retVal = 4;
      break;
    case STATION_E_CODE:
      Serial.println("Station E");
      retVal = 5;
      break;
    case STATION_F_CODE:
      Serial.println("Station F");
      retVal = 6;
      break;
    case STATION_G_CODE:
      Serial.println("Station G");
      retVal = 7;
      break;
    case STATION_H_CODE:
      Serial.println("Station H");
      retVal = 8;
      break;
    case PREDETERMINED_PATH:
      Serial.println("Predetermined Path");
      retVal = 20;
      break;
    default:
      Serial.println("No station selected");
      retVal = 0;
      break;
    }
    irrecv.resume(); // Will start looking for next value
  }
  return retVal;
}

void timeToSleep()
{
  #ifdef IR_PRINT_DEBUG
    Serial.println("Zzz...");  
  #endif   
  Serial.flush(); // Waits until all serial data is finished
  esp_sleep_enable_ext0_wakeup(ir_pin, 0); // Wake when IR is low (pressed)
  esp_light_sleep_start(); // Go to sleep
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