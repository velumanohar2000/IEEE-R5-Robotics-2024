/*
 * Bit Bangers Dummy Robot Project
 * github.com/Bit-Bangers-UTA/Senior-Design
 *
 * ** This code currently does not work **
 *
 * Hardware setup:
 * STBY: ESP32 0 -> SmartCar Pin 3
 * PWMA: ESP32 1 -> SmartCar Pin 5
 * PWMB: ESP32 GPIO 2 -> SmartCar Pin 6
 * AINA: ESP32 GPIO 3 -> SmartCar Pin 7
 * BIN1: ESP32 GPIO 10 -> SmartCar Pin 8
 * ESP32 3V3 -> SmartCar 5V0
 * ESP32 GND -> SmartCar GND
*/

// Libraries ------------------------------------------------------------------

#include <Arduino.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include <SparkFun_TB6612.h>

// Defines --------------------------------------------------------------------

#define AIN1 3
#define BIN1 10
#define PWMA 1
#define PWMB 2
#define STBY 0

// Variables & Constants ------------------------------------------------------

const int offsetA = 1;
const int offsetB = 1;

// Structures & Classes -------------------------------------------------------

XboxSeriesXControllerESP32_asukiaaa::Core xboxController;
Motor motor1 = Motor(AIN1, 0, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, 0, PWMB, offsetB, STBY);

// Functions ------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  xboxController.begin();
}

void loop() {
  xboxController.onLoop();

  if (xboxController.isConnected()) {
    // at the moment, only goes forward and backward
    /*
    // uses right stick to steer
    Serial.print("Steering: ");
    float joyRHoriNormalized = (float)xboxController.xboxNotif.joyRHori / XboxControllerNotificationParser::maxJoy;
    float joyRHoriMapped = joyRHoriNormalized * 2 - 1;
    Serial.println(joyRHoriMapped, 2);
    */

    // uses right trigger to accelerate
    Serial.print("Accelerating at: ");
    Serial.println((uint16_t)xboxController.xboxNotif.trigRT / 4);

    // uses left trigger to reverse
    Serial.print("Reversing at: ");
    Serial.println((uint16_t)xboxController.xboxNotif.trigLT / 4);

    Serial.println();
  }
  else {
    Serial.println("Not connected");
    
    if (xboxController.getCountFailedConnection() > 2) {
      ESP.restart();
    }
  }

  uint16_t rev = (uint16_t)xboxController.xboxNotif.trigLT / 4;
  uint16_t fwd = (uint16_t)xboxController.xboxNotif.trigRT / 4;

  int speeeeed = fwd - rev;

  Serial.println(speeeeed);

  motor1.drive(speeeeed);
  motor2.drive(speeeeed);

  delay(10);
}
