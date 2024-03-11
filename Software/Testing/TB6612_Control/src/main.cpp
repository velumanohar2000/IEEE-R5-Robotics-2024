/*
 * Bit Bangers Modified MotorTestRun.ino for ELEGOO SmartCar-Shield-V1.1
 * Use this as a test program. The library is in lib/TB6612_SmartCar
 * github.com/Bit-Bangers-UTA/Senior-Design
 * 
 * Authors:
 * Rolando Rosales
 *
 * Hardware setup:
 * STBY: ESP GPIO 0 -> SmartCar Pin 3
 * PWMA: ESP GPIO 1 -> SmartCar Pin 5
 * AIN1: ESP GPIO 2 -> SmartCar Pin 7
*/

// Libraries ------------------------------------------------------------------

#include <Arduino.h>
#include <TB6612_SmartCar.h>

// Defines --------------------------------------------------------------------

#define STBY 0
#define PWMA 1
#define AIN1 2

// Variables & Constants ------------------------------------------------------

const int offsetA = 1;

// Structures & Classes -------------------------------------------------------

Motor motor1 = Motor(AIN1, PWMA, offsetA, STBY);

// Functions ------------------------------------------------------------------

void setup()
{
  // Nothing here
}

void loop()
{
  // Use of the drive function which takes as arguements the speed
  // and optional duration.  A negative speed will cause it to go
  // backwards.  Speed can be from -255 to 255.  Also use of the
  // brake function which takes no arguements.
  motor1.drive(255, 1000);
  motor1.drive(-255, 1000);
  motor1.brake();
  delay(1000);

  // Use of the forward function, which takes as arguements two motors
  // and optionally a speed.  If a negative number is used for speed
  // it will go backwards
  forward(motor1, 150);
  delay(1000);

  // Use of the back function, which takes as arguments two motors
  // and optionally a speed.  Either a positive number or a negative
  // number for speed will cause it to go backwards
  back(motor1, -150);
  delay(1000);

  // Use of the brake function which takes as arguments two motors.
  // Note that functions do not stop motors on their own.
  brake(motor1);
  delay(1000);

  // Use of brake again.
  brake(motor1);
  delay(1000);
}