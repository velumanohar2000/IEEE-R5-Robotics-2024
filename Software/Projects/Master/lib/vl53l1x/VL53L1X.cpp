/*
  Reading distance from the laser based VL53L1X
  By: Nathan Seidle
  SparkFun Electronics
  Date: April 4th, 2018
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  SparkFun labored with love to create this code. Feel like supporting open source hardware?
  Buy a board from SparkFun! https://www.sparkfun.com/products/14667

  This example prints the distance to an object.

  Are you getting weird readings? Be sure the vacuum tape has been removed from the sensor.
*/

#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X
#include "VL53L1X.h"

//Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3

extern SFEVL53L1X distanceSensor;
//Uncomment the following line to use the optional shutdown and interrupt pins.
//SFEVL53L1X distanceSensor(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);

void initVL53L1X(void)
{
  Wire.begin(9, 8);

  // Serial.begin(115200);
  // Serial.println("VL53L1X Qwiic Test");

  if (distanceSensor.begin() != 0) //Begin returns 0 on a good init
  {
    // Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  // Serial.println("Sensor online!");
}

int getDistanceVL53L1X(uint8_t unit)
{
  distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
  while (!distanceSensor.checkForDataReady())
  {
    delay(1);
  }
  int distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
  distanceSensor.clearInterrupt();
  distanceSensor.stopRanging();

  if(unit == UNIT_INCHES) // inches
    return (distance * 0.0393701)/12;
  else if(unit == UNIT_FEET) // feet
    return distance * 0.0393701;
  else
    return distance; // mm

}

// void loop(void)
// {
//   distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
//   while (!distanceSensor.checkForDataReady())
//   {
//     delay(1);
//   }
//   int distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
//   distanceSensor.clearInterrupt();
//   distanceSensor.stopRanging();

//   Serial.print("Distance(mm): ");
//   Serial.print(distance);

//   float distanceInches = distance * 0.0393701;
//   float distanceFeet = distanceInches / 12.0;

//   Serial.print("\tDistance(in): ");
//   Serial.print(distanceInches, 2);

//   Serial.println();
// }