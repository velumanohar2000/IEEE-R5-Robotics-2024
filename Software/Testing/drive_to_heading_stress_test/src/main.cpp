#include <Arduino.h>
#include <stdbool.h>
#include <Adafruit_BNO08x.h>
#include <Wire.h>
#include <math.h>
#include <ESP32MotorControl.h>
#include <LiquidCrystal_I2C.h>
#include <string.h>
#include <Servo.h>

#include "motor_control_v2.h"
#include "BNO085_heading.h"

#define ELIMINATION
// #define TEST
// #define ANGLE
#define MOTOR_IN_1 7
#define MOTOR_IN_2 6
#define MOTOR_IN_3 5
#define MOTOR_IN_4 4

float currentCardinalHeading;

unsigned long startTime;
;

/*
 * Global Variables for IMU
 */

Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;
ESP32MotorControl motors;
float offset = 0;

void setup()
{
  Serial.begin(115200);
  Wire.begin(9, 8);
  motors.attachMotors(MOTOR_IN_1, MOTOR_IN_2, MOTOR_IN_3, MOTOR_IN_4);
  setupBNO085(&bno08x); // Initialize the IMU
  Serial.println("*****TEST HEADING******\n\n");
  delay(3000);
  offset = getCurrentAngle(); // Get the offset of the IMU
  Serial.println("offset: ");
  Serial.println(offset);
  float currentAngle = getCurrentAngle();
  Serial.print("Current Angle: ");
  Serial.println(currentAngle);
}
void printCurrentAngle()
{
  float currentAngle = getCurrentAngle();
  Serial.print("CurrentAngle: ");
  Serial.println(currentAngle);
  // Serial.print("Cardinal heading: ");
  // Serial.println(findCardinalheading());
}

void turnToHeading(float goal, uint8_t speed)
{
  float absVal;
  float angleDiff;
  uint8_t i = 0;

  float currentAngle = getCurrentAngle();

  // printToLcd("Current Angle: ", currentAngle);
  angleDiff = goal - currentAngle;
  absVal = abs(angleDiff);
  if (absVal > 350)
  {
    absVal = 359.99 - absVal;
    angleDiff = 359.99 - angleDiff;
  }
  // Serial.println(angleDiff);
  // Serial.println(absVal);
  // Serial.println();

  while (absVal > 10)
  {
    // setServoPosition();
    // Serial.print("abs: ");
    // Serial.print(absVal);
    // Serial.print(" angle: ");
    // Serial.print(currentAngle);

    if ((angleDiff >= 0) && (absVal <= 180))
    {
      // Serial.print(" case 1: ");
      turn(COUNTER_CLOCKWISE, speed);
    }
    else if ((angleDiff < 0) && (absVal <= 180))
    {
      // Serial.print(" case 2: ");
      turn(CLOCKWISE, speed);
    }
    else if ((angleDiff >= 0) && (absVal >= 180))
    {
      // Serial.print(" case 3: ");
      turn(CLOCKWISE, speed);
    }
    else
    {
      // Serial.print(" case 4: ");
      turn(COUNTER_CLOCKWISE, speed);
    }

    currentAngle = getCurrentAngle();
    angleDiff = goal - currentAngle;
    absVal = abs(angleDiff);
    if (absVal > 350)
    {
      absVal = 359.99 - absVal;
      angleDiff = 359.99 - angleDiff;
    }
  }
  stopMotors();
}

void driveToHeading(float goalHeading)
{
  float currentAngle = -1;
  float absVal;
  float angleDiff;

  uint32_t interval = 1000;
  uint16_t i = 0;

  // bool goToHeading = true;
  // while (goToHeading)
  // {
  currentAngle = getCurrentAngle();

  i++;
  if (i == 30)
  {
    i = 0;
  }
  angleDiff = goalHeading - currentAngle;
  absVal = abs(angleDiff);
  // Serial.println(absVal);
  if (absVal > 345)
  {
    absVal = 359.99 - absVal;
    angleDiff = 359.99 - angleDiff;
  }
  if (absVal > 15)
  {
    turnToHeading(goalHeading, 65); // turn to the desired heading because angle is to great to correct while driving
    //startTime = millis();           // but this is not used anymore
  }
  else if (absVal <= 15 && absVal >= 3)
  {
    if ((angleDiff >= 0) && (absVal <= 180))
    {
      turn2(COUNTER_CLOCKWISE, 65, 30); // Turn2 takes in the direction, speed, and an offset that will increase the turning wheel by the given amount
    }
    else if ((angleDiff < 0) && (absVal <= 180))
    {
      turn2(CLOCKWISE, 65, 30);
    }
    else if ((angleDiff >= 0) && (absVal >= 180))
    {
      turn2(CLOCKWISE, 65, 30);
    }
    else
    {
      turn2(COUNTER_CLOCKWISE, 65, 30);
    }
    //startTime = millis(); // but this is not used anymore
  }
  else if (absVal < 5 && absVal >= 2) // if the angle is less than 5 degrees, drive the turning wheel slightly faster than the straight wheel
  {
    if ((angleDiff >= 0) && (absVal <= 180))
    {
      turn2(COUNTER_CLOCKWISE, 65, 20);
    }
    else if ((angleDiff < 0) && (absVal <= 180))
    {
      turn2(CLOCKWISE, 65, 20);
    }
    else if ((angleDiff >= 0) && (absVal >= 180))
    {
      turn2(CLOCKWISE, 65, 20);
    }
    else
    {
      turn2(COUNTER_CLOCKWISE, 65, 20);
    }
   // startTime = millis(); // but this is not used anymore
  }
  else if (absVal < 2 && absVal >= 1) // if the angle is less than 2 degrees, drive the turning wheel barely faster than the straight wheel
  {
    if ((angleDiff >= 0) && (absVal <= 180))
    {
      turn2(COUNTER_CLOCKWISE, 65, 10);
    }
    else if ((angleDiff < 0) && (absVal <= 180))
    {
      turn2(CLOCKWISE, 65, 10);
    }
    else if ((angleDiff >= 0) && (absVal >= 180))
    {
      turn2(CLOCKWISE, 65, 10);
    }
    else
    {
      turn2(COUNTER_CLOCKWISE, 65, 10);
    }
   // startTime = millis(); // but this is not used anymore
  }
  else
  {
    move(FORWARD, 255); // if the angle is less than 1 degree, drive straight
  }
  // }
}

void driveToHeadingTimer(uint16_t heading, uint32_t time)
{
  unsigned long currentMillis = millis();
  startTime = millis();

  while (currentMillis - startTime < time)
  {
    currentMillis = millis();
    driveToHeading(heading);
   // startTime = millis();
  }
  stopMotors();
}

void loop()
{
  Serial.println("*****TEST HEADING******\n\n");
  driveToHeadingTimer(270, 5000);
  driveToHeadingTimer(0, 5000);
  driveToHeadingTimer(90, 5000);
  driveToHeadingTimer(180, 5000);
  driveToHeadingTimer(315, 5000);

  driveToHeadingTimer(135, 5000);
  driveToHeadingTimer(0, 5000);
  driveToHeadingTimer(225, 5000);
  driveToHeadingTimer(45, 5000);
  driveToHeadingTimer(0, 5000);

  while (1)
    ;
}
