#include <Arduino.h>
#include <stdbool.h>
#include <Adafruit_BNO08x.h>
#include <Wire.h>
#include <math.h>
#include <ESP32MotorControl.h>

#include "BNO085_heading_acceleration.h"
#include "motor_control_v2.h"

// After starting/resetting the robot you have 5 seconds to align it with 0 degress
// We need to implement a blovking function that waits unitl button pressed before locking in 0
// Also we need to implement a lcd screen to show us our angle without uart
/*

  Global Variable for the Motors

*/
ESP32MotorControl motors;
// LEFT MOTOR
const uint8_t MOTOR_A_IN_1 = 41;
const uint8_t MOTOR_A_IN_2 = 40;
// RIGHT MOTOR
const uint8_t MOTOR_B_IN_3 = 39;
const uint8_t MOTOR_B_IN_4 = 38;
/*

  Global Variable for IMU

*/
Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

float offset = 0;

float getHeading()
{

  float retVal = -1;

  if (bno08x.getSensorEvent(&sensorValue))
  {
    retVal = calculateHeading(sensorValue.un.arvrStabilizedRV.i, sensorValue.un.arvrStabilizedRV.j, sensorValue.un.arvrStabilizedRV.k, sensorValue.un.arvrStabilizedRV.real);

    retVal -= offset;
    if (retVal < 0)
    {
      retVal += 360;
    }
  }

  return retVal;
}

void turnToGoalHeading(float goal, uint8_t speed)
{
  float currentAngle = -1;
  float absVal;
  float angleDiff;

  while (currentAngle == -1)
    currentAngle = getHeading();

  angleDiff = goal - currentAngle;
  absVal = abs(angleDiff);

  while (absVal > 8)
  {
    Serial.print("abs: ");
    Serial.print(absVal);
    Serial.print(" angle: ");
    Serial.print(currentAngle);

    if ((angleDiff >= 0) && (absVal <= 180))
    {
      Serial.print(" case 1: ");
      turn(CLOCKWISE, speed);
    }
    else if ((angleDiff < 0) && (absVal <= 180))
    {
      Serial.print(" case 2: ");
      turn(COUNTER_CLOCKWISE, speed);
    }
    else if ((angleDiff >= 0) && (absVal >= 180))
    {
      Serial.print(" case 3: ");
      turn(COUNTER_CLOCKWISE, speed);
    }
    else
    {
      Serial.print(" case 4: ");
      turn(CLOCKWISE, speed);
    }
    currentAngle = -1;
    while (currentAngle == -1)
      currentAngle = getHeading();

    Serial.println(goal);
    angleDiff = goal - currentAngle;
    absVal = abs(angleDiff);
  }
}

void setup()
{
  Serial.begin(115200);
  Wire.begin(9, 8);
  setupBNO085(&bno08x);
  motors.attachMotors(MOTOR_A_IN_1, MOTOR_A_IN_2, MOTOR_B_IN_3, MOTOR_B_IN_4);
  uint8_t i = 0;
  Serial.println("*****TEST HEADING******\n\n");
  float currentAngle = -1;
  while (currentAngle == -1)
  {
    currentAngle = offset = getHeading();
  }

  Serial.println("offset: ");
  Serial.println(offset);

  currentAngle = -1;
  while (currentAngle == -1)
  {
    currentAngle = getHeading();
  }
  Serial.print("Current Angle: ");
  Serial.println(currentAngle);

  delay(5000);
}

void loop()
{
  turnToGoalHeading(90, 100);
  stop();
  delay(2000);
  turnToGoalHeading(0, 100);
  stop();
  delay(2000);
  turnToGoalHeading(180, 100);
  stop();
  delay(2000);
  turnToGoalHeading(0, 100);
  stop();
  delay(2000);
  turnToGoalHeading(270, 100);
  stop();
  delay(2000);
  turnToGoalHeading(45, 100);
  stop();
  delay(2000);
  turnToGoalHeading(225, 100);
  stop();
  delay(2000);
  turnToGoalHeading(135, 100);
  stop();
  delay(2000);
  turnToGoalHeading(315, 100);
  stop();
  delay(2000);
  while (1)
    ;
}
