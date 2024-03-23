/* 
 * Bit Bangers Bang Bus Master Code
 * github.com/Bit-Bangers-UTA/Senior-Design
 *
 * Authors:
 * Velu Manohar
 * Noor Abdullah
 * Rolando Rosales
 * 
 * Comments:
 * After starting/resetting the robot you have 2.5 seconds to align it with 0 degress
 * We need to implement a blocking function that waits unitl button pressed before locking in 0
 * Also we need to implement a lcd screen to show us our angle without uart
*/

// Libraries ------------------------------------------------------------------

#include <Arduino.h>
#include <stdbool.h>
#include <Adafruit_BNO08x.h>
#include <Wire.h>
#include <math.h>
#include <ESP32MotorControl.h>
#include "Ultrasonic.h"
#include "BNO085_heading_acceleration.h"
#include "motor_control_v2.h"
#include "SparkFun_VL53L1X.h"

// Defines --------------------------------------------------------------------

// Preprocessor Directives
#define TURN_TO_HEAD
// #define MAIN

// Motors
// Left
const uint8_t motor_a_in1 = 4;
const uint8_t motor_a_in2 = 5;
// Right
const uint8_t motor_b_in3 = 6;
const uint8_t motor_b_in4 = 7;

// Whisker
#define WHISKER_STOP_DIS 8
#define MAX_PRELIM_DIST 60

// Ultrasonic
#define ULTRAS_TRIG 1
#define ULTRAS_ECHO 0

// Variables & Constants ------------------------------------------------------

// Structures & Classes -------------------------------------------------------

/*
 * None of the things below are organized yet, but we can just do that later
 * Note: things like "ESP32MotorControl motors;" are classes, not variables
 * -RR
*/

/*
  Global Variable for the Motors
*/
ESP32MotorControl motors;

/*
  Global Variable for IMU
*/
Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;
float offset = 0;
float goal = 0;

/*
  Globals for Whiskers
*/
SFEVL53L1X distanceSensor;
int whiskDistance;                            // whisker
float whiskDistanceInch = 1000;               // whisker
bool wallFound = false;

/*
  Globals for ultrasonic
*/
Ultrasonic ultrasonic(ULTRAS_TRIG, ULTRAS_ECHO);
int ultraDistance = 100;                      // ultrasonic
float ultraDistanceInch;                      // ultrasonic

// Functions ------------------------------------------------------------------

void getWhiskerDistance()
{
  distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
  while (!distanceSensor.checkForDataReady())
  {
    delay(1);
  }
  whiskDistance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
  distanceSensor.clearInterrupt();
  distanceSensor.stopRanging();
  whiskDistanceInch = whiskDistance * 0.0393701;
}

void initVL53L1X(void)
{
  Wire.begin(9, 8);

  if (distanceSensor.begin() != 0) //Begin returns 0 on a good init
  {
    while (1)
      ;
  }
}

float getHeading()
{

  float retVal = -1;

  if (bno08x.getSensorEvent(&sensorValue))
  {
    retVal = calculateHeading(sensorValue.un.arvrStabilizedRV.i, sensorValue.un.arvrStabilizedRV.j, sensorValue.un.arvrStabilizedRV.k, sensorValue.un.arvrStabilizedRV.real);

    Serial.printf(" Raw angle = %f\n", retVal);
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

  while (absVal > 4)
  {
    // getWhiskerDistance();
    // Serial.printf("Whiskaaa is %f: \n", whiskDistanceInch);
    Serial.print("Offset ");
    Serial.print(offset);
    Serial.print(" Abs: ");
    Serial.print(absVal);
    Serial.print(" Current Angle: ");
    Serial.print(currentAngle);

    if ((angleDiff >= 0) && (absVal <= 180))
    {
      // Serial.print(" case 1: ");
      turn(CLOCKWISE, speed);
    }
    else if ((angleDiff < 0) && (absVal <= 180))
    {
      // Serial.print(" case 2: ");
      turn(COUNTER_CLOCKWISE, speed);
    }
    else if ((angleDiff >= 0) && (absVal >= 180))
    {
      // Serial.print(" case 3: ");
      turn(COUNTER_CLOCKWISE, speed);
    }
    else
    {
      // Serial.print(" case 4: ");
      turn(CLOCKWISE, speed);
    }
    currentAngle = -1;
    while (currentAngle == -1)
      currentAngle = getHeading();

    // Serial.println(goal);
    angleDiff = goal - currentAngle;
    absVal = abs(angleDiff);
  }
}

void calibrate()
{
  float currentAngle = -1;
  while (currentAngle == -1)
  {
    currentAngle = offset = getHeading();
  }

  Serial.println("offset: ");
  Serial.println(offset);
}

void setup()
{
  Serial.begin(115200);
  Wire.begin(9, 8);
  initVL53L1X();
  setupBNO085(&bno08x);
  motors.attachMotors(motor_a_in1, motor_a_in2, motor_b_in3, motor_b_in4);
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

  delay(2500);
}

void loop()
{
  #ifdef MAIN
  float currentAngle;
  if(!wallFound)
  {
    currentAngle = getHeading();
    Serial.printf("Whisker dist %f: \n", whiskDistance);
    if(whiskDistanceInch > WHISKER_STOP_DIS)
    {
      ultraDistance = ultrasonic.read();
      if(ultraDistance > MAX_PRELIM_DIST)
      {
        stop();
      }
      else if(ultraDistance > 8)
      {
        turn(COUNTER_CLOCKWISE, 80);
        delay(80);
        move(FORWARD, 80);
        delay(80);
      }
      else if(ultraDistance < 5)
      {
        turn(CLOCKWISE, 80);
        delay(80);
        move(FORWARD, 80);
        delay(80);
      }
      else
        move(FORWARD, 80);
    }
    else{
      stop();
      Serial.println("Wall Reached");
      delay(1500);
      wallFound = true;
      // calibrate();
      currentAngle = getHeading();
      // if(currentAngle > 180)
      // {
      //   goal = (360-currentAngle) + 85;
      // }
      // else
      // {
      //   goal = 85-currentAngle;
      // }


    }
  }
  else
  {
    turnToGoalHeading(/*getHeading() + */ 85, 80);
    stop();
    Serial.printf("Final reading %f\n", getHeading());
    while(1);
  }
  getWhiskerDistance();
  #endif

  #ifdef TURN_TO_HEAD
  uint16_t zoooooom = 200; // We're on 3v3 now, so just put a high value
  turnToGoalHeading(90, zoooooom);
  stop();
  delay(2000);
  turnToGoalHeading(0, zoooooom);
  stop();
  delay(2000);
  turnToGoalHeading(180, zoooooom);
  stop();
  delay(2000);
  turnToGoalHeading(0, zoooooom);
  stop();
  delay(2000);
  turnToGoalHeading(270, zoooooom);
  stop();
  delay(2000);
  turnToGoalHeading(45, zoooooom);
  stop();
  delay(2000);
  turnToGoalHeading(225, zoooooom);
  stop();
  delay(2000);
  turnToGoalHeading(135, zoooooom);
  stop();
  delay(2000);
  turnToGoalHeading(315, zoooooom);
  stop();
  delay(2000);
  while (1)
    ;
  #endif
}
