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
// #include <LiquidCrystal_I2C.h>
#include "Ultrasonic.h"
#include "BNO085_heading_acceleration.h"
#include "motor_control_v2.h"
#include "SparkFun_VL53L1X.h"
#include "OPT3101_whisker.h"

// Defines --------------------------------------------------------------------

// Preprocessor Directives
// #define TURN_TO_HEAD
// #define MAIN
// #define PRELIM
#define PRELIM_OPT

// Whisker
#define WHISKER_STOP_DIS 200


// Ultrasonic
#define ULTRAS_TRIG 14
#define ULTRAS_ECHO 11
#define MAX_PRELIM_DIST 60
#define MIN_WALL_DIST_CM 4
#define MAX_WALL_DIST_CM 6

// Variables & Constants ------------------------------------------------------

// Motors
// Left
const uint8_t motor_a_in1 = 7;
const uint8_t motor_a_in2 = 6;
// Right
const uint8_t motor_b_in3 = 4;
const uint8_t motor_b_in4 = 5;

int16_t A_LEFT_MOTOR_OFFSET = 0;
int16_t B_RIGHT_MOTOR_OFFSET = 0;

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
float goal = 85;

/*
  Globals for (VL53L1X) Whisker
*/
SFEVL53L1X distanceSensor;
int whiskDistance = 1000;                            // whisker
float whiskDistanceInch = 1000;               // whisker
int whiskOffset = 0;
bool wallFound = false;

/*
  Globals for ultrasonic
*/
Ultrasonic ultrasonic(ULTRAS_TRIG, ULTRAS_ECHO);
int ultraDistanceCm = 100;                      // ultrasonic
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
    {
      // neopixelWrite(LED_BUILTIN, 0xFF, 0, 0);
    }
  }
}

/*
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
      retVal += 359.99;
    }
  }

  return retVal;
}
*/

void turnToGoalHeading(float goal, uint8_t speed)
{
  float currentAngle = -1;
  float absVal;
  float angleDiff;
  if(goal >= 360)
    goal -= 359.99;

  while (currentAngle == -1)
    currentAngle = getHeading();

  angleDiff = goal - currentAngle;
  absVal = abs(angleDiff);

  while (absVal > 4)
  {
    // getWhiskerDistance();
    // Serial.printf("Whiskaaa is %f: \n", whiskDistanceInch);
    Serial.print("Offset ");
    Serial.println(offset);
    Serial.print(" Abs: ");
    Serial.println(absVal);
    Serial.print(" Current Angle: ");
    Serial.println(currentAngle);

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

/*
void driveToHeading(float goalHeading)
{
  float currentAngle = -1;
  float absVal;
  float angleDiff;
  unsigned long currentMillis;
  unsigned long previousMillis = 0;
  uint32_t interval = 1000;
  uint16_t i = 0;

  bool goToHeading = true;
  while (goToHeading)
  {
    // currentAngle = getCurrentAngle();

    // i++;
    // if (i == 30)
    // {
    //   printToLcd("Current Angle: ", currentAngle);
    //   i = 0;
    // }
    angleDiff = goalHeading - currentAngle;
    absVal = abs(angleDiff);
    // Serial.println(absVal);
    if (absVal > 345)
    {
      absVal = 359.99 - absVal;
      angleDiff = 359.99 - angleDiff;
    }
    Serial.println(angleDiff);
    Serial.println(absVal);
    Serial.println();

    if (absVal > 15)
    {
      turnToGoalHeading(goalHeading, 65);
      previousMillis = currentMillis = millis();
    }
    else if (absVal <= 15 && absVal >= 3)
    {
      if ((angleDiff >= 0) && (absVal <= 180))
      {
        // Serial.print(" case 1: ");
        turn2(COUNTER_CLOCKWISE, 65, 30);
      }
      else if ((angleDiff < 0) && (absVal <= 180))
      {
        // Serial.print(" case 2: ");
        turn2(CLOCKWISE, 65, 30);
      }
      else if ((angleDiff >= 0) && (absVal >= 180))
      {
        // Serial.print(" case 3: ");
        turn2(CLOCKWISE, 65, 30);
      }
      else
      {
        // Serial.print(" case 4: ");
        turn2(COUNTER_CLOCKWISE, 65, 30);
      }
      previousMillis = currentMillis = millis();
    }
    else if (absVal < 5 && absVal >= 2)
    {
      if ((angleDiff >= 0) && (absVal <= 180))
      {
        // Serial.print(" case 1: ");
        turn2(COUNTER_CLOCKWISE, 65, 10);
      }
      else if ((angleDiff < 0) && (absVal <= 180))
      {
        // Serial.print(" case 2: ");
        turn2(CLOCKWISE, 65, 10);
      }
      else if ((angleDiff >= 0) && (absVal >= 180))
      {
        // Serial.print(" case 3: ");
        turn2(CLOCKWISE, 65, 10);
      }
      else
      {
        // Serial.print(" case 4: ");
        turn2(COUNTER_CLOCKWISE, 65, 10);
      }
      unsigned long currentMillis = millis();

      if (currentMillis - previousMillis >= interval)
      {
        previousMillis = currentMillis;
        if (previousMillis != 0)
        {
          goToHeading = false;
          stop();
        }
      }
    }
    else
    {
      move(FORWARD, 65);

      if (currentMillis - previousMillis >= interval)
      {
        previousMillis = currentMillis;
        if (previousMillis != 0)
        {
          goToHeading = false;
          stop();
        }
      }
    }
  }
}
*/

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
  // initVL53L1X();
  initOPT3101();
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

  // neopixelWrite(LED_BUILTIN, 0, 0, 0xFF);

  delay(2500);
}

uint16_t states = 0;
bool firstWall = true;
#define RIDE_RIGHT_WALL 0
#define TURN_90_DEG 1
#define TRAVEL_12_INCHES 2
#define TURN_RIGHT_TOWARDS_BUTTON 3
#define HIT_BUTTON 4
#define MOVE_BACKWARDS 5
#define MOVE_BACKWARDS_2 6
#define TURN_LEFT_TOWARDS_WALL 7
#define GO_TO_WALL_END 8
#define TURN_180_DEG 9
#define STOP 0xFFFF

void loop()
{
  uint16_t speed = 255;
  float currentAngle;
  // int16_t whiskDistance = 0;

  #ifdef PRELIM
  switch(states)
  {
    case RIDE_RIGHT_WALL:
    {
      
      getWhiskerDistance();
      currentAngle = getHeading();
      if(whiskDistance> (WHISKER_STOP_DIS))
      {
        if(whiskDistance > 1200)
          speed = 255;
        else if( whiskDistance > 300)
          speed = 80;
        else
          speed = 50;
        ultraDistanceCm = ultrasonic.read();

        Serial.printf("ult dist %d: \n", ultraDistanceCm);
        if(ultraDistanceCm > MAX_PRELIM_DIST)
        {
          stop();
        }
        else if(ultraDistanceCm > MAX_WALL_DIST_CM)
        {
          turn(COUNTER_CLOCKWISE, speed);
          delay(80);
          move(FORWARD, speed);
          delay(80);
        }
        else if(ultraDistanceCm < MIN_WALL_DIST_CM)
        {
          turn(CLOCKWISE, speed);
          delay(80);
          move(FORWARD, speed);
          delay(80);
        }
        else
          move(FORWARD, speed);
      }
      else
      {
        stop();
        states = TURN_90_DEG;
        currentAngle = getHeading();
      }
      break;
    }
    case TURN_90_DEG:
    {
      goal = 85;
      if(!firstWall)
        goal += 180;
      turnToGoalHeading(/*getHeading() + */ goal, 80);
      stop();
      states = TRAVEL_12_INCHES;
      break;
    }
    case TRAVEL_12_INCHES:
    {
      getWhiskerDistance();
      currentAngle = getHeading();
      if(whiskDistance> (250))
      {
        if(whiskDistance > 1200)
          speed = 255;
        else if( whiskDistance > 300)
          speed = 80;
        else
          speed = 65;
        ultraDistanceCm = ultrasonic.read();

        Serial.printf("ult dist %d: \n", ultraDistanceCm);
        if(ultraDistanceCm > MAX_PRELIM_DIST)
        {
          stop();
        }
        else if(ultraDistanceCm > 15)
        {
          turn(COUNTER_CLOCKWISE, speed);
          delay(80);
          move(FORWARD, speed);
          delay(80);
        }
        else if(ultraDistanceCm < 10)
        {
          turn(CLOCKWISE, speed);
          delay(80);
          move(FORWARD, speed);
          delay(80);
        }
        else
          move(FORWARD, speed);
      }
      else
      {
        stop();
        states = TURN_RIGHT_TOWARDS_BUTTON;
        currentAngle = getHeading();
      }
      break;
    }
    case TURN_RIGHT_TOWARDS_BUTTON:
    {
      goal = 4;
      if(!firstWall)
        goal += 180;
      turnToGoalHeading(/*getHeading() + */ goal, 80);
      stop();
      states = HIT_BUTTON;
      break;
    }
    case HIT_BUTTON:
    {
      move(FORWARD, 255);
      delay(750);
      stop();
      states = MOVE_BACKWARDS;
      break;
    }
    case MOVE_BACKWARDS:
    {
      move(BACKWARD, 80);
      delay(750);
      stop();
      states = TURN_LEFT_TOWARDS_WALL;
      break;
    }
    case TURN_LEFT_TOWARDS_WALL:
    {
      goal = 85;
      if(!firstWall)
        goal += 180;
      turnToGoalHeading(/*getHeading() + */ goal, 80);
      stop();
      states = MOVE_BACKWARDS_2;
      break;
    }
    case MOVE_BACKWARDS_2:
    { 
      move(BACKWARD, 80);
      delay(750);
      stop();
      states = GO_TO_WALL_END;
      break;
    }
    case GO_TO_WALL_END:
    {
      getWhiskerDistance();
      currentAngle = getHeading();
       if(whiskDistance> (250))
      {
        if(whiskDistance > 1200)
          speed = 255;
        else if( whiskDistance > 300)
          speed = 80;
        else
          speed = 65;
        ultraDistanceCm = ultrasonic.read();

        Serial.printf("ult dist %d: \n", ultraDistanceCm);
        if(ultraDistanceCm > MAX_PRELIM_DIST)
        {
          stop();
        }
        else if(ultraDistanceCm > MAX_WALL_DIST_CM)
        {
          turn(COUNTER_CLOCKWISE, speed);
          delay(80);
          move(FORWARD, speed);
          delay(80);
        }
        else if(ultraDistanceCm < MIN_WALL_DIST_CM)
        {
          turn(CLOCKWISE, speed);
          delay(80);
          move(FORWARD, speed);
          delay(80);
        }
        else
          move(FORWARD, speed);
      }
      else
      {
        stop();
        states = TURN_180_DEG;
        currentAngle = getHeading();
      }
      break;
    }
    case TURN_180_DEG:
    {
      goal = 180;
      if(!firstWall)
        goal += 180;
      turnToGoalHeading(/*getHeading() + */ goal, 80);
      stop();
      if(firstWall)
      {
        states = RIDE_RIGHT_WALL;
        firstWall = false;
      }
      elseWhisker.distanceMillimeters
      {
        states = STOP;
      }

      break;
    }
    case STOP:
    {
      stop();
      while(1);
    }

  }
  

  #endif

  #ifdef PRELIM_OPT
  switch(states)
  {
    Serial.println(getWhiskerDistanceCm());
    case RIDE_RIGHT_WALL:
    {
      whiskDistance = getWhiskerDistanceCm();
      // getWhiskerDistance();
      currentAngle = getHeading();
      if(whiskDistance> (WHISKER_STOP_DIS / 10))
      {
        if(whiskDistance > 1200)
          speed = 255;
        else if( whiskDistance > 300)
          speed = 80;
        else
          speed = 50;
        ultraDistanceCm = ultrasonic.read();

        Serial.printf("ult dist %d: \n", ultraDistanceCm);
        if(ultraDistanceCm > MAX_PRELIM_DIST)
        {
          stop();
        }
        else if(ultraDistanceCm > MAX_WALL_DIST_CM)
        {
          turn(COUNTER_CLOCKWISE, speed);
          delay(80);
          move(FORWARD, speed);
          delay(80);
        }
        else if(ultraDistanceCm < MIN_WALL_DIST_CM)
        {
          turn(CLOCKWISE, speed);
          delay(80);
          move(FORWARD, speed);
          delay(80);
        }
        else
          move(FORWARD, speed);
      }
      else
      {
        stop();
        states = TURN_90_DEG;
        currentAngle = getHeading();
      }
      break;
    }
    case TURN_90_DEG:
    {
      goal = 85;
      if(!firstWall)
        goal += 180;
      turnToGoalHeading(/*getHeading() + */ goal, 80);
      stop();
      states = TRAVEL_12_INCHES;
      break;
    }
    case TRAVEL_12_INCHES:
    {
      whiskDistance = getWhiskerDistanceCm();
      currentAngle = getHeading();
      if(whiskDistance> (250))
      {
        if(whiskDistance > 1200)
          speed = 255;
        else if( whiskDistance > 300)
          speed = 80;
        else
          speed = 65;
        ultraDistanceCm = ultrasonic.read();

        Serial.printf("ult dist %d: \n", ultraDistanceCm);
        if(ultraDistanceCm > MAX_PRELIM_DIST)
        {
          stop();
        }
        else if(ultraDistanceCm > 15)
        {
          turn(COUNTER_CLOCKWISE, speed);
          delay(80);
          move(FORWARD, speed);
          delay(80);
        }
        else if(ultraDistanceCm < 10)
        {
          turn(CLOCKWISE, speed);
          delay(80);
          move(FORWARD, speed);
          delay(80);
        }
        else
          move(FORWARD, speed);
      }
      else
      {
        stop();
        states = TURN_RIGHT_TOWARDS_BUTTON;
        currentAngle = getHeading();
      }
      break;
    }
    case TURN_RIGHT_TOWARDS_BUTTON:
    {
      goal = 4;
      if(!firstWall)
        goal += 180;
      turnToGoalHeading(/*getHeading() + */ goal, 80);
      stop();
      // states = HIT_BUTTON;
      states = STOP;
      break;
    }
    case HIT_BUTTON:
    {
      move(FORWARD, 255);
      delay(750);
      stop();
      states = MOVE_BACKWARDS;
      break;
    }
    case MOVE_BACKWARDS:
    {
      move(BACKWARD, 80);
      delay(750);
      stop();
      states = TURN_LEFT_TOWARDS_WALL;
      break;
    }
    case TURN_LEFT_TOWARDS_WALL:
    {
      goal = 85;
      if(!firstWall)
        goal += 180;
      turnToGoalHeading(/*getHeading() + */ goal, 80);
      stop();
      states = MOVE_BACKWARDS_2;
      break;
    }
    case MOVE_BACKWARDS_2:
    { 
      move(BACKWARD, 80);
      delay(750);
      stop();
      states = GO_TO_WALL_END;
      break;
    }
    case GO_TO_WALL_END:
    {
      whiskDistance = getWhiskerDistanceCm();
      currentAngle = getHeading();
       if(whiskDistance> (250))
      {
        if(whiskDistance > 1200)
          speed = 255;
        else if( whiskDistance > 300)
          speed = 80;
        else
          speed = 65;
        ultraDistanceCm = ultrasonic.read();

        Serial.printf("ult dist %d: \n", ultraDistanceCm);
        if(ultraDistanceCm > MAX_PRELIM_DIST)
        {
          stop();
        }
        else if(ultraDistanceCm > MAX_WALL_DIST_CM)
        {
          turn(COUNTER_CLOCKWISE, speed);
          delay(80);
          move(FORWARD, speed);
          delay(80);
        }
        else if(ultraDistanceCm < MIN_WALL_DIST_CM)
        {
          turn(CLOCKWISE, speed);
          delay(80);
          move(FORWARD, speed);
          delay(80);
        }
        else
          move(FORWARD, speed);
      }
      else
      {
        stop();
        states = TURN_180_DEG;
        currentAngle = getHeading();
      }
      break;
    }
    case TURN_180_DEG:
    {
      goal = 180;
      if(!firstWall)
        goal += 180;
      turnToGoalHeading(/*getHeading() + */ goal, 80);
      stop();
      if(firstWall)
      {
        states = RIDE_RIGHT_WALL;
        firstWall = false;
      }
      else
      {
        states = STOP;
      }

      break;
    }
    case STOP:
    {
      stop();
      while(1);
    }

  }
  #endif

  #ifdef MAIN
  float currentAngle;
  if(!wallFound)
  {
    currentAngle = getHeading();
    if(whiskDistance> (WHISKER_STOP_DIS - whiskOffset))
    {
      ultraDistanceCm = ultrasonic.read();

      Serial.printf("ult dist %d: \n", ultraDistanceCm);
      if(ultraDistanceCm > MAX_PRELIM_DIST)
      {
        stop();
      }
      else if(ultraDistanceCm > MAX_WALL_DIST_CM)
      {
        turn(COUNTER_CLOCKWISE, 255);
        delay(80);
        move(FORWARD, 255);
        delay(80);
      }
      else if(ultraDistanceCm < MIN_WALL_DIST_CM)
      {
        turn(CLOCKWISE, 255);
        delay(80);
        move(FORWARD, 255);
        delay(80);
      }
      else
        move(FORWARD, 255);
    }
    else
    {
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
    turnToGoalHeading(/*getHeading() + */ goal, 80);
    stop();
    if(whiskOffset == 0)
    {
      whiskOffset = 10;
    }
    else
    {
      whiskOffset = 0;
    }
    goal += 90;
    if (goal > 359.999)
    {
      goal -= 359.999;
    }
    {
      /* code */
    }
    
    Serial.printf("Final reading %f\n", getHeading());
    delay(1000);
    wallFound = false;
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
