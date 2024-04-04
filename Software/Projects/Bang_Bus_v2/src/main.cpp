#include <Arduino.h>
#include <Servo.h>
#include <stdbool.h>
#include <Adafruit_BNO08x.h>
#include <Wire.h>
#include <math.h>
#include <ESP32MotorControl.h>
#include <VL53L1X_MULTIPLE.h>
#include "BNO085_heading.h"
#include "motor_control_v2.h"
#include "SparkFun_VL53L1X.h"

// Whisker
#define WHISKER_STOP_DIS 15

#define MAX_PRELIM_DIST 60
#define MIN_WALL_DIST_CM 10
#define MAX_WALL_DIST_CM 12

#define TURN_TO_ANGLE_DIFF 2
#define DRIVE_TO_ANGLE_DIFF 20

// Variables & Constants ------------------------------------------------------

/*
  Global Variable for the Motors
*/
ESP32MotorControl motors;
// Motors
// Left
const uint8_t MOTOR_A_IN_1 = 6;
const uint8_t MOTOR_A_IN_2 = 7;
int16_t A_LEFT_MOTOR_OFFSET = 20;

// RIGHT MOTOR
const uint8_t MOTOR_B_IN_3 = 5;
const uint8_t MOTOR_B_IN_4 = 4;
int16_t B_RIGHT_MOTOR_OFFSET = 0;

/*
  Global Variable for IMU
*/
Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;
float offsetForImu = 0;
float goal = 85;

/*
  Globals for (VL53L1X) Whisker
*/
SFEVL53L1X lrf1_init;
SFEVL53L1X lrf2_init;

float frontVLcm = 1000;
float sideVLcm;
// int frontVLcm = 1000;                            // whisker
float frontVLcmInch = 1000; // whisker
int whiskOffset = 0;
bool wallFound = false;

/*
  globals for servo
*/
Servo myservo = Servo();
uint8_t servoPin = 2;
uint16_t servoPosition = 90;

void setup()
{
  Serial.begin(115200);
  delay(10);
  Serial.println("*****BANGING THE BUS******\n\n");
  delay(1000);

  Wire.begin(9, 8); // init Wire
  init_2_VL53L1X(); // init periscopes

  setupBNO085(&bno08x);                                                        // init IMU
  motors.attachMotors(MOTOR_B_IN_3, MOTOR_B_IN_4, MOTOR_A_IN_1, MOTOR_A_IN_2); // init motors

  uint8_t i = 0;
  offsetForImu = getCurrentAngle();

  Serial.println("offset: ");
  Serial.println(offsetForImu);

  float currentAngle = getCurrentAngle();

  Serial.print("Current Angle: ");
  Serial.println(currentAngle);

  delay(3000);
}

// void driveToHeading(float goalHeading)
// {
//   float currentAngle = -1;
//   float absVal;
//   float angleDiff;
//   unsigned long currentMillis;
//   unsigned long previousMillis = 0;
//   uint32_t interval = 1000;
//   uint16_t i = 0;

//   // bool goToHeading = true;
//   // while (goToHeading)
//   // {
//   currentAngle = getCurrentAngle();

//   angleDiff = goalHeading - currentAngle;
//   absVal = abs(angleDiff);
//   uint32_t turnDiff = DRIVE_TO_ANGLE_DIFF;
//   // Serial.println(absVal);
//   if (absVal > (360 - turnDiff))
//   {
//     absVal = 359.99 - absVal;
//     angleDiff = 359.99 - angleDiff;
//   }
//   Serial.println(angleDiff);
//   Serial.println(absVal);
//   Serial.println();

//   if (absVal > turnDiff)
//   {
//     // stopMotors();
//     // delay(20);
//     turnToHeading(goalHeading-10, 60);
//   }
//   else if (absVal <= 15 && absVal >= 5)
//   {
//     if ((angleDiff >= 0) && (absVal <= 180))
//     {
//       turn2(COUNTER_CLOCKWISE, 65, 30);
//     }
//     else if ((angleDiff < 0) && (absVal <= 180))
//     {
//       turn2(CLOCKWISE, 65, 30);
//     }
//     else if ((angleDiff >= 0) && (absVal >= 180))
//     {
//       turn2(CLOCKWISE, 65, 30);
//     }
//     else
//     {
//       turn2(COUNTER_CLOCKWISE, 65, 30);
//     }
//     previousMillis = currentMillis = millis();
//   }
//   else if (absVal < 5 && absVal >= 2)
//   {
//     if ((angleDiff >= 0) && (absVal <= 180))
//     {
//       turn2(COUNTER_CLOCKWISE, 65, 10);
//     }
//     else if ((angleDiff < 0) && (absVal <= 180))
//     {
//       turn2(CLOCKWISE, 65, 10);
//     }
//     else if ((angleDiff >= 0) && (absVal >= 180))
//     {
//       turn2(CLOCKWISE, 65, 10);
//     }
//     else
//     {
//       turn2(COUNTER_CLOCKWISE, 65, 10);
//     }
//   }
//   else
//   {
//     move(FORWARD, 65);
//   }
//   // }
// }

void turnToHeading(float goal, uint8_t speed)
{
  stopMotors();
  // speed = 60;
  printf("Goal: %f\n", goal);
  // delay(3000);
  float absVal;
  float angleDiff;
  uint8_t i = 0;

  float currentAngle = getCurrentAngle();

  // printToLcd("Current Angle: ", currentAngle);
  angleDiff = goal - currentAngle;
  absVal = abs(angleDiff);
  uint32_t turnDiff = TURN_TO_ANGLE_DIFF;
  // Serial.println(absVal);
  if (absVal > 360 - turnDiff)
  {
    absVal = 359.99 - absVal;
    angleDiff = 359.99 - angleDiff;
  }
  // Serial.println(angleDiff);
  // Serial.println(absVal);
  // Serial.println();

  while (absVal > turnDiff)
  {

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

    // Serial.println(goal);
    angleDiff = goal - currentAngle;
    absVal = abs(angleDiff);
    if (absVal > (360 - TURN_TO_ANGLE_DIFF)) // was (absVal > 360 - turnDiff)
    {
      absVal = 359.99 - absVal;
      angleDiff = 359.99 - angleDiff;
    }
    // Serial.println(angleDiff);
    // Serial.println(absVal);
    // Serial.println();
  }
  stopMotors();
}
void driveToHeading(float goalHeading)
{
  float currentAngle = -1;
  float absVal;
  float angleDiff;
  unsigned long currentMillis;
  unsigned long previousMillis = 0;
  uint32_t interval = 1000;
  uint16_t i = 0;

  // bool goToHeading = true;
  // while (goToHeading)
  // {
  currentAngle = getCurrentAngle();

  angleDiff = goalHeading - currentAngle;
  absVal = abs(angleDiff);
  uint32_t turnDiff = DRIVE_TO_ANGLE_DIFF;
  // Serial.println(absVal);
  if (absVal > (360 - turnDiff))
  {
    absVal = 359.99 - absVal;
    angleDiff = 359.99 - angleDiff;
  }
  Serial.println(angleDiff);
  Serial.println(absVal);
  Serial.println();

  if (absVal > turnDiff)
  {
    // stopMotors();
    // delay(20);
    turnToHeading(goalHeading, 60);
  }
  else if (absVal <= 15 && absVal >= 5)
  {
    if ((angleDiff >= 0) && (absVal <= 180))
    {
      turn2(COUNTER_CLOCKWISE, 65, 30);
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
    previousMillis = currentMillis = millis();
  }
  else if (absVal < 5 && absVal >= 2)
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
  }
  else
  {
    move(FORWARD, 65);
  }
  // }
}

unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
uint32_t interval = 1000;

void loop()
{

  if (getLrfDistanceCm(2) >= 15)
  {
    turnToHeading(270, 60);
    currentMillis = millis();
    previousMillis = millis();
    while (currentMillis - previousMillis <= 2000)
    {
      printf("Current Angle: %f\n", getCurrentAngle());
      currentMillis = millis();
    }
    // turnToHeading(270, 60);

    //  delay(3000);
    while (getLrfDistanceCm(1) >= 20)
    {
      driveToHeading(270);
    }
  }

  stopMotors();
  delay(2000);
  turnToHeading(0, 60);
  delay(2000);
  while (getLrfDistanceCm(1) >= 20)
  {
    driveToHeading(0);
  }
  stopMotors();

  while (1)
  {
    currentMillis = millis();
    if (currentMillis - previousMillis >= interval)
    {
      printf("LRF 1 (cm): %f\tLRF 2 (cm): %f\n", getLrfDistanceCm(1), getLrfDistanceCm(2));
      printf("Current Angle: %f\n", getCurrentAngle());
      previousMillis = millis();
    }
  }
}

// int16_t findCardinalheading()
// {
//   float currentAngle = getCurrentAngle();
//   if (currentAngle >= 0 && currentAngle < 45 || currentAngle >= 315)
//   {
//     return 0; // North
//   }
//   else if (currentAngle >= 45 && currentAngle < 135)
//   {
//     return 90; // East
//   }
//   else if (currentAngle >= 135 && currentAngle < 225)
//   {
//     return 180; // South
//   }
//   else if (currentAngle >= 225 && currentAngle < 315)
//   {
//     return 270; // West
//   }
//   else
//   {
//     return -1; // Error
//   }
// }

// void jiggle()
// {
//   float lrf1;
//   float lrf2;
//   float currentHeading = getCurrentAngle();
//   uint16_t cardinalHeading = findCardinalheading();
//   float servoPosition = cardinalHeading - currentHeading; // Calculate the difference between the cardinal heading and the current heading
//   if (abs(servoPosition) > 90)
//   {
//     servoPosition = 359.99 - abs(servoPosition); // 359.99 is the max value for the servo
//   }

//   servoPosition += 90;                    // Add 90 because 90 degress is the middle position for the servo (therefore it is the current heading of the car)
//   myservo.write(servoPin, servoPosition); // tell servo to go to position in variable 'pos'
//   delay(15);                              // waits 15ms for the servo to reach the position
//   lrf1 = getLrfDistanceCm(1);
//   lrf2 = getLrfDistanceCm(2);
//   // Serial.print("LRF1: ");
//   // Serial.print(lrf1);
//   // Serial.print("\tLRF2: ");
//   // Serial.println(lrf2);
//   // findPosition(cardinalHeading, lrf1, lrf2); // Find the position of the car
// }