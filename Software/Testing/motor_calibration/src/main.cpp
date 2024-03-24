#include <Arduino.h>
#include <stdbool.h>
#include <Adafruit_BNO08x.h>
#include <Wire.h>
#include <math.h>
#include <ESP32MotorControl.h>
#include <LiquidCrystal_I2C.h>
#include <string.h>

#include "BNO085_heading_acceleration.h"
#include "motor_control_v2.h"
#include "printToLcd.h"

//#define CALIBRATE_STRAIGHT_LINE


// After starting/resetting the robot you have 5 seconds to align it with 0 degress
// We need to implement a blovking function that waits unitl button pressed before locking in 0
// Also we need to implement a lcd screen to show us our angle without uart

/*
 * Global Variables for LCD Display
 */
LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display
char lcdStr[16];
/*
 *Global Variable for the Motors
 */
ESP32MotorControl motors;
// LEFT MOTOR
const uint8_t MOTOR_A_IN_1 = 41;
const uint8_t MOTOR_A_IN_2 = 40;
int16_t A_LEFT_MOTOR_OFFSET = 0;

// RIGHT MOTOR
const uint8_t MOTOR_B_IN_3 = 39;
const uint8_t MOTOR_B_IN_4 = 38;
int16_t B_RIGHT_MOTOR_OFFSET = 5;



/*
 * Global Variable for IMU
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
      retVal += 359.99;
    }
  }

  return retVal;
}

void printCurrentAngle()
{
  float currentAngle = -1;
  while (currentAngle == -1)
  {
    currentAngle = getHeading();
  }
  printToLcd("Current Angle: ", currentAngle);
}

void turnToGoalHeading(float goal, uint8_t speed)
{
  float currentAngle = -1;
  float absVal;
  float angleDiff;
  uint8_t i = 0;

  while (currentAngle == -1)
    currentAngle = getHeading();
  printToLcd("Current Angle: ", currentAngle);
  angleDiff = goal - currentAngle;
  absVal = abs(angleDiff);

  while (absVal > 10)
  {
    // Serial.print("abs: ");
    // Serial.print(absVal);
    // Serial.print(" angle: ");
    // Serial.print(currentAngle);

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
    i++;
    if (i >= 30)
    {
      printCurrentAngle();
      i = 0;
    }

    Serial.println(goal);
    angleDiff = goal - currentAngle;
    absVal = abs(angleDiff);
  }
  stop();
}

void testTurns()
{
  turnToGoalHeading(90, 80);
  printCurrentAngle();
  delay(500);

  turnToGoalHeading(0, 80);
  printCurrentAngle();
  delay(500);

  turnToGoalHeading(180, 80);
  printCurrentAngle();
  delay(500);

  turnToGoalHeading(0, 80);
  printCurrentAngle();
  delay(500);

  turnToGoalHeading(270, 80);
  printCurrentAngle();
  delay(500);

  turnToGoalHeading(45, 80);
  printCurrentAngle();
  delay(500);

  turnToGoalHeading(225, 80);
  printCurrentAngle();
  delay(500);

  turnToGoalHeading(135, 80);
  printCurrentAngle();
  delay(500);

  turnToGoalHeading(315, 80);
  printCurrentAngle();
  delay(500);
}

void setup()
{
  Serial.begin(115200);
  Wire.begin(9, 8);

  lcd.init(); // initialize the lcd
  lcd.backlight();

  setupBNO085(&bno08x);
  motors.attachMotors(MOTOR_A_IN_1, MOTOR_A_IN_2, MOTOR_B_IN_3, MOTOR_B_IN_4);

  uint8_t i = 0;
  Serial.println("*****TEST HEADING******\n\n");

  printToLcd("HEADING", "TEST");
  delay(1000);
  printToLcd("Position to 0", "degrees");
  for (i = 3; i > 0; i--)
  {
    printToLcd("Time left: ", i);
    delay(1000);
  }

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

  printToLcd("OFFSET:", offset);
  delay(1000);
  printCurrentAngle();
  delay(1000);

  // testTurns();
}
void driveToHeading(float goalHeading)
{
  float currentAngle = -1;
  float absVal;
  float angleDiff;
  unsigned long currentMillis;
  unsigned long previousMillis = 0;
  uint32_t interval = 4000;
  uint16_t i = 0;

  bool goToHeading = true;
  while (goToHeading)
  {
    currentAngle = -1;
    while (currentAngle == -1)
      currentAngle = getHeading();
    i++;
    if (i == 30)
    {
      printToLcd("Current Angle: ", currentAngle);
      i = 0;
    }
    angleDiff = goalHeading - currentAngle;
    absVal = abs(angleDiff);
    if (absVal > 15)
    {
      turnToGoalHeading(goalHeading, 80);
      previousMillis = currentMillis = millis();
    }
    else if (absVal <= 15 && absVal >= 5)
    {
      if (angleDiff >= 0)
      {
        turn2(CLOCKWISE, 80, 10);
      }
      else
      {
        turn2(COUNTER_CLOCKWISE, 80, 50);
      }
      previousMillis = currentMillis = millis();
    }
    else if (absVal < 5 && absVal > 2)
    {
      if (angleDiff >= 0)
      {
        turn2(CLOCKWISE, 80, 5);
      }
      else
      {
        turn2(COUNTER_CLOCKWISE, 80, 50);
      }
      previousMillis = currentMillis = millis();
    }
    else
    {
      move(FORWARD, 100);
      unsigned long currentMillis = millis();

      if (currentMillis - previousMillis >= interval)
      {
        previousMillis = currentMillis;
        if (previousMillis != 0)
        {
          goToHeading = false;
        }
      }
    }
  }
}
void loop()
{
  static uint8_t i = 0;
#ifdef CALIBRATE_STRAIGHT_LINE

  move(FORWARD, 100);
  delay(4000);
  stop();
  move(BACKWARD, 100);
  delay(4000);
  stop();
#endif


  while (1)
  {
    i++;
    if (i >= 30)
    {
      printCurrentAngle();
      i = 0;
    }
  }
}
