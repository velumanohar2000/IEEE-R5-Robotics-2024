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

extern ESP32MotorControl motors;
extern int16_t A_LEFT_MOTOR_OFFSET;
extern int16_t B_RIGHT_MOTOR_OFFSET;
extern Adafruit_BNO08x bno08x;
extern sh2_SensorValue_t sensorValue;
extern float offset;

 


void setReports(Adafruit_BNO08x *bno08x, sh2_SensorId_t reportType, uint32_t interval)
{
  // Serial.println("Setting desired reports");
  if (!bno08x->enableReport(reportType, interval)) // Top frequency is about 250Hz but this report is more accurate
  {
    // Serial.println("Could not enable stabilized remote vector");
  }
}

void reports(Adafruit_BNO08x *bno08x)
{
  // setReports(bno08x, SH2_LINEAR_ACCELERATION, 500);
  setReports(bno08x, SH2_ARVR_STABILIZED_RV, 5000);
  // setReports(bno08x, SH2_GYROSCOPE_CALIBRATED, 2500);
}


void setupBNO085(Adafruit_BNO08x *bno08x) 
{
  // Serial.begin(115200);
  // Serial.println("Adafruit BNO08x test!");
  // Wire.begin(9, 8);
  if (!bno08x->begin_I2C())
  {
    // Serial.println("Failed to find BNO08x chip");
    while (1)
    {
      delay(10);
    }
  }
  // Serial.println("BNO08x Found!");
  reports(bno08x);
  // Serial.println("Reading events");
  delay(1000);
}

float calculateHeading(float i, float j, float k, float real)
{
  float yaw = atan2(2.0f*(i*j+real*k),real*real+i*i-j*j-k*k);
  yaw *= 180.0f / M_PI;
  // Normalize heading angle
  if (yaw < 0) {
    yaw += 359.99f;
  }
  return yaw;
}

void checkReset(Adafruit_BNO08x *bno08x)
{
  if (bno08x->wasReset())
  {
    // Serial.print("sensor was reset ");
    reports(bno08x);
  }
}

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
float getCurrentAngle()
{
  float currentAngle = -1;
  while (currentAngle == -1)
  {
    currentAngle = getHeading();
  }
  return currentAngle;
}


void turnToHeading(float goal, uint8_t speed)
{
  float absVal;
  float angleDiff;
  uint8_t i = 0;

  float currentAngle = getCurrentAngle();

  printToLcd("Current Angle: ", currentAngle);
  angleDiff = goal - currentAngle;
  absVal = abs(angleDiff);
  if (absVal > 350)
  {
    absVal = 359.99 - absVal;
    angleDiff = 359.99 - angleDiff;
  }
  Serial.println(angleDiff);
  Serial.println(absVal);
  Serial.println();

  while (absVal > 10)
  {
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

    // Serial.println(goal);
    angleDiff = goal - currentAngle;
    absVal = abs(angleDiff);
    if (absVal > 350)
    {
      absVal = 359.99 - absVal;
      angleDiff = 359.99 - angleDiff;
    }
    Serial.println(angleDiff);
    Serial.println(absVal);
    Serial.println();
  }
  stop();
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

  bool goToHeading = true;
  while (goToHeading)
  {
    currentAngle = getCurrentAngle();

    i++;
    if (i == 30)
    {
      printToLcd("Current Angle: ", currentAngle);
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
    Serial.println(angleDiff);
    Serial.println(absVal);
    Serial.println();

    if (absVal > 15)
    {
      turnToHeading(goalHeading, 65);
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
