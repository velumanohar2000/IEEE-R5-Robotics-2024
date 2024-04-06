#include <Arduino.h>
#include "SparkFun_VL53L1X.h"
#include "OPT3101_whisker.h"
#include "VL53L1X.h"
#include "motor_control_v2.h"

SFEVL53L1X distanceSensor;
OPT3101 opt3101;

#define MAX_SIZE 10
uint16_t optAverage[MAX_SIZE] = {0};
uint16_t index = 0;
uint16_t sumOPT = 0;

double optBuffer(double val)
{
  double avg;
  sumOPT -= optAverage[index];
  sumOPT += val;
  optAverage[index] = val;
  avg = (double)(sumOPT/MAX_SIZE);
  index = (index + 1) % MAX_SIZE;
  return avg;
}

void fillFirstBuffer()
{
  for(int i = 0; i < MAX_SIZE; i++)
  {
    optBuffer(getWhiskerDistanceCm());
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(9, 8);
  initOPT3101();
  initVL53L1X();
  distanceSensor.startRanging();
  fillFirstBuffer();
}

float getVL53L1XDistanceCm()
{
  while(!distanceSensor.checkForDataReady())
  {
    delay(1);
  }
  float distance = distanceSensor.getDistance()/10;
  distanceSensor.clearInterrupt();
  return distance;
}

void wallDetection()
{
  float distance = getWhiskerDistanceCm();
  if(distance < 20)
  {
    if(getVL53L1XDistanceCm() < 25)
    {
      Serial.println("Wall detected");
    }
    else
    {
      Serial.println("Object detected");
    }
  }
  else
  {
    Serial.println("No wall or object detected");
  }
}

float X_POS = 10;
float Y_POS = 10;
float nextX = 0;
float nextY = 0;



/*
* @param pseudo: Must always be set, but is not used
* @description: acts as a blocking function to goToCoordinates in order to avoid obstacles, then returns
* to the calling goToCoordinates function
*/
void wallDetection(bool pseudo)
{
  float optDistance;

  // we only want to run this when it matters (ie when we are withing the center 6x6 square) to minimize power consumption
  if(X_POS < 60.48 && Y_POS < 60.48)
  {
    
    optDistance = getWhiskerDistanceCm();
    if (optDistance < 15)
    {
      // optDistance = getWhiskerDistanceCm() redundant?
      // if(optDistance < 15)
      // {
      while(optDistance < 15)
      {
        turn(CLOCKWISE, 65);
        delay(500);
      }
      // }
      stopMotors();
      move(FORWARD, 255);
      delay(500);
      stopMotors();
      //   goToCoordinates(COORDINATES); no garbage collector, stack gets full
    }
  }
  else
  {
    distanceSensor.stopRanging();
  }
}

void loop() {

  wallDetection();

  #ifdef TEST_INIT
  Serial.print("VL53L1X: ");
  Serial.print(getVL53L1XDistanceCm());
  distanceSensor.clearInterrupt();
  Serial.print(" OPT3101: ");
  Serial.println(getWhiskerDistanceCm());
  delay(100);
  #endif
  // put your main code here, to run repeatedly:
}

