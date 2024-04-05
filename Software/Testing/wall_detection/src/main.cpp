#include <Arduino.h>
#include "SparkFun_VL53L1X.h"
#include "OPT3101_whisker.h"
#include "VL53L1X.h"

SFEVL53L1X distanceSensor;
OPT3101 opt3101;

void setup() {
  Serial.begin(115200);
  Wire.begin(9, 8);
  initOPT3101();
  initVL53L1X();
  distanceSensor.startRanging();
}

void loop() {
  while(!distanceSensor.checkForDataReady())
  {
    delay(1);
  }

  Serial.print("VL53L1X: ");
  Serial.print(distanceSensor.getDistance()/10);
  distanceSensor.clearInterrupt();
  Serial.print(" OPT3101: ");
  Serial.println(getWhiskerDistanceCm());
  delay(100);
  // put your main code here, to run repeatedly:
}

