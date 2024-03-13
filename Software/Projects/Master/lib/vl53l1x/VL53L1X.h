#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_VL53L1X.h"

#define UNIT_MILIMETER 0
#define UNIT_INCHES 1
#define UNIT_FEET 2

void initVL53L1X(void);
int getDistanceVL53L1X(uint8_t unit);