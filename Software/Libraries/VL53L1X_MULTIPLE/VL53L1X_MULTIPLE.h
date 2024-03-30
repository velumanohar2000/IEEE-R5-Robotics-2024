#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_VL53L1X.h"

void init_2_VL53L1X(void);
float getLrfDistanceCm(uint8_t lrfNum);
