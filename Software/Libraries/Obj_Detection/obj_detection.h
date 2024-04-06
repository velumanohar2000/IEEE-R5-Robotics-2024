#include <Arduino.h>
#include "SparkFun_VL53L1X.h"
#include "OPT3101_whisker.h"
#include "VL53L1X.h"

SFEVL53L1X distanceSensor;
OPT3101 opt3101;

#define MAX_SIZE 10
uint16_t optAverage[MAX_SIZE] = {0};
uint16_t index = 0;
uint16_t sumOPT = 0;

double optBuffer(double val);
void fillFirstBuffer();


float getVL53L1XDistanceCm();

void wallDetection();

extern float X_POS = 10;
extern float Y_POS = 10;
extern float nextX = 0;
extern float nextY = 0;

void wallDetection(bool pseudo);

