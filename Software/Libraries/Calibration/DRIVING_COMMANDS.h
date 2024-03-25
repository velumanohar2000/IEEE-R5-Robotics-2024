#include <Arduino.h>
#include <motor_control_v2.h>
#include <Adafruit_BNO08x.h>
// #include "DRIVING_COMMANDS.h"

float getHeading(sh2_SensorValue_t sensorValue);
float getCurrentAngle(sh2_SensorValue_t sensorValue);
void printCurrentAngle(sh2_SensorValue_t sensorValue);
void turnToHeading(sh2_SensorValue_t sensorValue, float goal, uint8_t speed);
void driveToHeading(sh2_SensorValue_t sensorValue, float goalHeading);
void calibrateStraightLine();