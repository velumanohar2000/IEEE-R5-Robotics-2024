#include <Arduino.h>
#include <stdbool.h>
#include <Adafruit_BNO08x.h>
#include <Wire.h>
#include <math.h>
#include <ESP32MotorControl.h>
#include <LiquidCrystal_I2C.h>
#include <string.h>
#include <Servo.h>

#include "motor_control_v2.h"
#include "BNO085_heading_acceleration.h"
#include "lrf.h"

#define LRF_ADDRESS_1 0x10
#define LRF_ADDRESS_2 0x20

#define ELIMINATION
// #define TEST
// #define ANGLE
#define MOTOR_IN_1  7
#define MOTOR_IN_2  6 
#define MOTOR_IN_3  4
#define MOTOR_IN_4  5

float currentCardinalHeading;

/*
 * Global Variables for IMU
 */
Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;
ESP32MotorControl motors;

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}