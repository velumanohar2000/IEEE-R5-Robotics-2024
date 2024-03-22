#include <Arduino.h>
#include <stdbool.h>
#include <Adafruit_BNO08x.h>
#include <Wire.h>
#include <math.h>

#include "BNO085_heading_acceleration.h"
#include "motor_control_v2.h"


/*LEFT MOTOR*/
const uint8_t MOTOR_A_IN_1 = 8;  
const uint8_t MOTOR_A_IN_2 = 7;

/*RIGHT MOTOR*/
const uint8_t MOTOR_B_IN_3 = 5;
const uint8_t MOTOR_B_IN_4 = 4;

ESP32MotorControl motor;

void setup() {
    motor.attachMotors(MOTOR_A_IN_1, MOTOR_A_IN_2, MOTOR_B_IN_3, MOTOR_B_IN_4);
}

void loop() {
}

