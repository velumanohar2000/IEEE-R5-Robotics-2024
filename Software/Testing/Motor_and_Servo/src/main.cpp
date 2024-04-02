#include <Arduino.h>
#include <ESP32MotorControl.h>
#include "motor_control_v2.h"
#include <Servo.h>

#define MOTORS
#define SERVO

const uint8_t MOTOR_A_IN_1 = 5;
const uint8_t MOTOR_A_IN_2 = 4;
int16_t A_LEFT_MOTOR_OFFSET = 0;

// RIGHT MOTOR
const uint8_t MOTOR_B_IN_3 = 7;
const uint8_t MOTOR_B_IN_4 = 6;
int16_t B_RIGHT_MOTOR_OFFSET = 2;

ESP32MotorControl motors;
Servo myservo = Servo();
const int servoPin = 2;
uint16_t servoPosition = 90;

void setup() {
  // put your setup code here, to run once:
  motors.attachMotors(MOTOR_A_IN_1, MOTOR_A_IN_2, MOTOR_B_IN_3, MOTOR_B_IN_4);
}

void loop() {
  // put your main code here, to run repeatedly:
  #ifdef MOTORS
  move(FORWARD);
  #endif

  #ifdef SERVO
  for (int pos = 0; pos <= 180; pos++) {  // go from 0-180 degrees
    myservo.write(servoPin, pos);        // set the servo position (degrees)
  }
  for (int pos = 180; pos >= 0; pos--) {  // go from 180-0 degrees
    myservo.write(servoPin, pos);        // set the servo position (degrees)
    delay(15);
  }
  delay(1000);
  #endif

}
