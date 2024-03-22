#include "Arduino.h"
#include <ESP32MotorControl.h>
#include "motor_control_v2.h"

/*LEFT MOTOR*/
const uint8_t MOTOR_A_IN_1 = 41;
const uint8_t MOTOR_A_IN_2 = 40;

/*RIGHT MOTOR*/
const uint8_t MOTOR_B_IN_3 = 39;
const uint8_t MOTOR_B_IN_4 = 38;

ESP32MotorControl motors;

void setup()
{
  motors.attachMotors(MOTOR_A_IN_1, MOTOR_A_IN_2, MOTOR_B_IN_3, MOTOR_B_IN_4);
}

void loop()
{
  move(FORWARD);
  delay(2000);
  move(BACKWARD);
  delay(2000);
  move(FORWARD, 60);
  delay(2000);
  move(BACKWARD, 60);
  delay(2000);
  turn(CLOCKWISE);
  delay(2000);
  turn(COUNTER_CLOCKWISE);
  delay(2000);
  delay(2000);
  turn(CLOCKWISE, 60);
  delay(2000);
  turn(COUNTER_CLOCKWISE, 60);
  delay(2000);
  stop();
  delay(2000);
}