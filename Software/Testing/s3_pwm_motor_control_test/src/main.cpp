#include "Arduino.h"
#include <ESP32MotorControl.h>
#include "motor_control_v2.h"

/*LEFT MOTOR*/
const uint8_t MOTOR_A_IN_1 = 8;  
const uint8_t MOTOR_A_IN_2 = 7;

/*RIGHT MOTOR*/
const uint8_t MOTOR_B_IN_3 = 5;
const uint8_t MOTOR_B_IN_4 = 4;




ESP32MotorControl motor;

void setup()
{
  motor.attachMotors(MOTOR_A_IN_1, MOTOR_A_IN_2, MOTOR_B_IN_3, MOTOR_B_IN_4);
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
  motor.motorsStop();
  delay(2000);
}