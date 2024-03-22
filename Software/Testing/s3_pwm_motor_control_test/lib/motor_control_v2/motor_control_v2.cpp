/*
 * Bit Bangers Motor Control Library
 * github.com/Bit-Bangers-UTA/Senior-Design
 *
 * Authors:
 * Noor Abdullah
 * Rolando Rosales
 * Velu Manohar
 * 
 * Works with any dual-motor driver that has two PWM inputs per motor
 * Tested with MX1508
*/

// Libraries ------------------------------------------------------------------

#include <Arduino.h>
#include <ESP32MotorControl.h>
#include "motor_control_v2.h"

// Defines --------------------------------------------------------------------
// #define MOTOR_A 0
// #define MOTOR_B 1

// #define FORWARD 1
// #define BACKWARD 0
// #define COUNTER_CLOCKWISE 1
// #define CLOCKWISE 0


// Variables & Constants ------------------------------------------------------

extern ESP32MotorControl motor;

// Structures & Classes -------------------------------------------------------

// Functions ------------------------------------------------------------------

void move(bool direction)
{
  if(direction == FORWARD)
  {
    motor.motorFullForward(MOTOR_A);
    motor.motorFullForward(MOTOR_B);

  }
  else
  {
    motor.motorFullReverse(MOTOR_A);
    motor.motorFullReverse(MOTOR_B);
  }
}

void move(bool direction, uint16_t speed)
{
  if(direction == FORWARD)
  {
    motor.motorForward(MOTOR_A, speed);
    motor.motorForward(MOTOR_B, speed);

  }
  else
  {
    motor.motorReverse(MOTOR_A, speed);
    motor.motorReverse(MOTOR_B, speed);
  }
}

void stop()
{
  motor.motorsStop();
}

void stop(uint8_t mot) //motor a or b
{
  motor.motorStop(mot);
}


void turn(bool direction)
{
  if(direction == COUNTER_CLOCKWISE)
  {
    motor.motorFullForward(MOTOR_A);   
    motor.motorStop(MOTOR_B);
  }
  else
  {
    motor.motorFullForward(MOTOR_B);
    motor.motorStop(MOTOR_A);
  }
}

void turn(bool direction, uint16_t speed)
{
  if(direction == COUNTER_CLOCKWISE)
  {
    motor.motorForward(MOTOR_A, speed);   
    motor.motorStop(MOTOR_B);
  }
  else
  {
    motor.motorForward(MOTOR_B, speed);
    motor.motorStop(MOTOR_A);
  }
}
