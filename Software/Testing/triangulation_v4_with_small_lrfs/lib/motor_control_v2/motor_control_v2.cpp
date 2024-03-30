/*
 * Bit Bangers motors Control Library
 * github.com/Bit-Bangers-UTA/Senior-Design
 *
 * Authors:
 * Noor Abdullah
 * Rolando Rosales
 * Velu Manohar
 * 
 * Works with any dual-motors driver that has two PWM inputs per motors
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

extern ESP32MotorControl motors;

// Functions ------------------------------------------------------------------

void move(bool direction)
{
  if(direction == FORWARD)
  {
    motors.motorFullForward(MOTOR_A);
    motors.motorFullForward(MOTOR_B);

  }
  else
  {
    motors.motorFullReverse(MOTOR_A);
    motors.motorFullReverse(MOTOR_B);
  }
}

void move(bool direction, uint16_t speed)
{
  if(direction == FORWARD)
  {
    motors.motorForward(MOTOR_A, speed);
    motors.motorForward(MOTOR_B, speed);

  }
  else
  {
    motors.motorReverse(MOTOR_A, speed);
    motors.motorReverse(MOTOR_B, speed);
  }
}

void stop()
{
  motors.motorsStop();
}

void stop(uint8_t mot) //motor a or b
{
  motors.motorStop(mot);
}


void turn(bool direction)
{
  if(direction == COUNTER_CLOCKWISE)
  {
    motors.motorFullForward(MOTOR_A);   
    motors.motorStop(MOTOR_B);
  }
  else
  {
    motors.motorFullForward(MOTOR_B);
    motors.motorStop(MOTOR_A);
  }
}

void turn(bool direction, uint16_t speed)
{
  if(direction == COUNTER_CLOCKWISE)
  {
    motors.motorForward(MOTOR_A, speed);   
    motors.motorStop(MOTOR_B);
  }
  else
  {
    motors.motorForward(MOTOR_B, speed);
    motors.motorStop(MOTOR_A);
  }
}


void turn2(bool direction)
{
  if(direction == COUNTER_CLOCKWISE)
  {
    motors.motorFullForward(MOTOR_A);   
    motors.motorFullReverse(MOTOR_B);
  }
  else
  {
    motors.motorFullForward(MOTOR_B);
    motors.motorFullReverse(MOTOR_A);
  }
}

void turn2(bool direction, uint16_t speed, uint8_t offset)
{
  if(direction == COUNTER_CLOCKWISE)
  {
    motors.motorForward(MOTOR_A, speed+offset);   
    motors.motorForward(MOTOR_B, speed);
  }
  else
  {
    motors.motorForward(MOTOR_B, speed+offset);
    motors.motorForward(MOTOR_A, speed);
  }
}