/*
 * Bit Bangers Motor Control Library
 * github.com/Bit-Bangers-UTA/Senior-Design
 *
 * Authors:
 * Noor Abdullah
 * Rolando Rosales
 * 
 * Works with any dual-motor driver that has two PWM inputs per motor
 * Tested with MX1508
*/

// Libraries ------------------------------------------------------------------

#include <Arduino.h>

// Defines --------------------------------------------------------------------

#define FORWARD 1
#define BACKWARD 0
#define RIGHT 1
#define LEFT 0
#define FULL_SPEED 255
#define NO_SPEED 0

// Variables & Constants ------------------------------------------------------

uint8_t in1, in2, in3, in4;
bool direction = true;

// Structures & Classes -------------------------------------------------------

// Functions ------------------------------------------------------------------

void move(bool direction)
{
  if(direction == FORWARD)
  {
    analogWrite(in1, FULL_SPEED);
    analogWrite(in2, NO_SPEED);
    analogWrite(in3, FULL_SPEED);
    analogWrite(in4, NO_SPEED);
  }
  else
  {
    analogWrite(in1, NO_SPEED);
    analogWrite(in2, FULL_SPEED);
    analogWrite(in3, NO_SPEED);
    analogWrite(in4, FULL_SPEED);
  }
}

void move(bool direction, uint16_t speed)
{
  if(direction == FORWARD)
  {
    analogWrite(in1, speed);
    analogWrite(in2, NO_SPEED);
    analogWrite(in3, speed);
    analogWrite(in4, NO_SPEED);

  }
  else
  {
    analogWrite(in1, NO_SPEED);
    analogWrite(in2, speed);
    analogWrite(in3, NO_SPEED);
    analogWrite(in4, speed);
  }
}

void stop()
{
  analogWrite(in1, FULL_SPEED);
  analogWrite(in2, FULL_SPEED);
  analogWrite(in3, FULL_SPEED);
  analogWrite(in4, FULL_SPEED);
}

void standby()
{
  analogWrite(in1, NO_SPEED);
  analogWrite(in2, NO_SPEED);
  analogWrite(in3, NO_SPEED);
  analogWrite(in4, NO_SPEED);
}


void initMotors(uint8_t gpo_in1, uint8_t gpo_in2, uint8_t gpo_in3, uint8_t gpo_in4) {
  in1 = gpo_in1;
  in2 = gpo_in2;
  in3 = gpo_in3;
  in4 = gpo_in4;

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
}

void turn(bool direction)
{
  if(direction == RIGHT)
  {
    analogWrite(in1, FULL_SPEED);
    analogWrite(in2, NO_SPEED);
    analogWrite(in3, NO_SPEED);
    analogWrite(in4, FULL_SPEED);
  }
  else
  {
    analogWrite(in1, NO_SPEED);
    analogWrite(in2, FULL_SPEED);
    analogWrite(in3, FULL_SPEED);
    analogWrite(in4, NO_SPEED);
  }
}

/*
  basic idea for PWM, instead of going full speed, provide a param to control speed
*/

void turn(bool direction, uint16_t speed)
{
  if(direction == RIGHT)
  {
    analogWrite(in1, speed);
    analogWrite(in2, NO_SPEED);
    analogWrite(in3, NO_SPEED);
    analogWrite(in4, speed);
  }
  else
  {
    analogWrite(in1, NO_SPEED);
    analogWrite(in2, speed);
    analogWrite(in3, speed);
    analogWrite(in4, NO_SPEED);
  }
}

void turn(bool direction, uint16_t speed, uint16_t currentAngle, uint16_t requiredAngle)
{
  /*
  coded this without testing. Idea behind is that when were turning, we will never get the exact angle unless we use interrupts
  so for now, i multiplied the angles with a large value so we have some room for error. We keep turning as long as we're within that error range.

  After coding this i realized that we need interrupts since having slight error in angle would completely change the vector.
  If we're planning on using the same while code then we need to turn very slowly
  */
  uint64_t currentAngleError = currentAngle * 1000;
  uint64_t requiredAngleError = requiredAngle * 1000;
  while((currentAngleError > (requiredAngleError+50)) ||  (currentAngleError < (requiredAngleError-50)))
  {
    if(direction == RIGHT)
    {
      analogWrite(in1, speed);
      analogWrite(in2, NO_SPEED);
      analogWrite(in3, NO_SPEED);
      analogWrite(in4, speed);
    }
    else
    {
      analogWrite(in1, NO_SPEED);
      analogWrite(in2, speed);
      analogWrite(in3, speed);
      analogWrite(in4, NO_SPEED);
    }
  }
}