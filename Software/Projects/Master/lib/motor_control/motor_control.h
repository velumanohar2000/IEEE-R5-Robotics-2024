/*
 * Bit Bangers Motor Control Library
 * github.com/Bit-Bangers-UTA/Senior-Design
 *
 * Authors:
 * Noor Abdullah
 * Rolando Rosales
*/

#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#include <Arduino.h>

#define FORWARD 1
#define BACKWARD 0
#define RIGHT 1
#define LEFT 0
#define FULL_SPEED 255
#define NO_SPEED 0

void move(bool direction);
void move(bool direction, uint16_t speed);
void stop();
void initMotors(uint8_t gpo_in1, uint8_t gpo_in2, uint8_t gpo_in3, uint8_t gpo_in4);
void turn(bool direction);
void turn(bool direction, uint16_t speed);
void turn(bool direction, uint16_t speed, uint16_t currentAngle, uint16_t requiredAngle);

#endif