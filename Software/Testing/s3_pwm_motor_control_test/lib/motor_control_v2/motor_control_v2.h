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
#define MOTOR_A 0
#define MOTOR_B 1

#define FORWARD 1
#define BACKWARD 0
#define COUNTER_CLOCKWISE 1
#define CLOCKWISE 0

void move(bool direction);
void move(bool direction, uint16_t speed);
void stop();
void initMotors(uint8_t gpo_in1, uint8_t gpo_in2, uint8_t gpo_in3, uint8_t gpo_in4);
void turn(bool direction);
void turn(bool direction, uint16_t speed);
void turn(bool direction, uint16_t speed, uint16_t currentAngle, uint16_t requiredAngle);
void standby();

#endif