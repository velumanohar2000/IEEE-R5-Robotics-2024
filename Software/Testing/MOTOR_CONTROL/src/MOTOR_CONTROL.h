#include <Arduino.h>
#include <Wire.h>

void move(bool direction);
void move(bool direction, uint16_t speed);
void stop();
void stop(bool PWM);
void standby();
void turn(bool direction);
void turn(bool direction);
void initMotors();

