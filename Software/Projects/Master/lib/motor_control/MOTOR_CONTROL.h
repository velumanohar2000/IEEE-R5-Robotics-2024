#include <Arduino.h>
#include <Wire.h>

#define LIGHT 3
#define MOTORA_IN_1 3
#define MOTORA_IN_2 2
#define MOTORB_IN_3 6
#define MOTORB_IN_4 7
#define BACK_MOTOR 0

#define TIMER_INTERVAL_MS 5000
#define FORWARD 1
#define BACKWARD 0
#define RIGHT 1
#define LEFT 0
#define STOP 0

void move(bool direction);
void move(bool direction, uint16_t speed);
void stop();
void stop(bool PWM);
void standby();
void turn(bool direction);
void turn(bool direction);
void initMotors();

