#include <Arduino.h>

#define A_IN_4 7
#define A_IN_3 6
#define A_IN_2 5
#define A_IN_1 4
#define B_IN_4 10
#define B_IN_3 1
#define B_IN_2 0
#define B_IN_1 2

void forward()
{
  digitalWrite(A_IN_1, 1);
  digitalWrite(A_IN_2, 0);
  digitalWrite(A_IN_3, 1);
  digitalWrite(A_IN_4, 0);
}

void reverse()
{
  digitalWrite(A_IN_1, 0);
  digitalWrite(A_IN_2, 1);
  digitalWrite(A_IN_3, 0);
  digitalWrite(A_IN_4, 1);
}

void stop()
{
  digitalWrite(A_IN_1, 0);
  digitalWrite(A_IN_2, 0);
  digitalWrite(A_IN_3, 0);
  digitalWrite(A_IN_4, 0);
}

void setup() {
  pinMode(A_IN_1, OUTPUT);
  pinMode(A_IN_2, OUTPUT);
  pinMode(A_IN_3, OUTPUT);
  pinMode(A_IN_4, OUTPUT);
}

void loop() {
  forward();
  delay(3000);
  stop();
  delay(1000);
  reverse();
  delay(3000);
  stop();
  delay(1000);
}
