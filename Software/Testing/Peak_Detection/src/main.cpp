
#include "Arduino.h"
#include <ESP32Servo.h>
#include "lrf.h"
#include "Wire.h"

Servo myservo; // create servo object to control a servo

int pos = 0; // variable to store the servo position
int servoPin = 2;

#define LRF_ADDRESS_1 0x10
#define BUFFER_SIZE 20

float avgDistance = 0;
uint16_t sumDistance = 0;
uint16_t cirBuffer[BUFFER_SIZE] = {0};
uint16_t bufferIndex = 0;
uint8_t fillBuffer = 0;
uint8_t restartBuffer = 0;
bool rotateTo180 = true;
uint16_t oldDistance;
bool positiveSlope = false;
uint8_t psCount = 0;
bool negativeSlope = false;

bool checkNs = true;
bool checkPs = true;

uint16_t inflectionPos;
uint8_t nsCount = 0;
// #define LRF_ADDRESS_2 0x20

void setup()
{
  Serial.begin(115200);
  Wire.begin(9, 8);
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50); // standard 50 hz servo
  myservo.attach(servoPin);   // using default min/max of 1000us and 2000us
}
int distance;

void loop()
{
  // while(1);
  if (rotateTo180)
  {
    pos += 2;
    if (pos == 180)
    {
      rotateTo180 = false;
    }
  }
  else
  {
    pos -= 2;
    if (pos == 0)
    {
      rotateTo180 = true;
    }
  }
  myservo.write((int)pos); // tell servo to go to position in variable 'pos'
  distance = getLrfDistance(LRF_ADDRESS_1);

  // for (; pos <= 180; pos += 2)
  // { // goes from 0 degrees to 180 degrees
  // in steps of 1 degree
  // myservo.write((int)pos); // tell servo to go to position in variable 'pos'
  if (distance >= 0 && distance < 400)
  {
    if (abs(distance - avgDistance) <= 50 || fillBuffer < BUFFER_SIZE)
    {
      sumDistance -= cirBuffer[bufferIndex];
      sumDistance += distance;
      avgDistance = sumDistance / BUFFER_SIZE;
      cirBuffer[bufferIndex] = distance;
      bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
      if (fillBuffer < BUFFER_SIZE)
      {
        fillBuffer++;
      }
      printf("%d %d\n", distance, pos);
      restartBuffer = 0;
      if (distance - oldDistance > 0 && checkPs)
      {
        positiveSlope = true;
        checkNs = false;
        psCount++;

        // if (!checkNs)
        // {
        //   inflectionPos = pos;
        // }

        if (psCount > 5)
        {
          checkPs = false;
          checkNs = true;
          psCount = 0;
        }
      }
      if (distance - oldDistance < 0 && checkNs)
      {
        nsCount++;
        negativeSlope = true;
        checkPs = false;

        //  if (!checkPs)
        //   {
        //     inflectionPos = pos;
        //   }

        if (nsCount > 5)
        {
          checkNs = false;
          checkPs = true;

          nsCount = 0;
        }
      }
      if (positiveSlope && negativeSlope)
      {
        // local max/min at current distance value
        // myservo.write(inflectionPos);
        // myservo.write(pos);
        delay(2000);

        positiveSlope = false;
        negativeSlope = false;
        checkNs = true;
        checkPs = true;
      }
      oldDistance = distance;
    }
    else
    {
      // if unable to get a reading for 40 times it will reset the buffer
      // Unsure if we want this or not
      restartBuffer++;
      if (restartBuffer >= 40)
      {
        fillBuffer = 0;
        restartBuffer = 0;
      }
    }
  }
}