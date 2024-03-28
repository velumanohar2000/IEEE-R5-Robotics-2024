
#include "Arduino.h"
#include <Servo.h>
#include "Wire.h"

#include "lrf.h"
#define LRF_ADDRESS_1 0x10
#define LRF_ADDRESS_2 0x20
Servo myservo; // create servo object to control a servo
// 16 servo objects can be created on the ESP32

int pos = 0; // variable to store the servo position

int servoPin = 2;
int distance;



void setup()
{
  Serial.begin(115200);
  Wire.begin(9, 8);
  // Allow allocation of all timers
  myservo.attach(servoPin); // using default min/max of 1000us and 2000us
}

void loop()
{
  for (pos = 0; pos <= 180; pos += 2)
  { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(servoPin, pos); // tell servo to go to position in variable 'pos'
    distance = getLrfDistance(LRF_ADDRESS_1);
    printf("LRF 1 (0x10): %d@%d\n", distance, pos);
    distance = getLrfDistance(LRF_ADDRESS_2);
    printf("LRF 2 (0x20): %d@%d\n", distance, pos);
   }
  delay(2000);
  myservo.write(servoPin, 0); // tell servo to go to position in variable 'pos'

  delay(1000);
}