#include "Arduino.h"
#include <ESP32Servo.h>
#include "lrf.h"
#include "Wire.h"
#include "map.h"
#include "MOTOR_CONTROL.h"
#include "VL53L1X.h"
#include "Ultrasonic.h"
// #define VELU
// #define MAIN
#define TURNS

Servo myservo; // create servo object to control a servo
SFEVL53L1X distanceSensor;

// 16 servo objects can be created on the ESP32

int pos = 0; // variable to store the servo position
// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33
int servoPin = 2;

#define LRF_ADDRESS_1 0x10
#define LRF_ADDRESS_2 0x20

// create map and buffer
struct Coordinate *local_maxes[4];
int local_max_index = 0;
Ultrasonic ultrasonic(0, 1);
int distanceUltrasonic = 100;
float distanceInUltrasonic;

void setup()
{
  // initMotors();
  initVL53L1X();
  #ifdef VELU
  // Serial.begin(115200);
  // Wire.begin(9,8);
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);          // standard 50 hz servo
  myservo.attach(servoPin, 500, 2400); // attaches the servo on pin 18 to the servo object
                                       // using default min/max of 1000us and 2000us
                                       // different servos may require different min/max settings
                                       // for an accurate 0 to 180 sweep
  #endif
}
int distance;
float distanceInches = 1000;

void loop(){
  #ifdef VELU
  for (pos = 0; pos <= 180; pos += 1)
  { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write((int)pos); // tell servo to go to position in variable 'pos'
    distance = getLrfDistance(LRF_ADDRESS_1);

    map_point(distance, pos);
    // printf("%d@%d\n", distance, pos);
    delay(15);                // waits 15ms for the servo to reach the position
  }
  printf("Local Maxes Clockwise: \n");
  local_max_index = 0;
  for(int i = 0; i < 4; i++){
    printf("X: %d, Y: %d\n", local_maxes[i]->x, local_maxes[i]->y);
  }

  delay(1000);
  for (pos = 180; pos >= 0; pos -= 1)
  {                          // goes from 180 degrees to 0 degrees
    myservo.write((int)pos); // tell servo to go to position in variable 'pos'
    distance = getLrfDistance(LRF_ADDRESS_1);
    map_point(distance, pos);
    // printf("%d@%d\n", distance, pos);
    delay(15);                // waits 15ms for the servo to reach the position
  }
  // printf("Local Maxes Counter Clockwise: \n");
  local_max_index = 0;
  for(int i = 0; i < 4; i++){
    printf("X: %d, Y: %d\n", local_maxes[i]->x, local_maxes[i]->y);
  }
  delay(1000);
  #endif

// move(FORWARD);
// delay(1000);
// move(BACKWARD);
// turn(LEFT);
// digitalWrite(MOTORA_IN_1, LOW);
//     digitalWrite(MOTORA_IN_2, HIGH);
//     digitalWrite(MOTORB_IN_3, LOW);
//     digitalWrite(MOTORB_IN_4, HIGH);
// delay(1000);
#ifdef MAIN
  // distanceUltrasonic = ultrasonic.read();
  // if(distanceUltrasonic > 25)
  // turn(LEFT);
  // else
  // turn(RIGHT);
  if(distanceInches > 10.0)
  {
    distanceUltrasonic = ultrasonic.read();
    if(distanceUltrasonic > 12)
    {
      // while(distanceInUltrasonic > 6)
      {
        turn(LEFT, 128);
        delay(25);
        move(FORWARD, 128);
        delay(25);
        turn(RIGHT, 128);
        delay(25);
        move(FORWARD, 128);
        delay(25);
        distanceUltrasonic = ultrasonic.read();
      }
    }
    else if(distanceUltrasonic < 8)
    {
      // while(distanceInUltrasonic < 5)
      {
        turn(RIGHT, 128);
        delay(25);
        move(FORWARD, 128);
        delay(25);
        turn(LEFT, 128);
        delay(25);
        move(FORWARD, 128);
        delay(25);
        distanceUltrasonic = ultrasonic.read();
      }
    }
    else
      move(FORWARD, 128);
  }
  else
    stop();
  
  // distanceInches = getDistanceVL53L1X(distanceSensor, UNIT_INCHES);

  distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
  while (!distanceSensor.checkForDataReady())
  {
    delay(1);
  }
  distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
  distanceSensor.clearInterrupt();
  distanceSensor.stopRanging();
  distanceInches = distance * 0.0393701;
  Serial.printf("Distance (in): %f\n", distanceInches);
#endif

#ifdef TURNS
  turn(RIGHT, 128);
        // delay(500);
        // move(FORWARD, 128);
        // delay(500);
        // turn(LEFT, 128);
        // delay(500);
        // move(FORWARD, 128);
        // delay(500);
#endif
}
