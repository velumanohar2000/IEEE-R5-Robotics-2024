#include "Arduino.h"
#include <ESP32Servo.h>
#include "lrf.h"
#include "Wire.h"
#include "map.h"
#include "MOTOR_CONTROL.h"
#include "VL53L1X.h"
#include "Ultrasonic.h"
// #define VELU
#define MAIN
// #define TURNS

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
int distanceUltrasonic = 100;         // ultrasonic
float distanceInUltrasonic;           // ultrasonic

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
int distance;                           // whisker
float distanceInches = 1000;            // whisker

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

#ifdef MAIN
  /*
    same main if else condition, if distance on whiskers read less than 10 inches then stop if not keep going
  */
  if(distanceInches > 10.0)
  {
    /*
      distanceUltrasonic is in centimeters, maybe i should use inches instead for testing
    */
    distanceUltrasonic = ultrasonic.read();

    /*
      Idea behind logic below is that we want to always be going forward until forward whisker tells us to stop.
      Also we need to be going in a straight line always so we don't run into the wall or the post

      So if we're too far away from the wall
        i)    we would need to turn into it slightly.
        ii)   go forward slightly in the new direction.
        iii)  turn away for the same amount of time as step 1 to straighten out the car so we don't run into the side wall

      else if we're too close to the wall
        i)    we would need to turn away from it slightly.
        ii)   go forward slightly in the new direction.
        iii)  turn towards the wall for the same amount of time as step 1 to straighten out the car so we don't run into the post

    This is obviously a draft algo (doesn't work either lol) so can be changed if needed, most of the time i was trying to get the whisker
    and the US to work together so i came up with this algo at the very end.

    Also I had wired the GPIO for motor control backwards so for the if statement, if we need to go towards the wall we need to be "turning right"
    since my gpio was backwards i had to turn left. but again, draft code, will be changed.
    */
    if(distanceUltrasonic > 12)
    {
      // while(distanceInUltrasonic > 6)
      {
        turn(LEFT);
        delay(25);
        move(FORWARD, 128);
        delay(25);
        turn(RIGHT);
        delay(25);
        move(FORWARD, 128);
        delay(25);
        // distanceUltrasonic = ultrasonic.read();
      }
    }
    else if(distanceUltrasonic < 8)
    {
      // while(distanceInUltrasonic < 5)
      {
        turn(RIGHT);
        delay(25);
        move(FORWARD, 128);
        delay(25);
        turn(LEFT);
        delay(25);
        move(FORWARD, 128);
        delay(25);
        // distanceUltrasonic = ultrasonic.read();
      }
    }
    else
      move(FORWARD, 128);
  }
  else
    stop();
  
  /*
    This fn suddenly stopped working so I had to copy the code instead of calling the fn from lib
  */

  // distanceInches = getDistanceVL53L1X(distanceSensor, UNIT_INCHES);

  /*
    same code for getting data for whisker
  */
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
/*
  also added code to turn using PWM but doesn't work with the test code below
*/

  // turn(RIGHT, 128);
  // delay(500);
  // move(FORWARD, 128);
  // delay(500);
  // turn(LEFT, 128);
  // delay(500);
  // move(FORWARD, 128);
  // delay(500);
#endif
}
