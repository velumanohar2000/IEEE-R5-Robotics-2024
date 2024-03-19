/*
 * Bit Bangers Master Robot
 * github.com/Bit-Bangers-UTA/Senior-Design
*/

// Libraries ------------------------------------------------------------------

#include "Arduino.h"
#include <stdbool.h>
#include <ESP32Servo.h>
#include "lrf.h"
#include "Wire.h"
#include "map.h"
#include "motor_control.h"
#include "VL53L1X.h"
#include "Ultrasonic.h"
#include <Adafruit_BNO08x.h>
#include "math.h"
#include "BNO085_heading_acceleration.h"

// Defines --------------------------------------------------------------------

// Modes

// Preprocessor Directives
// #define PRELIMS
#define PRELIMS_DRAFT
// #define CHECK_I2C
// #define VELU
// #define TURNS
// #define TURNS_2

// Motors
#define MOTORA_IN_1 3
#define MOTORA_IN_2 2
#define MOTORB_IN_3 6
#define MOTORB_IN_4 7

// Whisker
#define WHISKER_STOP_DIS 5
#define MAX_PRELIM_DIST 60

// LRF
#define LRF_ADDRESS_1 0x10
#define LRF_ADDRESS_2 0x20

// Variables & Constants ------------------------------------------------------

// Servo
int pos = 0; // variable to store the servo position
// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33
int servoPin = 2;

// Map
// create map and buffer
struct Coordinate *local_maxes[4];
int local_max_index = 0;

// Ultrasonic Sensor
int ultraDistance = 100;         // ultrasonic
float ultraDistanceInch;           // ultrasonic

// Whisker
int whiskDistance;                           // whisker
float whiskDistanceInch = 1000;            // whisker
bool wallFound = false;

// IMU
float heading = -1.0;             // IMU
float test = 45.0;
float firstHeading = -1.0;
bool northEstablished = false;
float nextAngle = 0.0;
float currentAngle = -1;
uint16_t speed = 128;
// Structures & Classes -------------------------------------------------------

// Servo
Servo myservo; // create servo object to control a servo
SFEVL53L1X distanceSensor;

// 16 servo objects can be created on the ESP32

// Ultrasonic Sensor
Ultrasonic ultrasonic(0, 1);

// IMU
Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

// Functions ------------------------------------------------------------------

float getHeading()
{
  float retVal = -1;
  // if (bno08x.getSensorEvent(&sensorValue))
  // {
  //   switch (sensorValue.sensorId)
  //   {
  //   case SH2_LINEAR_ACCELERATION:
  //   {
  //     retVal = sensorValue.un.linearAcceleration.x;
  //     retVal = -1.0;
  //     break;
  //   }
  //   case SH2_ARVR_STABILIZED_RV:
  //   {
  //     retVal = calculateHeading(sensorValue.un.arvrStabilizedRV.i, sensorValue.un.arvrStabilizedRV.j, sensorValue.un.arvrStabilizedRV.k, sensorValue.un.arvrStabilizedRV.real);
  //     break;
  //   }
  //   }
  // }
  if (bno08x.getSensorEvent(&sensorValue))
    retVal = calculateHeading(sensorValue.un.arvrStabilizedRV.i, sensorValue.un.arvrStabilizedRV.j, sensorValue.un.arvrStabilizedRV.k, sensorValue.un.arvrStabilizedRV.real);
  //     
  return retVal;
}

void getWhiskerDistance()
{
  distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
  while (!distanceSensor.checkForDataReady())
  {
    delay(1);
  }
  whiskDistance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
  distanceSensor.clearInterrupt();
  distanceSensor.stopRanging();
  whiskDistanceInch = whiskDistance * 0.0393701;
  // Serial.printf("Whisker dis (in): %f\n", whiskDistanceInch);
}

void setup()
{
  Serial.begin(115200);
  Wire.begin(9, 8);
  // Serial.println("Adafruit BNO08x test!");
  // Wire.begin(9, 8);
  initMotors(MOTORA_IN_1, MOTORA_IN_2, MOTORB_IN_3, MOTORB_IN_4);
  initVL53L1X();
  setupBNO085(&bno08x);
  // northEstablished = false;
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

#ifdef PRELIMS
  /*
    same main if else condition, if distance on whiskers read less than 10 inches then stop if not keep going
  */
  if(whiskDistanceInch > WHISKER_STOP_DIS)
  {
    /*
      distanceUltrasonic is in centimeters, maybe i should use inches instead for testing
    */
    ultraDistance = ultrasonic.read();

    Serial.printf("Ultras dis (cm): %d\n", ultraDistance);

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
    // if the Ultrasonic distance is longer than game field, just stop
    if(ultraDistance > MAX_PRELIM_DIST)
    {
      stop();
    }
    else if(ultraDistance > 8)
    {
      turn(LEFT, 128);
      delay(100);
      move(FORWARD, 128);
      delay(100);

      /*
      turn(RIGHT);
      delay(25);
      move(FORWARD, 128);
      delay(25);
      // distanceUltrasonic = ultrasonic.read();
      d*/
    }
    else if(ultraDistance < 5)
    {
      turn(RIGHT, 128);
      delay(100);
      
      move(FORWARD, 128);
      delay(25);

      /*
      turn(LEFT);
      delay(25);
      move(FORWARD, 128);
      delay(25);
      // distanceUltrasonic = ultrasonic.read();
      */
    }
    else
      move(FORWARD, 128);
  }
  else
  {
    stop();
  }
  
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
  whiskDistance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
  distanceSensor.clearInterrupt();
  distanceSensor.stopRanging();
  whiskDistanceInch = whiskDistance * 0.0393701;
  Serial.printf("Whisker dis (in): %f\n", whiskDistanceInch);
#endif

#ifdef PRELIMS_DRAFT
  while((firstHeading == -1.0) && (!northEstablished))
  {
    firstHeading = getHeading();
    // delay(100);
    if(firstHeading != -1.0)
    {
      northEstablished = true;
      Serial.print("heading is: ");
      Serial.println(firstHeading);
    }
  }
  if(!wallFound)
  {
    if(whiskDistanceInch > WHISKER_STOP_DIS)
    {
      ultraDistance = ultrasonic.read();
      if(ultraDistance > MAX_PRELIM_DIST)
      {
        stop();
      }
      else if(ultraDistance > 8)
      {
        turn(LEFT, 128);
        delay(100);
        move(FORWARD, 128);
        delay(100);
      }
      else if(ultraDistance < 5)
      {
        turn(RIGHT, 128);
        delay(100);
        
        move(FORWARD, 128);
        delay(25);
      }
      else
        move(FORWARD, 128);
    }
    else
    {
      stop();
      delay(1000);
      wallFound = true;
      nextAngle += 90.0;
      if(nextAngle > 360)
        nextAngle -= 360;
        Serial.print("nextAngle is: ");
      Serial.println(nextAngle);
    }
    getWhiskerDistance();
  }
  else
  {
    while(currentAngle == -1)
    {
      currentAngle = getHeading();
      // delay(100);
    }
    currentAngle = (currentAngle + firstHeading);

      Serial.print("currentAngle is: ");
      Serial.println(currentAngle);
    
    if(currentAngle > 360)
      currentAngle -= 360;
    while((currentAngle > (nextAngle + 10)) || (currentAngle < (nextAngle - 10)) )
    {
      currentAngle = abs(getHeading() - firstHeading);
      Serial.print("pointing at: ");
      Serial.println(currentAngle);
      delay(100);
      turn(RIGHT, 115);
    }
    stop();
    delay(1000);
    getWhiskerDistance();
    wallFound = false;
  }
  

#endif

#ifdef CHECK_I2C

  // Serial.println("begin: ");
  // if(northEstablished)
  // {
  //   Serial.println("true");
  // }
  // else
  //   Serial.println("false");

  // delay(5000);
  // if(northEstablished)
  // {
  //   Serial.print("Heading established: ");
  //   Serial.println(heading);
  // }
  // else
  // {
  //   Serial.println("no: ");
  //   firstHeading = getHeading();
  //   delay(100);
  //   Serial.println(firstHeading);
  //   if(firstHeading != -1.0)
  //     northEstablished = true;
  // }
  //   Serial.println("Heading not established");
  // else
  // {
  //   Serial.print("Heading established: ");
  //   Serial.println(heading);
  // }
  // while((firstHeading == -1.0) && (northEstablished == false))
  //   {
  //     // delay(5000);
  //     firstHeading = getHeading();
  //     delay(100);
  //     if(firstHeading != -1.0)
  //     {
  //       northEstablished = true;
  //       Serial.printf(" Inside First heading = %0.2f\n",firstHeading);
  //     }
  //     else
  //     {
  //       Serial.println("no");
  //     }
  //   }
  getWhiskerDistance();
  Serial.printf("Whisker: ");
  Serial.print(whiskDistanceInch);
  delay(10);
  heading = getHeading();
  if(heading != -1)
  {
  Serial.printf("\n\t\tIMU: ");
  Serial.println(heading);

  }
  else
  {
    Serial.println();
  }
  delay(100);

#endif

#ifdef TURNS
/*
  also added code to turn using PWM but doesn't work with the test code below
*/
  while((floor(heading) >= (test + 10)) || (floor(heading) <= (test - 10)) )
  {

          // Serial.print("cjec");
    // if (bno08x.getSensorEvent(&sensorValue))
      {
        // switch (sensorValue.sensorId)
        // {
        // case SH2_LINEAR_ACCELERATION:
        // {
        //   // TODO adjust reports function
        //   // Serial.print("Acceleration (x): ");
        //   float xAccel = sensorValue.un.linearAcceleration.x;
        //   Serial.println(xAccel);
        //   break;
        // }
        // case SH2_ARVR_STABILIZED_RV:
        {
          heading = getHeading(); //calculateHeading(sensorValue.un.arvrStabilizedRV.i, sensorValue.un.arvrStabilizedRV.j, sensorValue.un.arvrStabilizedRV.k, sensorValue.un.arvrStabilizedRV.real);
          Serial.print("Heading: ");
          Serial.print(heading);
          Serial.print("\ttest: ");
          Serial.println(test);
          // break;
        }
        // }
      }
      turn(LEFT, 120);
      delay(100);

  }
  stop();
  delay(500);
  // move(FORWARD, 100);
  // delay(500);
  // standby();
  // delay(100);
  // move(BACKWARD, 100);
  // delay(500);
  // standby();
  // delay(100);
  test += 45;
  if(test > 360)
    test -=360;




  
#endif

#ifdef TURNS_2
  // while((floor(heading) >= (test + 3)) || (floor(heading) <= (test - 3)) )
  while(1)
  {
    heading = getHeading();


  }
    // if((test == 0 ) || (test == 360 ))
    // {

    // }
    if(((test+2) >= heading) && ((test-2) <= heading))
    {
      // speed = 128;
      stop();
      test += 45;
      if(test > 360)
        test -=360;
        delay(500);

    }
    else if((test) <= heading)
    {
      turn(RIGHT, speed);
      // speed -= 10;
    }
    else
    {
      turn(LEFT, speed);
      // speed -= 10;
    }
    
#endif
}


/*
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include "math.h"
#include "BNO085_heading_acceleration.h"

#define BNO08X_RESET -1

Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

void setup()
{
  setupBNO085(&bno08x);
}

float fakeNorth = 0;;
bool firstReading = true;

void loop()
{
  checkReset(&bno08x);

  if (bno08x.getSensorEvent(&sensorValue))
  {
    switch (sensorValue.sensorId)
    {
    case SH2_LINEAR_ACCELERATION:
    {
      //TODO adjust reports function
      Serial.print("Acceleration (x): ");
      float xAccel = sensorValue.un.linearAcceleration.x;
      Serial.println(xAccel);
      break;
    }
    case SH2_ARVR_STABILIZED_RV:
    {
      float heading = calculateHeading(sensorValue.un.arvrStabilizedRV.i, sensorValue.un.arvrStabilizedRV.j, sensorValue.un.arvrStabilizedRV.k, sensorValue.un.arvrStabilizedRV.real);
      heading -= fakeNorth;
      if(heading < 0)
        heading += 360;
      if(firstReading)
      {
        fakeNorth = heading;
        firstReading = false;
      }
      Serial.print("Heading: ");
      Serial.println(heading);
      break;
    }
    }
  }
  delay(100);
}

*/