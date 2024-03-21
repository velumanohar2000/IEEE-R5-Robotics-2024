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
// #include "SafeString.h"
#include "millisDelay.h"

// Defines --------------------------------------------------------------------

// Modes

// Preprocessor Directives
// #define PRELIMS
// #define PRELIMS_DRAFT
// #define PRELIMS_DRAFT_V2
// #define CHECK_I2C
// #define VELU
// #define TURNS
// #define TURNS_2
// #define TURNS_3
#define TURNS_4
// #define INTERRUPTS_CHECK

// Motors
#define MOTORA_IN_1 7
#define MOTORA_IN_2 6
#define MOTORB_IN_3 2
#define MOTORB_IN_4 3

//IMU
#define IMU_INTERRUPT 10

// Whisker
#define WHISKER_STOP_DIS 8
#define MAX_PRELIM_DIST 60

// LRF
#define LRF_ADDRESS_1 0x10
#define LRF_ADDRESS_2 0x20

// Variables & Constants ------------------------------------------------------

/*
counter = 1
++
360 * counter
*/

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
float test = 45.0;
float nextAngle = 90.0;
uint16_t speed = 128;
// #define TIRE_DELAY 100
// millisDelay tireDelay;
// float angleDiff = 0;
// float startingangle = 0;
uint32_t m = 0;
// Structures & Classes -------------------------------------------------------

// Servo
Servo myservo; // create servo object to control a servo
SFEVL53L1X distanceSensor;
float dt = 0;
float millisOld = 0;
unsigned long lastTime, currentTime = 0;
float x, y, z = 0;
float thetaZ = 0;
// float angle = 0;
uint32_t timeDifference = 0;

// 16 servo objects can be created on the ESP32

// Ultrasonic Sensor
Ultrasonic ultrasonic(0, 1);

// IMU
Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

// Functions ------------------------------------------------------------------
// void IRAM_ATTR imuIsr() {
//   // newDataFlag = true;
//   Serial.println("triggered");
//   if (bno08x.getSensorEvent(&sensorValue))
//   {
//     switch (sensorValue.sensorId)
//     {
//       case SH2_GYROSCOPE_CALIBRATED:
//       {
//         z = sensorValue.un.gyroscope.z;
//         break;
//       }
//       default:
//         break;
//     }
//   currentTime = micros();
//   dt = (currentTime - lastTime - 0)/1000000.0;
//   lastTime = currentTime;
//   thetaZ += z*dt;
//   angle = thetaZ*(180/M_PI);
//   if(angle > 360)
//     angle -= 360;
//   else if(angle < 0)
//     angle += 360;
//   }
// }

float getHeading()
{

  float angle = 0;
  float retVal = -1;
  float z = 0;
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
  {
    switch (sensorValue.sensorId)
    {
      case SH2_GYROSCOPE_CALIBRATED:
      {
        z = sensorValue.un.gyroscope.z;
        break;
      }
      default:
        break;
    }
  currentTime = micros();
  dt = (currentTime - lastTime)/1000000.0;
  lastTime = currentTime;
  thetaZ += z*dt;
  angle = thetaZ*(180/M_PI);

  Serial.print(angle);
  if(angle >= 359)
    angle -= 359;
  else if(angle < 0)
    angle += 359;
  Serial.printf("\t");
  Serial.println(angle);
  // if(angle == 0);
    // angle = 359;
  retVal = angle;
  }
  return retVal;

}

void turnToGoalHeading(float goal)
{
  float currentAngle = -1;
  if(goal == 0)
    goal = 359;
  int32_t counter = 0;
  int32_t speed = 100;
  while(currentAngle == -1)
    currentAngle = getHeading();
  float absVal = 0;
  float angleDiff = goal - currentAngle;
  absVal = abs(angleDiff);

  while(absVal > 8)
  {
    Serial.print("abs: ");
    Serial.print(absVal);
    Serial.print(" angle: ");
    Serial.print(currentAngle);

    if((angleDiff >= 0) && (absVal <= 180))
    {
      Serial.print(" case 1: ");
      // Serial.printf("Speed = %f\n", speed + (-1 * counter));
      turn(LEFT, speed);
    }
    else if((angleDiff < 0) && (absVal <= 180))
    {
      Serial.print(" case 2: ");
      // Serial.printf("Speed = %f\n", speed + (-1 * counter));
      turn(RIGHT, speed);
    }
    else if((angleDiff >= 0) && (absVal >= 180))
    {
      Serial.print(" case 3: ");
      // Serial.printf("Speed = %f\n", speed + (-1 * counter));
      turn(RIGHT, speed);
    }
    else
    {
      Serial.print(" case 4: ");
      // Serial.printf("Speed = %f\n", speed + (-1 * counter));
      turn(LEFT, speed);
    }
    counter++;
    if(counter > 12)
      counter = 5;
    currentAngle = getHeading();
    if(currentAngle > 360)
      currentAngle -= 360;
    if(currentAngle < 0)
      currentAngle += 360;
    Serial.println(goal);
    absVal = 0;
    angleDiff = goal - currentAngle;
    absVal = abs(angleDiff);
  }

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
  // pinMode(IMU_INTERRUPT, INPUT);
  // attachInterrupt(digitalPinToInterrupt(IMU_INTERRUPT), imuIsr, FALLING);
  Serial.println("Adafruit BNO08x test!");
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
float currentAngle = -1;
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
  // while((firstHeading == -1.0) && (!northEstablished))
  // {
  //   firstHeading = getHeading();
  //   // delay(100);
  //   if(firstHeading != -1.0)
  //   {
  //     northEstablished = true;
  //     Serial.print("heading is: ");
  //     Serial.println(firstHeading);
  //   }
  // }
  if(!wallFound)
  {
    currentAngle = getHeading();
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
        tireDelay.start(TIRE_DELAY);
        while(!tireDelay.justFinished())
          currentAngle = getHeading();
        // delay(100);
        move(FORWARD, 128);
        tireDelay.start(TIRE_DELAY);
        while(!tireDelay.justFinished())
         currentAngle = getHeading();
        // delay(100);
      }
      else if(ultraDistance < 5)
      {
        turn(RIGHT, 128);
        tireDelay.start(TIRE_DELAY);
        while(!tireDelay.justFinished())
          currentAngle = getHeading();
        // delay(100);
        
        move(FORWARD, 128);
        tireDelay.start(TIRE_DELAY);
        while(!tireDelay.justFinished())
          currentAngle = getHeading();
        // delay(25);
      }
      else
        move(FORWARD, 128);
    }
    else
    {
      stop();
      delay(50);
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
    // currentAngle = getHeading();

      // Serial.print("currentAngle is: ");
      // Serial.println(currentAngle);
    
    // if(currentAngle > 360)
    //   currentAngle -= 360;
    while((currentAngle > (nextAngle + 1)) || (currentAngle < (nextAngle - 1)) )
    {
      currentAngle = getHeading();
      Serial.print("pointing at: ");
      Serial.println(currentAngle);
      // delay(100);
      turn(RIGHT, 90);
    }
    stop();
    // static uint32_t i = 0;
    // for(i = 0; i < 16*1000000; i++)
    //   asm volatile ("nop\n\t");
    tireDelay.start(TIRE_DELAY);
    while(!tireDelay.justFinished())
      currentAngle = getHeading();
    // delay(100);
    // timeDifference = 1000 * 1000;
    getWhiskerDistance();
    wallFound = false;
  }
  

#endif

#ifdef PRELIMS_DRAFT_V2
if(!wallFound)
  {
    currentAngle = getHeading();
    Serial.println("1");
    if(whiskDistanceInch > WHISKER_STOP_DIS)
    {
      ultraDistance = ultrasonic.read();
      if(ultraDistance > MAX_PRELIM_DIST)
      {

    Serial.println("2");
        currentAngle = getHeading();
        angleDiff = currentAngle - startingangle;
        stop();
      }
      else if(/*ultraDistance > 8 ||*/ (angleDiff > 1))
      {

    Serial.println("3");
        angleDiff = currentAngle - startingangle;
        while(currentAngle < 1)
        {

    Serial.println("4");
          turn(LEFT, 128);
          currentAngle = getHeading();
          angleDiff = currentAngle - startingangle;
        }
          move(FORWARD, 128);
      }
      else if(/*ultraDistance < 5 ||*/ (angleDiff < 1))
      {

    Serial.println("5");
        angleDiff = startingangle - currentAngle;
        while(currentAngle > 1)
        {

    Serial.println("6");
          turn(RIGHT, 128);        
          currentAngle = getHeading();
          angleDiff = startingangle - currentAngle;
        }
        move(FORWARD, 128);
      }
      else
      {
        move(FORWARD, 128);

    Serial.println("7");

      }
    }
    else
    {
      stop();
      for(m = 0; m < 5000; m++)
        if((m%100) == 0)
          currentAngle = getHeading();
      // delay(1000);
      wallFound = true;
      startingangle += 90;
      nextAngle += 90.0;
      if(nextAngle > 360)
        nextAngle -= 360;
      if(startingangle > 360)
        startingangle -= 360;
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
    // currentAngle = getHeading();

      // Serial.print("currentAngle is: ");
      // Serial.println(currentAngle);
    
    // if(currentAngle > 360)
    //   currentAngle -= 360;
    while((currentAngle > (nextAngle + 1)) || (currentAngle < (nextAngle - 1)) )
    {
      currentAngle = getHeading();
      Serial.print("pointing at: ");
      Serial.println(currentAngle);
      // delay(100);
      turn(RIGHT, 90);
    }
    stop();
    for(m = 0; m < 5000; m++)
        if((m%100) == 0)
          currentAngle = getHeading();
    // static uint32_t i = 0;
    // for(i = 0; i < 16*1000000; i++)
    //   asm volatile ("nop\n\t");
    // delay(100);
    // timeDifference = 1000 * 1000;
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
  while((floor(heading) >= (test+1)) || (floor(heading) <= (test - 1)) )
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
      turn(RIGHT, 90);
      // delay(50);
      // stop();
      // delay(5);

  }
  stop();
  delay(500);
  Serial.print(" current: ");
          Serial.print(currentTime);
          Serial.print("\tlast: ");
          Serial.print(lastTime);
          Serial.print("\tdiff: ");
          Serial.print(timeDifference);
  timeDifference = 500*1000;
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
  else if(test == 360)
    test = 0;




  
#endif

#ifdef TURNS_2

  currentAngle = getHeading();
  angleDiff = nextAngle - currentAngle;
  if(abs(angleDiff) > 2)
  {
    if(angleDiff > 2)
      while(angleDiff < nextAngle)
      {
        Serial.print("Current ");
        Serial.print(currentAngle);
        Serial.print("\tNext ");
        Serial.print(nextAngle);
        Serial.print("\tdiff ");
        Serial.println(angleDiff);
        turn(RIGHT, 128);
        currentAngle = getHeading();
        angleDiff = nextAngle - currentAngle;
      }
    else
      while(angleDiff < nextAngle)
        {
          Serial.print("Current ");
          Serial.print(currentAngle);
          Serial.print("\tNext ");
          Serial.print(nextAngle);
          Serial.print("\tdiff ");
          Serial.println(nextAngle);
          turn(LEFT, 128);
          currentAngle = getHeading();
          angleDiff = nextAngle - currentAngle;
        }

  }
  else
  {
    Serial.println("stop");
    stop();
    currentAngle = getHeading();
    // while(1);
    if(nextAngle == 90)
      nextAngle = 0;
      else
      nextAngle = 90;

  }
  // while((floor(heading) >= (test + 3)) || (floor(heading) <= (test - 3)) )
  // while(1)
  // {
  //   heading = getHeading();


  // }
  //   // if((test == 0 ) || (test == 360 ))
  //   // {

  //   // }
  //   if(((test+2) >= heading) && ((test-2) <= heading))
  //   {
  //     // speed = 128;
  //     stop();
  //     test += 45;
  //     if(test > 360)
  //       test -=360;
  //       delay(500);

  //   }
  //   else if((test) <= heading)
  //   {
  //     turn(RIGHT, speed);
  //     // speed -= 10;
  //   }
  //   else
  //   {
  //     turn(LEFT, speed);
  //     // speed -= 10;
  //   }
    
#endif

#ifdef INTERRUPTS_CHECK
  
#endif

#ifdef TURNS_3

  Serial.println("Start");
  turnToGoalHeading(180);
  Serial.print("Current Angle");
  Serial.println(getHeading());
  turnToGoalHeading(359);
  Serial.print("Current Angle");
  Serial.println(getHeading());
  turnToGoalHeading(180);
  Serial.print("Current Angle");
  Serial.println(getHeading());
  turnToGoalHeading(359);
  Serial.print("Current Angle");
  Serial.println(getHeading());
  // turnToGoalHeading(0);
  // Serial.print("Current Angle");
  // Serial.println(getHeading());
  // turnToGoalHeading(270);
  // Serial.print("Current Angle");
  // Serial.println(getHeading());
    stop();
    Serial.print("Stopped");
  while(1)
  {
  }
#endif

#ifdef TURNS_4
  float angle1 = getHeading();

#endif
}
