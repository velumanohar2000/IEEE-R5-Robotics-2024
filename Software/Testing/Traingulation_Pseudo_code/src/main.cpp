#include <Arduino.h>
#include <stdbool.h>
#include <Adafruit_BNO08x.h>
#include <Wire.h>
#include <math.h>
#include <ESP32MotorControl.h>
#include <LiquidCrystal_I2C.h>
#include <string.h>
#include <Servo.h>

#include "motor_control_v2.h"
#include "BNO085_heading_.h"
#include "lrf.h"

#define LRF_ADDRESS_1 0x10
#define LRF_ADDRESS_2 0x20

#define ELIMINATION
// #define TEST
// #define ANGLE
#define MOTOR_IN_1  7
#define MOTOR_IN_2  6 
#define MOTOR_IN_3  4
#define MOTOR_IN_4  5

/*
* README:
* This code is used to find the position of the car based on the distance from the walls
* The car has two LRFs that are used to find the distance from the walls
* The car has a servo that is used to rotate the LRFs
* The car also has an IMU that is used to find the heading of the car
* You must position the car at 0 degrees (So that the IMU heading will be calibrated to the gamefield heading)
*/


/*
 * Global Variables for X and Y position
 */
float X_POS;
float Y_POS;

float currentCardinalHeading;
/*
 * Global Variables for Servo
 */
Servo myservo = Servo();
uint8_t servoPin = 2;
uint16_t servoPosition = 90;
/*
 * Global Variables for IMU
 */
Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;
ESP32MotorControl motors;
float offset = 0;

void setup()
{
  Serial.begin(115200);
  Wire.begin(9, 8);

  // initMotors(MOTOR_IN_1, MOTOR_IN_2, MOTOR_IN_3, MOTOR_IN_4);
  motors.attachMotors(MOTOR_IN_1, MOTOR_IN_2, MOTOR_IN_3, MOTOR_IN_4);
  setupBNO085(&bno08x); // Initialize the IMU
  Serial.println("*****TEST HEADING******\n\n");
  delay(3000);
  offset = getCurrentAngle(); // Get the offset of the IMU
  Serial.println("offset: ");
  Serial.println(offset);
  float currentAngle = getCurrentAngle();
  Serial.print("Current Angle: ");
  Serial.println(currentAngle);
}
// Function to find the cardinal heading based on the current angle
int16_t findCardinalheading()
{
  float currentAngle = getCurrentAngle();
  if (currentAngle >= 0 && currentAngle < 45 || currentAngle >= 315)
  {
    return 0; // North
  }
  else if (currentAngle >= 45 && currentAngle < 135)
  {
    return 90; // East
  }
  else if (currentAngle >= 135 && currentAngle < 225)
  {
    return 180; // South
  }
  else if (currentAngle >= 225 && currentAngle < 315)
  {
    return 270; // West
  }
  else
  {
    return -1; // Error
  }
}
void findPosition(uint16_t cardinalheading, float lrf1, float lrf2)
{
  switch (cardinalheading)
  {
  case 0:
    X_POS = 243 - lrf2; // Subtract the distance from the wall to the LRF
    Y_POS = 243 - lrf1; // Subtract the distance from the wall to the LRF
    break;
  case 90:
    X_POS = lrf1;
    Y_POS = 243 - lrf2;
    break;
  case 180:
    X_POS = lrf2;
    Y_POS = lrf1;
    break;
  case 270:
    X_POS = 243 - lrf1;
    Y_POS = lrf2;
    break;
  default:
    Serial.println("Error");
    break;
  }
}

void setServoPosition()
{
  float lrf1;
  float lrf2;
  float currentHeading = getCurrentAngle();
  uint16_t cardinalHeading = findCardinalheading();
  float servoPosition = cardinalHeading - currentHeading; // Calculate the difference between the cardinal heading and the current heading
  if (abs(servoPosition) > 90)
  {
    servoPosition = 359.99 - abs(servoPosition); // 359.99 is the max value for the servo
  }

  servoPosition += 90;                    // Add 90 because 90 degress is the middle position for the servo (therefore it is the current heading of the car)
  myservo.write(servoPin, servoPosition); // tell servo to go to position in variable 'pos'
  lrf1 = getLrfDistance(LRF_ADDRESS_1);
  // printf("LRF 1 (0x10): %d@%d\n", lrf1, servoPosition);
  lrf2 = getLrfDistance(LRF_ADDRESS_2) + 3; // Subtract 3cm to account for the distance between the two LRFs
  // printf("LRF 2 (0x20): %d@%d\n", lrf2, servoPosition);

  findPosition(cardinalHeading, lrf1, lrf2); // Find the position of the car
}

void printCurrentAngle()
{
  float currentAngle = getCurrentAngle();
  Serial.print("CurrentAngle: ");
  Serial.println(currentAngle);
  // Serial.print("Cardinal heading: ");
  // Serial.println(findCardinalheading());
}

void printCurrentPos()
{
  Serial.println("Position in CM:");
  Serial.print("X: ");
  Serial.print(X_POS);
  Serial.print("\tY: ");
  Serial.println(Y_POS);

  Serial.println("Position in feet:");
  Serial.print("X: ");
  Serial.print(X_POS / 30.48); // Convert X_POS from cm to feet
  Serial.print("\tY: ");
  Serial.println(Y_POS / 30.48); // Convert Y_POS from cm to feet
}
float getNextAngle(float currentX, float currentY, float goalX, float goalY)
{
  Serial.print("goal x ");
  Serial.println(goalX);
  Serial.print("gaol y ");
  Serial.println(goalY);
  float diff_X = goalX - currentX;
  float diff_Y = goalY - currentY;
  // diff_X = -41.52;
  // diff_Y = 212.84;
  float angle = atan(diff_Y/diff_X);
  // atan2()
  Serial.printf("Angle rads: %f\n", angle);
  angle *= 180.0f / M_PI;
  Serial.print("Diff x ");
  Serial.println(diff_X);
  Serial.print("Diff y ");
  Serial.println(diff_Y);
  Serial.printf("differnatial angle = %f\n\n", angle);
  if(diff_X == 0 && diff_Y > 0)
  {
    angle = 0;
  }
  else if(diff_X == 0 && diff_Y < 0)
  {
    angle = 180;
  }
  else if(diff_Y == 0 && diff_X > 0)
  {
    angle = 270;
  }
  else if(diff_Y == 0 && diff_X < 0)
  {
    angle = 90;
  }
  else if(diff_X > 0 && diff_Y > 0)             // x+ y+
  {
    angle += 270;
  }
  else if(diff_X < 0 && diff_Y > 0)             // x- y+
  {
    angle += 90;
  }
  else if(diff_X < 0 && diff_Y < 0)             // x- y-
  {
    angle+=90;;
  }
  else                                          // x+ y-
  {
    angle+=270;;
  }
  if (angle > 359.99)
    angle -= 359.99;
  else if( angle < 0)
    angle += 359.99;
  return angle;
}

// void indicateAngle(float nextAngle)
// {

// }
void turnToHeading(float goal, uint8_t speed)
{
  float absVal;
  float angleDiff;
  uint8_t i = 0;

  float currentAngle = getCurrentAngle();

  // printToLcd("Current Angle: ", currentAngle);
  angleDiff = goal - currentAngle;
  absVal = abs(angleDiff);
  if (absVal > 350)
  {
    absVal = 359.99 - absVal;
    angleDiff = 359.99 - angleDiff;
  }
  // Serial.println(angleDiff);
  // Serial.println(absVal);
  // Serial.println();

  while (absVal > 10)
  {
    setServoPosition();
    // Serial.print("abs: ");
    // Serial.print(absVal);
    // Serial.print(" angle: ");
    // Serial.print(currentAngle);

    if ((angleDiff >= 0) && (absVal <= 180))
    {
      // Serial.print(" case 1: ");
      turn(COUNTER_CLOCKWISE, speed);
    }
    else if ((angleDiff < 0) && (absVal <= 180))
    {
      // Serial.print(" case 2: ");
      turn(CLOCKWISE, speed);
    }
    else if ((angleDiff >= 0) && (absVal >= 180))
    {
      // Serial.print(" case 3: ");
      turn(CLOCKWISE, speed);
    }
    else
    {
      // Serial.print(" case 4: ");
      turn(COUNTER_CLOCKWISE, speed);
    }

    currentAngle = getCurrentAngle();

    // Serial.println(goal);
    angleDiff = goal - currentAngle;
    absVal = abs(angleDiff);
    if (absVal > 350)
    {
      absVal = 359.99 - absVal;
      angleDiff = 359.99 - angleDiff;
    }
    // Serial.println(angleDiff);
    // Serial.println(absVal);
    // Serial.println();
  }
  stop();
}


void driveToHeading(float goalHeading, float goalX, float goalY)
{
  float diffX = abs(X_POS-goalX);
  float diffY = abs(Y_POS-goalY);
  float currentAngle = -1;
  float absVal;
  float angleDiff;
  unsigned long currentMillis;
  unsigned long previousMillis = 0;
  uint32_t interval = 1000;
  uint16_t i = 0;

  bool goToHeading = true;
  setServoPosition();
  if((diffX <= 5 && diffY <= 5))
  {
    stop();
    return;
  }
  while (goToHeading)
  {
    diffX = abs(X_POS-goalX);
    diffY = abs(Y_POS-goalY);
    Serial.printf("X = %f and Y = %f\n",diffX, diffY);
    currentAngle = getCurrentAngle();
    

    i++;
    if (i == 30)
    {
      // printToLcd("Current Angle: ", currentAngle);
      i = 0;
    }
    angleDiff = goalHeading - currentAngle;
    absVal = abs(angleDiff);
    // Serial.println(absVal);
    if (absVal > 345)
    {
      absVal = 359.99 - absVal;
      angleDiff = 359.99 - angleDiff;
    }
    // Serial.println(angleDiff);
    // Serial.println(absVal);
    // Serial.println();

    if (absVal > 15)
    {
      turnToHeading(goalHeading, 65);            // turn to the desired heading because angle is to great to correct while driving
      previousMillis = currentMillis = millis(); // this timer was used to stop the robot after one seconds when starts driving at the correct angle
                                                // but this is not used anymore
    }
    else if (absVal <= 15 && absVal >= 3)
    {
      if ((angleDiff >= 0) && (absVal <= 180))
      {
        // Serial.print(" case 1: ");
        turn2(COUNTER_CLOCKWISE, 65, 30); // Turn2 takes in the direction, speed, and an offset that will increase the turning wheel by the given amount
      }
      else if ((angleDiff < 0) && (absVal <= 180))
      {
        // Serial.print(" case 2: ");
        turn2(CLOCKWISE, 65, 30);
      }
      else if ((angleDiff >= 0) && (absVal >= 180))
      {
        // Serial.print(" case 3: ");
        turn2(CLOCKWISE, 65, 30);
      }
      else
      {
        // Serial.print(" case 4: ");
        turn2(COUNTER_CLOCKWISE, 65, 30);
      }
      previousMillis = currentMillis = millis();
    }
    else if (absVal < 5 && absVal >= 2) // if the angle is less than 5 degrees, drive the turning wheel slightly faster than the straight wheel
    {
      if ((angleDiff >= 0) && (absVal <= 180))
      {
        // Serial.print(" case 1: ");
        turn2(COUNTER_CLOCKWISE, 65, 20);
      }
      else if ((angleDiff < 0) && (absVal <= 180))
      {
        // Serial.print(" case 2: ");
        turn2(CLOCKWISE, 65, 20);
      }
      else if ((angleDiff >= 0) && (absVal >= 180))
      {
        // Serial.print(" case 3: ");
        turn2(CLOCKWISE, 65, 20);
      }
      else
      {
        // Serial.print(" case 4: ");
        turn2(COUNTER_CLOCKWISE, 65, 20);
      }
      previousMillis = currentMillis = millis();
    }
    else if (absVal < 2 && absVal >= 1) // if the angle is less than 2 degrees, drive the turning wheel barely faster than the straight wheel
    {
      if ((angleDiff >= 0) && (absVal <= 180))
      {
        // Serial.print(" case 1: ");
        turn2(COUNTER_CLOCKWISE, 65, 10);
      }
      else if ((angleDiff < 0) && (absVal <= 180))
      {
        // Serial.print(" case 2: ");
        turn2(CLOCKWISE, 65, 10);
      }
      else if ((angleDiff >= 0) && (absVal >= 180))
      {
        // Serial.print(" case 3: ");
        turn2(CLOCKWISE, 65, 10);
      }
      else
      {
        // Serial.print(" case 4: ");
        turn2(COUNTER_CLOCKWISE, 65, 10);
      }
      unsigned long currentMillis = millis();

      // if (currentMillis - previousMillis >= interval)
      // {
      //   previousMillis = currentMillis;
      //   if (previousMillis != 0)
      //   {
      //     goToHeading = false;
      //     stop();
      //   }
      // }
    }
    else
    {
      move(FORWARD, 255); // if the angle is less than 1 degree, drive straight

      // if (currentMillis - previousMillis >= interval)
      // {
      //   previousMillis = currentMillis;
      //   if (previousMillis != 0)
      //   {
      //     goToHeading = false;
      //     stop();
      //   }
      // }
    }
    setServoPosition();
     if((diffX <= 5 && diffY <= 5))
    {
      stop();
      return;
    }
  }
}



void testHeading(float x, float y)
{
  /*
  float nextX = 6;
  float nextY = 6;
  
  */
  float currentAngle = 0.0;
  float nextAngle = 0.0;
  setServoPosition();
  currentAngle = getCurrentAngle();
  nextAngle = getNextAngle(X_POS, Y_POS, x*30.48, y*30.48);
  // x = 121.92 y = 91.44
  
  // Serial.println("Current Angle: ");
  // Serial.println(currentAngle);
  printCurrentPos();

  Serial.println("**************");
  Serial.println("Next Angle: ");
  Serial.println(nextAngle);
  // Serial.println("Next Position in feet:");
  // Serial.print("X: ");
  // Serial.print(x); // Convert X_POS from cm to feet
  // Serial.print("\tY: ");
  // Serial.println(y); // Convert Y_POS from cm to feet
// Serial.println(getHeading());
//   delay(25);
  // turnToHeading(nextAngle, 80);
  driveToHeading(nextAngle, (x*30.48), (y*30.48));
  Serial.print("Angle reached: ");
  Serial.println(getCurrentAngle());
}


void loop()
{
  #ifdef ANGLE
  Serial.println(getHeading());
  delay(100);
  #endif
  
  #ifdef ELIMINATION
  testHeading(1, 4);
  delay(1000);
  // delay(2000);
  // stop();
  // delay(1000);
  // testHeading(1, 1);
  // delay(2000);
  // stop();
  // delay(1000);
  // testHeading(1, 6);
  // delay(2000);
  // stop();
  // delay(1000);
  // testHeading(7, 2);
  // delay(2000);
  // stop();
  // delay(1000);
  // testHeading(1, 6);
  // delay(2000);
  // stop();
  // delay(1000);
  stop();

  while(1);

  

  #endif

  #ifdef TEST
  static uint16_t i = 0;
  i++;
  setServoPosition();

  if (i >= 15)
  {
    printCurrentAngle();
    printCurrentPos();
    i = 0;
  }
  delay(10);
  #endif
}
