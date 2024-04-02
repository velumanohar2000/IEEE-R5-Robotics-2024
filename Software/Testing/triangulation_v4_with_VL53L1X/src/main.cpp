#include <Arduino.h>
#include <stdbool.h>
#include <Adafruit_BNO08x.h>
#include <Wire.h>
#include <math.h>
#include <ESP32MotorControl.h>
#include <LiquidCrystal_I2C.h>
#include <string.h>
#include <Servo.h>
#include "SparkFun_VL53L1X.h"


#include "BNO085_heading.h"
#include "VL53L1X_MULTIPLE.h"
#include "lrf.h"



SFEVL53L1X lrf1;
SFEVL53L1X lrf2;


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
float offset = 0;

void setup()
{
  Serial.begin(115200);
  Wire.begin(9, 8);
 // Wire1.begin(20, 21); //20 sda, 21 scl
  init_2_VL53L1X();
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
  lrf1 = getLrfDistanceCm(1);
  // printf("LRF 1 (0x10): %d@%d\n", lrf1, servoPosition);
  lrf2 = getLrfDistanceCm(2); // Subtract 3cm to account for the distance between the two LRFs
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
  // Serial.print("goal x ");
  // Serial.println(goalX);
  // Serial.print("gaol y ");
  // Serial.println(goalY);
  float diff_X = goalX - currentX;
  float diff_Y = goalY - currentY;
  // diff_X = -41.52;
  // diff_Y = 212.84;
  float angle = atan(diff_Y/diff_X);
  // atan2()
  // Serial.printf("Angle rads: %f\n", angle);
  // angle *= 180.0f / M_PI;
  // Serial.print("Diff x ");
  // Serial.println(diff_X);
  // Serial.print("Diff y ");
  // Serial.println(diff_Y);
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
  Serial.printf("next angle = %f\n\n", angle);
  return angle;
}

void loop()
{
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
}
