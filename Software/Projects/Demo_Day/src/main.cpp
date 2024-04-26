#include <Arduino.h>
#include <stdbool.h>
#include <Adafruit_BNO08x.h>
#include <Wire.h>
#include <math.h>
#include <ESP32MotorControl.h>
#include "motor_control_v2.h"
#include <LiquidCrystal_I2C.h>
#include <string.h>
#include <Servo.h>
#include "SparkFun_VL53L1X.h"
#include "BNO085_heading.h"
#include "VL53L1X_MULTIPLE.h"
#include "lrf.h"
#include "OPT3101_whisker.h"
#include "TCS_color_det.h"
#include "obj_detection.h"
#include "IR_switch.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// #define TEST_ALL_COMPONENTS
// #define TEST
// #define ELIM


#define XSHUT_PIN 18

// #define TEST_ELIM
// #define MOTORS

// #define TEST_TURN_TO_HEADING

#define TURN_TO_ANGLE_DIFF 20
#define DRIVE_TO_ANGLE_DIFF 30

#define R_AMB 17
#define G_AMB 23
#define B_AMB 18s
#define STATION_A 2, .75
#define STATION_B 6, .75
#define STATION_C 7.25, 2
#define STATION_D 7.25, 6
#define STATION_E 6, 7.25
#define STATION_F 2, 7.25
#define STATION_G 1, 6
#define STATION_H 0.75, 2

// TODO:
#define STATION_A_CODE 0x400E40BF // Change this to ir code
#define STATION_B_CODE 0x400EC03F // Change this to ir code
#define STATION_C_CODE 0x400E20DF // Change this to ir code
#define STATION_D_CODE 0x400EA05F // Change this to ir code
#define STATION_E_CODE 0x400E609F // Change this to ir code
#define STATION_F_CODE 0x400EE01F // Change this to ir code
#define STATION_G_CODE 0x400E10EF // Change this to ir code
#define STATION_H_CODE 0x400E906F // Change this to ir code

SFEVL53L1X lrf1_init;
SFEVL53L1X lrf2_init;

/*
 * README:
 * This code is used to find the position of the car based on the distance from the walls
 * The car has two LRFs that are used to find the distance from the walls
 * The car has a servo that is used to rotate the LRFs
 * The car also has an IMU that is used to find the heading of the car
 * You must position the car at 0 degrees (So that the IMU heading will be calibrated to the gamefield heading)
 */

/*
 * Global Variables for Motors
 */
const uint8_t MOTOR_A_IN_1 = 6;
const uint8_t MOTOR_A_IN_2 = 7;
int16_t A_LEFT_MOTOR_OFFSET = 20;

// RIGHT MOTOR
const uint8_t MOTOR_B_IN_3 = 5;
const uint8_t MOTOR_B_IN_4 = 4;
int16_t B_RIGHT_MOTOR_OFFSET = 20;

ESP32MotorControl motors;
bool direction = true;

/*
 * Global Variables for X and Y position

 */
float X_POS;
float Y_POS;

float OLD_X_POS = 60.96;
float OLD_Y_POS = 0;

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
float offsetForImu = 0;

// Buttons for TCL - Roku TV Remote
const uint32_t lit_code = 0xE0E040BF;     // Hulu code: follow path
const uint32_t unalive_code = 0x400E00FF; // Netflix code


gpio_num_t ir_pin = GPIO_NUM_11;

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void setup()
{
  Serial.begin(115200);
  Wire.begin(9, 8);

  setupBNO085(&bno08x); // Initialize the IMU
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();

  // Wire1.begin(20,21);s
  // initOPT3101();
  motors.attachMotors(MOTOR_B_IN_3, MOTOR_B_IN_4, MOTOR_A_IN_1, MOTOR_A_IN_2);

  // TODO: enter manual_code
  initIR(lit_code, unalive_code);
  while (!wokeFromIR())
  {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.println("Zzz...");
    display.display();
    timeToSleep();
  }
  // Wire1.begin(20, 21); //20 sda, 21 scl
  init_2_VL53L1X();
  // bno08x.hardwareReset();
  /// initTCS(R_AMB, G_AMB, B_AMB, 0x29, &Wire1);

  Serial.println("*****TEST HEADING******\n\n");
  delay(3000);
  offsetForImu = getCurrentAngle(); // Get the offset of the IMU
  Serial.println("offset: ");
  Serial.println(offsetForImu);
  float currentAngle = getCurrentAngle();
  Serial.print("Current Angle: ");
  Serial.println(currentAngle);
}

void sleephandler()
{
  stopMotors();
  Wire.end();
  digitalWrite(XSHUT_PIN, 0);
  delay(100);
}

void displayText(uint8_t x, uint8_t y, char station) {
  display.clearDisplay();
  display.setTextSize(3);
  display.setTextColor(WHITE);
  display.setCursor(0, 6);
  display.printf("DEST: %c",station);
  display.setCursor(13, 36);
  display.printf("(%d, %d)", x, y);
  display.display();
}

void checkSleep()
{
  if (sleepCodeReceived())
  {
    sleephandler();
    timeToSleep();
    ESP.restart();
  }
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

void jiggle()
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
  delay(100);                             // waits 15ms for the servo to reach the position
  lrf1 = getLrfDistanceCm(1);
  lrf2 = getLrfDistanceCm(2);
  findPosition(cardinalHeading, lrf1, lrf2); // Find the position of the car
}

float getNextAngle(float currentX, float currentY, float goalX, float goalY)
{

  Serial.print("Current x ");
  Serial.print(currentX);
  Serial.print("\tCurrent Y ");
  Serial.println(currentY);
  float diff_X = goalX - currentX;
  float diff_Y = goalY - currentY;
  Serial.print("Diff x ");
  Serial.print(diff_X);
  Serial.print("\tDiff y ");
  Serial.println(diff_Y);

  float angle;
  if (diff_X == 0 && diff_Y > 0)
  {
    angle = 0;
  }
  else if (diff_X == 0 && diff_Y < 0)
  {
    angle = 180;
  }
  else if (diff_Y == 0 && diff_X > 0)
  {
    angle = 270;
  }
  else if (diff_Y == 0 && diff_X < 0)
  {
    angle = 90;
  }
  else
  {
    angle = atan(diff_Y / diff_X);
    angle *= 180.0f / M_PI;

    if (diff_X > 0 && diff_Y > 0) // x+ y+
    {
      angle += 270;
    }
    else if (diff_X < 0 && diff_Y > 0) // x- y+
    {
      angle += 90;
    }
    else if (diff_X < 0 && diff_Y < 0) // x- y-
    {
      angle += 90;
    }
    else // x+ y-
    {
      angle += 270;
    }
  }
  if (angle > 359.99)
    angle -= 359.99;
  else if (angle < 0)
    angle += 359.99;
  Serial.printf("next angle = %f\n\n", angle);

  Serial.printf("Degrees away from Current Angle = %f\n\n", angle - getCurrentAngle());

  return angle;
}

void turnToHeading(float goal, uint8_t speed)
{
  printf("Goal: %f\n", goal);
  float absVal;
  float angleDiff;
  uint8_t i = 0;

  float currentAngle = getCurrentAngle();

  angleDiff = goal - currentAngle;
  absVal = abs(angleDiff);
  uint32_t turnDiff = TURN_TO_ANGLE_DIFF;
  if (absVal > 360 - turnDiff)
  {
    absVal = 359.99 - absVal;
    angleDiff = 359.99 - angleDiff;
  }
  Serial.println(angleDiff);
  Serial.println(absVal);
  Serial.println();

  while (absVal > turnDiff)
  {
    checkSleep();

    if ((angleDiff >= 0) && (absVal <= 180))
    {
      turn(COUNTER_CLOCKWISE, speed);
    }
    else if ((angleDiff < 0) && (absVal <= 180))
    {
      turn(CLOCKWISE, speed);
    }
    else if ((angleDiff >= 0) && (absVal >= 180))
    {
      turn(CLOCKWISE, speed);
    }
    else
    {
      turn(COUNTER_CLOCKWISE, speed);
    }

    currentAngle = getCurrentAngle();

    angleDiff = goal - currentAngle;
    absVal = abs(angleDiff);
    if (absVal > (360 - turnDiff)) // was (absVal > 360 - turnDiff)
    {
      absVal = 359.99 - absVal;
      angleDiff = 359.99 - angleDiff;
    }
  }
}

void driveToHeading(float goalHeading)
{
  float currentAngle = -1;
  float absVal;
  float angleDiff;
  unsigned long currentMillis;
  unsigned long previousMillis = 0;
  uint32_t interval = 1000;
  uint16_t i = 0;

  currentAngle = getCurrentAngle();

  angleDiff = goalHeading - currentAngle;
  absVal = abs(angleDiff);
  uint32_t turnDiff = DRIVE_TO_ANGLE_DIFF;
  if (absVal > (360 - turnDiff))
  {
    absVal = 359.99 - absVal;
    angleDiff = 359.99 - angleDiff;
  }
  printf("Angle Diff: %f", angleDiff);
  printf("\tAbsVal: %f", absVal);

  if (absVal > turnDiff)
  {
    turnToHeading(goalHeading, 60);
  }
  else if (absVal >= 5)
  {
    if ((angleDiff >= 0) && (absVal <= 180))
    {
      turn2(COUNTER_CLOCKWISE, 65, 30);
    }
    else if ((angleDiff < 0) && (absVal <= 180))
    {
      turn2(CLOCKWISE, 65, 30);
    }
    else if ((angleDiff >= 0) && (absVal >= 180))
    {
      turn2(CLOCKWISE, 65, 30);
    }
    else
    {
      turn2(COUNTER_CLOCKWISE, 65, 30);
    }
  }
  else if (absVal < 5 && absVal >= 2)
  {
    if ((angleDiff >= 0) && (absVal <= 180))
    {
      turn2(COUNTER_CLOCKWISE, 65, 10);
    }
    else if ((angleDiff < 0) && (absVal <= 180))
    {
      turn2(CLOCKWISE, 65, 10);
    }
    else if ((angleDiff >= 0) && (absVal >= 180))
    {
      turn2(CLOCKWISE, 65, 10);
    }
    else
    {
      turn2(COUNTER_CLOCKWISE, 65, 10);
    }
  }
  else
  {
    move(FORWARD, 65);
  }
}

void goToCoordinates(float nextX, float nextY, char station)
{
  nextX *= 30.48;
  nextY *= 30.48;
  float diffX = abs(nextX - X_POS);
  float diffY = abs(nextY - Y_POS);
  bool goToCoordinates = true;
  while (goToCoordinates)
  {
    displayText((uint8_t)((X_POS+0.5)/30.48), (uint8_t)((Y_POS+0.5)/30.48), station);
    checkSleep();

    jiggle();

    if (abs(X_POS - OLD_X_POS) > 30.48 * 3 || abs(Y_POS - OLD_Y_POS) > 30.48 * 3)
    {
      stopMotors();
      uint8_t i = 0;
      for (i = 0; i < 5; i++)
      {
        jiggle();
      }
    }
    OLD_X_POS = X_POS;
    OLD_Y_POS = Y_POS;

    // get angle for next coordinates
    float nextAngle = getNextAngle(X_POS, Y_POS, nextX, nextY);
    driveToHeading(nextAngle);
    diffX = abs(nextX - X_POS);
    diffY = abs(nextY - Y_POS);

    if (diffX <= 10 && diffY <= 10)
    {
      stopMotors();
      goToCoordinates = false;
    }
  }
}

bool firstRun = true;

// void loop()
// {
// #ifdef ELIM

//   if (firstRun)
//   {
//     uint16_t i = 0;
//     for (i = 0; i < 30; i++)
//       jiggle();
//     firstRun = false;
//   }
//   // set current coordinates starting at A

//   goToCoordinates(STATION_D);
//   delay(1500);
//   goToCoordinates(STATION_H);
//   delay(1500);
//   goToCoordinates(STATION_F);
//   delay(1500);
//   goToCoordinates(STATION_B);
//   delay(1500); // comment these
//   goToCoordinates(STATION_G);
//   delay(1500);
//   goToCoordinates(STATION_E);
//   delay(1500);
//   goToCoordinates(STATION_C);
//   delay(1500);
//   goToCoordinates(STATION_A);
//   delay(1500);

//   /*
//   goToCoordinates(STATION_G);
//   delay(1500);
//   goToCoordinates(STATION_E);
//   delay(1500);
//   goToCoordinates(STATION_C);
//   delay(1500);
//   goToCoordinates(STATION_A);
//   delay(1500);                    // comment these
//   goToCoordinates(STATION_D);
//   delay(1500);
//   goToCoordinates(STATION_H);
//   delay(1500);
//   goToCoordinates(STATION_F);
//   delay(1500);
//   goToCoordinates(STATION_B);
//   delay(1500);          //gecadhfb
//   */

// #endif
// }

void loop()
{
  // displayText("(2, 1)", 'A');
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.println("Init...");
  display.display();
  uint16_t i = 0;
  for (i = 0; i < 30; i++)
    jiggle();
  stopMotors();

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.println("Waiting");
  display.display();

  uint32_t input = 0; 
  while (1)
  {
    // TODO 
    // wait for ir input
    // read ir code and then go to coordinate
    // if incorrect code or no code then nothing should happen because motors should be stopped. No default case needed.
    while(input == 0)
    {
      input = getDestination();
    }

    Serial.println("out of while loop");

    switch (input)
    {
    case 1:
      goToCoordinates(STATION_A, 'A');
      // displayText("(2, 1)", 'A');
      stopMotors();
      break;
    case 2:
      goToCoordinates(STATION_B, 'B');
      // displayText("(6, 1)", 'B');
      stopMotors();
      break;
    case 3:
      goToCoordinates(STATION_C, 'C');
      // displayText("(7, 2)", 'C');
      stopMotors();
      break;
    case 4:
      goToCoordinates(STATION_D, 'D');
      // displayText("(7, 6)", 'D');
      stopMotors();
      break;
    case 5:
      goToCoordinates(STATION_E, 'E');
      // displayText("(6, 7)", 'E');
      stopMotors();
      break;
    case 6:
      goToCoordinates(STATION_F, 'F');
      // displayText("(2, 7)", 'F');
      stopMotors();
      break;
    case 7:
      goToCoordinates(STATION_G, 'G');
      // displayText("(1, 6)", 'G');
      stopMotors();
      break;
    case 8:
      goToCoordinates(STATION_H, 'H');
      // displayText("(1, 2)", 'H');
      stopMotors();
      break;
    case 20:
    {
      goToCoordinates(STATION_D, 'D');
      delay(1500);
      goToCoordinates(STATION_H, 'H');
      delay(1500);
      goToCoordinates(STATION_F, 'F');
      delay(1500);
      goToCoordinates(STATION_B, 'B');
      delay(1500); // comment these
      goToCoordinates(STATION_G, 'G');
      delay(1500);
      goToCoordinates(STATION_E, 'E');
      delay(1500);
      goToCoordinates(STATION_C, 'C');
      delay(1500);
      goToCoordinates(STATION_A, 'A');
      delay(1500);
      break;
    }
    }
    input = 0;
  }
}