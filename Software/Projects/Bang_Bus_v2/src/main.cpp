#include <Arduino.h>
#include <Servo.h>
#include <stdbool.h>
#include <Adafruit_BNO08x.h>
#include <Wire.h>
#include <math.h>
#include <ESP32MotorControl.h>
#include <VL53L1X_MULTIPLE.h>
#include "BNO085_heading.h"
#include "motor_control_v2.h"
#include "SparkFun_VL53L1X.h"
#include "IR_switch.h"

// Whisker
#define WHISKER_STOP_DIS 25

#define MAX_PRELIM_DIST 60
#define MIN_WALL_DIST_CM 11
#define MAX_WALL_DIST_CM 12

#define TURN_TO_ANGLE_DIFF 2
#define DRIVE_TO_ANGLE_DIFF 20

// Sleep
#define XSHUT_PIN 18

// Variables & Constants ------------------------------------------------------

/*
  Global Variable for the Motors
*/
ESP32MotorControl motors;
// Motors
// Left
const uint8_t MOTOR_A_IN_1 = 6;
const uint8_t MOTOR_A_IN_2 = 7;
int16_t A_LEFT_MOTOR_OFFSET = 2;

// RIGHT MOTOR
const uint8_t MOTOR_B_IN_3 = 5;
const uint8_t MOTOR_B_IN_4 = 4;
int16_t B_RIGHT_MOTOR_OFFSET = 14;

/*
  Global Variable for IMU
*/
Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;
float offsetForImu = 0;
float goal = 85;

/*
  Globals for (VL53L1X) Whisker
*/
SFEVL53L1X lrf1_init;
SFEVL53L1X lrf2_init;

float frontVLcm = 1000;
float sideVLcm;
// int frontVLcm = 1000;                            // whisker
float frontVLcmInch = 1000; // whisker
int whiskOffset = 0;
bool wallFound = false;

/*
  globals for servo
*/
Servo myservo = Servo();
uint8_t servoPin = 2;
uint16_t servoPosition = 90;

/*
  Globals for IR sensor & sleep
*/
const uint32_t lit_code = 0x574309F6; // Amazon button
const uint32_t unalive_code = 0x5743D32C; // Netflix button
gpio_num_t ir_pin = GPIO_NUM_11; // pin number of IR receiver(s)


void setup()
{
  Serial.begin(115200);
  Wire1.begin(20,21);
  setupBNO085(&bno08x); // Initialize the IMU
  // pinMode(XSHUT_PIN, OUTPUT);
  // digitalWrite(XSHUT_PIN, 0);

  initIR(lit_code, unalive_code);
  if (!wokeFromIR())
  {
    timeToSleep();
  }
  // digitalWrite(XSHUT_PIN, 1);

  delay(1000);

  delay(10);
  Serial.println("*****BANGING THE BUS******\n\n");
  delay(500);  

  Wire.begin(9, 8); // init Wire
  init_2_VL53L1X(); // init periscopes

  motors.attachMotors(MOTOR_B_IN_3, MOTOR_B_IN_4, MOTOR_A_IN_1, MOTOR_A_IN_2); // init motors

  uint8_t i = 0;
  offsetForImu = getCurrentAngle();

  Serial.println("offset: ");
  Serial.println(offsetForImu);

  float currentAngle = getCurrentAngle();

  Serial.print("Current Angle: ");
  Serial.println(currentAngle);

  delay(500);

  for (i = 0; i < 50; i++)
  {
    float test = getLrfDistanceCm(1);
    test = getLrfDistanceCm(2);
    Serial.printf("LRF1: %f\n", test);
  }
  myservo.write(servoPin, 90);
  delay(500);
}

void sleepHandler()
{
    stopMotors();
    Wire.end();
    digitalWrite(XSHUT_PIN, 0);
    delay(100);
}

void checkSleep()
{
  if (sleepCodeReceived())
  {
    sleepHandler();
    timeToSleep();
    ESP.restart();
  }
}

// void driveToHeading(float goalHeading)
// {
//   float currentAngle = -1;
//   float absVal;
//   float angleDiff;
//   unsigned long currentMillis;
//   unsigned long previousMillis = 0;
//   uint32_t interval = 1000;
//   uint16_t i = 0;

//   // bool goToHeading = true;
//   // while (goToHeading)
//   // {
//   currentAngle = getCurrentAngle();

//   angleDiff = goalHeading - currentAngle;
//   absVal = abs(angleDiff);
//   uint32_t turnDiff = DRIVE_TO_ANGLE_DIFF;
//   // Serial.println(absVal);
//   if (absVal > (360 - turnDiff))
//   {
//     absVal = 359.99 - absVal;
//     angleDiff = 359.99 - angleDiff;
//   }
//   Serial.println(angleDiff);
//   Serial.println(absVal);
//   Serial.println();

//   if (absVal > turnDiff)
//   {
//     // stopMotors();
//     // delay(20);
//     turnToHeading(goalHeading-10, 60);
//   }
//   else if (absVal <= 15 && absVal >= 5)
//   {
//     if ((angleDiff >= 0) && (absVal <= 180))
//     {
//       turn2(COUNTER_CLOCKWISE, 65, 30);
//     }
//     else if ((angleDiff < 0) && (absVal <= 180))
//     {
//       turn2(CLOCKWISE, 65, 30);
//     }
//     else if ((angleDiff >= 0) && (absVal >= 180))
//     {
//       turn2(CLOCKWISE, 65, 30);
//     }
//     else
//     {
//       turn2(COUNTER_CLOCKWISE, 65, 30);
//     }
//     previousMillis = currentMillis = millis();
//   }
//   else if (absVal < 5 && absVal >= 2)
//   {
//     if ((angleDiff >= 0) && (absVal <= 180))
//     {
//       turn2(COUNTER_CLOCKWISE, 65, 10);
//     }
//     else if ((angleDiff < 0) && (absVal <= 180))
//     {
//       turn2(CLOCKWISE, 65, 10);
//     }
//     else if ((angleDiff >= 0) && (absVal >= 180))
//     {
//       turn2(CLOCKWISE, 65, 10);
//     }
//     else
//     {
//       turn2(COUNTER_CLOCKWISE, 65, 10);
//     }
//   }
//   else
//   {
//     move(FORWARD, 65);
//   }
//   // }
// }

void turnToHeading(float goal, uint8_t speed)
{
  stopMotors();
  // speed = 60;
  printf("Goal: %f\n", goal);
  // delay(3000);
  float absVal;
  float angleDiff;
  uint8_t i = 0;

  float currentAngle = getCurrentAngle();

  // printToLcd("Current Angle: ", currentAngle);
  angleDiff = goal - currentAngle;
  absVal = abs(angleDiff);
  uint32_t turnDiff = TURN_TO_ANGLE_DIFF;
  // Serial.println(absVal);
  if (absVal > 360 - turnDiff)
  {
    absVal = 359.99 - absVal;
    angleDiff = 359.99 - angleDiff;
  }
  // Serial.println(angleDiff);
  // Serial.println(absVal);
  // Serial.println();

  while (absVal > turnDiff)
  {

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
    if (absVal > (360 - TURN_TO_ANGLE_DIFF)) // was (absVal > 360 - turnDiff)
    {
      absVal = 359.99 - absVal;
      angleDiff = 359.99 - angleDiff;
    }
    // Serial.println(angleDiff);
    // Serial.println(absVal);
    // Serial.println();
  }
  stopMotors();
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

  // bool goToHeading = true;
  // while (goToHeading)
  // {
  currentAngle = getCurrentAngle();

  angleDiff = goalHeading - currentAngle;
  absVal = abs(angleDiff);
  uint32_t turnDiff = DRIVE_TO_ANGLE_DIFF;
  // Serial.println(absVal);
  if (absVal > (360 - turnDiff))
  {
    absVal = 359.99 - absVal;
    angleDiff = 359.99 - angleDiff;
  }
  Serial.println(angleDiff);
  Serial.println(absVal);
  Serial.println();

  if (absVal > turnDiff)
  {
    // stopMotors();
    // delay(20);
    turnToHeading(goalHeading, 60);
  }
  else if (absVal <= 15 && absVal >= 5)
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
    previousMillis = currentMillis = millis();
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
  // }
}

// void driveToHeadingVl(float goalHeading)
// {
//   float currentReading = -1;
//   float absVal;
//   float posDiff;
//   unsigned long currentMillis;
//   unsigned long previousMillis = 0;
//   uint32_t interval = 1000;
//   uint16_t i = 0;

//   // bool goToHeading = true;
//   // while (goToHeading)
//   // {
//   currentReading = getLrfDistanceCm(2);

//   posDiff = 13 - currentReading;
//   absVal = abs(posDiff);
//   // uint32_t turnDiff = DRIVE_TO_ANGLE_DIFF;
//   // // Serial.println(absVal);
//   // if (absVal > (360 - turnDiff))
//   // {
//   //   absVal = 359.99 - absVal;
//   //   angleDiff = 359.99 - angleDiff;
//   // }
//   Serial.println(posDiff);
//   Serial.println(absVal);
//   Serial.println();

//   if (absVal > 20)
//   {
//     // stopMotors();
//     // delay(20);
//     turnToHeading(goalHeading, 60);
//   }
//   else if (absVal <= 15 && absVal >= 5)
//   {
//     if ((angleDiff >= 0))
//     {
//       turn2(COUNTER_CLOCKWISE, 65, 30);
//     }
//     else if ((angleDiff < 0) && (absVal <= 180))
//     {
//       turn2(CLOCKWISE, 65, 30);
//     }
//     else if ((angleDiff >= 0) && (absVal >= 180))
//     {
//       turn2(CLOCKWISE, 65, 30);
//     }
//     else
//     {
//       turn2(COUNTER_CLOCKWISE, 65, 30);
//     }
//     previousMillis = currentMillis = millis();
//   }
//   else if (absVal < 5 && absVal >= 2)
//   {
//     if ((angleDiff >= 0) && (absVal <= 180))
//     {
//       turn2(COUNTER_CLOCKWISE, 65, 10);
//     }
//     else if ((angleDiff < 0) && (absVal <= 180))
//     {
//       turn2(CLOCKWISE, 65, 10);
//     }
//     else if ((angleDiff >= 0) && (absVal >= 180))
//     {
//       turn2(CLOCKWISE, 65, 10);
//     }
//     else
//     {
//       turn2(COUNTER_CLOCKWISE, 65, 10);
//     }
//   }
//   else
//   {
//     move(FORWARD, 65);
//   }
//   // }
// }

unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
uint32_t interval = 1000;
uint16_t states = 0;

bool firstButton = true;
bool firstRun = true;
#define INITIAL 0
#define RIDE_RIGHT_WALL 1
#define TURN_90_DEG 2
#define TURN_TO_BUTTON 3
#define HIT_BUTTON 4
#define MOVE_BACKWARDS 5
#define TURN_90_DEGREE_AGAIN 6
#define TURN_OTHER_WAY 7
#define STOP 8  

void jiggleForRound1()
{
  uint16_t position = 0;
  float currentAngle = getCurrentAngle();
  if(currentAngle >= 0 && currentAngle <= 90)
  {
    position =(90-currentAngle);
  }
  else if(currentAngle >= 270)
  {
    position =((360-currentAngle)+90);
  }
  else if(currentAngle >= 180 && currentAngle <= 270)
  {
    position =(90-(currentAngle-180));
  }
  else if(currentAngle < 180 && currentAngle >= 90)
  {
    position =(180 - currentAngle + 90);
  }
  myservo.write(servoPin, position);
}

uint16_t speed;
void loop()
{
  checkSleep();
  // if (getLrfDistanceCm(2) >= 25)
  // {
  //   turnToHeading(270, 60);
  //   currentMillis = millis();
  //   previousMillis = millis();
  //   while (currentMillis - previousMillis <= 2000)
  //   {
  //     printf("Current Angle: %f\n", getCurrentAngle());
  //     currentMillis = millis();
  //   }
  //   // turnToHeading(270, 60);

  //   //  delay(3000);
  //   while (getLrfDistanceCm(1) >= 20)
  //   {
  //     driveToHeading(270);
  //   }
  // }


  // stopMotors();
  // delay(2000);
  // turnToHeading(0, 60);
  // stopMotors();
  float check = getLrfDistanceCm(2);
  switch(states)
  {
    case INITIAL:
    {
      Serial.printf("LRF 2: %f\n", check);
      if (check >= 19.0)
      {
        Serial.printf("here\n");
        turnToHeading(270, 70);
        currentMillis = millis();
        previousMillis = millis();
        while (currentMillis - previousMillis <= 2000)
        {
          printf("Current Angle: %f\n", getCurrentAngle());
          currentMillis = millis();
        }
        // turnToHeading(270, 60);

        //  delay(3000);
        while (getLrfDistanceCm(1) >= 12)
        {
        Serial.printf("in the while\n");
          driveToHeading(270);
        }

      }
        stopMotors();
        delay(500);
        turnToHeading(355, 70);
        stopMotors();
        delay(500);
        states = RIDE_RIGHT_WALL;
      break;
    }
    case RIDE_RIGHT_WALL:
    {
      jiggleForRound1();
      frontVLcm = getLrfDistanceCm(1);
      // frontVLcm = getLrfDistanceCm(1)
      // currentAngle = getHeading();
      if(frontVLcm > (WHISKER_STOP_DIS))
      {
        if(frontVLcm > 1200)
          speed = 255;
        else if( frontVLcm > 300)
          speed = 80;
        else
          speed = 50;
        sideVLcm = getLrfDistanceCm(2);

        Serial.printf("ult dist %f: \n", sideVLcm);
        if(sideVLcm > MAX_PRELIM_DIST)
        {
          stopMotors();
        }
        else if(sideVLcm > MAX_WALL_DIST_CM)
        {
          turn(CLOCKWISE, speed);
          delay(40);
          move(FORWARD, speed);
          delay(50);
        }
        else if(sideVLcm < MIN_WALL_DIST_CM)
        {
          turn(COUNTER_CLOCKWISE, speed);
          delay(40);
          move(FORWARD, speed);
          delay(50);
        }
        else
          move(FORWARD, speed);
      }
      else
      {
        stopMotors();
        states = TURN_90_DEG;
        // currentAngle = getHeading();
      }
      break;
    }
    case TURN_90_DEG:
    {
      float angle = 90;
      if(!firstButton)
      {
        turnToHeading(270, 70);
        stopMotors();
        delay(500);
        angle = 270.0;
      }
      else
      {
        turnToHeading(90, 70);
        stopMotors();
        delay(500);
      }
      myservo.write(servoPin, 90);
      if(getLrfDistanceCm(1) > 35)
      {
        while(getLrfDistanceCm(1) < 35)
        {
          driveToHeading(angle);
        }
      }
      states = TURN_TO_BUTTON;
      break;
    }
    case TURN_TO_BUTTON:
    {
      if(!firstButton)
      {
        turnToHeading(180, 70);
        stopMotors();
        delay(500);
      }
      else
      {
        turnToHeading(0, 70);
        stopMotors();
        delay(500);
      }
      states = HIT_BUTTON;
      break;
    }
    case HIT_BUTTON:
    {
      move(FORWARD, 240);
      delay(500);
      stopMotors();
      delay(500);
      states = MOVE_BACKWARDS;
      break;
    }
    case MOVE_BACKWARDS:
    {
      move(BACKWARD, 240);
      delay(500);
      stopMotors();
      delay(500);
      if(!firstButton)
      {
        states = STOP;
      }
      else
      {
        states = TURN_90_DEGREE_AGAIN;
      }
      break;
    }
    case TURN_90_DEGREE_AGAIN:
    {
        turnToHeading(90, 70);
        stopMotors();
        delay(500);
        while(getLrfDistanceCm(1) > 12)
        {
          driveToHeading(90);
        }
      
      states = TURN_OTHER_WAY;
      break;
    }
    case TURN_OTHER_WAY:
    {
      turnToHeading(180, 70);
      stopMotors();
      delay(500);
      if(firstButton)
      {
        states = RIDE_RIGHT_WALL;
        firstButton = false;
      }
      else
      {
        states = STOP;
      }
      break;
    }
    case STOP:
    {
      while(1)
      {
        void checkSleep();
        currentMillis = millis();
          jiggleForRound1();
        if (currentMillis - previousMillis >= interval)
        {
          printf("LRF 1 (cm): %f\tLRF 2 (cm): %f\n", getLrfDistanceCm(1), getLrfDistanceCm(2));
          printf("Current Angle: %f\n", getCurrentAngle());
          previousMillis = millis();
        }
      }

    }
  }
  // driveToHeading(0);
}

// int16_t findCardinalheading()
// {
//   float currentAngle = getCurrentAngle();
//   if (currentAngle >= 0 && currentAngle < 45 || currentAngle >= 315)
//   {
//     return 0; // North
//   }
//   else if (currentAngle >= 45 && currentAngle < 135)
//   {
//     return 90; // East
//   }
//   else if (currentAngle >= 135 && currentAngle < 225)
//   {
//     return 180; // South
//   }
//   else if (currentAngle >= 225 && currentAngle < 315)
//   {
//     return 270; // West
//   }
//   else
//   {
//     return -1; // Error
//   }
// }

// void jiggle()
// {
//   float lrf1;
//   float lrf2;
//   float currentHeading = getCurrentAngle();
//   uint16_t cardinalHeading = findCardinalheading();
//   float servoPosition = cardinalHeading - currentHeading; // Calculate the difference between the cardinal heading and the current heading
//   if (abs(servoPosition) > 90)
//   {
//     servoPosition = 359.99 - abs(servoPosition); // 359.99 is the max value for the servo
//   }

//   servoPosition += 90;                    // Add 90 because 90 degress is the middle position for the servo (therefore it is the current heading of the car)
//   myservo.write(servoPin, servoPosition); // tell servo to go to position in variable 'pos'
//   delay(15);                              // waits 15ms for the servo to reach the position
//   lrf1 = getLrfDistanceCm(1);
//   lrf2 = getLrfDistanceCm(2);
//   // Serial.print("LRF1: ");
//   // Serial.print(lrf1);
//   // Serial.print("\tLRF2: ");
//   // Serial.println(lrf2);
//   // findPosition(cardinalHeading, lrf1, lrf2); // Find the position of the car
// }