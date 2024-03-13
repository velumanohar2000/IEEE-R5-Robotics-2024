#include <Arduino.h>
#include <Wire.h>
// #include "SparkFun_VL53L1X.h" 

#define LIGHT 3
#define MOTORA_IN_1 3
#define MOTORA_IN_2 2
#define MOTORB_IN_3 6
#define MOTORB_IN_4 7
#define BACK_MOTOR 0

#define TIMER_INTERVAL_MS 5000
#define FORWARD 1
#define BACKWARD 0
#define RIGHT 1
#define LEFT 0
#define STOP 0

// #define MODE_TURN 1
// #define MODE_RAMP 1
#define MODE_HALF 1

bool direction = true;
int distance = 4000;
float distanceInches = 4000.0;

// SFEVL53L1X distanceSensor;

void move(bool direction)
{
  if(direction == FORWARD)
  {
    digitalWrite(MOTORA_IN_1, HIGH);
    digitalWrite(MOTORA_IN_2, LOW);
    digitalWrite(MOTORB_IN_3, HIGH);
    digitalWrite(MOTORB_IN_4, LOW);
    digitalWrite(BACK_MOTOR, HIGH);

  }
  else
  {
    digitalWrite(MOTORA_IN_1, LOW);
    digitalWrite(MOTORA_IN_2, HIGH);
    digitalWrite(MOTORB_IN_3, LOW);
    digitalWrite(MOTORB_IN_4, HIGH);
  }
}

void move(bool direction, uint16_t speed)
{
  if(direction == FORWARD)
  {
    analogWrite(MOTORA_IN_1, speed);
    analogWrite(MOTORA_IN_2, LOW);
    analogWrite(MOTORB_IN_3, speed);
    analogWrite(MOTORB_IN_4, LOW);

  }
  else
  {
    analogWrite(MOTORA_IN_1, LOW);
    analogWrite(MOTORA_IN_2, speed);
    analogWrite(MOTORB_IN_3, LOW);
    analogWrite(MOTORB_IN_4, speed);
  }
}

void stop()
{
  digitalWrite(MOTORA_IN_1, HIGH);
  digitalWrite(MOTORB_IN_3, HIGH);
  digitalWrite(MOTORA_IN_2, HIGH);
  digitalWrite(MOTORB_IN_4, HIGH);
  digitalWrite(BACK_MOTOR, LOW);
}

void stop(bool PWM)
{
  analogWrite(MOTORA_IN_1, 0);
  analogWrite(MOTORB_IN_3, 0);
  analogWrite(MOTORA_IN_2, 0);
  analogWrite(MOTORB_IN_4, 0);
}

void standby()
{
  digitalWrite(MOTORA_IN_1, LOW);
  digitalWrite(MOTORB_IN_3, LOW);
  digitalWrite(MOTORA_IN_2, LOW);
  digitalWrite(MOTORB_IN_4, LOW);
}


void initMotors() {
  pinMode(MOTORA_IN_1, OUTPUT);
  pinMode(MOTORA_IN_2, OUTPUT);
  pinMode(MOTORB_IN_3, OUTPUT);
  pinMode(MOTORB_IN_4, OUTPUT);
  pinMode(BACK_MOTOR, OUTPUT);
  // Wire.begin(9, 8);
  // Serial.begin(115200);
  // if (distanceSensor.begin() != 0) //Begin returns 0 on a good init
  // {
  //   Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
  //   while (1)
  //     ;
  // }
  // Serial.println("Sensor online!");

}

void turn(bool direction)
{
  if(direction == RIGHT)
  {
    digitalWrite(MOTORA_IN_1, HIGH);
    digitalWrite(MOTORA_IN_2, LOW);
    digitalWrite(MOTORB_IN_3, LOW);
    digitalWrite(MOTORB_IN_4, HIGH);
  }
  else
  {
    digitalWrite(MOTORA_IN_1, LOW);
    digitalWrite(MOTORA_IN_2, HIGH);
    digitalWrite(MOTORB_IN_3, HIGH);
    digitalWrite(MOTORB_IN_4, LOW);
  }
}

// void loop() {

//   // #ifdef MODE_HALF
//   // if(distanceInches > 10.0)
//   //   move(FORWARD);
//   // else
//   //   stop();

//   // distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
//   // while (!distanceSensor.checkForDataReady())
//   // {
//   //   delay(1);
//   // }
//   // distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
//   // distanceSensor.clearInterrupt();
//   // distanceSensor.stopRanging();
//   // distanceInches = distance * 0.0393701;
//   // Serial.printf("Distance (in): %f\n", distanceInches);
  
//   // if(distanceInches < 4.0)
//   // {
//   //   stop(1);
//   //   while(1);
//   // }
//   // #endif

//   #ifdef MODE_RAMP
//     static uint16_t i = 0;
//     for(i = 100; i < 256; i++)
//     {
//       move(FORWARD, i);
//       delay(50);
//     }
//     stop();
//     delay(5000);
//     for(i = 100; i < 256; i++)
//     {
//       move(BACKWARD, i);
//       delay(50);
//     }
//     stop();
//     delay(5000);
//   #endif

//   #ifdef MODE_TURN
//     turn(RIGHT);
//     delay(1000);
//     standby();
//     delay(100);
//     turn(LEFT);
//     delay(1000);
//     standby();
//     delay(100);

//   #endif
// }
