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
// int distance = 4000;
// float distanceInches = 4000.0;

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

/*
  basic idea for PWM, instead of going full speed, provide a param to control speed
*/

void turn(bool direction, uint16_t speed)
{
  if(direction == RIGHT)
  {
    analogWrite(MOTORA_IN_1, speed);
    analogWrite(MOTORA_IN_2, 0);
    analogWrite(MOTORB_IN_3, 0);
    analogWrite(MOTORB_IN_4, speed);
  }
  else
  {
    analogWrite(MOTORA_IN_1, 0);
    analogWrite(MOTORA_IN_2, speed);
    analogWrite(MOTORB_IN_3, speed);
    analogWrite(MOTORB_IN_4, 0);
  }
}

void turn(bool direction, uint16_t speed, uint16_t currentAngle, uint16_t requiredAngle)
{
  /*
  coded this without testing. Idea behind is that when were turning, we will never get the exact angle unless we use interrupts
  so for now, i multiplied the angles with a large value so we have some room for error. We keep turning as long as we're within that error range.

  After coding this i realized that we need interrupts since having slight error in angle would completely change the vector.
  If we're planning on using the same while code then we need to turn very slowly
  */
  uint64_t currentAngleError = currentAngle * 1000;
  uint64_t requiredAngleError = requiredAngle * 1000;
  while((currentAngleError > (requiredAngleError+50)) ||  (currentAngleError < (requiredAngleError-50)))
  {
    if(direction == RIGHT)
    {
      digitalWrite(MOTORA_IN_1, speed);
      digitalWrite(MOTORA_IN_2, 0);
      digitalWrite(MOTORB_IN_3, 0);
      digitalWrite(MOTORB_IN_4, speed);
    }
    else
    {
      digitalWrite(MOTORA_IN_1, 0);
      digitalWrite(MOTORA_IN_2, speed);
      digitalWrite(MOTORB_IN_3, speed);
      digitalWrite(MOTORB_IN_4, 0);
    }
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
