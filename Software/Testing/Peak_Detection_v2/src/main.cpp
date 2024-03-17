
#include "Arduino.h"
#include <ESP32Servo.h>
#include "lrf.h"
#include "Wire.h"
#include "math.h"
Servo myservo; // create servo object to control a servo

int pos = -2; // variable to store the servo position
int servoPin = 2;

#define LRF_ADDRESS_1 0x10

#define BUFFER_SIZE 20

bool rotateTo180 = true;

int32_t distance[200] = {0};
uint16_t distIndex = 0;

uint16_t inflectionPos;

uint8_t maximaCounter = 0;
uint8_t minimaCounter = 0;

uint8_t maximaValues[4] = {0};
uint8_t minimaValues[4] = {0};

uint16_t possiblePeak;
uint16_t possibleValley;

uint8_t negativeSlopeCounter = 0;
uint8_t positiveSlopeCounter = 0;

bool startNegativeCounter = false;
bool startPositiveCounter = false;

float x;
float y;

void setup()
{
  Serial.begin(115200);
  Wire.begin(9, 8);
  myservo.setPeriodHertz(50); // standard 50 hz servo
  myservo.attach(servoPin);   // using default min/max of 1000us and 2000us
}
void findLocationMinMax()
{
  y = minimaValues[1];
  float c = maximaValues[0];

  // Calculate x coordinate using pythagorean theorem
  x = sqrt(pow(c, 2) - pow(y, 2));
}

void findLocationMaxMax()
{
  float a = maximaValues[0];
  float b = maximaValues[1];
  float c = 243.84;

  // Calculate semi-perimeter
  float s = (a + b + c) / 2;
  // Calculate area using Heron's formula
  float area = sqrt(s * (s - a) * (s - b) * (s - c));
  // Calculate y coordinate given the area and base
  y = (2 * area) / c;
  // Calculate x coordinate using pythagorean theorem
  x = sqrt(pow(a, 2) - pow(y, 2));
}

void findLocalMinimaAndMaxima(int32_t arr[], int n, int neighborhood)
{

  uint32_t i = 0;
  while (i < n - 1)
  {
    if (arr[i + 1] - arr[i] > 0)
    {
      // slope is postiive or equal to zero
      while (arr[i + 1] - arr[i] >= 0)
      {
        i++;
        if (startPositiveCounter)
        {
          positiveSlopeCounter++;
          if (positiveSlopeCounter == 2) // for noisy data
          {
            // Valley Detected
            printf("Local Minima found at index %d, value: %d\n", i, arr[i]);
            myservo.write(i * 2);
            minimaValues[minimaCounter] = arr[possibleValley];
            minimaCounter++;
            delay(2000);
            startPositiveCounter = false;
            positiveSlopeCounter = 0;
          }
        }
      }
      if (!startPositiveCounter)
      {
        possiblePeak = i;
        startNegativeCounter = true;
        negativeSlopeCounter = 0;
      }
      else
      {
        startNegativeCounter = false;
        negativeSlopeCounter = 0;
      }
    }
    else if ((arr[i + 1] - arr[i] < 0))
    {
      // slope is negative
      while (arr[i + 1] - arr[i] <= 0)
      {
        i++;
        if (startNegativeCounter)
        {
          negativeSlopeCounter++;
          if (negativeSlopeCounter == 2) // for noisy data
          {
            // Peak Detected
            printf("Local Maxima found at index %d, value: %d\n", i, arr[i]);
            myservo.write(i * 2);
            maximaValues[maximaCounter] = arr[possiblePeak];
            maximaCounter++;
            delay(2000);
            startNegativeCounter = false;
            negativeSlopeCounter = 0;
          }
        }
      }
      if (!startNegativeCounter)
      {
        possibleValley = i;
        startPositiveCounter = true;
        positiveSlopeCounter = 0;
      }
      else
      {
        startNegativeCounter = false;
        negativeSlopeCounter = 0;
      }
    }
    else
    {
      i++;
    }
  }
}

void loop()
{
  // rotating to 0 to 180, getting data values and storing them into an array
  if (rotateTo180)
  {
    pos += 2;
    if (pos > 180)
    {
      rotateTo180 = false;
      int n = sizeof(distance) / sizeof(distance[0]);
      int neighborhood = 30; // You can adjust this to be larger or smaller

      findLocalMinimaAndMaxima(distance, n, neighborhood);
      if (maximaCounter == 2 && minimaCounter == 2)
      {

        findLocationMinMax();
        printf("location calculated with 1 max and 1 min value is:\nx: %f cm (%f ft)\ny: %f cm (%f ft)\n", x, x / (2.54 * 12), y, y / (2.54 * 12));

        findLocationMaxMax();
        printf("location calculated with 2 max valeus is:\nx: %f cm (%f ft)\ny: %f cm (%f ft)\n", x, x / (2.54 * 12), y, y / (2.54 * 12));
      }
      else
      {
        Serial.println("Incorrect number of Max and min values, expecting 2 min and and 2 max");
        printf("maxima counter: %d\nminima counter: %d\n", maximaCounter, minimaCounter);

      }
    }
    else
    {
      myservo.write((int)pos); // tell servo to go to position in variable 'pos'
      distance[distIndex] = getLrfDistance(LRF_ADDRESS_1);

      // Do some basic filtering, making sure it is less than the max distance of diagonl (344 cm)
      if (distIndex > 5)
      {
        if (distance[distIndex] < 0 || distance[distIndex] > 345)
        {
          distance[distIndex] = distance[distIndex - 1];
        }
        // if (abs(distance[distIndex] - distance[distIndex - 1]) > 50)
        //   distance[distIndex] = distance[distIndex - 1];
      }
      // printf("%d %d\n", distance[distIndex], pos);
      distIndex++;
    }
  }
  else
  {
    delay(1000);
    pos = 0;
    myservo.write((int)pos); // tell servo to go to position in variable 'pos'
    delay(1000);
    pos = -2;
    rotateTo180 = true;
    distIndex = 0;
    maximaCounter = 0;
    minimaCounter = 0;
  }
}
