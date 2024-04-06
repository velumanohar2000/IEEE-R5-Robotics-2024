
#include <Arduino.h>
#include <Wire.h>
// #include <SPI.h>
// #include "Adafruit_TCS34725.h"
#include <TCS_color_det.h>

// Ambient color values
#define R_AMB 7
#define G_AMB 6
#define B_AMB 3

#define NUMBER_OF_WAYPOINTS 8
#define SIZE_OF_COLOR_SENOR_ARRAY 100

// #define startingStation 2 // intgrate with color sensor
//  Define a struct for the stationWaypoints
typedef struct
{
  char name;
  uint8_t original_x, original_y;
  uint8_t new_x, new_y;
  uint8_t nextWaypoint;

} Waypoint;

// Initialize stationWaypoints
Waypoint stationWaypoints[] = {
    {'A', 2, 0, 0, 0, 3}, {'B', 6, 0, 0, 0, 6}, {'C', 8, 2, 0, 0, 0}, {'D', 8, 6, 0, 0, 7}, {'E', 6, 8, 0, 0, 2}, {'F', 2, 8, 0, 0, 1}, {'G', 0, 6, 0, 0, 4}, {'H', 0, 2, 0, 0, 5}};

// Function to transpose stationWaypoints
void transposeStationWaypoints(uint8_t startingStation)
{
  uint8_t traverseNum;

  if (startingStation > 1) // if starting at 0 or 1 there is no need to transpose
  {
    if (startingStation % 2 == 0)
    {
      // even number
      traverseNum = startingStation;
      //    printf("Even Number\n");
    }
    else
    {
      traverseNum = startingStation - 1;
    }
  }
  uint8_t i = 0;
  uint8_t index;
  int8_t copyIndex = 0;
  for (i = 0; i < NUMBER_OF_WAYPOINTS; i++)
  {
    index = startingStation + i;
    if (index > 7)
    {
      index = index - NUMBER_OF_WAYPOINTS;
    }
    // printf("Index: %d\n", index);

    copyIndex = index - traverseNum;
    if (copyIndex < 0)
    {
      copyIndex = NUMBER_OF_WAYPOINTS - abs(copyIndex);
    }
    //    printf("Copy Index: %d\n", copyIndex);

    stationWaypoints[index].new_x = stationWaypoints[copyIndex].original_x;
    stationWaypoints[index].new_y = stationWaypoints[copyIndex].original_y;
    // printf("%c: (%d, %d)\n", stationWaypoints[i].name, stationWaypoints[i].new_x, stationWaypoints[i].new_x);
  }
}

// Function to loop through stationWaypoints
void loopStationWaypoints(uint8_t startingStation)
{
  uint8_t i = 0;
  uint8_t index;
  printf("Starting Location: %c: (%d, %d) Next Waypoint: %d\n",
         stationWaypoints[startingStation].name,
         stationWaypoints[startingStation].new_x,
         stationWaypoints[startingStation].new_y, stationWaypoints[startingStation].nextWaypoint);

  uint8_t station = startingStation;
  uint8_t nextStation = stationWaypoints[startingStation].nextWaypoint;

  for (i = 0; i < NUMBER_OF_WAYPOINTS; i++)
  {

    printf("%c: (%d, %d)\n", stationWaypoints[nextStation].name, stationWaypoints[nextStation].new_x,
           stationWaypoints[nextStation].new_y);

    nextStation = stationWaypoints[nextStation].nextWaypoint;
  }
}

uint8_t findMode(uint16_t arr[], uint8_t n)
{
  uint8_t maxCount = 0;
  uint8_t mode = arr[0];
  uint8_t i = 0;
  uint8_t j = 0;
  for (i = 0; i < n; i++)
  {
    uint8_t count = 0;
    for (j = 0; j < n; j++)
    {
      if (arr[j] == arr[i])
        count++;
    }
    if (count > maxCount)
    {
      maxCount = count;
      mode = arr[i];
    }
  }
  return mode;
}

void getRoute(uint16_t startColorArray[])
{
  uint16_t i = 0;
  for (i = 0; i < SIZE_OF_COLOR_SENOR_ARRAY; i++)
  {
    startColorArray[i] = getColorCode();
    printf("Color Read: %d\n", getColorCode());
  }
  // uint16_t n = sizeof(startColorArray) / sizeof(startColorArray[0]);
  uint8_t startColorIndex = findMode(startColorArray, SIZE_OF_COLOR_SENOR_ARRAY);
  printf("Start Color Index: %d\n", startColorIndex);
  transposeStationWaypoints(startColorIndex);
  loopStationWaypoints(startColorIndex);

}

void setup()
{
  Serial.begin(115200);
  Wire.begin(7, 6);
  initTCS(R_AMB, G_AMB, B_AMB, 0x29, &Wire);
  uint16_t startColorArray[SIZE_OF_COLOR_SENOR_ARRAY];
  delay(3000);
  getRoute(startColorArray);
  // for (i = 0; i < SIZE_OF_COLOR_SENOR_ARRAY; i++)
  // {
  //   startColorArray[i] = getColorCode();
  //   printf("Color Read: %d\n", getColorCode());
  // }
  // // uint16_t n = sizeof(startColorArray) / sizeof(startColorArray[0]);
  // uint8_t startColorIndex = findMode(startColorArray, SIZE_OF_COLOR_SENOR_ARRAY);
  // printf("Start Color Index: %d\n", startColorIndex);

  //transposeStationWaypoints(startColorIndex);
}
void loop()
{
  // printf("Color Read: %d\n", getColorCode());

  while (1)
    ;
}
