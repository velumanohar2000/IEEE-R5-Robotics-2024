
#include <TCS_color_det.h>
#include <Arduino.h>
#include <Wire.h>
// Ambient color values
#define R_AMB 7
#define G_AMB 6
#define B_AMB 3

#define NUMBER_OF_WAYPOINTS 8

// #define startingStation 2 // intgrate with color sensor
//  Define a struct for the stationWaypoints
// typedef struct
// {
//   char name;
//   uint8_t original_x, original_y;
//   uint8_t new_x, new_y;
//   uint8_t nextWaypoint;

// } Waypoint;

// // Initialize stationWaypoints
// Waypoint stationWaypoints[] = {
//     {'A', 2, 0, 0, 0, 3}, {'B', 6, 0, 0, 0, 6}, {'C', 8, 2, 0, 0, 0}, {'D', 8, 6, 0, 0, 7}, {'E', 6, 8, 0, 0, 2}, {'F', 2, 8, 0, 0, 1}, {'G', 0, 6, 0, 0, 4}, {'H', 0, 2, 0, 0, 5}};

// // Function to transpose stationWaypoints
// void transposestationWaypoints(uint8_t startingStation)
// {
//   uint8_t traverseNum;

//   if (startingStation > 1) // if starting at 0 or 1 there is no need to transpose
//   {
//     if (startingStation % 2 == 0)
//     {
//       // even number
//       traverseNum = startingStation;
//       //    printf("Even Number\n");
//     }
//     else
//     {
//       traverseNum = startingStation - 1;
//     }
//   }
//   uint8_t i = 0;
//   uint8_t index;
//   int8_t copyIndex = 0;
//   for (i = 0; i < NUMBER_OF_WAYPOINTS; i++)
//   {
//     index = startingStation + i;
//     if (index > 7)
//     {
//       index = index - NUMBER_OF_WAYPOINTS;
//     }
//     // printf("Index: %d\n", index);

//     copyIndex = index - traverseNum;
//     if (copyIndex < 0)
//     {
//       copyIndex = NUMBER_OF_WAYPOINTS - abs(copyIndex);
//     }
//     //    printf("Copy Index: %d\n", copyIndex);

//     stationWaypoints[index].new_x = stationWaypoints[copyIndex].original_x;
//     stationWaypoints[index].new_y = stationWaypoints[copyIndex].original_y;
//     // printf("%c: (%d, %d)\n", stationWaypoints[i].name, stationWaypoints[i].new_x, stationWaypoints[i].new_x);
//   }
// }

// // Function to loop through stationWaypoints
// void loopstationWaypoints(uint8_t startingStation)
// {
//   uint8_t i = 0;
//   uint8_t index;
//   printf("Starting Location: %c: (%d, %d) Next Waypoint: %d\n",
//          stationWaypoints[startingStation].name,
//          stationWaypoints[startingStation].new_x,
//          stationWaypoints[startingStation].new_y, stationWaypoints[startingStation].nextWaypoint);

//   uint8_t station = startingStation;
//   uint8_t nextStation = stationWaypoints[startingStation].nextWaypoint;

//   for (i = 0; i < NUMBER_OF_WAYPOINTS; i++)
//   {

//     printf("%c: (%d, %d)\n", stationWaypoints[nextStation].name, stationWaypoints[nextStation].new_x,
//            stationWaypoints[nextStation].new_y);

//     nextStation = stationWaypoints[nextStation].nextWaypoint;
//   }
// }

void setup()
{
  Serial.begin(115200);
  Wire.begin(7, 6);
  initTCS(R_AMB, G_AMB, B_AMB);
  uint8_t i = 0;
  uint8_t startColorIndex;
  uint32_t startColorArray[50];
  delay(3000);
  for (i = 0; i < 50; i++)
  {
    startColorArray[i] = getColorCode();
  }

  // transposestationWaypoints(startColorIndex);
  // loopstationWaypoints(startColorIndex);
}
void loop()
{
  printf("Color Read: %d\n", getColorCode());

  // while (1)
  //   ;
}
