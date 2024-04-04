#include <stdio.h>
#include <stdlib.h>

#include <stdbool.h>
#include <inttypes.h>

#define NUMBER_OF_WAYPOINTS 8



#define OUTPUT_FROM_COLOR_SENSOR 2	// intgrate with color sensor
// Define a struct for the waypoints
typedef struct
{
  char name;
  uint8_t original_x, original_y;
  uint8_t new_x, new_y;
  uint8_t nextWaypoint;

} Waypoint;

// Initialize waypoints
Waypoint waypoints[] = {
  {'A', 2, 0, 0, 0, 3}, {'B', 6, 0, 0, 0, 6}, {'C', 8, 2, 0, 0, 0}, {'D', 8, 6, 0, 0, 7},
	{'E', 6, 8, 0, 0, 2}, {'F', 2, 8, 0, 0, 1}, {'G', 0, 6, 0, 0, 4}, {'H', 0, 2, 0, 0, 5}
};

// Function to transpose waypoints
void
transposeWaypoints ()
{
  uint8_t traverseNum;

  if (OUTPUT_FROM_COLOR_SENSOR > 1)	// if starting at 0 or 1 there is no need to transpose
	{
	  if (OUTPUT_FROM_COLOR_SENSOR % 2 == 0)
		{
		  // even number
		  traverseNum = OUTPUT_FROM_COLOR_SENSOR;
		  //    printf("Even Number\n");
		}
	  else
		{
		  traverseNum = OUTPUT_FROM_COLOR_SENSOR - 1;
		}
	}
  uint8_t i = 0;
  uint8_t index;
  int8_t copyIndex = 0;
  for (i = 0; i < NUMBER_OF_WAYPOINTS; i++)
	{
	  index = OUTPUT_FROM_COLOR_SENSOR + i;
	  if (index > 7)
		{
		  index = index - NUMBER_OF_WAYPOINTS;
		}
	  // printf("Index: %d\n", index);

	  copyIndex = index - traverseNum;
	  if (copyIndex < 0)
		{
		  copyIndex = NUMBER_OF_WAYPOINTS - abs (copyIndex);
		}
	  //    printf("Copy Index: %d\n", copyIndex);


	  waypoints[index].new_x = waypoints[copyIndex].original_x;
	  waypoints[index].new_y = waypoints[copyIndex].original_y;
	  //printf("%c: (%d, %d)\n", waypoints[i].name, waypoints[i].new_x, waypoints[i].new_x);

	}
}

// Function to loop through waypoints
void
loopWaypoints ()
{
  uint8_t i = 0;
  uint8_t index;
  printf ("Starting Location: %c: (%d, %d) Next Waypoint: %d\n",
		  waypoints[OUTPUT_FROM_COLOR_SENSOR].name,
		  waypoints[OUTPUT_FROM_COLOR_SENSOR].new_x,
		  waypoints[OUTPUT_FROM_COLOR_SENSOR].new_y, waypoints[OUTPUT_FROM_COLOR_SENSOR].nextWaypoint);
    
    uint8_t station = OUTPUT_FROM_COLOR_SENSOR;
    uint8_t nextStation = waypoints[OUTPUT_FROM_COLOR_SENSOR].nextWaypoint;
    
  for (i = 0; i < NUMBER_OF_WAYPOINTS; i++)
	{
		
	  printf ("%c: (%d, %d)\n", waypoints[nextStation].name, waypoints[nextStation].new_x,
			  waypoints[nextStation].new_y);
			  
	    nextStation = waypoints[nextStation].nextWaypoint;
	}
	
	

}

// void setup()
// {
//   Serial.begin(115200);
//   Wire.begin(9, 8);
// }

// void loop()
// {
// }

int
main ()
{

  // Transpose waypoints to the fixed new starting position
  transposeWaypoints ();

  // Loop through waypoints
  loopWaypoints ();

  return 0;
}
