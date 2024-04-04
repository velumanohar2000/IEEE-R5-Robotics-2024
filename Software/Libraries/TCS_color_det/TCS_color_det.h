/*
 * Bit Bangers TCS34725 Color Detection Library
 * github.com/Bit-Bangers-UTA/Senior-Design
 *
 * Authors:
 * Rolando Rosales
 * 
 * Comments:
 * Requires adafruit/Adafruit TCS34725@^1.4.4 library
*/

#ifndef TCS_COLOR_DET_H
#define TCS_COLOR_DET_H

// Libraries ------------------------------------------------------------------

#include <Arduino.h>

// Defines --------------------------------------------------------------------

#define RED 0
#define ORANGE 1
#define YELLOW 2
#define GREEN 3
#define GRAY 4
#define BLACK 5
#define PURPLE 6
#define BLUE 7

// Variables & Constants ------------------------------------------------------

// Structures & Classes -------------------------------------------------------

// Functions ------------------------------------------------------------------

void initTCS(uint16_t r, uint16_t g, uint16_t b);
uint8_t getColorCode(void);

#endif