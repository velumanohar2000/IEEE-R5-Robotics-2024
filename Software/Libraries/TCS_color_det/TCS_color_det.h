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

#define RED 1
#define ORANGE 2
#define YELLOW 3
#define GREEN 4
#define GRAY 5
#define BLACK 6
#define PURPLE 7
#define BLUE 8

// Variables & Constants ------------------------------------------------------

// Structures & Classes -------------------------------------------------------

// Functions ------------------------------------------------------------------

void initTCS(uint16_t r, uint16_t g, uint16_t b);
uint8_t getColorCode(void);

#endif