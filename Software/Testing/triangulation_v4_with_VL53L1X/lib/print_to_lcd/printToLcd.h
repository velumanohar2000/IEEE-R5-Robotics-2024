/*
 * Bit Bangers printing to LCD Control Library
 * github.com/Bit-Bangers-UTA/Senior-Design
 *
 * Authors:
 * Velu Manohar
 * 
*/

// Libraries ------------------------------------------------------------------

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#ifndef PRINT_TO_LCD_H_
#define PRINT_TO_LCD_H_


void printToLcd(float line1Float, float line2Float);
void printToLcd(const char* line1String, float line2Float);
void printToLcd(float line1Float, const char* line2String);
void printToLcd(const char* line1String, const char* line2String);
void printToLcd(const char* line1String);
void printToLcd(float line1Float);


#endif