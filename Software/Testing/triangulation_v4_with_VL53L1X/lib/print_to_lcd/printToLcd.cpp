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

extern LiquidCrystal_I2C lcd;

void printToLcd(float line1Float, float line2Float)
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(line1Float);
    lcd.setCursor(0, 1);
    lcd.print(line2Float);
}

void printToLcd(const char* line1String, float line2Float)
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(line1String);
    lcd.setCursor(0, 1);
    lcd.print(line2Float);
}

void printToLcd(float line1Float, const char* line2String)
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(line1Float);
    lcd.setCursor(0, 1);
    lcd.print(line2String);
}

void printToLcd(const char* line1String, const char* line2String)
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(line1String);
    lcd.setCursor(0, 1);
    lcd.print(line2String);
}

void printToLcd(const char* line1String)
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(line1String);
}

void printToLcd(float line1Float)
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(line1Float);
}

/*
 * With staring position
 */

// void printToLcd(float line1Float, float line2Float, uint8_t posLine1, uint8_t posLine2)
//  {

// }

// void printToLcd(char* line1String, float line2Float, uint8_t posLine1, uint8_t posLine2)
// {

// }

// void printToLcd(float line1Float, char* line2String, uint8_t posLine1, uint8_t posLine2)
// {

// }

// void printToLcd(char* line1String, char* line2String, uint8_t posLine1, uint8_t posLine2)
// {

// }
