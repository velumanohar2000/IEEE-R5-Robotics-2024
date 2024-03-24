/*
 * Displays text sent over the serial port (e.g. from the Serial Monitor) on
 * an attached LCD.
 * YWROBOT
 *Compatible with the Arduino IDE 1.0
 *Library version:1.1
 */
#include <Arduino.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

void setup()
{
  Wire.begin(9,8);
  lcd.init();                      // initialize the lcd 
  lcd.backlight();
}

void loop()
{
  lcd.clear(); 
  delay(1000);
  lcd.setCursor(0, 0);
  lcd.print("Noor is a");
  uint8_t i = 0;
  for (i = 0; i < 5; i++)
  {
    delay(350);
    lcd.print(".");
  }
  lcd.setCursor(0,1);
  lcd.print("******BITCH******");
  delay(1000);
}