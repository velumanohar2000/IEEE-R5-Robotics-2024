#include <Arduino.h>
#include "Wire.h"
#include "Adafruit_BNO08x.h"
#include "math.h"

#define BNO08X_RESET -1

Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

float xAccel;


void setReports(sh2_SensorId_t reportType, uint32_t interval)
{
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(reportType, interval)) // Top frequency is about 250Hz but this report is more accurate
  {
    Serial.println("Could not enable stabilized remote vector");
  }
}
void reports()
{
  setReports(SH2_LINEAR_ACCELERATION, 2000);
  setReports(SH2_MAGNETIC_FIELD_CALIBRATED, 1000);
}
void setup(void)
{

  Serial.begin(115200);

  Serial.println("Adafruit BNO08x test!");

  Wire.begin(9, 8);
  if (!bno08x.begin_I2C())
  {
    Serial.println("Failed to find BNO08x chip");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("BNO08x Found!");

  reports();

  Serial.println("Reading events");
  delay(1000);
}

// Constants for compass calibration
float offsetX = 0.0; // Offset along the x-axis
float offsetY = 0.0; // Offset along the y-axis
float magX;
float magY;
float heading;
// Function to calculate compass heading
float calculateHeading(float x, float y)
{
  // Apply calibration offsets
  x -= offsetX;
  y -= offsetY;

  // Calculate heading angle
  float heading = atan2(y, x) * 180.0 / M_PI;

  // Ensure the angle is within [0, 360) range
  if (heading < 0)
  {
    heading += 360.0;
  }

  return heading;
}

void loop()
{
  // Read magnetic field strength along x and y axes
  if (bno08x.wasReset())
  {
    Serial.print("sensor was reset ");
    reports();
  }
  if (bno08x.getSensorEvent(&sensorValue))
  {
    switch (sensorValue.sensorId)
    {
    case SH2_MAGNETIC_FIELD_CALIBRATED:
    {
      magX = sensorValue.un.magneticField.x; 
      magY = sensorValue.un.magneticField.y; 
    }
    }
  

  // Calculate compass heading
  heading = calculateHeading(magX, magY);

  // Print heading
  Serial.print("Heading: ");
  Serial.println(heading);
  }
  delay(100); 
}
