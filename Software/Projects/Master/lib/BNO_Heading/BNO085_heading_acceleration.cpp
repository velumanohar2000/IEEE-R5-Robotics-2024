#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include "math.h"
#include "BNO085_heading_acceleration.h"


void setReports(Adafruit_BNO08x *bno08x, sh2_SensorId_t reportType, uint32_t interval)
{
  // Serial.println("Setting desired reports");
  if (!bno08x->enableReport(reportType, interval)) // Top frequency is about 250Hz but this report is more accurate
  {
    // Serial.println("Could not enable stabilized remote vector");
  }
}

void reports(Adafruit_BNO08x *bno08x)
{
  // setReports(bno08x, SH2_LINEAR_ACCELERATION, 500);
  // setReports(bno08x, SH2_ARVR_STABILIZED_RV, 1000);
  setReports(bno08x, SH2_GYROSCOPE_CALIBRATED, 2500);
}


void setupBNO085(Adafruit_BNO08x *bno08x) 
{
  // Serial.begin(115200);
  // Serial.println("Adafruit BNO08x test!");
  // Wire.begin(9, 8);
  if (!bno08x->begin_I2C())
  {
    // Serial.println("Failed to find BNO08x chip");
    while (1)
    {
      delay(10);
    }
  }
  // Serial.println("BNO08x Found!");
  reports(bno08x);
  // Serial.println("Reading events");
  delay(1000);
}

float calculateHeading(float i, float j, float k, float real)
{
  float yaw = atan2(2.0f*(i*j+real*k),real*real+i*i-j*j-k*k);
  yaw *= 180.0f / M_PI;

  // Normalize heading angle
  if (yaw < 0) {
    yaw += 360.0f;
  }
  return yaw;
}

void checkReset(Adafruit_BNO08x *bno08x)
{
  if (bno08x->wasReset())
  {
    // Serial.print("sensor was reset ");
    reports(bno08x);
  }
}
