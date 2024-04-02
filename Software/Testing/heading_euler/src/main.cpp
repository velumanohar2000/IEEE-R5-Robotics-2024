#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include "math.h"
#include "BNO085_heading.h"

/*
* Make sure to use pull up resistors!!!
* I am using 1k resitors in this demo
*/


#define BNO08X_RESET -1


Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

void setup()
{
  Serial.begin(115200);
  Serial.println("Adafruit BNO08x test!");
  Wire.begin(20, 21);
  setupBNO085(&bno08x);
}


void loop()
{
 // checkReset(&bno08x);

  if (bno08x.getSensorEvent(&sensorValue))
  {
    switch (sensorValue.sensorId)
    {
    case SH2_LINEAR_ACCELERATION:
    {
      //TODO adjust reports function
      Serial.print("Acceleration (x): ");
      float xAccel = sensorValue.un.linearAcceleration.x;
      Serial.println(xAccel);
      break;
    }
    case SH2_ARVR_STABILIZED_RV:
    {
      float heading = calculateHeading(sensorValue.un.arvrStabilizedRV.i, sensorValue.un.arvrStabilizedRV.j, sensorValue.un.arvrStabilizedRV.k, sensorValue.un.arvrStabilizedRV.real);
      Serial.print("Heading: ");
      Serial.println(heading);
      break;
    }
    }
  }
  delay(100);
}
