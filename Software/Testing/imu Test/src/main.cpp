#include <Arduino.h>
#include "SparkFun_BNO08x_Arduino_Library.h"  // CTRL+Click here to get the library: http://librarymanager/All#SparkFun_BNO08x
BNO08x myIMU;

//#define BNO08X_INT  A4
#define BNO08X_INT  -1
//#define BNO08X_RST  A5
#define BNO08X_RST  -1
#define BNO08X_ADDR 0x4B 
// In the documetation it says I2C protocols are broken and may not work well with ESPRESSIF ESP32

void setReports(void) {
  Serial.println("Setting desired reports");
  if (myIMU.enableRotationVector() == true) {
    Serial.println(F("Rotation vector enabled"));
    Serial.println(F("Output in form i, j, k, real, accuracy"));
  } else {
    Serial.println("Could not enable rotation vector");
  }
}
void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  while(!Serial) delay(10); // Wait for Serial to become available.
  // Necessary for boards with native USB (like the SAMD51 Thing+).
  // For a final version of a project that does not need serial debug (or a USB cable plugged in),
  // Comment out this while loop, or it will prevent the remaining code from running.
  
  Serial.println();
  Serial.println("BNO08x Acceleration and Heading Example");

  Wire.begin(9,8);

  // if (myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST) == false) {
  //   Serial.println("BNO08x not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
  //   while (1)
  //     ;
  //}
  Serial.println("BNO08x found!");

  setReports();

  Serial.println("Reading events");
  delay(100);
}



void loop() {
  digitalWrite(LED_BUILTIN,~digitalRead(BUILTIN_LED));
  delay(1000);
}

