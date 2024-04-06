#include <Adafruit_BNO08x.h>

#define BNO08X_RESET -1

void setReports(Adafruit_BNO08x *bno08x, sh2_SensorId_t reportType, uint32_t interval);
void reports(Adafruit_BNO08x *bno08x);
void setupBNO085(Adafruit_BNO08x *bno08x);
void checkReset(Adafruit_BNO08x *bno08x);
float calculateHeading(float i, float j, float k, float real);
float getHeading();
float getCurrentAngle();
void printCurrentAngle();
// void turnToHeading(float goal, uint8_t speed);
// void driveToHeading(float goalHeading);