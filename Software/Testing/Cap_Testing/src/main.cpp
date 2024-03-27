#include <Arduino.h>

#define MOTORA_IN_1 2
#define MOTORA_IN_2 3
#define MOTORB_IN_3 1
#define MOTORB_IN_4 0

void setup() {
  Serial.begin(115200);
  pinMode(MOTORA_IN_1, OUTPUT);
  pinMode(MOTORA_IN_2, OUTPUT);
  pinMode(MOTORB_IN_3, OUTPUT);
  pinMode(MOTORB_IN_4, OUTPUT);

}
void loop() {
  uint8_t speed = 255;
  analogWrite(MOTORA_IN_1, speed);
  analogWrite(MOTORA_IN_2, 0);
  analogWrite(MOTORB_IN_3, speed);
  analogWrite(MOTORB_IN_4, 0);
}
