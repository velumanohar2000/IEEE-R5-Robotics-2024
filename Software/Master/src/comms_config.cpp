
#include "comms_config.h"


#define TF_MINI_S_I2C_ADDR  0x10 
#define CMD_SWITCH_TO_I2C   0x5A

void switch_tfmini_to_i2c() {
  uint8_t data[] = {CMD_SWITCH_TO_I2C, 0x05};
  Wire.beginTransmission(TF_MINI_S_I2C_ADDR);
  Wire.write(data, sizeof(data));
  Wire.endTransmission();

}
