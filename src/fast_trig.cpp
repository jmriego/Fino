#include "fast_trig.h"
#include <EEPROM.h>


float fast_sin(uint8_t angle) {
  int16_t val;
  EEPROM.get(angle*2, val);
  return val / 65535.0;
  // float radians = (angle) * 360.0 / 255.0 * DEG_TO_RAD;
  // return sin(radians);
}

float fast_cos(uint8_t angle) {
  // return fast_cos((angle + 180)%360);
  return fast_cos((angle + 64)%256);
}
