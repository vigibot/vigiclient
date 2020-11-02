#include <math.h>
#include "sin16.hpp"

int16_t sin16(uint16_t angle) {
 return sinTable16[angle] - ONE16;
}

int16_t cos16(uint16_t angle) {
 return sin16(HALFPI16 - angle);
}

uint16_t angleFloatToAngle16(float angle) {
 return int(angle * float(PI16) / M_PI);
}

float angle16ToAngleFloat(int16_t angle) {
 return float(angle) * M_PI / float(PI16);
}

float un16ToUnFloat(int16_t i16) {
 return float(i16) / float(ONE16);
}

float sinFloat(float angle) {
 //return sin(angle);
 return un16ToUnFloat(sin16(angleFloatToAngle16(angle)));
}

float cosFloat(float angle) {
 //return cos(angle);
 return un16ToUnFloat(cos16(angleFloatToAngle16(angle)));
}

int32_t tanQ16(uint16_t angle) {
 if(angle > PI16)
  angle -= PI16;
 if(angle <= HALFPI16 && angle > HALFPI16 - 3)
  return 2147483647;
 else if(angle > HALFPI16 && angle < HALFPI16 + 3)
  return -2147483648;
 else
  return sin16(angle) * ONE16 / cos16(angle);
}
