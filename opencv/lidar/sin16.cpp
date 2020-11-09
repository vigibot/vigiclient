#include <math.h>
#include "sin16.hpp"

int16_t sin16(uint16_t angle) {
 return sinTable16[angle] - ONE16;
}

int16_t cos16(uint16_t angle) {
 return sin16(HALFPI16 - angle);
}

uint16_t angleDoubleToAngle16(double angle) {
 return int(angle * double(PI16) / M_PI);
}

double angle16ToAngleDouble(int16_t angle) {
 return double(angle) * M_PI / double(PI16);
}

double un16ToUnDouble(int16_t i16) {
 return double(i16) / double(ONE16);
}

double sinDouble(double angle) {
 //return sin(angle);
 return un16ToUnDouble(sin16(angleDoubleToAngle16(angle)));
}

double cosDouble(double angle) {
 //return cos(angle);
 return un16ToUnDouble(cos16(angleDoubleToAngle16(angle)));
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
