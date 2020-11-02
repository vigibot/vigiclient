#include <wiringSerial.h>
#include <vector>
#include "lidars.hpp"

#ifdef LDLIDAR
void startLidar(int ld) {
}

void stopLidar(int ld) {
}

bool readLidar(int ld, std::vector<PointPolar> &pointsOut) {
 uint8_t current;
 static uint8_t n = 0;
 static uint8_t o = 0;
 static uint8_t p = 0;
 static uint16_t motorSpeed;
 static uint16_t startAngle;
 static uint16_t distances[NBMEASURESPACK];
 static uint8_t confidences[NBMEASURESPACK];
 static uint16_t endAngle;
 static uint16_t timestamp;
 static uint8_t crc = 0;
 static uint8_t packs = 0;
 static std::vector<PointPolar> points;
 bool done = false;

 while(serialDataAvail(ld)) {
  current = serialGetchar(ld);

  if(n < 46)
   crc = LDCRC[crc ^ current];

  switch(n) {

   case 0:
    if(current == 0x54)
     n = 1;
    break;

   case 1:
    if(current == 0x2c)
     n = 2;
    else
     n = 0;
    break;

   case 2:
    motorSpeed = current;
    n = 3;
    break;

   case 3:
    motorSpeed |= current << 8;
    n = 4;
    break;

   case 4:
    startAngle = current;
    n = 5;
    break;

   case 5:
    startAngle |= current << 8;
    n = 6;
    break;

   default:
    switch(o) {
     case 0:
      distances[p] = current;
      o = 1;
      break;
     case 1:
      distances[p] |= current << 8;
      o = 2;
      break;
     case 2:
      confidences[p] = current;
      o = 0;
      p++;
      break;
    }
    n++;
    break;

   case 42: // 6 + NBMEASURESPACK * 3
    endAngle = current;
    n = 43;
    break;

   case 43: {
    endAngle |= current << 8;
    n = 44;
   } break;

   case 44:
    timestamp = current;
    n = 45;
    break;

   case 45:
    timestamp |= current << 8;
    n = 46;
    break;

   case 46:
    if(current == crc) {
     uint16_t diff = (endAngle + 36000 - startAngle) % 36000;

     for(uint8_t i = 0; i < NBMEASURESPACK; i++) {
      if(confidences[i] < CONFIDENCEMIN)
       continue;

      uint16_t angle = startAngle + diff * i / (NBMEASURESPACK - 1);
      angle = angle * 65536 / 36000;
      points.push_back({distances[i], angle});
     }
    }
    packs++;

    if(packs == 26 && points.size()) {
     packs = 0;
     pointsOut = points;
     points.clear();
     done = true;
    }

    n = 0;
    p = 0;
    crc = 0;
    break;

  }
 }

 return done;
}
#endif

#ifdef RPLIDAR
void startLidar(int ld) {
 serialPutchar(ld, 0xa5);
 serialPutchar(ld, 0x82);
 serialPutchar(ld, 0x05);
 serialPutchar(ld, 0x00);
 serialPutchar(ld, 0x00);
 serialPutchar(ld, 0x00);
 serialPutchar(ld, 0x00);
 serialPutchar(ld, 0x00);
 serialPutchar(ld, 0x22);
}

void stopLidar(int ld) {
 serialPutchar(ld, 0xa5);
 serialPutchar(ld, 0x25);
}

bool readLidar(int ld, std::vector<PointPolar> &pointsOut) {
 static uint8_t init = 0;
 uint8_t current;
 static uint8_t n = 0;
 static uint8_t checksum;
 static uint8_t sum = 0;
 static uint16_t startAngleQ6;
 static uint16_t oldStartAngleQ6 = 0;
 static int32_t oldAngleBrutQ6 = 0;
 static uint8_t deltaAnglesQ3[NBMEASURESCABIN];
 static uint16_t distances[NBMEASURESCABIN];
 static uint8_t o = 0;
 static uint8_t p = 0;
 static uint16_t j = 0;
 static std::vector<PointPolar> points;
 bool done = false;

 while(serialDataAvail(ld)) {
  current = serialGetchar(ld);

  switch(n) {

   case 0:                                                   // Début de réception de l'en-tête
    if(current >> 4 == 0xA) {
     checksum = current & 0xF;
     n = 1;
    }
    break;

   case 1:
    if(current >> 4 == 0x5) {
     checksum |= current << 4;
     n = 2;
    } else
     n = 0;
    break;

   case 2:
    sum ^= current;
    startAngleQ6 = current;
    n = 3;
    break;

   case 3: {
    sum ^= current;
    startAngleQ6 |= (current & 0x7F) << 8;
    //bool start = current >> 7;                             // Fin de réception de l'en-tête

    if(init < NBINITS) {                                     // Ne pas calculer pendant la synchronisation ou sans les cabines
     init++;
     n = 4;
     break;
    }

    uint16_t diffAngleQ6 = startAngleQ6 - oldStartAngleQ6;   // Calculer l'angle entre deux mesures de référence
    if(oldStartAngleQ6 > startAngleQ6)
     diffAngleQ6 += FULLTURNQ6;

    int32_t diffAngleTotalQ6 = 0;
    for(uint8_t i = 0; i < NBMEASURESCABIN; i++) {

     // Calculer l'angle non compensé
     int32_t angleBrutQ6 = (oldStartAngleQ6 + diffAngleTotalQ6 / NBMEASURESCABIN) % FULLTURNQ6;
     diffAngleTotalQ6 += diffAngleQ6;

     if(oldAngleBrutQ6 > angleBrutQ6 && points.size()) {     // Détection du passage par zéro de l'angle non compensé
      pointsOut = points;
      points.clear();
      done = true;
     }
     oldAngleBrutQ6 = angleBrutQ6;

     if(distances[i]) {                                      // Si la lecture est valide et si il reste de la place dans les tableaux
      int32_t angle = angleBrutQ6 - (deltaAnglesQ3[i] << 3); // Calculer l'angle compensé
      angle = angle * 65536 / FULLTURNQ6;                    // Remise à l'échelle de l'angle
      points.push_back({distances[i], uint16_t(angle)});
     }

    }
    oldStartAngleQ6 = startAngleQ6;

    n = 4;
   } break;

   default:                                                  // Début de réception des cabines
    sum ^= current;
    switch(o) {
     case 0:
      deltaAnglesQ3[p] = (current & 0b11) << 4;
      distances[p] = current >> 2;
      o = 1;
      break;
     case 1:
      distances[p] |= current << 6;
      o = 2;
      break;
     case 2:
      deltaAnglesQ3[p + 1] = (current & 0b11) << 4;
      distances[p + 1] = current >> 2;
      o = 3;
      break;
     case 3:
      distances[p + 1] |= current << 6;
      o = 4;
      break;
     case 4:
      deltaAnglesQ3[p] |= current & 0b1111;
      deltaAnglesQ3[p + 1] |= current >> 4;
      o = 0;
      p += 2;
      if(p == NBMEASURESCABIN) {
       if(sum != checksum)
        init = NBINITS - 1;                                  // Ne pas faire les calculs pour ces cabines
       n = 0;
       p = 0;
       sum = 0;
      }
      break;
    }
    break;

  }
 }

 return done;
}
#endif
