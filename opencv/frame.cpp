#include "frame.hpp"

bool readModem(int fd) {
 uint8_t octet;
 static uint8_t pos = 0;
 uint8_t p;

 while(serialDataAvail(fd)) {
  octet = serialGetchar(fd);

  switch(pos) {

   case 0:
    if(octet == '$')
     pos = 1;
    break;

   case 1:
    if(octet == 'S')
     pos = 2;
    else
     pos = 0;
    break;

   case 2:
   case 3:
    if(octet == ' ')
     pos++;
    else
     pos = 0;
    break;

   default:
    remoteFrame.bytes[pos++] = octet;
    if(pos == REMOTEFRAMESIZE) {
     pos = 0;
     serialFlush(fd);
     return true;
    }

  }
 }

 return false;
}

void writeModem(int fd) {
 for(int i = 0; i < TELEMETRYFRAMESIZE; i++)
  serialPutchar(fd, telemetryFrame.bytes[i]);
}
