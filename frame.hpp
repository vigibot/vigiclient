#include <stdint.h>

#define DEVROBOT "/dev/pts/1"
#define DEVDEBIT 115200
#define REMOTEFRAMESIZE 17
#define TELEMETRYFRAMESIZE 25
#define HEADERSIZE 4
#define NBCOMMANDS 2

typedef struct {
 union {
  struct {
   uint8_t header[HEADERSIZE];
   int16_t xy[NBCOMMANDS][2];
   uint8_t z;
   int8_t vx;
   int8_t vy;
   int8_t vz;
   uint8_t switchs;
  };

  uint8_t bytes[REMOTEFRAMESIZE];
 };
} RemoteFrame;

typedef struct {
 union {
  struct {
   uint8_t header[HEADERSIZE];
   //uint32_t lat;
   //uint32_t lon;
   int16_t xy[NBCOMMANDS][2];
   uint16_t voltage;
   uint16_t battery;
   uint8_t z;
   int8_t vx;
   int8_t vy;
   int8_t vz;
   uint8_t switchs;
   uint8_t cpuload;
   uint8_t soctemp;
   uint8_t link;
   uint8_t rssi;
   //uint8_t satsActive;
   //uint8_t speed;
   //uint8_t track;
  };

  uint8_t bytes[TELEMETRYFRAMESIZE];
 };
} TelemetryFrame;

RemoteFrame remoteFrame;
TelemetryFrame telemetryFrame;
