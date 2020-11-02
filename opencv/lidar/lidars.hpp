#include <stdint.h>

#define LIDARPORT "/dev/serial0"

#define LDLIDAR
#define LIDARRATE 230400
#define NBMEASURESPACK 12

//#define RPLIDAR
//#define LIDARRATE 115200
//#define NBSYNC 3
//#define NBMEASURESCABIN 32
//#define UNTOURQ6 (360 << 6)

typedef struct PointPolar {
 int distance;
 uint16_t theta;
} PointPolar;

void startLidar(int ld);
void stopLidar(int ld);
bool readLidar(int ld, std::vector<PointPolar> &pointsOut);
