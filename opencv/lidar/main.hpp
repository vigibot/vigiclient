#define WIDTH 640
#define HEIGHT 480
#define FPS 30

#define NBITERATIONSSYNCHRO 3
#define NBMESURESCABINES 32
#define UNTOURQ6 (360 << 6)

#define LIDARX 0
#define LIDARY 0
#define ROBOTWIDTH 300
#define ROBOTHEIGHT 300

#define ROBOTXMIN -(ROBOTWIDTH / 2)
#define ROBOTXMAX (ROBOTWIDTH / 2)
#define ROBOTYMIN -(ROBOTHEIGHT / 2)
#define ROBOTYMAX (ROBOTHEIGHT / 2)

#define EPSILON 30.0
#define NBPOINTSMIN 4
#define SQDISTMAX 30000

typedef struct PointPolar {
 int distance;
 uint16_t theta;
} PointPolar;

int width;
int height;
int fps;

volatile bool run = true;

RemoteFrame remoteFrame;
TelemetryFrame telemetryFrame;

RTIMU_DATA imuData;
