#define WIDTH 640
#define HEIGHT 480
#define FPS 30

#define DIRZ -1

#define LIDARX 0
#define LIDARY 0
#define ROBOTWIDTH 10
#define ROBOTHEIGHT 10

#define ROBOTXMIN (-ROBOTWIDTH / 2)
#define ROBOTXMAX (ROBOTWIDTH / 2)
#define ROBOTYMIN (-ROBOTHEIGHT / 2)
#define ROBOTYMAX (ROBOTHEIGHT / 2)

#define EPSILON 30.0
#define NBPOINTSMIN 4
#define SQDISTMAX (150 * 150)

int width;
int height;
int fps;

volatile bool run = true;

RemoteFrame remoteFrame;
TelemetryFrame telemetryFrame;

RTIMU_DATA imuData;
