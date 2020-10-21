#define WIDTH 640
#define HEIGHT 480
#define FPS 30

#define MARGIN 5
#define DIAM1 20
#define DIAM2 10
#define COEF2 10.0

int width;
int height;
int fps;

volatile bool run = true;

RemoteFrame remoteFrame;
TelemetryFrame telemetryFrame;

RTIMU_DATA imuData;
