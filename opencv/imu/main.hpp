#define WIDTH 640
#define HEIGHT 480
#define FPS 30

#define MARGIN 10
#define LINELEN 15
#define ROLLCOEF 4.0
#define PITCHCOEF -4.0
#define YAWCOEF 4.0

int width;
int height;
int fps;

volatile bool run = true;

RemoteFrame remoteFrame;
TelemetryFrame telemetryFrame;

RTIMU_DATA imuData;
