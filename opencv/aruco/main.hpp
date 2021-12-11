#define WIDTH 640
#define HEIGHT 480
#define FPS 30

int width;
int height;
int fps;

volatile bool run = true;

RemoteFrame remoteFrame;
TelemetryFrame telemetryFrame;
