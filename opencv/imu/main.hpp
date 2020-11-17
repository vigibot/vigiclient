#define WIDTH 640
#define HEIGHT 480
#define FPS 30

//#define BODYPAN
#define TIMEOUT 30
#define DIRX 1.0
#define DIRY 1.0
#define DIRZ -1.0
#define OFFSETX M_PI
#define OFFSETY 0.0
#define OFFSETZ 0.0
#define IMUSLERPPOWER 0.01
#define TRIGGERVX 64
#define DIVVZ 20
#define KPVZ 3
#define KIVZ 20
#define KDVZ 6
#define MARGIN 5
#define DIAM1 20
#define DIAM2 10
#define COEF2 4.0

int width;
int height;
int fps;

volatile bool run = true;

RemoteFrame remoteFrame;
TelemetryFrame telemetryFrame;

RTIMU_DATA imuData;
