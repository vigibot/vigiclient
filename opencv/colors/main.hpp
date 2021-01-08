#define WIDTH 640
#define HEIGHT 480
#define FPS 30

#define BUTTONSLONGPRESS 15

#define BINNING 2
#define NBCOLORS 7
#define THRESHOLD 60
#define EPSILON 1.0
#define MINVERTICES 3
#define MINAREA 20.0
#define MINCOLORDIST 2
#define MINPOINTPOLYGONDIST -20.0

#define HEADPAN
#define HEADTILT
#define TIMEOUT 30
#define NEUTRAL 10
#define KPX 3
#define KDX 9
#define KPY 2
#define KDY 4
#define XMIN -13500
#define XMAX 13500
#define YMIN -5500
#define YMAX 5500
#define KVY 200
#define KPVZ 4
#define KDVZ 2
#define VMIN 16

enum {
 SELECTCAMERA,
 SELECTCAMERASHORTS,
 SELECTCONFTHRESHOLD,
 SELECTCONFFIRSTCOLOR
};

const char *COLORS[] = {"Red", "Orange", "Yellow", "Green", "Cyan", "Blue", "Magenta"};
const char *SHORTS[] = {"R", "O", "Y", "G", "C", "B", "M"};

typedef struct Feature {
 int color;
 std::vector<cv::Point> polygon;
 float area;
 cv::Point2f center;
 cv::Point circleCenter;
 int circleRadius;
 bool filtered;
} Feature;

int width;
int height;
int fps;

volatile bool run = true;

RemoteFrame remoteFrame;
TelemetryFrame telemetryFrame;

uchar hues[] = {7, 22, 37, 82, 97, 134, 164};

uchar hueToColor[180];
uchar colorToHue[NBCOLORS];
cv::Scalar hueToBgr[180];

bool blacks[NBCOLORS] = {false};

std::vector<Feature> features;
