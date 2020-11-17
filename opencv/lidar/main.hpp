#define WIDTH 640
#define HEIGHT 480
#define FPS 30

#define IMU
#define DIRX 1.0
#define DIRY 1.0
#define DIRZ -1.0
#define OFFSETX M_PI
#define OFFSETY 0.0
#define OFFSETZ 0.0
#define IMUSLERPPOWER 0.01
#define REFTILTINITDELAY 100
#define MARGIN 5
#define DIAM1 20
#define DIAM2 10

#define LIDARXMIN -100
#define LIDARXMAX 100
#define LIDARYMIN -100
#define LIDARYMAX 100

#define EPSILON 50.0
#define DISTMARGIN 3
#define DISTCLAMP 50
#define NBPOINTSMIN 5
#define DISTMIN 100

#define VXDIV 10
#define VYDIV 10
#define VZMUL 10

#define MAPDIVMIN 5
#define MAPDIVMAX 50

#define LARGEDISTTOLERANCE 200
#define LARGEANGULARTOLERANCE (20.0 * M_PI / 180.0)
#define SMALLDISTTOLERANCE 100
#define SMALLANGULARTOLERANCE (10.0 * M_PI / 180.0)

#define CONFIDENCEDISTTOLERANCE 40
#define CONFIDENCEANGULARTOLERANCE (4.0 * M_PI / 180.0)
#define CONFIDENCEMAXVELOCITY 64
#define CONFIDENCEMAXTILT (4.0 * M_PI / 180.0)
#define CONFIDENCEDELAY 10

#define NBITERATIONS 10
#define ODOMETRYCORRECTORDIV 5
#define IMUTHETACORRECTORDIV 100
#define THETACORRECTORDIV 10

enum {
 SELECTNONE,
 SELECTMAP,
 SELECTMAPBEAMS
};

const std::vector<cv::Point> robotIcon = {
 cv::Point(-30, -40),
 cv::Point{30, -40},
 cv::Point(30, 40),
 cv::Point(0, 60),
 cv::Point{-30, 40}
};

typedef struct Line {
 cv::Point a;
 cv::Point b;
} Line;

int width;
int height;
int fps;

volatile bool run = true;

RemoteFrame remoteFrame;
TelemetryFrame telemetryFrame;

RTIMU_DATA imuData;
RTIMU *imu;
uint16_t thetaCorrector = 0;

cv::Scalar hueToBgr[180];
