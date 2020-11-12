#define WIDTH 640
#define HEIGHT 480
#define FPS 30

#define IMU
#define DIRZ -1
#define REFTILTINITDELAY 20

#define LIDARX 0
#define LIDARY 0
#define LIDARXMIN -100
#define LIDARXMAX 100
#define LIDARYMIN -100
#define LIDARYMAX 100

#define EPSILON 50.0
#define DISTMARGIN 3
#define DISTCLAMP 50
#define NBPOINTSMIN 6
#define DISTMIN 200

#define VXDIV 10
#define VYDIV 10
#define VZMUL 10

#define MAPDIVMIN 5
#define MAPDIVMAX 50

#define LARGEDISTERROR 200
#define LARGEANGULARERROR (20.0 * M_PI / 180.0)
#define SMALLDISTERROR 50
#define SMALLANGULARERROR (5.0 * M_PI / 180.0)

#define CONFIDENCEMAXVELOCITY 64
#define CONFIDENCEMAXTILT (3.0 * M_PI / 180.0)
#define CONFIDENCEDELAY 10
#define CONFIDENCEDISTERROR 30
#define CONFIDENCEANGULARERROR (3.0 * M_PI / 180.0)

#define ODOMETRYCORRECTORDIV 2
#define IMUTHETACORRECTORDIV 10
#define THETACORRECTORDIV 4

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
