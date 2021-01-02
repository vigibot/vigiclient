#define WIDTH 640
#define HEIGHT 480
#define FPS 30

#define IMU
#define DIRZ -1.0
#define IMUSLERPPOWER 0.01
#define IMUTHETACORRECTORDIV 100

#define EPSILON 50.0
#define DISTCOEF 4
#define DISTCLAMP 50
#define NBPOINTSMIN 5
#define LINESIZEMIN 200

#define VXDIV 10
#define VYDIV 10
#define VZMUL 10

#define MAPFILE "/tmp/map.json"
#define MAPDIV 10
#define MAPDIVMIN 1
#define MAPDIVMAX 100

#define LARGEDISTTOLERANCE 300
#define LARGEANGULARTOLERANCE (30.0 * M_PI / 180.0)
#define SMALLDISTTOLERANCE 40
#define SMALLANGULARTOLERANCE (2.0 * M_PI / 180.0)

#define INTERSECTDETMIN 0.0001
#define INTERSECTMAX 10000

#define VALIDATIONFILTERKILL -5
#define VALIDATIONFILTERSTART 0
#define VALIDATIONFILTERKEEP 5
#define SHRINKFILTER 5
#define MAPFILTERSDECAY 2

#define NBITERATIONS 10
#define HIST 500

#define GOTOPOINTDISTTOLERANCE 100
#define GOTOPOINTANGLEREVERSEGEAR 135
#define GOTOPOINTVELOCITY 127
#define GOTOPOINTBRAKEDIST 400
#define GOTOPOINTANGLEBRAKE (90.0 * M_PI / 180.0)
#define GOTOPOINTANGLESTOP 180
#define KPTHETA 100
#define KITHETA 200
#define KDTHETA 100

enum {
 SELECTNONE,
 SELECTMAP,
 SELECTMAPPOINTS,
 SELECTMAPPOINTSBEAMS,
 SELECTPOINTSBEAMS
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
 cv::Point intega;
 cv::Point integb;
 int integ;
 int validation;
 int shrinka;
 int shrinkb;
 bool locka;
 bool lockb;
} Line;

int width;
int height;
int fps;

volatile bool run = true;

RemoteFrame remoteFrame;
TelemetryFrame telemetryFrame;

RTIMU_DATA imuData;
RTIMU *imu;
uint16_t robotThetaCorrector = 0;

cv::Scalar hueToBgr[180];
