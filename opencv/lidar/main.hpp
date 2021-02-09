#define WIDTH 640
#define HEIGHT 480
#define FPS 30

#define GOTOTOOL 0
#define BUTTONSLONGPRESS 15

#define IMU
#define DIRZ -1.0
#define IMUSLERPPOWER 0.01
#define IMUTHETACORRECTORDIV 100

#define EPSILON 50.0
#define DISTCOEF 4
#define DISTCLAMP 50
#define NBPOINTSMIN 5
#define LIDARLINESMINLEN 100
#define MAPLINESMINLEN 200

#define VXDIV 10
#define VYDIV 10
#define VZMUL 10

#define MAPFILE "lidar.json"
#define MAPDIV 100
#define MAPDIVMIN 10
#define MAPDIVMAX 1000

#define AXES 2
#define LARGEDISTTOLERANCE 300
#define LARGEANGULARTOLERANCE (30.0 * M_PI / 180.0)
#define SMALLDISTTOLERANCE 40
#define SMALLANGULARTOLERANCE (2.0 * M_PI / 180.0)
#define INTERSECTMAX 10000
#define MAPCLEANERDISTPERCENT 50
#define MAPCLEANERANGULARTOLERANCE (15.0 * M_PI / 180.0)

#define RECOVERYCONFIDENCE 10
#define RECOVERYDISTTOLERANCE 1000

#define VALIDATIONFILTERKILL -5
#define VALIDATIONFILTERSTART 0
#define VALIDATIONFILTERKEEP 5
#define SHRINKFILTER 5
#define MAPFILTERSDECAY 2

#define NBITERATIONS 10
#define HIST 500

#define GOTOPOINTDISTTOLERANCE 150
#define GOTOPOINTANGLEREVERSEGEAR 150
#define GOTOPOINTVELOCITY 127
#define GOTOPOINTVELOCITYTHETA 127
#define GOTOPOINTANGLESTOP 180
#define KPTHETA 200
#define KITHETA 2000
#define KDTHETA 100

#define LINKSLENGTHMIN 100
#define LINKSLENGTHMAX 300
#define OBSTACLENBPOINTS 2
#define OBSTACLEROBOTWIDTH 50
#define OBSTACLEROBOTLENGTH 100
#define DISTFROMOBSTACLE 150
#define GRAPHINGSLOWDOWN 30

enum {
 STATUSWAITING,
 STATUSSUCCESS,
 STATUSERROR
};

enum {
 GOTOWAITING,
 GOTONODE,
 GOTOPOINT
};

enum {
 SELECTFIXEDMINIMAL,
 SELECTFIXEDAUTOPILOT,
 SELECTFIXEDWAYPOINTS,
 SELECTFIXEDGRAPHING,
 SELECTFIXEDMAPPING,
 SELECTAUTOPILOT,
 SELECTWAYPOINTS,
 SELECTGRAPHING,
 SELECTMAPPING,
 SELECTLIDARONLY
};

const std::vector<cv::Point> robotIcon = {
 cv::Point(-30, -40),
 cv::Point{30, -40},
 cv::Point(30, 40),
 cv::Point(0, 60),
 cv::Point{-30, 40}
};

const char *OFFON[] = {"off", "on"};

typedef struct Line {
 cv::Point a;
 cv::Point b;
 cv::Point intega;
 cv::Point integb;
 int integ;
 int validation;
 int shrinka;
 int shrinkb;
} Line;

typedef std::pair<int, int> Pair;

int width;
int height;
int fps;

volatile bool run = true;

RemoteFrame remoteFrame;
TelemetryFrame telemetryFrame;

RTIMU_DATA imuData;
RTIMU *imu;
volatile int imuThreadStatus = STATUSWAITING;
uint16_t robotThetaCorrector = 0;

cv::Scalar hueToBgr[180];
