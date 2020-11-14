#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <wiringSerial.h>
#include <thread>
#include <RTIMULib.h>
#include "../common.hpp"
#include "../frame.hpp"
#include "lidars.hpp"
#include "sin16.hpp"
#include "main.hpp"

using namespace std;
using namespace cv;

void signal_callback_handler(int signum) {
 fprintf(stderr, "Caught signal %d\n", signum);
 run = false;
}

void imuThread() {
 int oldStdout = dup(fileno(stdout));
 dup2(fileno(stderr), fileno(stdout));

 RTIMUSettings *settings = new RTIMUSettings("RTIMULib");
 imu = RTIMU::createIMU(settings);
 if(imu == NULL || imu->IMUType() == RTIMU_TYPE_NULL) {
  fprintf(stderr, "No IMU found\n");
  return;
 }

 imu->IMUInit();
 imu->setSlerpPower(0.002);
 imu->setGyroEnable(true);
 imu->setAccelEnable(true);
 imu->setCompassEnable(false);

 dup2(oldStdout, fileno(stdout));

 while(run) {
  usleep(imu->IMUGetPollInterval() * 1000);
  while(imu->IMURead())
   imuData = imu->getIMUData();
 }
}

int sqNorm(Point point) {
 return point.x * point.x + point.y * point.y;
}

int sqDist(Line line) {
 return sqNorm(line.b - line.a);
}

int sqDist(Point point1, Point point2) {
 Point diff = point2 - point1;
 return sqNorm(diff);
}

void extractRawLines(vector<PolarPoint> &polarPoints, vector<Point> &points, vector<vector<Point>> &lines) {
 vector<Point> pointsDp;
 vector<Point> pointsNoDp;
 Point oldPoint = Point(0, 0);

 approxPolyDP(points, pointsDp, EPSILON, true);

 for(int i = 0; i < points.size() * 2; i++) {
  int ii = i % points.size();

  bool dp = false;
  for(int j = 0; j < pointsDp.size(); j++) {
   if(points[ii] == pointsDp[j]) {
    dp = true;
    break;
   }
  }

  int sqDst = sqDist(points[ii], oldPoint);
  oldPoint = points[ii];

  uint16_t angle = 2 * PI16 / polarPoints.size();
  int distMax = polarPoints[ii].distance * sin16(angle) * DISTMARGIN / ONE16;
  if(distMax < DISTCLAMP)
   distMax = DISTCLAMP;

  if(dp || sqDst > distMax * distMax) {
   int size = pointsNoDp.size();
   if(size >= NBPOINTSMIN && i > size + 1 &&
      sqDist(pointsNoDp[0], pointsNoDp[size -1]) >= DISTMIN * DISTMIN) {
    lines.push_back(pointsNoDp);
    if(i > ii)
     break;
   }
   pointsNoDp.clear();

  } else
   pointsNoDp.push_back(points[ii]);
 }
}

void fitLines(vector<vector<Point>> &rawLinesIn, vector<Line> &linesOut) {
 for(int i = 0; i < rawLinesIn.size(); i++) {
  vector<double> fit;
  fitLine(rawLinesIn[i], fit, CV_DIST_L2, 0.0, 0.01, 0.01);

  Point point0 = Point(fit[2], fit[3]);

  double dist1 = sqrt(sqDist(point0, rawLinesIn[i][0]));
  double dist2 = sqrt(sqDist(point0, rawLinesIn[i][rawLinesIn[i].size() - 1]));

  Point point1 = Point(fit[0] * dist1 + fit[2],
                       fit[1] * dist1 + fit[3]);

  int sqDist3 = sqDist(point1, rawLinesIn[i][0]);
  int sqDist4 = sqDist(point1, rawLinesIn[i][rawLinesIn[i].size() - 1]);

  Point point2;
  if(sqDist3 > sqDist4) {
   point1 = Point(fit[0] * -dist1 + fit[2],
                  fit[1] * -dist1 + fit[3]);
   point2 = Point(fit[0] * dist2 + fit[2],
                  fit[1] * dist2 + fit[3]);
  } else
   point2 = Point(fit[0] * -dist2 + fit[2],
                  fit[1] * -dist2 + fit[3]);

  linesOut.push_back({point1, point2});
 }
}

void lidarToRobot(vector<PolarPoint> &pointsIn, vector<Point> &pointsOut) {
 for(int i = 0; i < pointsIn.size(); i++) {
  int x = pointsIn[i].distance * sin16(pointsIn[i].theta) / ONE16;
  int y = pointsIn[i].distance * cos16(pointsIn[i].theta) / ONE16;

  if(x > LIDARXMAX || x < LIDARXMIN ||
     y > LIDARYMAX || y < LIDARYMIN)
   pointsOut.push_back(Point(x, y));
 }
}

void robotToMap(vector<Line> &linesIn, vector<Line> &linesOut, Point odometryPoint, uint16_t theta) {
 for(int i = 0; i < linesIn.size(); i++) {
  linesOut.push_back({
   Point(odometryPoint.x + (linesIn[i].a.x * cos16(theta) - linesIn[i].a.y * sin16(theta)) / ONE16,
         odometryPoint.y + (linesIn[i].a.x * sin16(theta) + linesIn[i].a.y * cos16(theta)) / ONE16),
   Point(odometryPoint.x + (linesIn[i].b.x * cos16(theta) - linesIn[i].b.y * sin16(theta)) / ONE16,
         odometryPoint.y + (linesIn[i].b.x * sin16(theta) + linesIn[i].b.y * cos16(theta)) / ONE16)
  });
 }
}

void mapToRobot(vector<Line> &linesIn, vector<Line> &linesOut, Point odometryPoint, uint16_t theta) {
 for(int i = 0; i < linesIn.size(); i++) {
  Point point1 = linesIn[i].a - odometryPoint;
  Point point2 = linesIn[i].b - odometryPoint;
  linesOut.push_back({
   Point((point1.x * cos16(-theta) - point1.y * sin16(-theta)) / ONE16,
         (point1.x * sin16(-theta) + point1.y * cos16(-theta)) / ONE16),
   Point((point2.x * cos16(-theta) - point2.y * sin16(-theta)) / ONE16,
         (point2.x * sin16(-theta) + point2.y * cos16(-theta)) / ONE16)
  });
 }
}

double lineAngle(Line line) {
 Point diff = line.b - line.a;
 return atan2(diff.y, diff.x);
}

double ratioPointLine(Point point, Line line) {
 Point diff = line.b - line.a;
 double scalarProduct;
 double ratio = 0;

 if(diff.x || diff.y) {
  scalarProduct = (point.x - line.a.x) * diff.x +
                  (point.y - line.a.y) * diff.y;
  ratio = scalarProduct / sqNorm(diff);
 }

 return ratio;
}

Point pointDistancePointLine(Point point, Line line) {
 double ratio = ratioPointLine(point, line);
 Point h;
 Point diff = line.b - line.a;
 Point result = Point(0, 0);

 if(ratio) {
  h.x = line.a.x + int(double(diff.x) * ratio);
  h.y = line.a.y + int(double(diff.y) * ratio);
  result = point - h;
 }

 return result;
}

int distancePointLine(Point point, Line line) {
 return sqrt(sqNorm(pointDistancePointLine(point, line)));
}

bool growLine(Point point, Line &line) {
 Point diff = line.b - line.a;
 double ratio = ratioPointLine(point, line);
 Point h;

 if(ratio < 0 || ratio > 1) {
  h.x = line.a.x + int(double(diff.x) * ratio);
  h.y = line.a.y + int(double(diff.y) * ratio);

  if(ratio < 0)
   line.a = h;
  else
   line.b = h;

  return true;
 }
 return false;
}

double diffAngle(Line ligne1, Line ligne2) {
 double angle1 = lineAngle(ligne1);
 double angle2 = lineAngle(ligne2);
 double result = angle2 - angle1;

 if(result > M_PI)
  result -= 2.0 * M_PI;
 else if(result < -M_PI)
  result += 2.0 * M_PI;

 return result;
}

bool getConfidence(double refTilt[], Point pointError, double angularError) {
 static int delay = 0;
 bool moveFast = abs(remoteFrame.vx) > CONFIDENCEMAXVELOCITY ||
                 abs(remoteFrame.vy) > CONFIDENCEMAXVELOCITY ||
                 abs(remoteFrame.vz) > CONFIDENCEMAXVELOCITY;
 bool tilt = abs(imuData.fusionPose.x() - refTilt[0]) > CONFIDENCEMAXTILT ||
             abs(imuData.fusionPose.y() - refTilt[1]) > CONFIDENCEMAXTILT;

 if(moveFast || tilt)
  delay = CONFIDENCEDELAY;

 if(delay)
  delay--;

 if(!delay &&
    sqNorm(pointError) < CONFIDENCEDISTTOLERANCE * CONFIDENCEDISTTOLERANCE &&
    abs(angularError) < CONFIDENCEANGULARTOLERANCE)
  return true;

 return false;
}

bool ccw(Point point1, Point point2, Point point3) {
 return (point3.y - point1.y) * (point2.x - point1.x) >
        (point2.y - point1.y) * (point3.x - point1.x);
}

bool intersect(Line line1, Line line2) {
 return ccw(line1.a, line2.a, line2.b) != ccw(line1.b, line2.a, line2.b) &&
        ccw(line1.a, line1.b, line2.a) != ccw(line1.a, line1.b, line2.b);
}

void localization(vector<Line> &lines, vector<Line> &map,
                  Point &odometryPoint, uint16_t &theta,
                  double refTilt[], bool &confidence) {

 Point pointErrorSum = Point(0, 0);
 double angularErrorSum = 0;
 int weightSum = 0;
 int distErrorMax = 0;
 int distErrorMaxId = -1;
 double absAngularErrorMax = 0;
 int absAngularErrorMaxId = -1;

 for(int i = 0; i < lines.size(); i++) {
  for(int j = 0; j < map.size(); j++) {

   double angularError = diffAngle(lines[i], map[j]);
   double absAngularError = abs(angularError);
   if(absAngularError > LARGEANGULARTOLERANCE)
    continue;

   Point pointError = pointDistancePointLine((lines[i].a + lines[i].b) / 2, map[j]);
   int distError = sqrt(sqNorm(pointError));
   if(distError > LARGEDISTTOLERANCE)
    continue;

   int refNorm = sqrt(sqDist(map[j]));
   int distance1 = ratioPointLine(lines[i].a, map[j]) * refNorm;
   int distance2 = ratioPointLine(lines[i].b, map[j]) * refNorm;
   if((distance1 < 0 || distance1 > refNorm) &&
      (distance2 < 0 || distance2 > refNorm) &&
      distance1 * distance2 > 0)
    continue;

   pointErrorSum += pointError * refNorm;
   angularErrorSum += angularError * refNorm;
   weightSum += refNorm;

   if(distError > distErrorMax) {
    distErrorMax = distError;
    distErrorMaxId = j;
   }

   if(absAngularError > absAngularErrorMax) {
    absAngularErrorMax = absAngularError;
    absAngularErrorMaxId = j;
   }
  }
 }

 if(weightSum) {
  confidence = getConfidence(refTilt, pointErrorSum / weightSum, angularErrorSum / weightSum);
  odometryPoint -= pointErrorSum / weightSum / ODOMETRYCORRECTORDIV;

#ifdef IMU
  thetaCorrector += int(angularErrorSum * double(PI16) / M_PI) / weightSum / IMUTHETACORRECTORDIV;
#else
  theta += int(angularErrorSum * double(PI16) / M_PI) / weightSum / THETACORRECTORDIV;
#endif

  if(!confidence)
   return;

  if(distErrorMax > SMALLDISTTOLERANCE)
   map.erase(map.begin() + distErrorMaxId);

  if(absAngularErrorMax > SMALLANGULARTOLERANCE)
   map.erase(map.begin() + absAngularErrorMaxId);

 } else
  confidence = false;
}

bool testLines(Line line1, Line line2) {
 double absAngularError = abs(diffAngle(line1, line2));
 if(absAngularError > SMALLANGULARTOLERANCE)
  return false;

 int distError = distancePointLine((line1.a + line1.b) / 2, line2);
 if(distError > SMALLDISTTOLERANCE)
  return false;

 int refNorm = sqrt(sqDist(line2));
 int distance1 = ratioPointLine(line1.a, line2) * refNorm;
 int distance2 = ratioPointLine(line1.b, line2) * refNorm;
 if((distance1 < 0 || distance1 > refNorm) &&
    (distance2 < 0 || distance2 > refNorm) &&
    distance1 * distance2 > 0)
  return false;

 return true;
}

void mapping(vector<Line> &lines, vector<Line> &map, bool &confidence) {
 vector<Line> newLines;
 bool change = false;

 if(!confidence && map.size())
  return;

 for(int i = 0; i < lines.size(); i++) {
  bool newLine = true;

  for(int j = 0; j < map.size(); j++) {
   double absAngularError = abs(diffAngle(lines[i], map[j]));
   if(absAngularError > LARGEANGULARTOLERANCE)
    continue;

   int distError = distancePointLine((lines[i].a + lines[i].b) / 2, map[j]);
   if(distError > LARGEDISTTOLERANCE)
    continue;

   int refNorm = sqrt(sqDist(map[j]));
   int distance1 = ratioPointLine(lines[i].a, map[j]) * refNorm;
   int distance2 = ratioPointLine(lines[i].b, map[j]) * refNorm;
   if((distance1 < 0 || distance1 > refNorm) &&
      (distance2 < 0 || distance2 > refNorm) &&
      distance1 * distance2 > 0)
    continue;

   newLine = false;

   if(absAngularError > SMALLANGULARTOLERANCE)
    continue;

   if(distError > SMALLDISTTOLERANCE)
    continue;

   bool merged = false;
   merged |= growLine(lines[i].a, map[j]);
   merged |= growLine(lines[i].b, map[j]);
   if(!merged)
    continue;
   else
    change = true;

   int mergeIndex = j;
   for(int k = 0; k < map.size(); k++) {
    if(k == mergeIndex || !testLines(map[mergeIndex], map[k]))
     continue;

    int bigger;
    int smaller;
    if(sqDist(map[mergeIndex]) > sqDist(map[k])) {
     bigger = mergeIndex;
     smaller = k;
    } else {
     bigger = k;
     smaller = mergeIndex;
     mergeIndex = bigger;
    }
    bool merged = false;
    merged |= growLine(map[smaller].a, map[bigger]);
    merged |= growLine(map[smaller].b, map[bigger]);
    if(!merged)
     continue;

    map.erase(map.begin() + smaller);
    if(smaller < bigger) {
     mergeIndex--;
     k--;
    }
   }

   break;
  }

  if(newLine) {
   newLines.push_back(lines[i]);
   change = true;
  }
 }

 for(int i = 0; i < newLines.size(); i++)
  map.push_back(newLines[i]);

 if(change) {
  sort(map.begin(), map.end(), [](const Line &a, const Line &b) {
   return sqDist(a) > sqDist(b);
  });
 }
}

void drawPoints(Mat &image, vector<Point> &points, bool beams, int mapDiv) {
 const Point centerPoint = Point(width / 2, height / 2);

 for(int i = 0; i < points.size(); i++) {
  Point point = points[i];
  point.x /= mapDiv;
  point.y /= -mapDiv;

  if(beams)
   line(image, centerPoint, centerPoint + point, Scalar::all(i ? 64 : 255), 1, LINE_AA);
  circle(image, centerPoint + point, i ? 1 : 3, Scalar::all(255), FILLED, LINE_AA);
 }
}

void drawLines(Mat &image, vector<Line> &lines, vector<Line> &colors, bool colored, int mapDiv) {
 const Point centerPoint = Point(width / 2, height / 2);

 for(int i = 0; i < lines.size(); i++) {
  Point point1 = lines[i].a;
  Point point2 = lines[i].b;

  point1.x /= mapDiv;
  point2.x /= mapDiv;
  point1.y /= -mapDiv;
  point2.y /= -mapDiv;
  point1 += centerPoint;
  point2 += centerPoint;

  Point diff = colors[i].b - colors[i].a;
  double angleDeg = atan2(diff.y, diff.x) * 180.0 / M_PI;

  Scalar color;
  if(colored) {
   uchar hue = uchar(angleDeg / 2.0 + 90.0) % 180;
   line(image, point1, point2, hueToBgr[hue], 2, LINE_AA);
  } else
   line(image, point1, point2, Scalar::all(255), 2, LINE_AA);
 }
}

void drawRobot(Mat &image, vector<Point> robotIcon, int thickness, Point odometryPoint, uint16_t theta, int mapDiv) {
 const Point centerPoint = Point(width / 2, height / 2);

 vector<Point> polygon;
 for(int i = 0; i < robotIcon.size(); i++) {
  Point point = odometryPoint;
  point.x += (robotIcon[i].x * cos16(theta) - robotIcon[i].y * sin16(theta)) / ONE16;
  point.y += (robotIcon[i].x * sin16(theta) + robotIcon[i].y * cos16(theta)) / ONE16;
  point.x /= mapDiv;
  point.y /= -mapDiv;
  point += centerPoint;
  polygon.push_back(point);
 }

 vector<vector<Point>> tmp(1, polygon);
 drawContours(image, tmp, -1, Scalar::all(255), thickness, LINE_AA);
}

void ui(Mat &image, vector<Point> &robotPoints, vector<Line> &robotLines,
                    vector<Line> &map, vector<Line> &mapRobot,
                    Point &odometryPoint, uint16_t &theta,
                    double refTilt[], bool confidence) {

 bool buttonLess = remoteFrame.switchs & 0b00010000;
 bool buttonMore = remoteFrame.switchs & 0b00100000;
 bool buttonReset = remoteFrame.switchs & 0b01000000;
 bool buttonOk = remoteFrame.switchs & 0b10000000;
 static bool oldButtonLess = false;
 static bool oldButtonMore = false;
 static bool oldButtonReset = false;
 static bool oldButtonOk = false;
 static bool tune = false;
 static int select = SELECTMAP;
 static int mapDiv = MAPDIVMIN;

 if(!buttonReset && oldButtonReset) {
  map.clear();
  odometryPoint = Point(0, 0);
  theta = 0;

#ifdef IMU
  imu->resetFusion();
  thetaCorrector = 0;
  refTilt[0] = imuData.fusionPose.x();
  refTilt[1] = imuData.fusionPose.y();
#endif
 }

 if(!buttonOk && oldButtonOk)
  tune = !tune;

 if(tune) {
  if(buttonLess && mapDiv < MAPDIVMAX)
   mapDiv++;
  else if(buttonMore && mapDiv > MAPDIVMIN)
   mapDiv--;
 } else {
  if(!buttonMore && oldButtonMore) {
   if(select < SELECTMAPBEAMS)
    select++;
   else
    select = SELECTNONE;
  } else if(!buttonLess && oldButtonLess) {
   if(select > SELECTNONE)
    select--;
   else
    select = SELECTMAPBEAMS;
  }
 }
 oldButtonLess = buttonLess;
 oldButtonMore = buttonMore;
 oldButtonReset = buttonReset;
 oldButtonOk = buttonOk;

 if(select == SELECTNONE)
  return;

 if(select == SELECTMAP) {
  drawPoints(image, robotPoints, false, mapDiv);
  drawLines(image, robotLines, robotLines, false, mapDiv);
  drawLines(image, mapRobot, map, true, mapDiv);
  drawRobot(image, robotIcon, 1, Point(0, 0), 0, mapDiv);
 } else {
  drawPoints(image, robotPoints, true, mapDiv);
  drawLines(image, robotLines, robotLines, false, mapDiv);
  drawLines(image, mapRobot, map, true, mapDiv);
  drawRobot(image, robotIcon, FILLED, Point(0, 0), 0, mapDiv);
 }

 char text[80];
 if(tune)
  sprintf(text, "%d mm per pixel", mapDiv);
 else if(select == SELECTMAPBEAMS)
  sprintf(text, "%d points %d lines %d on map", robotPoints.size(), robotLines.size(), mapRobot.size());
 else
  sprintf(text, "X %03d Y %03d Theta %03d", odometryPoint.x / 10, odometryPoint.y / 10, theta * 180 / PI16);
 putText(image, text, Point(5, 15), FONT_HERSHEY_PLAIN, 1.0, Scalar::all(0), 1);
 putText(image, text, Point(6, 16), FONT_HERSHEY_PLAIN, 1.0, Scalar::all(confidence ? 255 : 128), 1);
}

void odometry(Point &odometryPoint, uint16_t &theta) {
#ifdef IMU
 theta = angleDoubleToAngle16(imuData.fusionPose.z()) * DIRZ + thetaCorrector;
#else
 theta += remoteFrame.vz * VZMUL;
#endif

 odometryPoint.x += (remoteFrame.vx * cos16(theta) - remoteFrame.vy * sin16(theta)) / ONE16 / VXDIV;
 odometryPoint.y += (remoteFrame.vx * sin16(theta) + remoteFrame.vy * cos16(theta)) / ONE16 / VYDIV;
}

void bgrInit() {
 for(uchar i = 0; i < 180; i++) {
  Mat imageHsv = Mat(1, 1, CV_8UC3, Scalar(i, 255, 255));
  Mat imageBgr;
  cvtColor(imageHsv, imageBgr, COLOR_HSV2BGR);
  hueToBgr[i] = imageBgr.at<Vec3b>(0, 0);
 }
}

int main(int argc, char* argv[]) {
 if(argc != 4) {
  width = WIDTH;
  height = HEIGHT;
  fps = FPS;
 } else {
  sscanf(argv[1], "%d", &width);
  sscanf(argv[2], "%d", &height);
  sscanf(argv[3], "%d", &fps);
 }

 int fd = serialOpen(DEVROBOT, DEVDEBIT);
 if(fd == -1) {
  fprintf(stderr, "Error opening serial port\n");
  return 1;
 }

 int ld = serialOpen(LIDARPORT, LIDARRATE);
 if(ld == -1) {
  fprintf(stderr, "Error opening serial port\n");
  return 1;
 }

#ifdef IMU
 thread imuThr(imuThread);
#endif

 startLidar(ld);

 Mat image;
 int size = width * height * 3;

 telemetryFrame.header[0] = '$';
 telemetryFrame.header[1] = 'R';
 telemetryFrame.header[2] = ' ';
 telemetryFrame.header[3] = ' ';

 Point odometryPoint = Point(0, 0);
 uint16_t theta = 0;
 vector<PolarPoint> polarPoints;
 vector<Point> robotPoints;
 vector<vector<Point>> robotRawLines;
 vector<Line> robotLines;
 vector<Line> mapLines;
 vector<Line> map;
 vector<Line> mapRobot;
 double refTilt[] = {0, 0};
 int refTiltInitDelay = REFTILTINITDELAY;
 bool confidence = false;

 bgrInit();

 VideoCapture capture;
 capture.open(0);
 capture.set(CAP_PROP_FRAME_WIDTH, width);
 capture.set(CAP_PROP_FRAME_HEIGHT, height);
 capture.set(CAP_PROP_FPS, fps);
 while(run) {
  capture.read(image);
  //usleep(20000);
  //image = Mat::zeros(Size(width, height), CV_8UC3);

  bool updated = readModem(fd, remoteFrame);

  if(updated)
   odometry(odometryPoint, theta);

  if(readLidar(ld, polarPoints)) {
   robotPoints.clear();
   lidarToRobot(polarPoints, robotPoints);

   robotRawLines.clear();
   extractRawLines(polarPoints, robotPoints, robotRawLines);

   robotLines.clear();
   fitLines(robotRawLines, robotLines);

   mapLines.clear();
   robotToMap(robotLines, mapLines, odometryPoint, theta);

   sort(mapLines.begin(), mapLines.end(), [](const Line &a, const Line &b) {
    return sqDist(a) > sqDist(b);
   });

   localization(mapLines, map, odometryPoint, theta, refTilt, confidence);
   mapping(mapLines, map, confidence);

   mapRobot.clear();
   mapToRobot(map, mapRobot, odometryPoint, theta);
  }

  ui(image, robotPoints, robotLines, map, mapRobot, odometryPoint, theta, refTilt, confidence);

  if(refTiltInitDelay == 1) {
   refTilt[0] = imuData.fusionPose.x();
   refTilt[1] = imuData.fusionPose.y();
  }

  if(refTiltInitDelay)
   refTiltInitDelay--;

  if(updated) {
   for(int i = 0; i < NBCOMMANDS; i++) {
    telemetryFrame.xy[i][0] = remoteFrame.xy[i][0];
    telemetryFrame.xy[i][1] = remoteFrame.xy[i][1];
   }
   telemetryFrame.z = remoteFrame.z;
   telemetryFrame.vx = remoteFrame.vx;
   telemetryFrame.vy = remoteFrame.vy;
   telemetryFrame.vz = remoteFrame.vz;
   telemetryFrame.switchs = remoteFrame.switchs;

   writeModem(fd, telemetryFrame);
  }

  fwrite(image.data, size, 1, stdout);
 }

 stopLidar(ld);

 return 0;
}
