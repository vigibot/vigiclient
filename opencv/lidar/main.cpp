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
 imu->setSlerpPower(IMUSLERPPOWER);
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
      sqDist(pointsNoDp[0], pointsNoDp[size - 1]) >= DISTMIN * DISTMIN) {
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

double ratioPointLine(Point point, Line line) {
 Point diff = line.b - line.a;
 double scalarProduct;
 double ratio = 0.0;

 if(diff.x || diff.y) {
  scalarProduct = (point.x - line.a.x) * diff.x +
                  (point.y - line.a.y) * diff.y;
  ratio = scalarProduct / sqNorm(diff);
 }

 return ratio;
}

bool growLine(Point point, Line &line, Line &grownLine) {
 Point diff = line.b - line.a;
 double ratio = ratioPointLine(point, line);
 Point h;
 grownLine = line;

 if(ratio < 0 || ratio > 1) {
  h.x = line.a.x + int(double(diff.x) * ratio);
  h.y = line.a.y + int(double(diff.y) * ratio);

  if(ratio < 0) {
   if(line.growa == 0) {
    grownLine.a = h;
    line.growa = GROWFILTER;
    return true;
   } else
    line.growa--;
  } else {
   if(line.growb == 0) {
    grownLine.b = h;
    line.growb = GROWFILTER;
    return true;
   } else
    line.growb--;
  }
 }

 return false;
}

double lineAngle(Line line) {
 Point diff = line.b - line.a;
 return atan2(diff.y, diff.x);
}

double diffAngle(Line line1, Line line2) {
 double angle1 = lineAngle(line1);
 double angle2 = lineAngle(line2);
 double result = angle2 - angle1;

 if(result > M_PI)
  result -= 2.0 * M_PI;
 else if(result < -M_PI)
  result += 2.0 * M_PI;

 return result;
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

bool testLines(Line line1, Line line2, double &angularError,
               Point &pointError, int &distError, int &refNorm) {

 angularError = diffAngle(line1, line2);
 if(fabs(angularError) > LARGEANGULARTOLERANCE)
  return false;

 pointError = pointDistancePointLine((line1.a + line1.b) / 2, line2);
 distError = int(sqrt(sqNorm(pointError)));
 if(distError > LARGEDISTTOLERANCE)
  return false;

 refNorm = int(sqrt(sqDist(line2)));
 int distance1 = ratioPointLine(line1.a, line2) * refNorm;
 int distance2 = ratioPointLine(line1.b, line2) * refNorm;
 if((distance1 < 0 || distance1 > refNorm) &&
    (distance2 < 0 || distance2 > refNorm) &&
    distance1 * distance2 > 0)
  return false;

 return true;
}

bool growLineMap(Point point, vector<Line> &map, int n) {
 Line grownLine;

 if(!growLine(point, map[n], grownLine))
  return false;

 for(int i = 0; i < map.size(); i++) {
  if(i == n)
   continue;

  double angularError;
  Point pointError;
  int distError;
  int refNorm;
  if(!testLines(map[i], grownLine, angularError, pointError, distError, refNorm))
   continue;

  map.erase(map.begin() + i);
 }

 map[n] = grownLine;

 return true;
}

bool intersect(Line line1, Line line2, Point &intersectPoint) {
 double det = (line1.a.x - line1.b.x) * (line2.a.y - line2.b.y) -
              (line1.a.y - line1.b.y) * (line2.a.x - line2.b.x);

 if(det == 0)
  return false;

 double pre = line1.a.x * line1.b.y - line1.a.y * line1.b.x;
 double post = line2.a.x * line2.b.y - line2.a.y * line2.b.x;
 double x = (pre * (line2.a.x - line2.b.x) - (line1.a.x - line1.b.x) * post) / det;
 double y = (pre * (line2.a.y - line2.b.y) - (line1.a.y - line1.b.y) * post) / det;

 if(x < min(line1.a.x, line1.b.x) || x > max(line1.a.x, line1.b.x) ||
    x < min(line2.a.x, line2.b.x) || x > max(line2.a.x, line2.b.x))
  return false;

 if(y < min(line1.a.y, line1.b.y) || y > max(line1.a.y, line1.b.y) ||
    y < min(line2.a.y, line2.b.y) || y > max(line2.a.y, line2.b.y))
  return false;

 intersectPoint = Point(x, y);

 return true;
}

void mapCleaner(vector<PolarPoint> &polarPoints, vector<Line> &map, Point odometryPoint, uint16_t theta) {
 vector<Point> closerPoints;

 for(int i = 0; i < polarPoints.size(); i++) {
  Point closerPoint = Point((polarPoints[i].distance - LARGEDISTTOLERANCE) * sin16(polarPoints[i].theta) / ONE16,
                            (polarPoints[i].distance - LARGEDISTTOLERANCE) * cos16(polarPoints[i].theta) / ONE16);

  closerPoints.push_back(Point(odometryPoint.x + (closerPoint.x * cos16(theta) - closerPoint.y * sin16(theta)) / ONE16,
                               odometryPoint.y + (closerPoint.x * sin16(theta) + closerPoint.y * cos16(theta)) / ONE16));
 }

 for(int i = 0; i < closerPoints.size(); i++) {
  Line shorterLine = {odometryPoint, closerPoints[i]};
  for(int j = 0; j < map.size(); j++) {

   Point intersectPoint;
   if(intersect(shorterLine, map[j], intersectPoint)) {

    double absAngularError = fabs(diffAngle(shorterLine, map[j]));
    if(absAngularError < LARGEANGULARTOLERANCE ||
       absAngularError > M_PI - LARGEANGULARTOLERANCE)
     continue;

    if(sqDist(map[j].a, intersectPoint) < sqDist(map[j].b, intersectPoint)) {
     if(map[j].shrinka == 0) {
      map[j].a = intersectPoint;
      map[j].shrinka = SHRINKFILTER;
     } else
      map[j].shrinka--;
    } else {
     if(map[j].shrinkb == 0) {
      map[j].b = intersectPoint;
      map[j].shrinkb = SHRINKFILTER;
     } else
      map[j].shrinkb--;
    }

    if(sqDist(map[j]) < DISTMIN * DISTMIN) {
     map.erase(map.begin() + j);
     j--;
    }
   }

  }
 }

}

bool computeErrors(vector<Line> &robotLines, vector<Line> &lines, vector<Line> &map,
                   Point &pointErrorOut, double &angularErrorOut) {

 Point pointErrorSum = Point(0, 0);
 double angularErrorSum = 0.0;
 int weightSum = 0;

 for(int i = 0; i < lines.size(); i++) {
  Line line = {Point(0, 0), Point((robotLines[i].a + robotLines[i].b) / 2)};
  double angle = fabs(diffAngle(line, robotLines[i]));
  if(angle < LARGEANGULARTOLERANCE || fabs(angle - M_PI) < LARGEANGULARTOLERANCE)
   continue;

  for(int j = 0; j < map.size(); j++) {
   double angularError;
   Point pointError;
   int distError;
   int refNorm;
   if(!testLines(lines[i], map[j], angularError, pointError, distError, refNorm))
    continue;

   pointErrorSum += pointError * refNorm;
   angularErrorSum += angularError * refNorm;
   weightSum += refNorm;
  }
 }

 if(weightSum) {
  pointErrorOut = pointErrorSum / weightSum;
  angularErrorOut = angularErrorSum / weightSum;

  return true;
 } else
  return false;
}

void mapping(vector<Line> &robotLines, vector<Line> &lines, vector<Line> &map) {
 vector<Line> newLines;
 bool change = false;

 for(int i = 0; i < lines.size(); i++) {
  bool newLine = true;

  Line line = {Point(0, 0), Point((robotLines[i].a + robotLines[i].b) / 2)};
  double angle = fabs(diffAngle(line, robotLines[i]));
  if(angle < LARGEANGULARTOLERANCE || fabs(angle - M_PI) < LARGEANGULARTOLERANCE)
   continue;

  for(int j = 0; j < map.size(); j++) {
   double angularError;
   Point pointError;
   int distError;
   int refNorm;
   if(!testLines(lines[i], map[j], angularError, pointError, distError, refNorm))
    continue;

   newLine = false;

   if(fabs(angularError) > SMALLANGULARTOLERANCE)
    continue;

   if(distError > SMALLDISTTOLERANCE)
    continue;

   bool merged = false;
   merged |= growLineMap(lines[i].a, map, j);
   merged |= growLineMap(lines[i].b, map, j);
   if(!merged)
    continue;
   else
    change = true;

   break;
  }

  if(newLine) {
   newLines.push_back(lines[i]);
   change = true;
  }
 }

 for(int i = 0; i < newLines.size(); i++) {
  newLines[i].growa = GROWFILTER;
  newLines[i].growb = GROWFILTER;
  newLines[i].shrinka = SHRINKFILTER;
  newLines[i].shrinkb = SHRINKFILTER;
  map.push_back(newLines[i]);
 }

 if(change) {
  sort(map.begin(), map.end(), [](const Line &a, const Line &b) {
   return sqDist(a) > sqDist(b);
  });
 }
}

void mapFiltersDecay(vector<Line> &map) {
 static int n = 0;

 if(n == MAPFILTERSDECAY) {
  for(int i = 0; i < map.size(); i++) {
   if(map[i].growa < GROWFILTER)
    map[i].growa++;
   if(map[i].growb < GROWFILTER)
    map[i].growb++;
   if(map[i].shrinka < SHRINKFILTER)
     map[i].shrinka++;
   if(map[i].shrinkb < SHRINKFILTER)
    map[i].shrinkb++;
  }
  n = 0;
 }

 n++;
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

void watch(Mat &image, double angle, Point center, int diam, Scalar color1, Scalar color2) {
 double angleDeg = angle * 180.0 / M_PI;
 ellipse(image, center, Point(diam, diam), angleDeg, 0.0, 180.0, color1, FILLED, LINE_AA);
 ellipse(image, center, Point(diam, diam), angleDeg, 0.0, -180.0, color2, FILLED, LINE_AA);
}

void ui(Mat &image, vector<Point> &robotPoints, vector<Line> &robotLines,
                    vector<Line> &map, vector<Line> &mapRobot,
                    Point &odometryPoint, uint16_t &theta, bool confidence) {

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
 static int mapDiv = MAPDIV;

 if(!buttonReset && oldButtonReset) {
  map.clear();
  odometryPoint = Point(0, 0);
  theta = 0;

#ifdef IMU
  imu->resetFusion();
  thetaCorrector = 0;
#endif
 }

 if(select != SELECTNONE &&
    !buttonOk && oldButtonOk)
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
  sprintf(text, "X %04d Y %04d Theta %03d", odometryPoint.x, odometryPoint.y, theta * 180 / PI16);
 putText(image, text, Point(5, 15), FONT_HERSHEY_PLAIN, 1.0, Scalar::all(0), 1);
 putText(image, text, Point(6, 16), FONT_HERSHEY_PLAIN, 1.0, Scalar::all(confidence ? 255 : 128), 1);
}

void odometry(Point &odometryPoint, uint16_t &theta) {
#ifdef IMU
 theta = angleDoubleToAngle16(imuData.fusionPose.z() * DIRZ) + thetaCorrector;
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

bool computeConfidence(Point pointError, double angularError) {
 static int delay = 0;
 bool moveFast = abs(remoteFrame.vx) > CONFIDENCEMAXVELOCITY ||
                 abs(remoteFrame.vy) > CONFIDENCEMAXVELOCITY ||
                 abs(remoteFrame.vz) > CONFIDENCEMAXANGULARVELOCITY;

 if(moveFast)
  delay = CONFIDENCEDELAY;

 if(delay)
  delay--;

 if(!delay && sqNorm(pointError) < CONFIDENCEDISTTOLERANCE * CONFIDENCEDISTTOLERANCE &&
              fabs(angularError) < CONFIDENCEANGULARTOLERANCE)
  return true;

 return false;
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
 bool confidence = false;

 bgrInit();

 VideoCapture capture;
 capture.open(0);
 capture.set(CAP_PROP_FRAME_WIDTH, width);
 capture.set(CAP_PROP_FRAME_HEIGHT, height);
 capture.set(CAP_PROP_FPS, fps);
 while(run) {
  capture.read(image);
  //usleep(10000);
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

   sort(robotLines.begin(), robotLines.end(), [](const Line &a, const Line &b) {
    return sqDist(a) > sqDist(b);
   });

   Point pointError = Point(0, 0);
   double angularError = 0.0;
   for(int i = 0; i < NBITERATIONS; i++) {
    mapLines.clear();
    robotToMap(robotLines, mapLines, odometryPoint, theta);

    if(computeErrors(robotLines, mapLines, map,
                     pointError, angularError)) {

     odometryPoint -= pointError / ODOMETRYCORRECTORDIV;
#ifdef IMU
     thetaCorrector += int(angularError * double(PI16) / M_PI) / IMUTHETACORRECTORDIV;
#else
     theta += int(angularError * double(PI16) / M_PI) / THETACORRECTORDIV;
#endif
    }
   }

   confidence = computeConfidence(pointError, angularError);

   if(confidence || !map.size()) {
    mapping(robotLines, mapLines, map);
    mapCleaner(polarPoints, map, odometryPoint, theta);
   }

   mapFiltersDecay(map);

   mapRobot.clear();
   mapToRobot(map, mapRobot, odometryPoint, theta);
  }

  ui(image, robotPoints, robotLines, map, mapRobot, odometryPoint, theta, confidence);

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
