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
 fprintf(stderr, "Starting IMU thread\n");

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

 fprintf(stderr, "Stopping IMU thread\n");
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

void extractRawLinesPascal(vector<PolarPoint> &polarPoints, vector<Point> &robotPoints, vector<vector<Point>> &robotRawLines) {
 vector<Point> pointsDp;
 vector<Point> pointsNoDp;
 Point oldRobotPoints = Point(0, 0);
 uint16_t angle = 2 * PI16 / polarPoints.size();

 approxPolyDP(robotPoints, pointsDp, EPSILON, true);

 for(int i = 0; i < robotPoints.size() * 2; i++) {
  int ii = i % robotPoints.size();

  bool dp = false;
  for(int j = 0; j < pointsDp.size(); j++) {
   if(robotPoints[ii] == pointsDp[j]) {
    dp = true;
    break;
   }
  }

  int sqDst = sqDist(robotPoints[ii], oldRobotPoints);
  oldRobotPoints = robotPoints[ii];

  int distMax = polarPoints[ii].distance * sin16(angle) * DISTCOEF / ONE16;
  if(distMax < DISTCLAMP)
   distMax = DISTCLAMP;

  if(dp || sqDst > distMax * distMax) {
   int size = pointsNoDp.size();
   if(size >= NBPOINTSMIN && i > size + 1 &&
      sqDist(pointsNoDp[0], pointsNoDp[size - 1]) >= DISTMIN * DISTMIN) {
    robotRawLines.push_back(pointsNoDp);
    if(i > robotPoints.size())
     break;
   }
   pointsNoDp.clear();

  } else
   pointsNoDp.push_back(robotPoints[ii]);
 }
}

void extractRawLinesMike118(vector<PolarPoint> &polarPoints, vector<Point> &robotPoints, vector<vector<Point>> &robotRawLines) {
 vector<Point> pointsDp;
 vector<Point> pointsNoDp;
 int i = 0;
 int j = 0;
 bool newLine = false;
 Point oldRobotPoints = robotPoints[robotPoints.size() - 1];
 uint16_t angle = 2 * PI16 / polarPoints.size();

 approxPolyDP(robotPoints, pointsDp, EPSILON, true);
 while(pointsDp[0] != robotPoints[i])
  i++;

 while(j < pointsDp.size() + 1) {
  int ii = i % robotPoints.size();
  int jj = j % pointsDp.size();

  if(robotPoints[ii] == pointsDp[jj]) {
   j++;
   newLine = true;
  } else {
   int sqDst = sqDist(robotPoints[ii], oldRobotPoints);

   int distMax = polarPoints[ii].distance * sin16(angle) * DISTCOEF / ONE16;
   if(distMax < DISTCLAMP)
    distMax = DISTCLAMP;

   if(sqDst < distMax * distMax)
    pointsNoDp.push_back(robotPoints[ii]);
   else
    newLine = true;
  }

  if(newLine) {
   if(pointsNoDp.size() >= NBPOINTSMIN &&
      sqDist(pointsNoDp[0], pointsNoDp[pointsNoDp.size() - 1]) >= DISTMIN * DISTMIN)
    robotRawLines.push_back(pointsNoDp);
   pointsNoDp.clear();
   newLine = false;
  }

  oldRobotPoints = robotPoints[ii];
  i++;
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
  pointsOut.push_back(Point(x, y));
 }
}

void robotToLidar(vector<Point> &pointsIn, vector<PolarPoint> &pointsOut) {
 for(int i = 0; i < pointsIn.size(); i++) {
  int distance = int(sqrt(sqNorm(pointsIn[i])));
  uint16_t theta = angleDoubleToAngle16(atan2(pointsIn[i].x, pointsIn[i].y));
  pointsOut.push_back({distance, theta});
 }
}

Point rotate(Point point, uint16_t theta) {
 return Point((point.x * cos16(theta) - point.y * sin16(theta)) / ONE16,
              (point.x * sin16(theta) + point.y * cos16(theta)) / ONE16);
}

void robotToMap(vector<Line> &linesIn, vector<Line> &linesOut, Point robotPoint, uint16_t robotTheta) {
 for(int i = 0; i < linesIn.size(); i++) {
  linesOut.push_back({robotPoint + rotate(linesIn[i].a, robotTheta),
                      robotPoint + rotate(linesIn[i].b, robotTheta)});
 }
}

/*void mapToRobot(vector<Line> &linesIn, vector<Line> &linesOut, Point robotPoint, uint16_t robotTheta) {
 for(int i = 0; i < linesIn.size(); i++) {
  linesOut.push_back({rotate(linesIn[i].a - robotPoint, -robotTheta),
                      rotate(linesIn[i].b - robotPoint, -robotTheta)});
 }
}*/

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
   if(line.growa)
    line.growa--;
   if(line.growa == 0) {
    grownLine.a = h;
    grownLine.growa = GROWFILTER;
    return true;
   }
  } else {
   if(line.growb)
    line.growb--;
   if(line.growb == 0) {
    grownLine.b = h;
    grownLine.growb = GROWFILTER;
    return true;
   }
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

bool testLines(Line line1, Line line2, int distTolerance, double angularTolerance, int lengthMargin,
               Point &pointError, double &angularError, int &distError, int &refNorm) {

 angularError = diffAngle(line1, line2);
 if(fabs(angularError) > angularTolerance)
  return false;

 pointError = pointDistancePointLine((line1.a + line1.b) / 2, line2);
 distError = int(sqrt(sqNorm(pointError)));
 if(distError > distTolerance)
  return false;

 refNorm = int(sqrt(sqDist(line2)));
 int distance1 = ratioPointLine(line1.a, line2) * refNorm;
 int distance2 = ratioPointLine(line1.b, line2) * refNorm;

 if(distance1 < -lengthMargin && distance2 < -lengthMargin ||
    distance1 > refNorm + lengthMargin && distance2 > refNorm + lengthMargin)
  return false;

 return true;
}

bool growLineMap(Point point, vector<Line> &map, int n) {
 Line grownLine;

 if(!growLine(point, map[n], grownLine))
  return false;

 bool found = false;
 for(int i = 0; i < map.size(); i++) {
  if(i == n)
   continue;

  Point pointError;
  double angularError;
  int distError;
  int refNorm;
  if(testLines(map[i], grownLine, LARGEDISTTOLERANCE, LARGEANGULARTOLERANCE, 0, pointError, angularError, distError, refNorm)) {
   if(distError < SMALLDISTTOLERANCE && fabs(angularError) < SMALLANGULARTOLERANCE) {

    if(sqDist(grownLine) > sqDist(map[i])) {
     growLine(map[i].a, grownLine, grownLine);
     growLine(map[i].b, grownLine, grownLine);
    } else {
     growLine(grownLine.a, map[i], grownLine);
     growLine(grownLine.b, map[i], grownLine);
    }
    map.erase(map.begin() + i);
    i--;

    for(int j = 0; j < map.size(); j++) {
     if(j == n)
      continue;

     if(testLines(map[j], grownLine, SMALLDISTTOLERANCE, SMALLANGULARTOLERANCE, 0, pointError, angularError, distError, refNorm)) {
      if(sqDist(grownLine) > sqDist(map[j]))
       map.erase(map.begin() + j);
      else {
       map.erase(map.begin() + n);
       break;
      }
      j--;
     }
    }

   } else
    found = true;

   break;
  }
 }

 if(!found) {
  map[n] = grownLine;
  return true;
 }

 return false;
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

void mapCleaner(vector<PolarPoint> &polarPoints, vector<Line> &map, Point robotPoint, uint16_t robotTheta) {
 vector<Point> closerPoints;

 for(int i = 0; i < polarPoints.size(); i++) {
  Point closerPoint = Point((polarPoints[i].distance / 2) * sin16(polarPoints[i].theta) / ONE16,
                            (polarPoints[i].distance / 2) * cos16(polarPoints[i].theta) / ONE16);

  closerPoints.push_back(robotPoint + rotate(closerPoint, robotTheta));
 }

 for(int i = 0; i < map.size(); i++) {
  bool shrinka = true;
  bool shrinkb = true;
  for(int j = 0; j < closerPoints.size(); j++) {
   Line shorterLine = {robotPoint, closerPoints[j]};

   Point intersectPoint;
   if(!intersect(shorterLine, map[i], intersectPoint))
    continue;

   double absAngularError = fabs(diffAngle(shorterLine, map[i]));
   if(absAngularError < SMALLANGULARTOLERANCE || absAngularError > M_PI - SMALLANGULARTOLERANCE)
    continue;

   if(sqDist(map[i].a, intersectPoint) < sqDist(map[i].b, intersectPoint)) {
    if(shrinka) {
     shrinka = false;
     map[i].shrinka--;
    }
    if(map[i].shrinka == 0)
     map[i].a = intersectPoint;
   } else {
    if(shrinkb) {
     shrinkb = false;
     map[i].shrinkb--;
    }
    if(map[i].shrinkb == 0)
     map[i].b = intersectPoint;
   }

   if(sqDist(map[i]) < DISTMIN * DISTMIN) {
    map.erase(map.begin() + i);
    i--;
   }
  }

  if(map[i].shrinka == 0)
   map[i].shrinka = SHRINKFILTER;
  if(map[i].shrinkb == 0)
   map[i].shrinkb = SHRINKFILTER;
 }
}

bool computeErrors(vector<Line> &mapLines, vector<Line> &map,
                   Point &pointErrorOut, double &angularErrorOut) {

 Point pointErrorSum = Point(0, 0);
 double angularErrorSum = 0.0;
 int n = 0;

 for(int i = 0; i < mapLines.size(); i++) {
  /*Line line = {Point(0, 0), Point((robotLines[i].a + robotLines[i].b) / 2)};
  double angle = fabs(diffAngle(line, robotLines[i]));
  if(angle < LARGEANGULARTOLERANCE || fabs(angle - M_PI) < LARGEANGULARTOLERANCE)
   continue;*/

  for(int j = 0; j < map.size(); j++) {
   Point pointError;
   double angularError;
   int distError;
   int refNorm;
   if(map[j].validation >= VALIDATIONFILTERKEEP &&
      testLines(mapLines[i], map[j], LARGEDISTTOLERANCE, LARGEANGULARTOLERANCE, 0, pointError, angularError, distError, refNorm)) {

    pointErrorSum += pointError;
    angularErrorSum += angularError;
    n++;
   }
  }
 }

 if(n) {
  pointErrorOut = pointErrorSum / n;
  angularErrorOut = angularErrorSum / n;

  return true;
 } else
  return false;
}

void mapping(vector<Line> &mapLines, vector<Line> &map) {
 vector<Line> newLines;
 bool change = false;

 for(int i = 0; i < mapLines.size(); i++) {
  /*Line line = {Point(0, 0), Point((robotLines[i].a + robotLines[i].b) / 2)};
  double angle = fabs(diffAngle(line, robotLines[i]));
  if(angle < LARGEANGULARTOLERANCE || fabs(angle - M_PI) < LARGEANGULARTOLERANCE)
   continue;*/

  bool newLine = true;
  for(int j = 0; j < map.size(); j++) {
   Point pointError;
   double angularError;
   int distError;
   int refNorm;
   if(!testLines(mapLines[i], map[j], LARGEDISTTOLERANCE, LARGEANGULARTOLERANCE * 2, 0, pointError, angularError, distError, refNorm))
    continue;

   newLine = false;

   if(distError > SMALLDISTTOLERANCE || fabs(angularError) > SMALLANGULARTOLERANCE)
    break;

   if(map[j].validation < VALIDATIONFILTERKEEP) {
    map[j].intega += mapLines[i].a;
    map[j].integb += mapLines[i].b;
    map[j].integ++;
    map[j].validation++;

    map[j].a = map[j].intega / map[j].integ;
    map[j].b = map[j].integb / map[j].integ;
    break;
   }

   bool merged = false;
   merged |= growLineMap(mapLines[i].a, map, j);
   merged |= growLineMap(mapLines[i].b, map, j);
   if(merged)
    change = true;
   break;
  }

  if(newLine) {
   newLines.push_back(mapLines[i]);
   change = true;
  }
 }

 for(int i = 0; i < newLines.size(); i++) {
  newLines[i].intega = newLines[i].a;
  newLines[i].integb = newLines[i].b;
  newLines[i].integ = 1;
  newLines[i].validation = 0;
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

   if(map[i].validation > VALIDATIONFILTERKILL && map[i].validation < VALIDATIONFILTERKEEP)
    map[i].validation--;
   else if(map[i].validation <= VALIDATIONFILTERKILL) {
    map.erase(map.begin() + i);
    i--;
   }

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

void localization(vector<Line> &robotLines, vector<Line> &mapLines, vector<Line> &map,
                  Point &robotPoint, uint16_t &robotTheta) {

 Point pointError = Point(0, 0);
 double angularError = 0.0;

 for(int i = 0; i < NBITERATIONS; i++) {
  mapLines.clear();
  robotToMap(robotLines, mapLines, robotPoint, robotTheta);

  if(computeErrors(mapLines, map, pointError, angularError)) {
   robotPoint -= pointError / (i + 1);

#ifdef IMU
   robotThetaCorrector += int(angularError * double(PI16) / M_PI) / IMUTHETACORRECTORDIV;
#else
   robotTheta += int(angularError * double(PI16) / M_PI) / (i + 1);
#endif

  }
 }
}

void drawLidarPoints(Mat &image, vector<Point> &points, bool beams, int mapDiv) {
 const Point centerPoint = Point(width / 2, height / 2);

 for(int i = 0; i < points.size(); i++) {
  Point point = points[i];
  point.x /= mapDiv;
  point.y /= -mapDiv;

  if(beams)
   line(image, centerPoint, centerPoint + point, Scalar::all(i ? 64 : 255), 1, LINE_AA);
  else
   circle(image, centerPoint + point, i ? 1 : 3, i ? Scalar::all(255) : Scalar(0, 0, 255), FILLED, LINE_AA);
 }
}

void drawMap(Mat &image, vector<Line> &map, bool colored, Point robotPoint, uint16_t robotTheta, int mapDiv) {
 const Point centerPoint = Point(width / 2, height / 2);

 for(int i = 0; i < map.size(); i++) {
  Point point1 = rotate(map[i].a - robotPoint, -robotTheta);
  Point point2 = rotate(map[i].b - robotPoint, -robotTheta);

  point1.x /= mapDiv;
  point2.x /= mapDiv;
  point1.y /= -mapDiv;
  point2.y /= -mapDiv;
  point1 += centerPoint;
  point2 += centerPoint;

  Scalar color;
  if(colored) {
   if(map[i].validation < VALIDATIONFILTERKEEP)
    color = Scalar::all(mapInteger(map[i].validation, VALIDATIONFILTERKILL, VALIDATIONFILTERKEEP, 0, 255));
   else {
    Point diff = map[i].b - map[i].a;
    double angleDeg = atan2(diff.y, diff.x) * 180.0 / M_PI;
    uchar hue = uchar(angleDeg / 2.0 + 90.0) % 180;
    color = hueToBgr[hue];
   }
  } else
   color = Scalar::all(255);

  line(image, point1, point2, color, 2, LINE_AA);
 }
}

void drawHist(Mat &image, Point robotPoint, uint16_t robotTheta, int mapDiv) {
 static Point hist[HIST] = {Point(0, 0)};
 static int n = 0;
 const Point centerPoint = Point(width / 2, height / 2);
 static Point oldPoint = Point(0, 0);

 hist[n++] = robotPoint;
 if(n == HIST)
  n = 0;

 for(int i = 0; i < HIST; i++) {
  Point point = rotate(hist[(i + n) % HIST] - robotPoint, -robotTheta);
  point.x /= mapDiv;
  point.y /= -mapDiv;
  point += centerPoint;

  if(i != 0) {
   int sqDistTolerancePixels = LARGEDISTTOLERANCE / mapDiv;
   if(sqDist(oldPoint, point) < sqDistTolerancePixels * sqDistTolerancePixels)
    line(image, oldPoint, point, Scalar::all(128), 1, LINE_AA);
  }

  oldPoint = point;
 }
}

void drawPatrolPoints(Mat &image, vector<Point> &points, Point robotPoint, uint16_t robotTheta, int mapDiv) {
 const Point centerPoint = Point(width / 2, height / 2);

 for(int i = 0; i < points.size(); i++) {
  Point point = rotate(points[i] - robotPoint, -robotTheta);
  point.x /= mapDiv;
  point.y /= -mapDiv;
  point += centerPoint;

  char text[8];
  sprintf(text, "%d", i);

  int baseline;
  Size textSize = getTextSize(text, FONT_HERSHEY_PLAIN, 1.0, 1, &baseline);
  Point textPoint = Point(-textSize.width / 2, textSize.height / 2) + point;

  putText(image, text, textPoint, FONT_HERSHEY_PLAIN, 1.0, Scalar::all(0), 1);
  putText(image, text, textPoint + Point(1, 1), FONT_HERSHEY_PLAIN, 1.0, Scalar::all(255), 1);
 }
}

void drawRobot(Mat &image, vector<Point> robotIcon, int thickness, int mapDiv) {
 const Point centerPoint = Point(width / 2, height / 2);

 vector<Point> polygon;
 for(int i = 0; i < robotIcon.size(); i++) {
  Point point = robotIcon[i];
  point.x /= mapDiv;
  point.y /= -mapDiv;
  point += centerPoint;
  polygon.push_back(point);
 }

 vector<vector<Point>> tmp(1, polygon);
 drawContours(image, tmp, -1, Scalar::all(255), thickness, LINE_AA);
}

void ui(Mat &image, vector<Point> &robotPoints, vector<Line> &robotLines,
                    vector<Line> &map, vector<Point> &patrolPoints,
                    Point &robotPoint, Point &oldRobotPoint,
                    uint16_t &robotTheta, uint16_t &oldRobotTheta, int time) {

 bool buttonLess = remoteFrame.switchs & 0b00010000;
 bool buttonMore = remoteFrame.switchs & 0b00100000;
 bool buttonOk = remoteFrame.switchs & 0b10000000;
 bool buttonCancel = remoteFrame.switchs & 0b01000000;
 static bool oldButtonLess = false;
 static bool oldButtonMore = false;
 static bool oldButtonOk = false;
 static bool oldButtonCancel = false;
 static int buttonOkCount = 0;
 static int buttonCancelCount = 0;
 static bool tune = false;
 static int select = SELECTMAPPOINTS;
 static int mapDiv = MAPDIV;

 if(buttonOk) {
  buttonOkCount++;
  if(buttonOkCount == 15)
   tune = !tune;

 } else if(buttonCancel) {
  buttonCancelCount++;
  if(buttonCancelCount == 15) {

   map.clear();
   robotPoint = Point(0, 0);
   oldRobotPoint = Point(0, 0);
   robotTheta = 0;
   oldRobotTheta = 0;
#ifdef IMU
   imu->resetFusion();
   robotThetaCorrector = 0;
#endif

  }
 } else if(!buttonOk && oldButtonOk) {
  if(buttonOkCount < 15) {

   bool found = false;
   for(int i = 0; i < patrolPoints.size(); i++) {
    if(sqDist(robotPoint, patrolPoints[i]) < SMALLDISTTOLERANCE * SMALLDISTTOLERANCE) {
     patrolPoints[i] = robotPoint;
     found = true;
     break;
    }
   }
   if(!found)
    patrolPoints.push_back(robotPoint);

  }
  buttonOkCount = 0;
 } else if(!buttonCancel && oldButtonCancel) {
  if(buttonCancelCount < 15 && !patrolPoints.empty()) {

   bool found = false;
   for(int i = 0; i < patrolPoints.size(); i++) {
    if(sqDist(robotPoint, patrolPoints[i]) < SMALLDISTTOLERANCE * SMALLDISTTOLERANCE) {
     patrolPoints.erase(patrolPoints.begin() + i);
     found = true;
     break;
    }
   }
   if(!found)
    patrolPoints.pop_back();

  }
  buttonCancelCount = 0;
 }

 if(tune) {
  if(buttonLess && mapDiv < MAPDIVMAX)
   mapDiv++;
  else if(buttonMore && mapDiv > MAPDIVMIN)
   mapDiv--;
 } else {
  if(!buttonMore && oldButtonMore) {
   if(select < SELECTPOINTSBEAMS)
    select++;
   else
    select = SELECTNONE;
  } else if(!buttonLess && oldButtonLess) {
   if(select > SELECTNONE)
    select--;
   else
    select = SELECTPOINTSBEAMS;
  }
 }

 oldButtonLess = buttonLess;
 oldButtonMore = buttonMore;
 oldButtonCancel = buttonCancel;
 oldButtonOk = buttonOk;

 char text[80];
 switch(select) {
  case SELECTNONE:
   drawPatrolPoints(image, patrolPoints, robotPoint, robotTheta, mapDiv);
   drawRobot(image, robotIcon, 1, mapDiv);
   if(tune)
    sprintf(text, "%d mm per pixel", mapDiv);
   else
    sprintf(text, "");
   break;

  case SELECTMAP:
   drawMap(image, map, true, robotPoint, robotTheta, mapDiv);
   drawHist(image, robotPoint, robotTheta, mapDiv);
   drawPatrolPoints(image, patrolPoints, robotPoint, robotTheta, mapDiv);
   drawRobot(image, robotIcon, 1, mapDiv);
   if(tune)
    sprintf(text, "%d mm per pixel", mapDiv);
   else
    sprintf(text, "X %04d Y %04d Theta %03d", robotPoint.x, robotPoint.y, robotTheta * 180 / PI16);
   break;

  case SELECTMAPPOINTS:
   drawLidarPoints(image, robotPoints, false, mapDiv);
   drawMap(image, robotLines, false, Point(0, 0), 0.0, mapDiv);
   drawMap(image, map, true, robotPoint, robotTheta, mapDiv);
   drawHist(image, robotPoint, robotTheta, mapDiv);
   drawPatrolPoints(image, patrolPoints, robotPoint, robotTheta, mapDiv);
   drawRobot(image, robotIcon, 1, mapDiv);
   if(tune)
    sprintf(text, "%d mm per pixel", mapDiv);
   else
    sprintf(text, "X %04d Y %04d Theta %03d", robotPoint.x, robotPoint.y, robotTheta * 180 / PI16);
   break;

  case SELECTMAPPOINTSBEAMS:
   drawLidarPoints(image, robotPoints, true, mapDiv);
   drawMap(image, robotLines, false, Point(0, 0), 0.0, mapDiv);
   drawMap(image, map, true, robotPoint, robotTheta, mapDiv);
   drawRobot(image, robotIcon, FILLED, mapDiv);
   if(tune)
    sprintf(text, "%d mm per pixel", mapDiv);
   else
    sprintf(text, "%d points %d lines %d map lines %d ms", robotPoints.size(), robotLines.size(), map.size(), time);
   break;

  case SELECTPOINTSBEAMS:
   drawLidarPoints(image, robotPoints, true, mapDiv);
   drawMap(image, robotLines, false, Point(0, 0), 0.0, mapDiv);
   drawRobot(image, robotIcon, FILLED, mapDiv);
   if(tune)
    sprintf(text, "%d mm per pixel", mapDiv);
   else
    sprintf(text, "%d points %d lines %d map lines %d ms", robotPoints.size(), robotLines.size(), map.size(), time);
   break;
 }

 putText(image, text, Point(5, 15), FONT_HERSHEY_PLAIN, 1.0, Scalar::all(0), 1);
 putText(image, text, Point(6, 16), FONT_HERSHEY_PLAIN, 1.0, Scalar::all(255), 1);
}

void bgrInit() {
 for(uchar i = 0; i < 180; i++) {
  Mat imageHsv = Mat(1, 1, CV_8UC3, Scalar(i, 255, 255));
  Mat imageBgr;
  cvtColor(imageHsv, imageBgr, COLOR_HSV2BGR);
  hueToBgr[i] = imageBgr.at<Vec3b>(0, 0);
 }
}

void dedistortTheta(vector<PolarPoint> &polarPoints, uint16_t robotTheta, uint16_t &oldRobotTheta) {
 int16_t size = polarPoints.size();
 int16_t deltaTheta = robotTheta - oldRobotTheta;

 for(int i = 0; i < size; i++)
  polarPoints[i].theta += (size - i) * deltaTheta / size;

 //while(polarPoints[0].theta > PI16 && polarPoints.size() > 1)
 while(polarPoints[0].theta - polarPoints[polarPoints.size() - 1].theta > PI16 && polarPoints.size() > 1)
  polarPoints.erase(polarPoints.begin());

 oldRobotTheta = robotTheta;
}

/*void dedistortLinear(vector<Point> &robotPoints, Point robotPoint, Point &oldRobotPoint, uint16_t robotTheta) {
 int size = robotPoints.size();
 Point delta = robotPoint - oldRobotPoint;

 for(int i = 0; i < size; i++) {
  Point correction = (size - i) * delta / size;
  robotPoints[i] -= rotate(correction, -robotTheta);
 }

 oldRobotPoint = robotPoint;
}*/

void writeMapFile(vector<Line> &map, vector<Point> patrolPoints, Point robotPoint, uint16_t robotTheta) {
 FileStorage fs(MAPFILE, FileStorage::WRITE);

 if(fs.isOpened()) {
  fs << "map" << "[";
  for(int i = 0; i < map.size(); i++) {
   fs << "{";
   fs << "a" << map[i].a;
   fs << "b" << map[i].b;
   fs << "}";
  }
  fs << "]";

  fs << "patrolPoints" << "[";
  for(int i = 0; i < patrolPoints.size(); i++)
   fs << patrolPoints[i];
  fs << "]";

  fs << "robotPoint" << robotPoint;
  fs << "robotTheta" << robotTheta;

  fs.release();
 } else
  fprintf(stderr, "Error writing map file\n");
}

void readMapFile(vector<Line> &map, vector<Point> &patrolPoints, Point &robotPoint, uint16_t &robotTheta) {
 FileStorage fs(MAPFILE, FileStorage::READ);

 if(fs.isOpened()) {
  FileNode fn1 = fs["map"];
  for(FileNodeIterator it = fn1.begin(); it != fn1.end(); it++) {
   FileNode item = *it;
   Point a;
   Point b;
   item["a"] >> a;
   item["b"] >> b;
   map.push_back({a, b, Point(0, 0), Point(0, 0), 0, VALIDATIONFILTERKEEP,
                  GROWFILTER, GROWFILTER, SHRINKFILTER, SHRINKFILTER});
  }

  FileNode fn2 = fs["patrolPoints"];
  for(FileNodeIterator it = fn2.begin(); it != fn2.end(); it++) {
   FileNode item = *it;
   Point point;
   item >> point;
   patrolPoints.push_back(point);
  }

  fs["robotPoint"] >> robotPoint;
  fs["robotTheta"] >> robotTheta;

  fs.release();
 } else
  fprintf(stderr, "Error reading map file\n");
}

void autopilot(vector<Point> patrolPoints, Point &robotPoint, uint16_t &robotTheta) {
 static bool enabled = false;
 int vx;
 int vy;
 int vz;

 if(patrolPoints.size() >= 2 && sqDist(robotPoint, patrolPoints[0]) < SMALLDISTTOLERANCE * SMALLDISTTOLERANCE)
  enabled = true;
 else if(remoteFrame.vx || remoteFrame.vy || remoteFrame.vz)
  enabled = false;

 if(!enabled) {
  telemetryFrame.vx = remoteFrame.vx;
  telemetryFrame.vy = remoteFrame.vy;
  telemetryFrame.vz = remoteFrame.vz;
  return;
 }


 // TODO
 vx = 0;
 vy = 0;
 vz = 0;


 telemetryFrame.vx = vx;
 telemetryFrame.vy = vy;
 telemetryFrame.vz = vz;

#ifndef IMU
 robotTheta += vz * VZMUL;
#endif

 Point point = rotate(Point(vx, vy), robotTheta);
 point.x /= VXDIV;
 point.y /= VYDIV;
 robotPoint += point;
}

int main(int argc, char* argv[]) {
 fprintf(stderr, "Starting\n");

 signal(SIGTERM, signal_callback_handler);

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
  fprintf(stderr, "Error opening Vigibot serial port\n");
  return 1;
 }

 int ld = serialOpen(LIDARPORT, LIDARRATE);
 if(ld == -1) {
  fprintf(stderr, "Error opening lidar serial port\n");
  return 1;
 }

#ifdef IMU
 thread imuThr(imuThread);
#endif

 fprintf(stderr, "Starting lidar\n");
 startLidar(ld);

 Mat image;
 int size = width * height * 3;

 telemetryFrame.header[0] = '$';
 telemetryFrame.header[1] = 'R';
 telemetryFrame.header[2] = ' ';
 telemetryFrame.header[3] = ' ';

 vector<PolarPoint> polarPoints;
 vector<Point> robotPoints;
 vector<vector<Point>> robotRawLines;
 vector<Line> robotLines;
 vector<Line> mapLines;
 vector<Line> map;
 vector<Point> patrolPoints;

 Point robotPoint = Point(0, 0);
 uint16_t robotTheta = 0;
 fprintf(stderr, "Reading map file\n");
 readMapFile(map, patrolPoints, robotPoint, robotTheta);
 Point oldRobotPoint = robotPoint;
 uint16_t oldRobotTheta = robotTheta;
 robotThetaCorrector = robotTheta;

 bgrInit();

 fprintf(stderr, "Starting capture\n");
 VideoCapture capture;
 capture.open(0);

 TickMeter tickMeter;
 int time = 0;

 bool captureEnabled = capture.isOpened();
 if(captureEnabled) {
  fprintf(stderr, "Configuring capture\n");
  capture.set(CAP_PROP_FRAME_WIDTH, width);
  capture.set(CAP_PROP_FRAME_HEIGHT, height);
  capture.set(CAP_PROP_FPS, fps);
 } else
  fprintf(stderr, "Error starting capture\n");

 while(run) {
  if(captureEnabled) {
   capture.read(image);
  } else {
   image = Mat::zeros(Size(width, height), CV_8UC3);
   int wait = 1000000 / fps - time * 1000;
   if(wait > 0)
    usleep(wait);
  }

  tickMeter.start();

  bool updated = readModem(fd, remoteFrame);

#ifdef IMU
  robotTheta = angleDoubleToAngle16(imuData.fusionPose.z() * DIRZ) + robotThetaCorrector;
#else
  robotTheta += remoteFrame.vz * VZMUL;
#endif

  Point point = rotate(Point(remoteFrame.vx, remoteFrame.vy), robotTheta);
  point.x /= VXDIV;
  point.y /= VYDIV;
  robotPoint += point;

  if(readLidar(ld, polarPoints)) {
   dedistortTheta(polarPoints, robotTheta, oldRobotTheta);

   robotPoints.clear();
   lidarToRobot(polarPoints, robotPoints);

   //dedistortLinear(robotPoints, robotPoint, oldRobotPoint, robotTheta);
   //polarPoints.clear();
   //robotToLidar(robotPoints, polarPoints);

   robotRawLines.clear();
   extractRawLinesMike118(polarPoints, robotPoints, robotRawLines);

   robotLines.clear();
   fitLines(robotRawLines, robotLines);

   sort(robotLines.begin(), robotLines.end(), [](const Line &a, const Line &b) {
    return sqDist(a) > sqDist(b);
   });

   localization(robotLines, mapLines, map, robotPoint, robotTheta);
   mapping(mapLines, map);
   mapCleaner(polarPoints, map, robotPoint, robotTheta);
   mapFiltersDecay(map);
  }

  autopilot(patrolPoints, robotPoint, robotTheta);

  ui(image, robotPoints, robotLines, map, patrolPoints, robotPoint, oldRobotPoint, robotTheta, oldRobotTheta, time);

  if(updated) {
   for(int i = 0; i < NBCOMMANDS; i++) {
    telemetryFrame.xy[i][0] = remoteFrame.xy[i][0];
    telemetryFrame.xy[i][1] = remoteFrame.xy[i][1];
   }
   telemetryFrame.z = remoteFrame.z;
   telemetryFrame.switchs = remoteFrame.switchs;

   writeModem(fd, telemetryFrame);
  }

  fwrite(image.data, size, 1, stdout);

  tickMeter.stop();
  time = tickMeter.getTimeMilli();
  tickMeter.reset();

  if(time > 1000 / fps)
   fprintf(stderr, "Timeout error %d ms\n", time);
 }

 if(captureEnabled) {
  fprintf(stderr, "Stopping capture\n");
  capture.release();
 }

 fprintf(stderr, "Stopping lidar\n");
 stopLidar(ld);

 fprintf(stderr, "Writing map file\n");
 writeMapFile(map, patrolPoints, robotPoint, robotTheta);

 fprintf(stderr, "Stopping\n");
 return 0;
}
