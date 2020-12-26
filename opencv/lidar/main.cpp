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

void robotToMap(vector<Line> &linesIn, vector<Line> &linesOut, Point odometryPoint, uint16_t theta) {
 for(int i = 0; i < linesIn.size(); i++) {
  linesOut.push_back({odometryPoint + rotate(linesIn[i].a, theta),
                      odometryPoint + rotate(linesIn[i].b, theta)});
 }
}

void mapToRobot(vector<Line> &linesIn, vector<Line> &linesOut, Point odometryPoint, uint16_t theta) {
 for(int i = 0; i < linesIn.size(); i++) {
  linesOut.push_back({rotate(linesIn[i].a - odometryPoint, -theta),
                      rotate(linesIn[i].b - odometryPoint, -theta)});
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
   if(line.growa)
    line.growa--;
   if(line.growa == 0) {
    grownLine.a = h;
    return true;
   }
  } else {
   if(line.growb)
    line.growb--;
   if(line.growb == 0) {
    grownLine.b = h;
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

   if(testLines(map[i], grownLine, SMALLDISTTOLERANCE, SMALLANGULARTOLERANCE, 0, pointError, angularError, distError, refNorm)) {
    if(sqDist(grownLine) > sqDist(map[i])) {
     growLine(map[i].a, grownLine, grownLine);
     growLine(map[i].b, grownLine, grownLine);
    } else {
     growLine(grownLine.a, map[i], grownLine);
     growLine(grownLine.b, map[i], grownLine);
    }
    map.erase(map.begin() + i);
    i--;
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

void mapCleaner(vector<PolarPoint> &polarPoints, vector<Line> &map, Point odometryPoint, uint16_t theta) {
 vector<Point> closerPoints;

 for(int i = 0; i < polarPoints.size(); i++) {
  if(polarPoints[i].distance < LARGEDISTTOLERANCE)
   continue;

  Point closerPoint = Point((polarPoints[i].distance - LARGEDISTTOLERANCE) * sin16(polarPoints[i].theta) / ONE16,
                            (polarPoints[i].distance - LARGEDISTTOLERANCE) * cos16(polarPoints[i].theta) / ONE16);

  closerPoints.push_back(odometryPoint + rotate(closerPoint, theta));
 }

 for(int i = 0; i < map.size(); i++) {
  bool shrinka = true;
  bool shrinkb = true;
  for(int j = 0; j < closerPoints.size(); j++) {
   Line shorterLine = {odometryPoint, closerPoints[j]};

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

   if(fabs(angularError) > SMALLANGULARTOLERANCE || distError > SMALLDISTTOLERANCE)
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

 for(int i = 0; i < map.size(); i++) {
  if(map[i].growa == 0)
   map[i].growa = GROWFILTER;
  if(map[i].growb == 0)
   map[i].growb = GROWFILTER;
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
                  Point &odometryPoint, uint16_t &theta) {

 Point pointError = Point(0, 0);
 double angularError = 0.0;

 for(int i = 0; i < NBITERATIONS; i++) {
  mapLines.clear();
  robotToMap(robotLines, mapLines, odometryPoint, theta);

  if(computeErrors(mapLines, map, pointError, angularError)) {
   odometryPoint -= pointError / (i + 1);

#ifdef IMU
   thetaCorrector += int(angularError * double(PI16) / M_PI) / IMUTHETACORRECTORDIV;
#else
   theta += int(angularError * double(PI16) / M_PI) / (i + 1);
#endif

  }
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

void drawMap(Mat &image, vector<Line> &mapRobot, vector<Line> &map, bool colored, int mapDiv) {
 const Point centerPoint = Point(width / 2, height / 2);

 for(int i = 0; i < mapRobot.size(); i++) {
  Point point1 = mapRobot[i].a;
  Point point2 = mapRobot[i].b;

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

void drawRobot(Mat &image, vector<Point> robotIcon, int thickness, Point odometryPoint, uint16_t theta, int mapDiv) {
 const Point centerPoint = Point(width / 2, height / 2);

 vector<Point> polygon;
 for(int i = 0; i < robotIcon.size(); i++) {
  Point point = odometryPoint + rotate(robotIcon[i], theta);
  point.x /= mapDiv;
  point.y /= -mapDiv;
  point += centerPoint;
  polygon.push_back(point);
 }

 vector<vector<Point>> tmp(1, polygon);
 drawContours(image, tmp, -1, Scalar::all(255), thickness, LINE_AA);
}

void drawHist(Mat &image, Point odometryPoint, uint16_t theta, int mapDiv) {
 static Point hist[HIST] = {Point(0, 0)};
 static int n = 0;
 const Point centerPoint = Point(width / 2, height / 2);
 static Point oldPoint = {Point(0, 0)};

 hist[n++] = odometryPoint;
 if(n == HIST)
  n = 0;

 for(int i = 0; i < HIST; i++) {
  Point point = rotate(hist[i] - odometryPoint, -theta);
  point.x /= mapDiv;
  point.y /= -mapDiv;
  point += centerPoint;

  if(i != n) {
   int sqDistTolerancePixels = LARGEDISTTOLERANCE / mapDiv;
   if(sqDist(oldPoint, point) < sqDistTolerancePixels * sqDistTolerancePixels)
    line(image, oldPoint, point, Scalar::all(255), 1, LINE_AA);
  }
  oldPoint = point;
 }
}

void ui(Mat &image, vector<Point> &robotPoints, vector<Line> &robotLines,
                    vector<Line> &map, vector<Line> &mapRobot,
                    Point &odometryPoint, Point &oldOdometryPoint,
                    uint16_t &theta, uint16_t &oldTheta, int time) {

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
  oldOdometryPoint = Point(0, 0);
  theta = 0;
  oldTheta = 0;

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
 oldButtonReset = buttonReset;
 oldButtonOk = buttonOk;

 switch(select) {
  case SELECTNONE:
   return;

  case SELECTMAP:
   drawMap(image, mapRobot, map, true, mapDiv);
   drawRobot(image, robotIcon, 1, Point(0, 0), 0, mapDiv);
   break;

  case SELECTMAPPOINTS:
   drawPoints(image, robotPoints, false, mapDiv);
   drawMap(image, robotLines, robotLines, false, mapDiv);
   drawMap(image, mapRobot, map, true, mapDiv);
   drawRobot(image, robotIcon, 1, Point(0, 0), 0, mapDiv);
   break;

  case SELECTMAPPOINTSBEAMS:
   drawPoints(image, robotPoints, true, mapDiv);
   drawMap(image, robotLines, robotLines, false, mapDiv);
   drawMap(image, mapRobot, map, true, mapDiv);
   drawRobot(image, robotIcon, FILLED, Point(0, 0), 0, mapDiv);
   break;

  case SELECTPOINTSBEAMS:
   drawPoints(image, robotPoints, true, mapDiv);
   drawMap(image, robotLines, robotLines, false, mapDiv);
   drawRobot(image, robotIcon, FILLED, Point(0, 0), 0, mapDiv);
   break;
 }

 drawHist(image, odometryPoint, theta, mapDiv);

 char text[80];
 if(tune)
  sprintf(text, "%d mm per pixel", mapDiv);
 else if(select == SELECTMAPPOINTSBEAMS)
  sprintf(text, "%d points %d lines %d map lines %d ms", robotPoints.size(), robotLines.size(), map.size(), time);
 else
  sprintf(text, "X %04d Y %04d Theta %03d", odometryPoint.x, odometryPoint.y, theta * 180 / PI16);
 putText(image, text, Point(5, 15), FONT_HERSHEY_PLAIN, 1.0, Scalar::all(0), 1);
 putText(image, text, Point(6, 16), FONT_HERSHEY_PLAIN, 1.0, Scalar::all(255), 1);
}

void odometry(Point &odometryPoint, uint16_t &theta) {
#ifdef IMU
 theta = angleDoubleToAngle16(imuData.fusionPose.z() * DIRZ) + thetaCorrector;
#else
 theta += remoteFrame.vz * VZMUL;
#endif

 Point point = rotate(Point(remoteFrame.vx, remoteFrame.vy), theta);
 point.x /= VXDIV;
 point.y /= VYDIV;
 odometryPoint += point;
}

void bgrInit() {
 for(uchar i = 0; i < 180; i++) {
  Mat imageHsv = Mat(1, 1, CV_8UC3, Scalar(i, 255, 255));
  Mat imageBgr;
  cvtColor(imageHsv, imageBgr, COLOR_HSV2BGR);
  hueToBgr[i] = imageBgr.at<Vec3b>(0, 0);
 }
}

void dedistortTheta(vector<PolarPoint> &polarPoints, uint16_t theta, uint16_t &oldTheta) {
 int16_t size = polarPoints.size();
 int16_t deltaTheta = theta - oldTheta;

 for(int i = 0; i < size; i++)
  polarPoints[i].theta += (size - i) * deltaTheta / size;

 //while(polarPoints[0].theta > PI16 && polarPoints.size() > 1)
 while(polarPoints[0].theta - polarPoints[polarPoints.size() - 1].theta > PI16 && polarPoints.size() > 1)
  polarPoints.erase(polarPoints.begin());

 oldTheta = theta;
}

void dedistortOdometry(vector<Point> &robotPoints, Point odometryPoint, Point &oldOdometryPoint, uint16_t theta) {
 int size = robotPoints.size();
 Point deltaOdometry = odometryPoint - oldOdometryPoint;

 for(int i = 0; i < size; i++) {
  Point correction = (size - i) * deltaOdometry / size;
  robotPoints[i] -= rotate(correction, -theta);
 }

 oldOdometryPoint = odometryPoint;
}

void writeMapFile(vector<Line> &map, Point odometryPoint, uint16_t theta) {
 FileStorage fs(MAPFILE, FileStorage::WRITE);

 if(fs.isOpened()) {
  fs << "odometryPoint" << odometryPoint;
  fs << "theta" << theta;

  fs << "map" << "[";
  for(int i = 0; i < map.size(); i++) {
   fs << "{";
   fs << "a" << map[i].a;
   fs << "b" << map[i].b;
   fs << "}";
  }
  fs << "]";

  fs.release();
 } else
  fprintf(stderr, "Error writing map file\n");
}

void readMapFile(vector<Line> &map, Point &odometryPoint, uint16_t &theta) {
 FileStorage fs(MAPFILE, FileStorage::READ);

 if(fs.isOpened()) {
  fs["odometryPoint"] >> odometryPoint;
  fs["theta"] >> theta;

  FileNode fn = fs["map"];
  for(FileNodeIterator it = fn.begin(); it != fn.end(); it++) {
   FileNode item = *it;
   Point a;
   Point b;
   item["a"] >> a;
   item["b"] >> b;
   map.push_back({a, b, Point(0, 0), Point(0, 0), 0, VALIDATIONFILTERKEEP,
                  GROWFILTER, GROWFILTER, SHRINKFILTER, SHRINKFILTER});
  }

  fs.release();
 } else
  fprintf(stderr, "Error reading map file\n");
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
 vector<Line> mapRobot;

 Point odometryPoint = Point(0, 0);
 uint16_t theta = 0;
 fprintf(stderr, "Reading map file\n");
 readMapFile(map, odometryPoint, theta);
 Point oldOdometryPoint = odometryPoint;
 uint16_t oldTheta = theta;
 thetaCorrector = theta;

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

  if(updated)
   odometry(odometryPoint, theta);

  if(readLidar(ld, polarPoints)) {
   dedistortTheta(polarPoints, theta, oldTheta);

   robotPoints.clear();
   lidarToRobot(polarPoints, robotPoints);

   //dedistortOdometry(robotPoints, odometryPoint, oldOdometryPoint, theta);
   //polarPoints.clear();
   //robotToLidar(robotPoints, polarPoints);

   robotRawLines.clear();
   extractRawLines(polarPoints, robotPoints, robotRawLines);

   robotLines.clear();
   fitLines(robotRawLines, robotLines);

   sort(robotLines.begin(), robotLines.end(), [](const Line &a, const Line &b) {
    return sqDist(a) > sqDist(b);
   });

   localization(robotLines, mapLines, map, odometryPoint, theta);
   mapping(mapLines, map);
   mapCleaner(polarPoints, map, odometryPoint, theta);
   mapFiltersDecay(map);

   mapRobot.clear();
   mapToRobot(map, mapRobot, odometryPoint, theta);
  }

  ui(image, robotPoints, robotLines, map, mapRobot, odometryPoint, oldOdometryPoint, theta, oldTheta, time);

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

  tickMeter.stop();
  time = tickMeter.getTimeMilli();
  tickMeter.reset();
 }

 if(captureEnabled) {
  fprintf(stderr, "Stopping capture\n");
  capture.release();
 }

 fprintf(stderr, "Stopping lidar\n");
 stopLidar(ld);

 fprintf(stderr, "Writing map file\n");
 writeMapFile(map, odometryPoint, theta);

 fprintf(stderr, "Stopping\n");
 return 0;
}
