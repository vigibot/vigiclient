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
 fprintf(stderr, "IMU thread starting\n");

 int oldStdout = dup(fileno(stdout));
 dup2(fileno(stderr), fileno(stdout));

 RTIMUSettings *settings = new RTIMUSettings("RTIMULib");
 imu = RTIMU::createIMU(settings);
 if(imu == NULL || imu->IMUType() == RTIMU_TYPE_NULL) {
  fprintf(stderr, "IMU thread initialization error\n");
  imuThreadStatus = STATUSERROR;
  return;
 }

 imu->IMUInit();
 imu->setSlerpPower(IMUSLERPPOWER);
 imu->setGyroEnable(true);
 imu->setAccelEnable(true);
 imu->setCompassEnable(false);

 dup2(oldStdout, fileno(stdout));
 fprintf(stderr, "IMU thread initialization success\n");
 imuThreadStatus = STATUSSUCCESS;

 while(run) {
  usleep(imu->IMUGetPollInterval() * 1000);
  while(imu->IMURead())
   imuData = imu->getIMUData();
 }

 fprintf(stderr, "IMU thread stopping\n");
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
      sqDist(pointsNoDp[0], pointsNoDp[size - 1]) >= LINESIZEMIN * LINESIZEMIN) {
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
      sqDist(pointsNoDp[0], pointsNoDp[pointsNoDp.size() - 1]) >= LINESIZEMIN * LINESIZEMIN)
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

void robotToMap(vector<Point> &pointsIn, vector<Point> &pointsOut, Point robotPoint, uint16_t robotTheta) {
 for(int i = 0; i < pointsIn.size(); i++)
  pointsOut.push_back(robotPoint + rotate(pointsIn[i], robotTheta));
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

bool growLine(Line &line, Point point) {
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

double lineAngle(Line line) {
 Point diff = line.b - line.a;
 return atan2(diff.y, diff.x);
}

double diffAngle(double angle1, double angle2) {
 double result = angle2 - angle1;

 if(result > M_PI)
  result -= 2.0 * M_PI;
 else if(result < -M_PI)
  result += 2.0 * M_PI;

 return result;
}

double diffAngle(Line line1, Line line2) {
 double angle1 = lineAngle(line1);
 double angle2 = lineAngle(line2);

 return diffAngle(angle1, angle2);
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

bool testPointLine(Point point, Line line, int distTolerance, int lengthMargin) {
 if(sqNorm(pointDistancePointLine(point, line)) < distTolerance * distTolerance) {
  int refNorm = int(sqrt(sqDist(line)));
  int distance = ratioPointLine(point, line) * refNorm;

  if(distance > -lengthMargin && distance < refNorm + lengthMargin)
   return true;
 }

 return false;
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

bool intersect(Line line1, Line line2, Point &intersectPoint) {
 Point2d x = line2.a - line1.a;
 Point2d d1 = line1.b - line1.a;
 Point2d d2 = line2.b - line2.a;

 double det = d1.x * d2.y - d1.y * d2.x;
 if(det == 0)
  return false;

 double t1 = (x.x * d2.y - x.y * d2.x) / det;
 intersectPoint = Point2d(line1.a) + d1 * t1;

 if(intersectPoint.x >= min(line1.a.x, line1.b.x) && intersectPoint.x <= max(line1.a.x, line1.b.x) &&
    intersectPoint.x >= min(line2.a.x, line2.b.x) && intersectPoint.x <= max(line2.a.x, line2.b.x) &&
    intersectPoint.y >= min(line1.a.y, line1.b.y) && intersectPoint.y <= max(line1.a.y, line1.b.y) &&
    intersectPoint.y >= min(line2.a.y, line2.b.y) && intersectPoint.y <= max(line2.a.y, line2.b.y))
  return true;

 return false;
}

bool intersectLine(Line line1, Line line2, Point &intersectPoint) {
 Point2d x = line2.a - line1.a;
 Point2d d1 = line1.b - line1.a;
 Point2d d2 = line2.b - line2.a;

 double det = d1.x * d2.y - d1.y * d2.x;
 if(det == 0)
  return false;

 double t1 = (x.x * d2.y - x.y * d2.x) / det;
 intersectPoint = Point2d(line1.a) + d1 * t1;

 Point averagePoint = (line1.a + line1.b + line2.a + line2.b) / 4;
 if(abs(intersectPoint.x - averagePoint.x) <= INTERSECTMAX &&
    abs(intersectPoint.y - averagePoint.y) <= INTERSECTMAX)
  return true;

 return false;
}

void sortLines(vector<Line> &map) {
 sort(map.begin(), map.end(), [](const Line &a, const Line &b) {
  return sqDist(a) > sqDist(b);
 });
}

void mapCleaner(vector<PolarPoint> &polarPoints, vector<Line> &map, Point robotPoint, uint16_t robotTheta) {
 vector<Point> closerPoints;
 bool sort = false;

 for(int i = 0; i < polarPoints.size(); i++) {
  Point closerPoint = Point((polarPoints[i].distance * MAPCLEANERDISTPERCENT / 100) * sin16(polarPoints[i].theta) / ONE16,
                            (polarPoints[i].distance * MAPCLEANERDISTPERCENT / 100) * cos16(polarPoints[i].theta) / ONE16);

  closerPoints.push_back(robotPoint + rotate(closerPoint, robotTheta));
 }

 for(int i = 0; i < map.size(); i++) {
  bool shrinka = true;
  bool shrinkb = true;

  for(int j = 0; j < closerPoints.size(); j++) {
   Line shorterLine = {robotPoint, closerPoints[j]};

   double angularError = diffAngle(map[i], shorterLine);
   if(angularError < MAPCLEANERANGULARTOLERANCE || angularError > M_PI - MAPCLEANERANGULARTOLERANCE)
    continue;

   Point intersectPoint;
   if(!intersect(shorterLine, map[i], intersectPoint))
    continue;

   if(sqDist(map[i].a, intersectPoint) < sqDist(map[i].b, intersectPoint)) {
    if(shrinka) {
     shrinka = false;
     map[i].shrinka--;
    }
    if(map[i].shrinka == 0) {
     map[i].a = intersectPoint;
     sort = true;
    }
   } else {
    if(shrinkb) {
     shrinkb = false;
     map[i].shrinkb--;
    }
    if(map[i].shrinkb == 0) {
     map[i].b = intersectPoint;
     sort = true;
    }
   }

   if(sqDist(map[i]) < LINESIZEMIN * LINESIZEMIN) {
    map.erase(map.begin() + i);
    i--;
   }
  }

  if(map[i].shrinka == 0)
   map[i].shrinka = SHRINKFILTER;
  if(map[i].shrinkb == 0)
   map[i].shrinkb = SHRINKFILTER;
 }

 if(sort)
  sortLines(map);
}

void mapDeduplicateAverage(vector<Line> &map) {
 bool sort = false;

 for(int i = 0; i < map.size(); i++) {
  vector<int> id;

  if(map[i].validation < VALIDATIONFILTERKEEP)
   continue;

  id.push_back(i);
  for(int j = i + 1; j < map.size(); j++) {
   if(map[j].validation < VALIDATIONFILTERKEEP)
    continue;

   Point pointError;
   double angularError;
   int distError;
   int refNorm;
   if(testLines(map[i], map[j], SMALLDISTTOLERANCE, SMALLANGULARTOLERANCE, -SMALLDISTTOLERANCE,
                pointError, angularError, distError, refNorm))
    id.push_back(j);
  }

  int nbLines = id.size();
  if(nbLines > 1) {

   for(int j = 0; j < nbLines; j++) {
    for(int k = 0; k < nbLines; k++) {
     growLine(map[id[j]], map[id[k]].a);
     growLine(map[id[j]], map[id[k]].b);
    }
   }

   Line averageLine = map[i];
   int nbAverages = 1;
   for(int j = nbLines - 1; j > 0; j--) {
    if(sqDist(map[i].a, map[j].a) < SMALLDISTTOLERANCE * SMALLDISTTOLERANCE &&
       sqDist(map[i].b, map[j].b) < SMALLDISTTOLERANCE * SMALLDISTTOLERANCE) {
     averageLine.a += map[id[j]].a;
     averageLine.b += map[id[j]].b;
     map.erase(map.begin() + id[j]);
     nbAverages++;
    }
   }

   averageLine.a /= nbAverages;
   averageLine.b /= nbAverages;
   map[i] = averageLine;
   sort = true;
  }
 }

 if(sort)
  sortLines(map);
}

void mapDeduplicateErase(vector<Line> &map) {
 for(int i = 0; i < map.size(); i++) {
  if(map[i].validation < VALIDATIONFILTERKEEP)
   continue;

  for(int j = i + 1; j < map.size(); j++) {
   if(map[j].validation < VALIDATIONFILTERKEEP)
    continue;

   Point pointError;
   double angularError;
   int distError;
   int refNorm;
   if(testLines(map[i], map[j], LARGEDISTTOLERANCE, LARGEANGULARTOLERANCE, -SMALLDISTTOLERANCE,
                pointError, angularError, distError, refNorm)) {
    map.erase(map.begin() + j);
    j--;
   }
  }
 }
}

bool computeErrors(vector<Line> &mapLines, vector<Line> &map,
                   Point &pointErrorOut, double &angularErrorOut) {

 Point pointErrorSum = Point(0, 0);
 int pointErrorWeightSum = 0;
 double angularErrorSum = 0.0;
 int angularErrorWeightSum = 0;

 for(int i = 0; i < mapLines.size(); i++) {
  for(int j = 0; j < map.size(); j++) {
   Point pointError;
   double angularError;
   int distError;
   int refNorm;

   if(map[j].validation >= VALIDATIONFILTERSTART &&
      testLines(mapLines[i], map[j], LARGEDISTTOLERANCE, LARGEANGULARTOLERANCE, -SMALLDISTTOLERANCE,
                pointError, angularError, distError, refNorm)) {
    pointErrorSum += pointError * refNorm;
    pointErrorWeightSum += refNorm;

    if(map[j].validation >= VALIDATIONFILTERKEEP) {
     angularErrorSum += angularError * refNorm;
     angularErrorWeightSum += refNorm;
    }
   }

  }
 }

 if(pointErrorWeightSum) {
  pointErrorOut = pointErrorSum / pointErrorWeightSum;
  if(angularErrorWeightSum)
   angularErrorOut = angularErrorSum / angularErrorWeightSum;

  return true;
 } else
  return false;
}

void mapping(vector<Line> &mapLines, vector<Line> &map) {
 vector<Line> newLines;
 bool sort = false;

 for(int i = 0; i < mapLines.size(); i++) {
  bool newLine = true;

  for(int j = 0; j < map.size(); j++) {
   Point pointError;
   double angularError;
   int distError;
   int refNorm;
   if(!testLines(mapLines[i], map[j], LARGEDISTTOLERANCE * 2, LARGEANGULARTOLERANCE, -SMALLDISTTOLERANCE,
      pointError, angularError, distError, refNorm))
    continue;

   newLine = false;

   if(map[j].validation < VALIDATIONFILTERKEEP) {
    map[j].intega += mapLines[i].a;
    map[j].integb += mapLines[i].b;
    map[j].integ++;
    map[j].validation++;

    map[j].a = map[j].intega / map[j].integ;
    map[j].b = map[j].integb / map[j].integ;
    break;
   }

   if(distError > SMALLDISTTOLERANCE || fabs(angularError) > SMALLANGULARTOLERANCE)
    break;

   bool grown = false;
   grown |= growLine(map[j], mapLines[i].a);
   grown |= growLine(map[j], mapLines[i].b);
   if(grown)
    sort = true;
  }

  if(newLine)
   newLines.push_back(mapLines[i]);
 }

 for(int i = 0; i < newLines.size(); i++) {
  newLines[i].intega = newLines[i].a;
  newLines[i].integb = newLines[i].b;
  newLines[i].integ = 1;
  newLines[i].validation = VALIDATIONFILTERSTART;
  newLines[i].shrinka = SHRINKFILTER;
  newLines[i].shrinkb = SHRINKFILTER;
  map.push_back(newLines[i]);
  sort = true;
 }

 if(sort)
  sortLines(map);
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

   if(map[i].shrinka < SHRINKFILTER)
    map[i].shrinka++;
   if(map[i].shrinkb < SHRINKFILTER)
    map[i].shrinkb++;
  }
  n = 0;
 }

 n++;
}

void splitAxes(vector<Line> &robotLines, vector<Line> robotLinesAxes[]) {
 robotLinesAxes[0].push_back(robotLines[0]);

 for(int i = 1; i < robotLines.size(); i++) {
  double angle1 = diffAngle(robotLines[0], robotLines[i]);

  for(int j = 0; j < AXES; j++) {
   double angle2 = abs(diffAngle(angle1, j * M_PI / AXES));

   if(angle2 <= M_PI / AXES / 2 || angle2 > M_PI - M_PI / AXES / 2) {
    robotLinesAxes[j].push_back(robotLines[i]);
    break;
   }
  }
 }
}

void localization(vector<Line> robotLinesAxes[], vector<Line> &map, Point &robotPoint, uint16_t &robotTheta) {
 vector<Line> mapLines;

 for(int i = 0; i < NBITERATIONS; i++) {
  for(int j = 0; j < AXES; j++) {
   mapLines.clear();
   robotToMap(robotLinesAxes[j], mapLines, robotPoint, robotTheta);

   Point pointError;
   double angularError;
   if(computeErrors(mapLines, map, pointError, angularError)) {
    robotPoint -= pointError / AXES / (i + 1);

#ifdef IMU
    robotThetaCorrector += int(angularError * double(PI16) / M_PI) / AXES / IMUTHETACORRECTORDIV;
#else
    robotTheta += int(angularError * double(PI16) / M_PI) / AXES / (i + 1);
#endif

   }
  }
 }
}

Point rescaleTranslate(Point point, int mapDiv) {
 point.x /= mapDiv;
 point.y /= -mapDiv;
 return Point(width / 2, height / 2) + point;
}

void drawLidarPoints(Mat &image, vector<Point> &points, bool beams, int mapDiv) {
 for(int i = 0; i < points.size(); i++) {
  Point point = rescaleTranslate(points[i], mapDiv);

  if(beams)
   line(image, Point(width / 2, height / 2), point, Scalar::all(64), 1, LINE_AA);
  else if(point.x >= 0 && point.x < width &&
          point.y >= 0 && point.y < height)
   image.at<Vec3b>(point.y, point.x) = Vec3b(255, 255, 255);
 }
}

void drawLidarLines(Mat &image, vector<Line> robotLinesAxes[], int mapDiv) {
 for(int i = 0; i < AXES; i++) {
  for(int j = 0; j < robotLinesAxes[i].size(); j++) {
   Point point1 = rescaleTranslate(robotLinesAxes[i][j].a, mapDiv);
   Point point2 = rescaleTranslate(robotLinesAxes[i][j].b, mapDiv);

   Scalar color;
   switch(i) {
    case 0:
     color = Scalar::all(255);
     break;
    case 1:
     color = Scalar(128, 128, 255);
     break;
    case 2:
     color = Scalar(128, 255, 128);
     break;
    case 3:
     color = Scalar(255, 128, 128);
     break;
   }

   line(image, point1, point2, color, 1, LINE_AA);
  }
 }
}

void drawMap(Mat &image, vector<Line> &map, bool light, Point robotPoint, uint16_t robotTheta, int mapDiv) {
 for(int i = 0; i < map.size(); i++) {
  Point point1 = rescaleTranslate(rotate(map[i].a - robotPoint, -robotTheta), mapDiv);
  Point point2 = rescaleTranslate(rotate(map[i].b - robotPoint, -robotTheta), mapDiv);

  if(light) {
   if(map[i].validation >= VALIDATIONFILTERKEEP)
    line(image, point1, point2, Scalar::all(128), 1, LINE_AA);
  } else {
   Scalar color;
   if(map[i].validation < VALIDATIONFILTERKEEP)
    color = Scalar::all(mapInteger(map[i].validation, VALIDATIONFILTERKILL, VALIDATIONFILTERKEEP, 0, 255));
   else {
    Point diff = map[i].b - map[i].a;
    double angleDeg = atan2(diff.y, diff.x) * 180.0 / M_PI;
    uchar hue = uchar(angleDeg / 2.0 + 90.0) % 180;
    color = hueToBgr[hue];
   }
   line(image, point1, point2, color, 2, LINE_AA);
  }

 }
}

void drawIntersects(Mat &image, vector<Line> &map, Point robotPoint, uint16_t robotTheta, int mapDiv) {
 for(int i = 0; i < map.size(); i++) {
  if(map[i].validation < VALIDATIONFILTERKEEP)
   continue;

  for(int j = i + 1; j < map.size(); j++) {
   if(map[j].validation < VALIDATIONFILTERKEEP)
    continue;

   Point intersectPoint;
   if(!intersectLine(map[i], map[j], intersectPoint))
    continue;

   Point point = rescaleTranslate(rotate(intersectPoint - robotPoint, -robotTheta), mapDiv);

   circle(image, point, 2, Scalar::all(255), FILLED, LINE_AA);
  }
 }
}

void drawHist(Mat &image, Point robotPoint, uint16_t robotTheta, int mapDiv) {
 static Point hist[HIST] = {Point(0, 0)};
 static int n = 0;
 static Point oldPoint = Point(0, 0);

 hist[n++] = robotPoint;
 if(n == HIST)
  n = 0;

 for(int i = 0; i < HIST; i++) {
  Point point = rescaleTranslate(rotate(hist[(i + n) % HIST] - robotPoint, -robotTheta), mapDiv);

  if(i != 0) {
   int sqDistTolerancePixels = LARGEDISTTOLERANCE / mapDiv;
   if(sqDist(oldPoint, point) < sqDistTolerancePixels * sqDistTolerancePixels)
    line(image, oldPoint, point, Scalar::all(128), 1, LINE_AA);
  }

  oldPoint = point;
 }
}

void drawNodes(Mat &image, vector<Point> &nodes, int targetNode, Point robotPoint, uint16_t robotTheta, int mapDiv) {
 for(int i = 0; i < nodes.size(); i++) {
  Point point = rescaleTranslate(rotate(nodes[i] - robotPoint, -robotTheta), mapDiv);

  if(i == targetNode) {
   circle(image, point, 4, Scalar::all(0), 1, LINE_AA);
   circle(image, point + Point(1, 1), 4, Scalar::all(255), 1, LINE_AA);
  }

  circle(image, point, 1, Scalar::all(0), FILLED, LINE_AA);
  circle(image, point + Point(1, 1), 1, Scalar::all(255), FILLED, LINE_AA);
 }
}

void drawNumbers(Mat &image, vector<Point> &nodes, int targetNode, Point robotPoint, uint16_t robotTheta, int mapDiv) {
 for(int i = 0; i < nodes.size(); i++) {
  Point point = rescaleTranslate(rotate(nodes[i] - robotPoint, -robotTheta), mapDiv);

  char text[8];
  sprintf(text, "%d", i);

  int baseline;
  Size textSize = getTextSize(text, FONT_HERSHEY_PLAIN, 1.0, 1, &baseline);
  Point textPoint = Point(-textSize.width / 2, textSize.height / 2) + point;

  putText(image, text, textPoint, FONT_HERSHEY_PLAIN, 1.0, Scalar::all(0), 1);
  putText(image, text, textPoint + Point(1, 1), FONT_HERSHEY_PLAIN, 1.0, Scalar::all(255), 1);

  if(i == targetNode) {
   circle(image, point, textSize.width / 2 + 3, Scalar::all(0), 1, LINE_AA);
   circle(image, point + Point(1, 1), textSize.width / 2 + 3, Scalar::all(255), 1, LINE_AA);
  }
 }
}

void drawLinks(Mat &image, vector<Point> &nodes, vector<array<int, 2>> &links, Point robotPoint, uint16_t robotTheta, int mapDiv) {
 for(int i = 0; i < links.size(); i++) {
  Point point1 = rescaleTranslate(rotate(nodes[links[i][0]] - robotPoint, -robotTheta), mapDiv);
  Point point2 = rescaleTranslate(rotate(nodes[links[i][1]] - robotPoint, -robotTheta), mapDiv);

  line(image, point1, point2, Scalar::all(128), 1, LINE_AA);
 }
}

void drawPath(Mat &image, vector<Point> &nodes, vector<int> &paths, int start, int end, Point robotPoint, uint16_t robotTheta, int mapDiv) {
 int n = end;
 Point oldPoint = rescaleTranslate(rotate(nodes[n] - robotPoint, -robotTheta), mapDiv);

 while(n != -1) {
  Point point = rescaleTranslate(rotate(nodes[n] - robotPoint, -robotTheta), mapDiv);

  line(image, oldPoint, point, Scalar::all(128), 1, LINE_AA);
  oldPoint = point;

  n = paths[n];
 }
}

void drawRobot(Mat &image, vector<Point> robotIcon, int thickness, int mapDiv) {
 vector<Point> polygon;
 for(int i = 0; i < robotIcon.size(); i++) {
  Point point = rescaleTranslate(robotIcon[i], mapDiv);
  polygon.push_back(point);
 }

 vector<vector<Point>> tmp(1, polygon);
 drawContours(image, tmp, -1, Scalar::all(255), thickness, LINE_AA);
}

bool obstacle(vector<Point> &mapPoints, Point robotPoint, Point targetPoint, int obstacleDetectionRange) {
 for(int i = 0; i < mapPoints.size(); i++) {
  Line line = {robotPoint, targetPoint};
  if(sqNorm(pointDistancePointLine(mapPoints[i], line)) < ROBOTWIDTH * ROBOTWIDTH) {
   int refNorm = int(sqrt(sqDist(line)));
   int distance = ratioPointLine(mapPoints[i], line) * refNorm;

   if(distance > 0 && distance <= obstacleDetectionRange)
    return true;
  }
 }

 return false;
}

void dijkstra(list<pair<int, int>> adjacent[], int nbNodes, int start, vector<int> &pathsOut) {
 priority_queue<Pair, vector<Pair>, greater<Pair>> pq;
 vector<int> dists(nbNodes, INT_MAX);
 vector<bool> flags(nbNodes, false);
 vector<int> paths(nbNodes, -1);

 pq.push(make_pair(0, start));
 dists[start] = 0;

 while(!pq.empty()) {
  int a = pq.top().second;
  pq.pop();
  flags[a] = true;

  list<pair<int, int>>::iterator i;
  for(i = adjacent[a].begin(); i != adjacent[a].end(); ++i) {
   int b = (*i).first;
   int weight = (*i).second;

   if(flags[b] == false && dists[b] > dists[a] + weight) {
    dists[b] = dists[a] + weight;
    paths[b] = a;
    pq.push(make_pair(dists[b], b));
   }
  }
 }

 pathsOut = paths;
}

void addLink(list<pair<int, int>> adjacent[], int a, int b, int weight) {
 adjacent[a].push_back(make_pair(b, weight));
 adjacent[b].push_back(make_pair(a, weight));
}

void computePaths(vector<Point> &nodes, vector<array<int, 2>> &links, int start, vector<int> &paths) {
 list<pair<int, int>> *adjacent;
 adjacent = new list<Pair>[nodes.size()];

 for(int i = 0; i < links.size(); i++) {
  int dist = int(sqrt(sqDist(nodes[links[i][0]], nodes[links[i][1]])));
  addLink(adjacent, links[i][0], links[i][1], dist);
 }

 dijkstra(adjacent, nodes.size(), start, paths);
}

void delNode(vector<Point> &nodes, vector<array<int, 2>> &links, int nodeIndex) {
 nodes.erase(nodes.begin() + nodeIndex);
 for(int i = 0; i < links.size(); i++) {
  if(nodeIndex == links[i][0] ||
     nodeIndex == links[i][1]) {
   links.erase(links.begin() + i);
   i--;
  } else {
   if(nodeIndex < links[i][0])
    links[i][0]--;
   if(nodeIndex < links[i][1])
    links[i][1]--;
  }
 }
}

void addNode(vector<Point> &mapPoints, vector<Point> &nodes, vector<array<int, 2>> &links, Point node) {
 for(int i = 0; i < nodes.size(); i++) {
  if(sqDist(node, nodes[i]) <= LINKSIZEMAX * LINKSIZEMAX &&
     !obstacle(mapPoints, node, nodes[i], LINKSIZEMAX))
   links.push_back({int(nodes.size()), i});
 }
 nodes.push_back(node);
}

int closestPoint(vector<Point> &points, Point point) {
 int distMin = sqDist(point, points[0]);
 int closest = 0;

 for(int i = 1; i < points.size(); i++) {
  int dist = sqDist(point, points[i]);
  if(dist < distMin) {
   distMin = dist;
   closest = i;
  }
 }

 return closest;
}

void ui(Mat &image, vector<Point> &robotPoints, vector<Line> robotLinesAxes[], vector<Line> &map,
                    vector<Point> &mapPoints, vector<Point> &nodes, vector<array<int, 2>> &links, vector<int> &paths, int &targetNode,
                    Point &robotPoint, Point &oldRobotPoint, uint16_t &robotTheta, uint16_t &oldRobotTheta,
                    bool &mappingEnabled, bool &running, int &select, int &mapDiv, int time) {

 static Point oldRemoteFramePoint = Point(0, 0);
 int closestRobot;
 static int oldTargetNode = 0;
 static int oldClosestRobot = 0;
 static int start = 0;
 static int end = 0;
 bool buttonLess = remoteFrame.switchs & 0b00010000;
 bool buttonMore = remoteFrame.switchs & 0b00100000;
 bool buttonOk = remoteFrame.switchs & 0b10000000;
 bool buttonCancel = remoteFrame.switchs & 0b01000000;
 static bool oldButtonLess = false;
 static bool oldButtonMore = false;
 static bool oldButtonOk = false;
 static bool oldButtonCancel = false;
 static int buttonLessCount = 0;
 static int buttonMoreCount = 0;
 static int buttonOkCount = 0;
 static int buttonCancelCount = 0;

 if(!nodes.empty()) {
  Point remoteFramePoint = Point(remoteFrame.xy[GOTOTOOL][0], remoteFrame.xy[GOTOTOOL][1]);
  if(remoteFramePoint != oldRemoteFramePoint) {
   oldRemoteFramePoint = remoteFramePoint;

   Point targetPoint;
   targetPoint.x = (remoteFramePoint.x * width * mapDiv) / 65535;
   targetPoint.y = (remoteFramePoint.y * height * mapDiv) / 65535;
   targetPoint = rotate(targetPoint, robotTheta) + robotPoint;

   targetNode = closestPoint(nodes, targetPoint);
  }

  closestRobot = closestPoint(nodes, robotPoint);

  if(targetNode != oldTargetNode) {
   start = closestRobot;
   end = targetNode;
   computePaths(nodes, links, start, paths);
   oldTargetNode = targetNode;
  }

  if(closestRobot != oldClosestRobot && !running) {
   start = targetNode;
   end = closestRobot;
   computePaths(nodes, links, start, paths);
   oldClosestRobot = closestRobot;
  }
 }

 if(buttonOk) {
  buttonOkCount++;
  if(buttonOkCount == BUTTONSLONGPRESS) {

   if(select <= SELECTFULL)
    running = !running;
   else if(select == SELECTDEBUGMAP) {
    mappingEnabled = !mappingEnabled;
    for(int i = 0; i < map.size(); i++) {
     if(map[i].validation < VALIDATIONFILTERKEEP) {
      map.erase(map.begin() + i);
      i--;
     }
    }
   }

  }
 } else if(buttonCancel) {
  buttonCancelCount++;
  if(buttonCancelCount == BUTTONSLONGPRESS) {

   if(select <= SELECTDEBUGMAP) {
    map.clear();
    if(nodes.empty()) {
     robotPoint = Point(0, 0);
     oldRobotPoint = Point(0, 0);
     robotTheta = 0;
     oldRobotTheta = 0;
#ifdef IMU
     imu->resetFusion();
     robotThetaCorrector = 0;
#endif
    }
   }

  }
 } else if(buttonMore) {
  buttonMoreCount++;
  if(buttonMoreCount >= BUTTONSLONGPRESS) {

   if(buttonMoreCount % 2 && mapDiv > MAPDIVMIN)
    mapDiv--;

  }
 } else if(buttonLess) {
  buttonLessCount++;
  if(buttonLessCount >= BUTTONSLONGPRESS) {

   if(buttonLessCount % 2 && mapDiv < MAPDIVMAX)
    mapDiv++;

  }
 } else if(!buttonOk && oldButtonOk) {
  if(buttonOkCount < BUTTONSLONGPRESS) {

   if(select <= SELECTFULL) {
    bool found = false;
    for(int i = 0; i < nodes.size(); i++) {
     if(sqDist(robotPoint, nodes[i]) < GOTOPOINTDISTTOLERANCE * GOTOPOINTDISTTOLERANCE) {
      delNode(nodes, links, i);
      addNode(mapPoints, nodes, links, robotPoint);
      computePaths(nodes, links, 0, paths);
      found = true;
      break;
     }
    }
    if(!found) {
     addNode(mapPoints, nodes, links, robotPoint);
     computePaths(nodes, links, 0, paths);
    }
   }

  }
  buttonOkCount = 0;
 } else if(!buttonCancel && oldButtonCancel) {
  if(buttonCancelCount < BUTTONSLONGPRESS) {

   if(select <= SELECTFULL) {
    bool found = false;
    for(int i = 0; i < nodes.size(); i++) {
     if(sqDist(robotPoint, nodes[i]) < GOTOPOINTDISTTOLERANCE * GOTOPOINTDISTTOLERANCE) {
      delNode(nodes, links, i);
      if(!nodes.empty())
       computePaths(nodes, links, 0, paths);
      found = true;
      break;
     }
    }
    if(!found && !nodes.empty()) {
     delNode(nodes, links, nodes.size() - 1);
     if(!nodes.empty())
      computePaths(nodes, links, 0, paths);
    }
    if(targetNode >= nodes.size() && targetNode >= 1)
     targetNode--;
   }

  }
  buttonCancelCount = 0;
 } else if(!buttonMore && oldButtonMore) {
  if(buttonMoreCount < BUTTONSLONGPRESS) {

   if(select < SELECTDEBUGLIDAR)
    select++;
   else
    select = SELECTNONE;

  }
  buttonMoreCount = 0;
 } else if(!buttonLess && oldButtonLess) {
  if(buttonLessCount < BUTTONSLONGPRESS) {

   if(select > SELECTNONE)
    select--;
   else
    select = SELECTDEBUGLIDAR;

  }
  buttonLessCount = 0;
 }

 oldButtonLess = buttonLess;
 oldButtonMore = buttonMore;
 oldButtonCancel = buttonCancel;
 oldButtonOk = buttonOk;

 char text[80];
 switch(select) {
  case SELECTNONE:
   sprintf(text, "");
   break;

  case SELECTHIST:
   drawHist(image, robotPoint, robotTheta, mapDiv);
   drawRobot(image, robotIcon, 1, mapDiv);
   sprintf(text, "");
   break;

  case SELECTLIGHT:
   drawLidarPoints(image, robotPoints, false, mapDiv);
   drawMap(image, map, true, robotPoint, robotTheta, mapDiv);
   //drawHist(image, robotPoint, robotTheta, mapDiv);
   if(!nodes.empty())
    drawPath(image, nodes, paths, start, end, robotPoint, robotTheta, mapDiv);
   drawNodes(image, nodes, targetNode, robotPoint, robotTheta, mapDiv);
   drawRobot(image, robotIcon, 1, mapDiv);
   sprintf(text, "");
   break;

  case SELECTFULL:
   drawLidarPoints(image, robotPoints, false, mapDiv);
   drawMap(image, map, false, robotPoint, robotTheta, mapDiv);
   //drawHist(image, robotPoint, robotTheta, mapDiv);
   //drawLinks(image, nodes, links, robotPoint, robotTheta, mapDiv);
   if(!nodes.empty())
    for(int i = 0; i < nodes.size(); i++)
     drawPath(image, nodes, paths, start, i, robotPoint, robotTheta, mapDiv);
   drawNodes(image, nodes, targetNode, robotPoint, robotTheta, mapDiv);
   //drawNumbers(image, nodes, targetNode, robotPoint, robotTheta, mapDiv);
   drawRobot(image, robotIcon, 1, mapDiv);
   if(nodes.empty())
    sprintf(text, "X %04d | Y %04d | Theta %03d", robotPoint.x, robotPoint.y, robotTheta * 180 / PI16);
   else
    sprintf(text, "Nodes %02d/%02d | Links %03d | X %04d/%04d | Y %04d/%04d | Theta %03d", end, nodes.size(),
            links.size(), robotPoint.x, nodes[end].x, robotPoint.y, nodes[end].y, robotTheta * 180 / PI16);
   break;

  case SELECTDEBUGMAP:
   drawLidarPoints(image, robotPoints, true, mapDiv);
   drawLidarLines(image, robotLinesAxes, mapDiv);
   drawMap(image, map, false, robotPoint, robotTheta, mapDiv);
   //drawIntersects(image, map, robotPoint, robotTheta, mapDiv);
   drawRobot(image, robotIcon, FILLED, mapDiv);
   {
    int n = 0;
    for(int i = 0; i < map.size(); i++)
     if(map[i].validation < VALIDATIONFILTERKEEP)
      n++;
    sprintf(text, "Pending %02d | Lines %03d | Scale %03d mm | Time %02d ms | Mapping %s", n, map.size() - n, mapDiv, time, OFFON[mappingEnabled]);
   }
   break;

  case SELECTDEBUGLIDAR:
   drawLidarPoints(image, robotPoints, true, mapDiv);
   drawLidarLines(image, robotLinesAxes, mapDiv);
   drawRobot(image, robotIcon, FILLED, mapDiv);
   sprintf(text, "Points %03d | Lines %02d/%02d | Scale %03d mm | Time %02d ms",
           robotPoints.size(), robotLinesAxes[0].size(), robotLinesAxes[1].size(), mapDiv, time);
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

 while(polarPoints[0].theta - polarPoints[size - 1].theta > PI16 && size > 1)
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

void writeMapFile(vector<Line> &map, vector<Point> &nodes, vector<array<int, 2>> &links, Point robotPoint,
                  uint16_t robotTheta, bool mappingEnabled, int select, int mapDiv) {
 FileStorage fs(MAPFILE, FileStorage::WRITE);

 if(fs.isOpened()) {
  fs << "map" << "[";
  for(int i = 0; i < map.size(); i++) {
   if(map[i].validation < VALIDATIONFILTERKEEP)
    continue;
   fs << "{";
   fs << "a" << map[i].a;
   fs << "b" << map[i].b;
   fs << "}";
  }
  fs << "]";

  fs << "nodes" << "[";
  for(int i = 0; i < nodes.size(); i++)
   fs << nodes[i];
  fs << "]";

  fs << "links" << "[";
  for(int i = 0; i < links.size(); i++) {
   fs << "[";
   fs << links[i][0];
   fs << links[i][1];
   fs << "]";
  }
  fs << "]";

  fs << "robotPoint" << robotPoint;
  fs << "robotTheta" << robotTheta;

  fs << "mappingEnabled" << mappingEnabled;
  fs << "select" << select;
  fs << "mapDiv" << mapDiv;

  fs.release();
 } else
  fprintf(stderr, "Error writing map file\n");
}

void readMapFile(vector<Line> &map, vector<Point> &nodes, vector<array<int, 2>> &links, Point &robotPoint,
                 uint16_t &robotTheta, bool &mappingEnabled, int &select, int &mapDiv) {
 FileStorage fs(MAPFILE, FileStorage::READ);

 if(fs.isOpened()) {
  FileNode fn1 = fs["map"];
  for(FileNodeIterator it = fn1.begin(); it != fn1.end(); it++) {
   FileNode item = *it;
   Point a;
   Point b;
   item["a"] >> a;
   item["b"] >> b;
   map.push_back({a, b, Point(0, 0), Point(0, 0), 0, VALIDATIONFILTERKEEP, SHRINKFILTER, SHRINKFILTER});
  }

  FileNode fn2 = fs["nodes"];
  for(FileNodeIterator it = fn2.begin(); it != fn2.end(); it++) {
   FileNode item = *it;
   Point point;
   item >> point;
   nodes.push_back(point);
  }

  FileNode fn3 = fs["links"];
  for(FileNodeIterator it = fn3.begin(); it != fn3.end(); it++) {
   FileNode item = *it;
   int a;
   int b;
   item[0] >> a;
   item[1] >> b;
   links.push_back({a, b});
  }

  fs["robotPoint"] >> robotPoint;
  fs["robotTheta"] >> robotTheta;

  fs["mappingEnabled"] >> mappingEnabled;
  fs["select"] >> select;
  fs["mapDiv"] >> mapDiv;

  fs.release();
 } else
  fprintf(stderr, "Error reading map file\n");
}

bool gotoPoint(Point targetPoint, int8_t &vy, int8_t &vz, Point robotPoint, uint16_t robotTheta) {
 Point deltaPoint = targetPoint - robotPoint;
 int dist = int(sqrt(sqNorm(deltaPoint)));
 static int integTheta = 0;
 static int16_t oldDeltaTheta = 0;

 if(dist <= GOTOPOINTDISTTOLERANCE) {
  integTheta = 0;
  return true;
 }

 uint16_t gotoTheta = angleDoubleToAngle16(atan2(deltaPoint.y, deltaPoint.x)) - HALFPI16;
 int16_t deltaTheta = gotoTheta - robotTheta;

 bool reverseGear = false;
 if(abs(deltaTheta) > PI16 / 180 * GOTOPOINTANGLEREVERSEGEAR) {
  deltaTheta += PI16;
  reverseGear = true;
 }

 if(deltaTheta >= 0 && oldDeltaTheta < 0 ||
    deltaTheta <= 0 && oldDeltaTheta > 0)
  integTheta = 0;
 integTheta += deltaTheta;
 integTheta = constrain(integTheta, -GOTOPOINTVELOCITYTHETA * KITHETA, GOTOPOINTVELOCITYTHETA * KITHETA);

 int16_t derivTheta = deltaTheta - oldDeltaTheta;
 oldDeltaTheta = deltaTheta;

 vy = constrain(GOTOPOINTVELOCITY - abs(deltaTheta) * GOTOPOINTVELOCITY * 180 / PI16 / GOTOPOINTANGLESTOP, 0, GOTOPOINTVELOCITY);
 if(reverseGear)
  vy = -vy;
 vz = constrain(deltaTheta / KPTHETA + integTheta / KITHETA + derivTheta / KDTHETA, -GOTOPOINTVELOCITYTHETA, GOTOPOINTVELOCITYTHETA);

 return false;
}

int nextNode(vector<int> &paths, int currentNode, int targetNode) {
 int n = targetNode;

 while(n != -1) {
  if(paths[n] == currentNode)
   return n;
  n = paths[n];
 }

 return -1;
}

void autopilot(vector<Point> &mapPoints, vector<Point> &nodes, vector<int> &paths, int targetNode, Point &robotPoint, uint16_t &robotTheta, bool &running) { // TODO
 static bool oldRunning = false;
 static int currentNode = 0;
 static int8_t vx = 0;
 static int8_t vy = 0;
 static int8_t vz = 0;

 if(running && !oldRunning && !nodes.empty())
  currentNode = closestPoint(nodes, robotPoint);

 else if(remoteFrame.vx || remoteFrame.vy || remoteFrame.vz)
  running = false;
 oldRunning = running;

 if(!running) {
  telemetryFrame.vx = remoteFrame.vx;
  telemetryFrame.vy = remoteFrame.vy;
  telemetryFrame.vz = remoteFrame.vz;
  return;
 }

 if(gotoPoint(nodes[currentNode], vy, vz, robotPoint, robotTheta)) {
  currentNode = nextNode(paths, currentNode, targetNode);
  if(currentNode == -1)
   running = false;
 }

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

void mapIntersects(vector<Line> &map) {
 bool sort = false;

 for(int i = 0; i < map.size(); i++) {
  if(map[i].validation < VALIDATIONFILTERKEEP)
   continue;

  for(int j = 0; j < map.size(); j++) {
   if(i == j || map[j].validation < VALIDATIONFILTERKEEP)
    continue;

   Point intersectPoint;
   if(!intersectLine(map[i], map[j], intersectPoint))
    continue;

   if(testPointLine(map[i].a, map[j], LINESIZEMIN, LINESIZEMIN) &&
      sqDist(map[i].a, intersectPoint) < LINESIZEMIN * LINESIZEMIN) {
    map[i].a = intersectPoint;
    sort = true;
   }

   if(testPointLine(map[i].b, map[j], LINESIZEMIN, LINESIZEMIN) &&
      sqDist(map[i].b, intersectPoint) < LINESIZEMIN * LINESIZEMIN) {
    map[i].b = intersectPoint;
    sort = true;
   }

   if(sqDist(map[i]) < LINESIZEMIN * LINESIZEMIN) {
    map.erase(map.begin() + i);
    if(j >= i)
     j--;
    i--;
    sort = false;
   }

  }
 }

 if(sort)
  sortLines(map);
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

 int fd = serialOpen(SERIALPORT, SERIALRATE);
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
 while(imuThreadStatus == STATUSWAITING);
 if(imuThreadStatus == STATUSERROR) {
  fprintf(stderr, "No IMU found\n");
  return 1;
 }
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
 vector<Line> robotLinesAxes[AXES];
 vector<Line> mapLines;
 vector<Line> map;
 vector<Point> mapPoints;
 vector<Point> nodes;
 vector<array<int, 2>> links;
 vector<int> paths;

 Point robotPoint = Point(0, 0);
 uint16_t robotTheta = 0;
 bool mappingEnabled = true;
 int targetNode = 0;
 bool running = false;
 int select = SELECTLIGHT;
 int mapDiv = MAPDIV;
 fprintf(stderr, "Reading map file\n");
 readMapFile(map, nodes, links, robotPoint, robotTheta, mappingEnabled, select, mapDiv);
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

   if(!robotLines.empty()) {
    sortLines(robotLines);

    for(int i = 0; i < AXES; i++)
     robotLinesAxes[i].clear();
    splitAxes(robotLines, robotLinesAxes);
    localization(robotLinesAxes, map, robotPoint, robotTheta);

    mapLines.clear();
    robotToMap(robotLines, mapLines, robotPoint, robotTheta);

    if(mappingEnabled) {
     mapping(mapLines, map);
     mapCleaner(polarPoints, map, robotPoint, robotTheta);
     mapDeduplicateAverage(map);
     mapDeduplicateErase(map);
     mapIntersects(map);
     mapFiltersDecay(map);
    }

    mapPoints.clear();
    robotToMap(robotPoints, mapPoints, robotPoint, robotTheta);
   }
  }

  autopilot(mapPoints, nodes, paths, targetNode, robotPoint, robotTheta, running);

  ui(image, robotPoints, robotLinesAxes, map,
     mapPoints, nodes, links, paths, targetNode,
     robotPoint, oldRobotPoint, robotTheta, oldRobotTheta,
     mappingEnabled, running, select, mapDiv, time);

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
 writeMapFile(map, nodes, links, robotPoint, robotTheta, mappingEnabled, select, mapDiv);

 fprintf(stderr, "Stopping\n");
 return 0;
}
