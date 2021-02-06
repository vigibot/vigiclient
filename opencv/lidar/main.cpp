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

/*void extractRawLinesPascal(vector<PolarPoint> &polarPoints, vector<Point> &robotPoints, vector<vector<Point>> &robotRawLines) {
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
      sqDist(pointsNoDp[0], pointsNoDp[size - 1]) >= LIDARLINESMINLEN * LIDARLINESMINLEN) {
    robotRawLines.push_back(pointsNoDp);
    if(i > robotPoints.size())
     break;
   }
   pointsNoDp.clear();

  } else
   pointsNoDp.push_back(robotPoints[ii]);
 }
}*/

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
      sqDist(pointsNoDp[0], pointsNoDp[pointsNoDp.size() - 1]) >= LIDARLINESMINLEN * LIDARLINESMINLEN)
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

void robotToMap(Line lineIn, Line &lineOut, Point robotPoint, uint16_t robotTheta) {
 lineOut = {robotPoint + rotate(lineIn.a, robotTheta),
            robotPoint + rotate(lineIn.b, robotTheta)};
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
               Point &pointError, double &angularError, int &distError) {

 pointError = pointDistancePointLine((line1.a + line1.b) / 2, line2);
 distError = int(sqrt(sqNorm(pointError)));
 if(distError > distTolerance)
  return false;

 angularError = diffAngle(line1, line2);
 if(fabs(angularError) > angularTolerance)
  return false;

 int refNorm = int(sqrt(sqDist(line2)));
 int distance1 = ratioPointLine(line1.a, line2) * refNorm;
 int distance2 = ratioPointLine(line1.b, line2) * refNorm;

 if(distance1 < -lengthMargin && distance2 < -lengthMargin ||
    distance1 > refNorm + lengthMargin && distance2 > refNorm + lengthMargin)
  return false;

 return true;
}

bool testLines(Line line1, Line line2, int distTolerance, double angularTolerance, int lengthMargin,
               Point &pointError, double &angularError, int &distError, int &length) {

 pointError = pointDistancePointLine((line1.a + line1.b) / 2, line2);
 distError = int(sqrt(sqNorm(pointError)));
 if(distError > distTolerance)
  return false;

 angularError = diffAngle(line1, line2);
 if(fabs(angularError) > angularTolerance)
  return false;

 int refNorm = int(sqrt(sqDist(line2)));
 double ratio1 = ratioPointLine(line1.a, line2);
 double ratio2 = ratioPointLine(line1.b, line2);
 int distance1 = ratio1 * refNorm;
 int distance2 = ratio2 * refNorm;

 if(distance1 < -lengthMargin && distance2 < -lengthMargin ||
    distance1 > refNorm + lengthMargin && distance2 > refNorm + lengthMargin)
  return false;

 bool test1 = ratio1 >= 0 && ratio1 <= 1; // Point line1.a is in line2
 bool test2 = ratio2 >= 0 && ratio2 <= 1; // Point line1.b is in line2

 if(test1 && test2)
  length = int(sqrt(sqDist(line1)));
 else if(test1 && !test2) {
  if(ratio2 <= 0)
   length = int(sqrt(sqDist(line1.a, line2.a)));
  else
   length = int(sqrt(sqDist(line1.a, line2.b)));
 } else if(!test1 && test2) {
  if(ratio1 <= 0)
   length = int(sqrt(sqDist(line1.b, line2.a)));
  else
   length = int(sqrt(sqDist(line1.b, line2.b)));
 } else
  length = refNorm;

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

   if(sqDist(map[i]) < MAPLINESMINLEN * MAPLINESMINLEN) {
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
   if(testLines(map[i], map[j], SMALLDISTTOLERANCE, SMALLANGULARTOLERANCE, -SMALLDISTTOLERANCE,
                pointError, angularError, distError))
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
   if(testLines(map[i], map[j], LARGEDISTTOLERANCE, LARGEANGULARTOLERANCE, -SMALLDISTTOLERANCE,
                pointError, angularError, distError)) {
    map.erase(map.begin() + j);
    j--;
   }
  }
 }
}

bool computeErrors(vector<Line> &mapLines, vector<Line> &map,
                   Point &pointErrorOut, double &angularErrorOut, int &confidence,
                   int distTolerance, double angularTolerance) {

 int mapLinesWeightSum = 0;
 Point pointErrorSum = Point(0, 0);
 int pointErrorWeightSum = 0;
 double angularErrorSum = 0.0;
 int angularErrorWeightSum = 0;

 for(int i = 0; i < mapLines.size(); i++) {
  mapLinesWeightSum += int(sqrt(sqDist(mapLines[i])));

  for(int j = 0; j < map.size(); j++) {
   Point pointError;
   double angularError;
   int distError;
   int length;

   if(map[j].validation >= VALIDATIONFILTERSTART &&
      testLines(mapLines[i], map[j], distTolerance, angularTolerance, -SMALLDISTTOLERANCE,
                pointError, angularError, distError, length)) {

    pointErrorSum += pointError * length;
    pointErrorWeightSum += length;

    if(map[j].validation >= VALIDATIONFILTERKEEP) {
     angularErrorSum += angularError * length;
     angularErrorWeightSum += length;
    }

    break;
   }
  }
 }

 if(mapLinesWeightSum)
  confidence = pointErrorWeightSum * 100 / mapLinesWeightSum;
 else
  confidence = 100;

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
  if(sqDist(mapLines[i]) < MAPLINESMINLEN * MAPLINESMINLEN)
   continue;

  bool newLine = true;
  for(int j = 0; j < map.size(); j++) {
   Point pointError;
   double angularError;
   int distError;
   if(!testLines(mapLines[i], map[j], LARGEDISTTOLERANCE * 2, LARGEANGULARTOLERANCE, -SMALLDISTTOLERANCE,
      pointError, angularError, distError))
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

 if(n++ == MAPFILTERSDECAY)
  n = 0;
 else
  return;

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

void localization(vector<Line> robotLinesAxes[], vector<Line> &map, int confidences[], Point &robotPoint, uint16_t &robotTheta) {
 static int c[AXES] = {0};

 for(int i = 0; i < NBITERATIONS; i++) {
  for(int j = 0; j < AXES; j++) {
   vector<Line> mapLinesAxe;
   int distTolerance;
   Point pointError;
   double angularError;
   int confidence;

   robotToMap(robotLinesAxes[j], mapLinesAxe, robotPoint, robotTheta);

   if(c[j] <= RECOVERYCONFIDENCE)
    distTolerance = RECOVERYDISTTOLERANCE;
   else
    distTolerance = LARGEDISTTOLERANCE;

   if(computeErrors(mapLinesAxe, map, pointError, angularError, confidence,
      distTolerance / (i + 1), LARGEANGULARTOLERANCE / (i + 1))) {

    robotPoint -= pointError / (i + 1);
#ifdef IMU
    robotThetaCorrector += int(angularError * double(PI16) / M_PI) / IMUTHETACORRECTORDIV;
#else
    robotTheta += int(angularError * double(PI16) / M_PI) / (i + 1);
#endif
   }

   if(i == 0)
    confidences[j] = confidence;
  }
 }

 for(int i = 0; i < AXES; i++)
  c[i] = confidences[i];
}

Point rescaleTranslate(Point point, int mapDiv) {
 point.x = point.x * 10 / mapDiv;
 point.y = point.y * 10 / -mapDiv;
 return Point(width / 2, height / 2) + point;
}

void drawLidarPoints(Mat &image, vector<Point> &points, bool beams, Point beamsSource, Point offset, int mapDiv) {
 for(int i = 0; i < points.size(); i++) {
  Point point = rescaleTranslate(points[i] - offset, mapDiv);

  if(beams)
   line(image, beamsSource, point, Scalar::all(64), 1, LINE_AA);
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

   if(i == 0)
    line(image, point1, point2, Scalar(128, 128, 255), 1, LINE_AA);
   else
    line(image, point1, point2, Scalar(255, 128, 128), 1, LINE_AA);
  }
 }
}

void drawLidarLines(Mat &image, vector<Line> &lines, Point offset, int mapDiv) {
 for(int i = 0; i < lines.size(); i++) {
  Point point1 = rescaleTranslate(lines[i].a - offset, mapDiv);
  Point point2 = rescaleTranslate(lines[i].b - offset, mapDiv);

  line(image, point1, point2, Scalar::all(255), 1, LINE_AA);
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

void drawHist(Mat &image, Point histPoint, Point robotPoint, uint16_t robotTheta, int mapDiv) {
 static Point hist[HIST] = {Point(0, 0)};
 static int n = 0;
 static Point oldPoint = Point(0, 0);

 hist[n++] = histPoint;
 if(n == HIST)
  n = 0;

 for(int i = 0; i < HIST; i++) {
  Point point = rescaleTranslate(rotate(hist[(i + n) % HIST] - robotPoint, -robotTheta), mapDiv);

  if(i != 0) {
   int distTolerancePixels = LARGEDISTTOLERANCE * 10 / mapDiv;
   if(sqDist(oldPoint, point) < distTolerancePixels * distTolerancePixels)
    line(image, oldPoint, point, Scalar(0, 128, 128), 1, LINE_AA);
  }

  oldPoint = point;
 }
}

void drawColoredPoint(Mat &image, Point point, Scalar color, Point robotPoint, uint16_t robotTheta, int mapDiv) {
 point = rescaleTranslate(rotate(point - robotPoint, -robotTheta), mapDiv);
 circle(image, point, 2, color, FILLED, LINE_AA);
}

void drawPath(Mat &image, vector<Point> &nodes, vector<int> &paths, int end, Point robotPoint, uint16_t robotTheta, int mapDiv) {
 int n = end;
 Point oldPoint = rescaleTranslate(rotate(nodes[n] - robotPoint, -robotTheta), mapDiv);

 while(n != -1) {
  Point point = rescaleTranslate(rotate(nodes[n] - robotPoint, -robotTheta), mapDiv);

  line(image, oldPoint, point, Scalar(128, 128, 0), 1, LINE_AA);
  oldPoint = point;

  n = paths[n];
 }
}

void drawTargetPoint(Mat &image, Point targetPoint, Point robotPoint, uint16_t robotTheta, int mapDiv) {
 Point point = rescaleTranslate(rotate(targetPoint - robotPoint, -robotTheta), mapDiv);

 line(image, point + Point(-5, -5), point + Point(5, 5), Scalar(0, 255, 255), 1, LINE_AA);
 line(image, point + Point(-5, 5), point + Point(5, -5), Scalar(0, 255, 255), 1, LINE_AA);
}

void drawRobot(Mat &image, vector<Point> robotIcon, int thickness, Point robotPoint, uint16_t robotTheta, int mapDiv) {
 vector<Point> polygon;

 for(int i = 0; i < robotIcon.size(); i++) {
  Point point = rescaleTranslate(robotPoint + rotate(robotIcon[i], robotTheta), mapDiv);
  polygon.push_back(point);
 }

 vector<vector<Point>> tmp(1, polygon);
 drawContours(image, tmp, -1, Scalar::all(255), thickness, LINE_AA);
}

void drawNodes(Mat &image, vector<Point> &nodes, Point robotPoint, uint16_t robotTheta, int mapDiv) {
 for(int i = 0; i < nodes.size(); i++) {
  Point point = rescaleTranslate(rotate(nodes[i] - robotPoint, -robotTheta), mapDiv);
  circle(image, point, 1, Scalar::all(128), FILLED, LINE_AA);
 }
}

/*void drawNumbers(Mat &image, vector<Point> &nodes, int targetNode, Point robotPoint, uint16_t robotTheta, int mapDiv) {
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
}*/

bool computePose(Line mapRef1, Line mapRef2, Line robotLine1, Line robotLine2,
                 Point distance1, Point distance2, Line &mapLine1, Line &mapLine2, Point &robotPoint, uint16_t &robotTheta) {

 if(sqDist(mapRef1) < sqDist(robotLine1) * LINELENGTHTOLERANCEPERCENT / 100 ||
    sqDist(mapRef2) < sqDist(robotLine2) * LINELENGTHTOLERANCEPERCENT / 100)
  return false;

 robotTheta = int(diffAngle(lineAngle(robotLine1), lineAngle(mapRef1)) * double(PI16) / M_PI);

 robotPoint = -pointDistancePointLine({0, 0}, mapRef1) + rotate(distance1, robotTheta) +
              -pointDistancePointLine({0, 0}, mapRef2) + rotate(distance2, robotTheta);

 robotToMap(robotLine1, mapLine1, robotPoint, robotTheta);
 robotToMap(robotLine2, mapLine2, robotPoint, robotTheta);

 return true;
}

bool probabilisticLocalization(vector<Line> robotLinesAxes[], vector<Line> &map, Point &robotPointFound, uint16_t &robotThetaFound) {
 Line robotLine1 = robotLinesAxes[0][0];
 Line robotLine2 = robotLinesAxes[1][0];
 double refAngle = diffAngle(robotLine1, robotLine2);
 Point distance1 = pointDistancePointLine({0, 0}, robotLine1);
 Point distance2 = pointDistancePointLine({0, 0}, robotLine2);
 int confidenceMax = 0;
 bool found = false;

 for(int i = 0; i < map.size(); i++) {
  for(int j = i + 1; j < map.size(); j++) {
   double angle = diffAngle(map[i], map[j]);
   bool ok = false;
   Line mapLine1;
   Line mapLine2;
   Point robotPoint;
   uint16_t robotTheta;

   if(fabs(angle - refAngle) < LARGEANGULARTOLERANCE)
    ok = computePose(map[i], map[j], robotLine1, robotLine2,
                     distance1, distance2, mapLine1, mapLine2, robotPoint, robotTheta);
   else if(fabs(angle + refAngle) < LARGEANGULARTOLERANCE)
    ok = computePose(map[j], map[i], robotLine1, robotLine2,
                     distance1, distance2, mapLine2, mapLine1, robotPoint, robotTheta);

   if(ok &&
      testPointLine(mapLine1.a, map[i], LARGEDISTTOLERANCE, LARGEDISTTOLERANCE) &&
      testPointLine(mapLine1.b, map[i], LARGEDISTTOLERANCE, LARGEDISTTOLERANCE) &&
      testPointLine(mapLine2.a, map[j], LARGEDISTTOLERANCE, LARGEDISTTOLERANCE) &&
      testPointLine(mapLine2.b, map[j], LARGEDISTTOLERANCE, LARGEDISTTOLERANCE)) {

    int confidenceSum = 0;
    for(int k = 0; k < AXES; k++) {
     vector<Line> mapLinesAxe;
     Point pointError;
     double angularError;
     int confidence;

     robotToMap(robotLinesAxes[k], mapLinesAxe, robotPoint, robotTheta);
     computeErrors(mapLinesAxe, map, pointError, angularError, confidence,
                   LARGEDISTTOLERANCE, LARGEANGULARTOLERANCE);

     confidenceSum += confidence;
    }
    confidenceSum /= AXES;

    if(confidenceSum > confidenceMax) {
     confidenceMax = confidenceSum;
     robotPointFound = robotPoint;
     robotThetaFound = robotTheta;
     found = true;
    }
   }

  }
 }

 return found;
}

bool obstacle(vector<Point> &mapPoints, Point robotPoint, Point targetPoint, int obstacleDetectionRange) {
 int n = 0;

 for(int i = 0; i < mapPoints.size(); i++) {
  Line line = {robotPoint, targetPoint};
  if(sqNorm(pointDistancePointLine(mapPoints[i], line)) < OBSTACLEROBOTWIDTH * OBSTACLEROBOTWIDTH) {
   int refNorm = int(sqrt(sqDist(line)));
   int distance = ratioPointLine(mapPoints[i], line) * refNorm;

   if(distance > 0 && distance <= obstacleDetectionRange) {
    n++;
    if(n == OBSTACLENBPOINTS)
     return true;
   }
  }
 }

 return false;
}

void dijkstra(list<pair<int, int>> adjacent[], int nbNodes, int start, vector<int> &pathsOut, vector<int> &distsOut) {
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
 distsOut = dists;
}

void addLink(list<pair<int, int>> adjacent[], int a, int b, int weight) {
 adjacent[a].push_back(make_pair(b, weight));
 adjacent[b].push_back(make_pair(a, weight));
}

void computePaths(vector<Point> &nodes, vector<array<int, 2>> &links, int start, vector<int> &paths, vector<int> &dists) {
 fprintf(stderr, "Launching Dijkstra's algorithm with %d nodes and %d links for the node %d\n", nodes.size(), links.size(), start);

 list<pair<int, int>> *adjacent;
 adjacent = new list<Pair>[nodes.size()];

 for(int i = 0; i < links.size(); i++) {
  int dist = int(sqrt(sqDist(nodes[links[i][0]], nodes[links[i][1]])));
  addLink(adjacent, links[i][0], links[i][1], dist);
 }

 dijkstra(adjacent, nodes.size(), start, paths, dists);

 delete [] adjacent;
 fprintf(stderr, "Ending Dijkstra's algorithm\n");
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

bool addNodeAndLinks(vector<Point> &mapPoints, vector<Point> &nodes, vector<array<int, 2>> &links, Point node) {
 if(nodes.empty()) {
  nodes.push_back(node);
  return true;
 }

 int closest = closestPoint(nodes, node);
 if(sqDist(node, nodes[closest]) < LINKSLENGTHMIN * LINKSLENGTHMIN)
  return false;

 vector<array<int, 2>> linksBuffer;
 for(int i = 0; i < nodes.size(); i++) {
  int dist = sqDist(node, nodes[i]);

  if(dist <= LINKSLENGTHMAX * LINKSLENGTHMAX)
   linksBuffer.push_back({int(nodes.size()), i});
 }

 if(!linksBuffer.empty()) {
  bool ok = true;

  for(int i = 1; i < mapPoints.size(); i++)
   if(testPointLine(node, {mapPoints[i - 1], mapPoints[i]}, DISTFROMOBSTACLE, DISTFROMOBSTACLE))
    ok = false;

  if(ok) {
   fprintf(stderr, "Adding the node %d with %d link(s)\n", nodes.size(), linksBuffer.size());
   nodes.push_back(node);
   for(int i = 0; i < linksBuffer.size(); i++)
    links.push_back(linksBuffer[i]);

   return true;
  }
 }

 return false;
}

void delNodeAndLinks(vector<Point> &nodes, vector<array<int, 2>> &links, int nodeIndex) {
 fprintf(stderr, "Deleting the node %d\n", nodeIndex);

 nodes.erase(nodes.begin() + nodeIndex);
 for(int i = 0; i < links.size(); i++) {
  if(nodeIndex == links[i][0] ||
     nodeIndex == links[i][1]) {
   fprintf(stderr, "Deleting the associated link %d\n", i);
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

void delNode(vector<Point> &nodes, vector<array<int, 2>> &links, int nodeIndex) {
 fprintf(stderr, "Deleting the node %d\n", nodeIndex);

 nodes.erase(nodes.begin() + nodeIndex);
 for(int i = 0; i < links.size(); i++) {
  if(nodeIndex < links[i][0])
   links[i][0]--;
  if(nodeIndex < links[i][1])
   links[i][1]--;
 }
}

void delLinkAndNodes(vector<Point> &nodes, vector<array<int, 2>> &links, int a, int b) {
 bool del = false;
 int minab = min(a, b);
 int maxab = max(a, b);
 bool delmin = true;
 bool delmax = true;

 for(int i = 0; i < links.size(); i++) {
  if(!del && (links[i][0] == a && links[i][1] == b || links[i][0] == b && links[i][1] == a)) {
   fprintf(stderr, "Deleting the link %d\n", i);
   links.erase(links.begin() + i);
   i--;
   del = true;
  } else {
   if(links[i][0] == minab || links[i][1] == minab)
    delmin = false;
   if(links[i][0] == maxab || links[i][1] == maxab)
    delmax = false;
  }
 }

 if(delmax) {
  fprintf(stderr, "Deleting the node without link %d\n", maxab);
  delNode(nodes, links, maxab);
 }
 if(delmin) {
  fprintf(stderr, "Deleting the node without link %d\n", minab);
  delNode(nodes, links, minab);
 }
}

/*void delLink(vector<array<int, 2>> &links, int a, int b) {
 for(int i = 0; i < links.size(); i++) {
  if(links[i][0] == a && links[i][1] == b ||
     links[i][1] == a && links[i][0] == b) {
   fprintf(stderr, "Deleting the link %d\n", i);
   links.erase(links.begin() + i);
   break;
  }
 }
}*/

void graphing(vector<PolarPoint> &polarPoints, vector<Point> &mapPoints, vector<Point> &nodes, vector<array<int, 2>> &links,
              vector<int> &paths, vector<int> &dists, Point targetPoint, int &targetNode, Point robotPoint, uint16_t robotTheta) {

 static int n = 0;

 if(n++ == GRAPHINGSLOWDOWN)
  n = 0;
 else
  return;

 bool added = false;
 for(int i = 0; i < polarPoints.size(); i++) {
  for(int j = polarPoints[i].distance - LINKSLENGTHMIN; j > LINKSLENGTHMIN; j -= LINKSLENGTHMIN / 2) {
   Point closerPoint = Point(j * sin16(polarPoints[i].theta) / ONE16,
                             j * cos16(polarPoints[i].theta) / ONE16);

   Point closerMapPoint = robotPoint + rotate(closerPoint, robotTheta);
   added |= addNodeAndLinks(mapPoints, nodes, links, closerMapPoint);
  }
 }

 if(added) {
  targetNode = closestPoint(nodes, targetPoint);
  computePaths(nodes, links, targetNode, paths, dists);
 }
}

void ui(Mat &image, vector<Point> &robotPoints, vector<Line> robotLinesAxes[], vector<Line> &mapLines, vector<Line> &map, vector<Point> &mapPoints,
                    vector<Point> &nodes, vector<array<int, 2>> &links, vector<int> &paths, vector<int> &dists, Point &targetPoint, int &targetNode, int &closestRobot,
                    Point &robotPoint, Point &oldRobotPoint, uint16_t &robotTheta, uint16_t &oldRobotTheta,
                    bool &mappingEnabled, bool &graphingEnabled, bool &running, int &select, int &mapDiv, int confidences[], int time) {

 int xmin = INT_MAX;
 int xmax = INT_MIN;
 int ymin = INT_MAX;
 int ymax = INT_MIN;
 Point offsetPoint = Point(0, 0);
 int mapDivFixed = mapDiv;
 static Point oldRemoteFramePoint = robotPoint;
 static int oldTargetNode = 0;
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

 if(!map.empty()) {
  for(int i = 0; i < map.size(); i++) {
   if(map[i].validation < VALIDATIONFILTERKEEP)
    continue;

   if(xmin > map[i].a.x)
    xmin = map[i].a.x;
   if(xmin > map[i].b.x)
    xmin = map[i].b.x;
   if(xmax < map[i].a.x)
    xmax = map[i].a.x;
   if(xmax < map[i].b.x)
    xmax = map[i].b.x;
   if(ymin > map[i].a.y)
    ymin = map[i].a.y;
   if(ymin > map[i].b.y)
    ymin = map[i].b.y;
   if(ymax < map[i].a.y)
    ymax = map[i].a.y;
   if(ymax < map[i].b.y)
    ymax = map[i].b.y;
  }

  offsetPoint = Point(xmax + xmin, ymax + ymin) / 2;
  mapDivFixed = constrain(max((xmax - xmin) * 10 / (width - 40),
                              (ymax - ymin) * 10 / (height - 40)), MAPDIVMIN, MAPDIVMAX);
 }

 Point remoteFramePoint = Point(remoteFrame.xy[GOTOTOOL][0], remoteFrame.xy[GOTOTOOL][1]);
 if(remoteFramePoint != oldRemoteFramePoint) {
  oldRemoteFramePoint = remoteFramePoint;

  if(select >= SELECTNONE && select <= SELECTFIXEDDEBUGMAP) {
   targetPoint.x = (remoteFramePoint.x * mapDivFixed / 10) * width / 65535;
   targetPoint.y = (remoteFramePoint.y * mapDivFixed / 10) * height / 65535;
   targetPoint += offsetPoint;
  } else {
   targetPoint.x = (remoteFramePoint.x * mapDiv / 10) * width / 65535;
   targetPoint.y = (remoteFramePoint.y * mapDiv / 10) * height / 65535;
   targetPoint = rotate(targetPoint, robotTheta) + robotPoint;
  }
 }

 if(!nodes.empty()) {
  targetNode = closestPoint(nodes, targetPoint);
  if(targetNode != oldTargetNode) {
   computePaths(nodes, links, targetNode, paths, dists);
   oldTargetNode = targetNode;
  }
 }

 if(buttonOk) {
  buttonOkCount++;
  if(buttonOkCount == BUTTONSLONGPRESS) {

   if(select == SELECTFIXEDFULL || select == SELECTFULL)
    graphingEnabled = !graphingEnabled;
   else if(select == SELECTFIXEDDEBUGMAP || select == SELECTDEBUGMAP) {
    mappingEnabled = !mappingEnabled;
    for(int i = 0; i < map.size(); i++) {
     if(map[i].validation < VALIDATIONFILTERKEEP) {
      map.erase(map.begin() + i);
      i--;
     }
    }
   } else
    running = !running;

  }
 } else if(buttonCancel) {
  buttonCancelCount++;
  if(buttonCancelCount == BUTTONSLONGPRESS) {

   if(select == SELECTFIXEDFULL || select == SELECTFULL) {
    running = false;
    nodes.clear();
    links.clear();
    paths.clear();
    paths.push_back(-1);
   } else if(select == SELECTFIXEDDEBUGMAP || select == SELECTDEBUGMAP) {
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

   if(select >= SELECTLIGHT && select <= SELECTDEBUGLIDAR) {
    if(mapDiv > MAPDIVMIN)
     mapDiv -= 2;
   }

  }
 } else if(buttonLess) {
  buttonLessCount++;
  if(buttonLessCount >= BUTTONSLONGPRESS) {

   if(select >= SELECTLIGHT && select <= SELECTDEBUGLIDAR) {
    if(mapDiv < MAPDIVMAX)
     mapDiv += 2;
   }

  }
 } else if(!buttonOk && oldButtonOk) {
  if(buttonOkCount < BUTTONSLONGPRESS) {

   if(select == SELECTFIXEDFULL || select == SELECTFULL) {
    if(addNodeAndLinks(mapPoints, nodes, links, targetPoint)) {
     targetNode = closestPoint(nodes, targetPoint);
     computePaths(nodes, links, targetNode, paths, dists);
     closestRobot = closestPoint(nodes, robotPoint);
    }
   }

  }
  buttonOkCount = 0;
 } else if(!buttonCancel && oldButtonCancel) {
  if(buttonCancelCount < BUTTONSLONGPRESS) {

   if(select == SELECTFIXEDFULL || select == SELECTFULL) {
    if(!nodes.empty())
     delNodeAndLinks(nodes, links, targetNode);
    if(!nodes.empty()) {
     targetNode = closestPoint(nodes, targetPoint);
     computePaths(nodes, links, targetNode, paths, dists);
     closestRobot = closestPoint(nodes, robotPoint);
    }
   } else if(select == SELECTFIXEDDEBUGMAP || select == SELECTDEBUGMAP) {
    for(int i = 0; i < map.size(); i++) {
     if(testPointLine(targetPoint, {map[i].a, map[i].b}, DISTFROMOBSTACLE, DISTFROMOBSTACLE)) {
      map.erase(map.begin() + i);
      break;
     }
    }
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
   drawRobot(image, robotIcon, 1, robotPoint - offsetPoint, robotTheta, mapDivFixed);
   drawTargetPoint(image, targetPoint, offsetPoint, 0, mapDivFixed);
   if(!nodes.empty()) {
    Point robotPointFound;
    uint16_t robotThetaFound;
    if(probabilisticLocalization(robotLinesAxes, map, robotPointFound, robotThetaFound))
     drawRobot(image, robotIcon, FILLED, robotPointFound - offsetPoint, robotThetaFound, mapDivFixed);
   }
   sprintf(text, "");
   break;

  case SELECTFIXEDLIGHT:
   drawHist(image, robotPoint, offsetPoint, 0, mapDivFixed);
   drawLidarPoints(image, mapPoints, false, Point(0, 0), offsetPoint, mapDivFixed);
   drawMap(image, map, true, offsetPoint, 0, mapDivFixed);
   if(!nodes.empty()) {
    if(closestRobot != -1) {
     drawPath(image, nodes, paths, closestRobot, offsetPoint, 0, mapDivFixed);
     drawColoredPoint(image, nodes[closestRobot], Scalar(0, 0, 255), offsetPoint, 0, mapDivFixed);
     if(paths[closestRobot] != -1)
      drawColoredPoint(image, nodes[paths[closestRobot]], Scalar(0, 255, 255), offsetPoint, 0, mapDivFixed);
    }
    drawColoredPoint(image, nodes[targetNode], Scalar(0, 255, 0), offsetPoint, 0, mapDivFixed);
   }
   drawRobot(image, robotIcon, FILLED, robotPoint - offsetPoint, robotTheta, mapDivFixed);
   drawTargetPoint(image, targetPoint, offsetPoint, 0, mapDivFixed);
   {
    int dist = int(sqrt(sqDist(robotPoint, targetPoint)));
    if(nodes.empty())
     sprintf(text, "Target %05d mm | Autopilot %s", dist, OFFON[running]);
    else {
     if(closestRobot != -1 && dists[closestRobot] != INT_MAX)
      sprintf(text, "Target %05d mm | Route %05d mm | Autopilot %s", dist, dists[closestRobot], OFFON[running]);
     else
      sprintf(text, "Target %05d mm | No route | Autopilot %s", dist, OFFON[running]);
    }
   }
   break;

  case SELECTFIXEDFULL:
   drawHist(image, robotPoint, offsetPoint, 0, mapDivFixed);
   drawLidarPoints(image, mapPoints, false, Point(0, 0), offsetPoint, mapDivFixed);
   drawMap(image, map, false, offsetPoint, 0, mapDivFixed);
   //for(int i = 0; i < nodes.size(); i++)
    //drawPath(image, nodes, paths, i, offsetPoint, 0, mapDivFixed);
   if(!nodes.empty()) {
    drawNodes(image, nodes, offsetPoint, 0, mapDivFixed);
    if(closestRobot != -1) {
     drawPath(image, nodes, paths, closestRobot, offsetPoint, 0, mapDivFixed);
     drawColoredPoint(image, nodes[closestRobot], Scalar(0, 0, 255), offsetPoint, 0, mapDivFixed);
     if(paths[closestRobot] != -1)
      drawColoredPoint(image, nodes[paths[closestRobot]], Scalar(0, 255, 255), offsetPoint, 0, mapDivFixed);
    }
    drawColoredPoint(image, nodes[targetNode], Scalar(0, 255, 0), offsetPoint, 0, mapDivFixed);
   }
   drawRobot(image, robotIcon, FILLED, robotPoint - offsetPoint, robotTheta, mapDivFixed);
   drawTargetPoint(image, targetPoint, offsetPoint, 0, mapDivFixed);
   {
    int thetaDeg = robotTheta * 180 / PI16;
    if(nodes.empty())
     sprintf(text, "X %06d/%06d mm | Y %06d/%06d mm | Theta %03d | Graph %s",
             robotPoint.x, targetPoint.x, robotPoint.y, targetPoint.y, thetaDeg, OFFON[graphingEnabled]);
    else
     sprintf(text, "Nodes %04d | Links %05d | X %06d | Y %06d | Theta %03d | Graph %s",
             nodes.size(), links.size(), robotPoint.x, robotPoint.y, thetaDeg, OFFON[graphingEnabled]);
   }
   break;

  case SELECTFIXEDDEBUGMAP:
   drawLidarPoints(image, mapPoints, true, rescaleTranslate(robotPoint - offsetPoint, mapDivFixed), offsetPoint, mapDivFixed);
   drawLidarLines(image, mapLines, offsetPoint, mapDivFixed);
   drawMap(image, map, false, offsetPoint, 0, mapDivFixed);
   drawRobot(image, robotIcon, FILLED, robotPoint - offsetPoint, robotTheta, mapDivFixed);
   drawTargetPoint(image, targetPoint, offsetPoint, 0, mapDivFixed);
   {
    int n = 0;
    for(int i = 0; i < map.size(); i++)
     if(map[i].validation < VALIDATIONFILTERKEEP)
      n++;
    sprintf(text, "Pending %02d | Lines %03d | Scale %03d mm | %03d %03d % | %02d ms | Map %s",
            n, map.size() - n, mapDiv / 10, confidences[0], confidences[1], time, OFFON[mappingEnabled]);
   }
   break;

  case SELECTLIGHT:
   drawHist(image, robotPoint, robotPoint, robotTheta, mapDiv);
   drawLidarPoints(image, robotPoints, false, Point(0, 0), Point(0, 0), mapDiv);
   drawMap(image, map, true, robotPoint, robotTheta, mapDiv);
   if(!nodes.empty()) {
    if(closestRobot != -1) {
     drawPath(image, nodes, paths, closestRobot, robotPoint, robotTheta, mapDiv);
     drawColoredPoint(image, nodes[closestRobot], Scalar(0, 0, 255), robotPoint, robotTheta, mapDiv);
     if(paths[closestRobot] != -1)
      drawColoredPoint(image, nodes[paths[closestRobot]], Scalar(0, 255, 255), robotPoint, robotTheta, mapDiv);
    }
    drawColoredPoint(image, nodes[targetNode], Scalar(0, 255, 0), robotPoint, robotTheta, mapDiv);
   }
   drawRobot(image, robotIcon, 1, Point(0, 0), 0, mapDiv);
   drawTargetPoint(image, targetPoint, robotPoint, robotTheta, mapDiv);
   {
    int dist = int(sqrt(sqDist(robotPoint, targetPoint)));
    if(nodes.empty())
     sprintf(text, "Target %05d mm | Autopilot %s", dist, OFFON[running]);
    else {
     if(closestRobot != -1 && dists[closestRobot] != INT_MAX)
      sprintf(text, "Target %05d mm | Route %05d mm | Autopilot %s", dist, dists[closestRobot], OFFON[running]);
     else
      sprintf(text, "Target %05d mm | No route | Autopilot %s", dist, OFFON[running]);
    }
   }
   break;

  case SELECTFULL:
   drawHist(image, robotPoint, robotPoint, robotTheta, mapDiv);
   drawLidarPoints(image, robotPoints, false, Point(0, 0), Point(0, 0), mapDiv);
   drawMap(image, map, false, robotPoint, robotTheta, mapDiv);
   //for(int i = 0; i < nodes.size(); i++)
    //drawPath(image, nodes, paths, i, robotPoint, robotTheta, mapDiv);
   if(!nodes.empty()) {
    drawNodes(image, nodes, robotPoint, robotTheta, mapDiv);
    if(closestRobot != -1) {
     drawPath(image, nodes, paths, closestRobot, robotPoint, robotTheta, mapDiv);
     drawColoredPoint(image, nodes[closestRobot], Scalar(0, 0, 255), robotPoint, robotTheta, mapDiv);
     if(paths[closestRobot] != -1)
      drawColoredPoint(image, nodes[paths[closestRobot]], Scalar(0, 255, 255), robotPoint, robotTheta, mapDiv);
    }
    drawColoredPoint(image, nodes[targetNode], Scalar(0, 255, 0), robotPoint, robotTheta, mapDiv);
   }
   drawRobot(image, robotIcon, 1, Point(0, 0), 0, mapDiv);
   drawTargetPoint(image, targetPoint, robotPoint, robotTheta, mapDiv);
   {
    int thetaDeg = robotTheta * 180 / PI16;
    if(nodes.empty())
     sprintf(text, "X %06d/%06d mm | Y %06d/%06d mm | Theta %03d | Graph %s",
             robotPoint.x, targetPoint.x, robotPoint.y, targetPoint.y, thetaDeg, OFFON[graphingEnabled]);
    else
     sprintf(text, "Nodes %04d | Links %05d | X %06d | Y %06d | Theta %03d | Graph %s",
             nodes.size(), links.size(), robotPoint.x, robotPoint.y, thetaDeg, OFFON[graphingEnabled]);
   }
   break;

  case SELECTDEBUGMAP:
   drawLidarPoints(image, robotPoints, true, Point(width / 2, height / 2), Point(0, 0), mapDiv);
   drawLidarLines(image, robotLinesAxes, mapDiv);
   drawMap(image, map, false, robotPoint, robotTheta, mapDiv);
   drawRobot(image, robotIcon, FILLED, Point(0, 0), 0, mapDiv);
   drawTargetPoint(image, targetPoint, robotPoint, robotTheta, mapDiv);
   {
    int n = 0;
    for(int i = 0; i < map.size(); i++)
     if(map[i].validation < VALIDATIONFILTERKEEP)
      n++;
    sprintf(text, "Pending %02d | Lines %03d | Scale %03d mm | %03d %03d % | %02d ms | Map %s",
            n, map.size() - n, mapDiv / 10, confidences[0], confidences[1], time, OFFON[mappingEnabled]);
   }
   break;

  case SELECTDEBUGLIDAR:
   drawLidarPoints(image, robotPoints, true, Point(width / 2, height / 2), Point(0, 0), mapDiv);
   drawLidarLines(image, robotLinesAxes, mapDiv);
   drawRobot(image, robotIcon, FILLED, Point(0, 0), 0, mapDiv);
   sprintf(text, "Points %03d | Lines %02d/%02d | Scale %03d mm | Time %02d ms",
           robotPoints.size(), robotLinesAxes[0].size(), robotLinesAxes[1].size(), mapDiv / 10, time);
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
                  uint16_t robotTheta, bool mappingEnabled, bool graphingEnabled, int select, int mapDiv) {
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
  fs << "graphingEnabled" << graphingEnabled;
  fs << "select" << select;
  fs << "mapDiv" << mapDiv;

  fs.release();
 } else
  fprintf(stderr, "Error writing map file\n");
}

void readMapFile(vector<Line> &map, vector<Point> &nodes, vector<array<int, 2>> &links, Point &robotPoint,
                 uint16_t &robotTheta, bool &mappingEnabled, bool &graphingEnabled, int &select, int &mapDiv) {
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
  fs["graphingEnabled"] >> graphingEnabled;
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

/*int nextNode(vector<int> &paths, int currentNode, int targetNode) {
 int n = targetNode;

 while(n != -1) {
  if(paths[n] == currentNode)
   return n;
  n = paths[n];
 }

 return -1;
}*/

void autopilot(vector<Point> &mapPoints, vector<Point> &nodes, vector<array<int, 2>> &links, vector<int> &paths, vector<int> &dists,
               Point targetPoint, int &targetNode, int closestRobot, Point &robotPoint, uint16_t &robotTheta, bool &running) {

 static int state = GOTOPOINT;
 static Point oldTargetPoint = robotPoint;
 static int currentNode = closestRobot;
 static int8_t vx = 0;
 static int8_t vy = 0;
 static int8_t vz = 0;

 if(remoteFrame.vx || remoteFrame.vy || remoteFrame.vz)
  running = false;

 if(!running) {
  telemetryFrame.vx = remoteFrame.vx;
  telemetryFrame.vy = remoteFrame.vy;
  telemetryFrame.vz = remoteFrame.vz;
  oldTargetPoint = robotPoint;
  return;
 }

 if(targetPoint != oldTargetPoint) {
  if(closestRobot != -1 && sqDist(robotPoint, nodes[closestRobot]) < sqDist(robotPoint, targetPoint)) {
   currentNode = paths[closestRobot];
   state = GOTONODE;
  } else
   state = GOTOPOINT;
  oldTargetPoint = targetPoint;
 }

 if(state != GOTOWAITING && closestRobot == -1)
  state = GOTOPOINT;
 else if(currentNode == -1)
  state = GOTOWAITING;

 switch(state) {
  case GOTONODE:
   if(obstacle(mapPoints, robotPoint, nodes[currentNode],
               int(sqrt(sqDist(robotPoint, nodes[currentNode]))) + OBSTACLEROBOTLENGTH)) {
    //delLinkAndNodes(nodes, links, closestRobot, currentNode);
    delNodeAndLinks(nodes, links, currentNode);
    if(!nodes.empty()) {
     targetNode = closestPoint(nodes, targetPoint);
     computePaths(nodes, links, targetNode, paths, dists);
     currentNode = closestPoint(nodes, robotPoint);
    }
   } else if(gotoPoint(nodes[currentNode], vy, vz, robotPoint, robotTheta)) {
    if(currentNode == targetNode)
     state = GOTOPOINT;
    else
     currentNode = paths[currentNode];
   }
   break;

  case GOTOPOINT:
   {
    int dist = int(sqrt(sqDist(robotPoint, targetPoint))) + OBSTACLEROBOTLENGTH;
    if(dist > DISTFROMOBSTACLE)
     dist = DISTFROMOBSTACLE;
    if(obstacle(mapPoints, robotPoint, targetPoint, dist) ||
       gotoPoint(targetPoint, vy, vz, robotPoint, robotTheta))
     state = GOTOWAITING;
    else
     break;
   }

  case GOTOWAITING:
   vx = 0;
   vy = 0;
   vz = 0;
   break;
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

   if(testPointLine(map[i].a, map[j], MAPLINESMINLEN, MAPLINESMINLEN) &&
      sqDist(map[i].a, intersectPoint) < MAPLINESMINLEN * MAPLINESMINLEN) {
    map[i].a = intersectPoint;
    sort = true;
   }

   if(testPointLine(map[i].b, map[j], MAPLINESMINLEN, MAPLINESMINLEN) &&
      sqDist(map[i].b, intersectPoint) < MAPLINESMINLEN * MAPLINESMINLEN) {
    map[i].b = intersectPoint;
    sort = true;
   }

   if(sqDist(map[i]) < MAPLINESMINLEN * MAPLINESMINLEN) {
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
 vector<Point> mapPoints;
 vector<Line> mapLines;
 vector<Line> map;
 vector<Point> nodes;
 vector<array<int, 2>> links;
 vector<int> paths;
 vector<int> dists;

 Point robotPoint = Point(0, 0);
 uint16_t robotTheta = 0;
 bool mappingEnabled = true;
 bool graphingEnabled = false;
 Point targetPoint = Point(0, 0);
 int targetNode = 0;
 int closestRobot = -1;
 bool running = false;
 int select = SELECTFIXEDFULL;
 int mapDiv = MAPDIV;
 int confidences[AXES] = {0};
 fprintf(stderr, "Reading map file\n");
 readMapFile(map, nodes, links, robotPoint, robotTheta, mappingEnabled, graphingEnabled, select, mapDiv);
 Point oldRobotPoint = robotPoint;
 uint16_t oldRobotTheta = robotTheta;
 robotThetaCorrector = robotTheta;
 if(!nodes.empty())
  computePaths(nodes, links, targetNode, paths, dists);

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
    localization(robotLinesAxes, map, confidences, robotPoint, robotTheta);

    mapPoints.clear();
    robotToMap(robotPoints, mapPoints, robotPoint, robotTheta);

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

    if(graphingEnabled)
     graphing(polarPoints, mapPoints, nodes, links, paths, dists, targetPoint, targetNode, robotPoint, robotTheta);
   }

   if(!nodes.empty())
    closestRobot = closestPoint(nodes, robotPoint);
  }

  ui(image, robotPoints, robotLinesAxes, mapLines, map, mapPoints,
     nodes, links, paths, dists, targetPoint, targetNode, closestRobot,
     robotPoint, oldRobotPoint, robotTheta, oldRobotTheta,
     mappingEnabled, graphingEnabled, running, select, mapDiv, confidences, time);

  autopilot(mapPoints, nodes, links, paths, dists,
            targetPoint, targetNode, closestRobot, robotPoint, robotTheta, running);

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
 writeMapFile(map, nodes, links, robotPoint, robotTheta, mappingEnabled, graphingEnabled, select, mapDiv);

 fprintf(stderr, "Stopping\n");
 return 0;
}
