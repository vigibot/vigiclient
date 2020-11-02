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
 RTIMU *imu = RTIMU::createIMU(settings);
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

void extractLines(vector<Point> &pointsIn, vector<vector<Point>> &linesOut) {
 vector<Point> pointsDp;
 vector<Point> pointsNoDp;
 Point oldPoint;

 approxPolyDP(pointsIn, pointsDp, EPSILON, true);

 for(int i = 0; i < pointsIn.size() * 2; i++) {
  int ii = i % pointsIn.size();

  bool dp = false;
  for(int j = 0; j < pointsDp.size(); j++) {
   if(pointsIn[ii] == pointsDp[j]) {
    dp = true;
    break;
   }
  }

  Point diff = pointsIn[ii] - oldPoint;
  oldPoint = pointsIn[ii];
  int sqDist = diff.x * diff.x + diff.y * diff.y;

  if(dp || sqDist > SQDISTMAX) {
   if(pointsNoDp.size() >= NBPOINTSMIN && i > pointsNoDp.size() + 1) {
    linesOut.push_back(pointsNoDp);
    if(i > ii)
     break;
   }
   pointsNoDp.clear();

  } else
   pointsNoDp.push_back(pointsIn[ii]);
 }
}

void lidarToRobot(vector<PointPolar> &pointsIn, vector<Point> &pointsOut) {
 for(int i = 0; i < pointsIn.size(); i++) {
  int x = LIDARX + pointsIn[i].distance * sin16(pointsIn[i].theta) / ONE16;
  int y = LIDARY + pointsIn[i].distance * cos16(pointsIn[i].theta) / ONE16;

  if(x > ROBOTXMAX || x < ROBOTXMIN ||
     y > ROBOTYMAX || y < ROBOTYMIN)
   pointsOut.push_back(Point(x, y));
 }
}

void robotToMap(vector<Point> &pointsIn, vector<Point> &pointsOut, Point pointOdometry, uint16_t theta) {
 for(int i = 0; i < pointsIn.size(); i++)
  pointsOut.push_back(Point(pointOdometry.x + (pointsIn[i].x * cos16(theta) - pointsIn[i].y * sin16(theta)) / ONE16,
                            pointOdometry.y + (pointsIn[i].x * sin16(theta) + pointsIn[i].y * cos16(theta)) / ONE16));
}

void drawPoints(Mat &image, vector<Point> &pointsIn, int mapDiv, bool beams) {
 Point pointCenter = Point(width / 2, height / 2);

 for(int i = 0; i < pointsIn.size(); i++) {
  Point point = pointsIn[i];
  point.x /= mapDiv;
  point.y /= -mapDiv;
  if(beams)
   line(image, pointCenter, pointCenter + point, Scalar::all(i ? 64 : 255), 1, LINE_AA);
  circle(image, pointCenter + point, i ? 1 : 3, Scalar::all(i ? 128 : 255), 1, LINE_AA);
 }
}

void drawLines(Mat &image, vector<vector<Point>> &linesIn, int mapDiv) {
 Point pointCenter = Point(width / 2, height / 2);

 for(int i = 0; i < linesIn.size(); i++) {
  Point point1 = (linesIn[i][0] + linesIn[i][1]) / 2;
  Point point2 = (linesIn[i][linesIn[i].size() - 1] + linesIn[i][linesIn[i].size() - 2]) / 2;
  point1.x /= mapDiv;
  point2.x /= mapDiv;
  point1.y /= -mapDiv;
  point2.y /= -mapDiv;
  point1 += pointCenter;
  point2 += pointCenter;
  line(image, point1, point2, Scalar::all(255), 1, LINE_AA);
 }
}

void ui(Mat &image, vector<Point> &pointsRobot,
                    vector<vector<Point>> &linesRobot,
                    vector<Point> &pointsMap,
                    vector<vector<Point>> &linesMap) {
 bool buttonLess = remoteFrame.switchs & 0b00010000;
 bool buttonMore = remoteFrame.switchs & 0b00100000;
 bool buttonOk = remoteFrame.switchs & 0b10000000;
 static bool oldButtonLess = false;
 static bool oldButtonMore = false;
 static bool oldButtonOk = false;
 static bool tune = false;
 static int select = SELECTROBOT;
 static int mapDiv = MAPDIVMIN;

 if(!tune) {
  if(!buttonMore && oldButtonMore) {
   if(select < SELECTMAP)
    select++;
   else
    select = SELECTNONE;
  } else if(!buttonLess && oldButtonLess) {
   if(select > SELECTNONE)
    select--;
   else
    select = SELECTMAP;
  }
 }

 if(select == SELECTROBOTBEAMS) {
  if(!buttonOk && oldButtonOk)
   tune = !tune;

  if(tune) {
   if(buttonLess && mapDiv < MAPDIVMAX)
    mapDiv++;
   else if(buttonMore && mapDiv > MAPDIVMIN)
    mapDiv--;
  }
 }
 oldButtonLess = buttonLess;
 oldButtonMore = buttonMore;
 oldButtonOk = buttonOk;

 if(select == SELECTNONE)
  return;

 if(select == SELECTMAP) {
  drawPoints(image, pointsMap, mapDiv, false);
  drawLines(image, linesMap, mapDiv);
 } else {
  drawPoints(image, pointsRobot, mapDiv, select == SELECTROBOTBEAMS);
  drawLines(image, linesRobot, mapDiv);
 }

 if(select == SELECTROBOTBEAMS) {
  char text[80];
  sprintf(text, "%d mm per pixel", mapDiv);
  putText(image, text, Point(5, 15), FONT_HERSHEY_PLAIN, 1.0, Scalar::all(0), 1 + tune);
  putText(image, text, Point(6, 16), FONT_HERSHEY_PLAIN, 1.0, Scalar::all(255), 1 + tune);
 } else {
  char text[80];
  sprintf(text, "%d points %d lines", pointsRobot.size(), linesRobot.size());
  putText(image, text, Point(5, 15), FONT_HERSHEY_PLAIN, 1.0, Scalar::all(0), 1 + tune);
  putText(image, text, Point(6, 16), FONT_HERSHEY_PLAIN, 1.0, Scalar::all(255), 1 + tune);
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

 thread imuThr(imuThread);

 startLidar(ld);

 Mat image;
 int size = width * height * 3;

 telemetryFrame.header[0] = '$';
 telemetryFrame.header[1] = 'R';
 telemetryFrame.header[2] = ' ';
 telemetryFrame.header[3] = ' ';

 Point pointOdometry;
 uint16_t theta;
 vector<PointPolar> pointsPolar;
 vector<Point> pointsRobot;
 vector<vector<Point>> linesRobot;
 vector<Point> pointsMap;
 vector<vector<Point>> linesMap;

 VideoCapture capture;
 capture.open(0);
 capture.set(CAP_PROP_FRAME_WIDTH, width);
 capture.set(CAP_PROP_FRAME_HEIGHT, height);
 capture.set(CAP_PROP_FPS, fps);
 while(run) {
  capture.read(image);
  //image = Mat::zeros(Size(width, height), CV_8UC3);

  bool updated = readModem(fd, remoteFrame);

  if(updated) {
   pointOdometry.x = 0;
   pointOdometry.y = 0;
   theta = int(imuData.fusionPose.z() * double(PI16) / M_PI) * DIRZ;
  }

  if(readLidar(ld, pointsPolar)) {
   pointsRobot.clear();
   linesRobot.clear();
   pointsMap.clear();
   linesMap.clear();

   lidarToRobot(pointsPolar, pointsRobot);
   extractLines(pointsRobot, linesRobot);

   robotToMap(pointsRobot, pointsMap, pointOdometry, theta);
   extractLines(pointsMap, linesMap);
  }

  ui(image, pointsRobot, linesRobot, pointsMap, linesMap);

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
