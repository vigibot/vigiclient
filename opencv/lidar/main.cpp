#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <wiringSerial.h>
#include <thread>
#include <RTIMULib.h>
#include <chrono>
#include "../common.hpp"
#include "../frame.hpp"
#include "main.hpp"
#include "sin16.hpp"

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

bool readRpLidar(int ld, vector<PointPolar> &pointsOut) {
 static uint8_t init = 0;
 uint8_t current;
 static uint8_t n = 0;
 static uint8_t checksum;
 static uint8_t sum = 0;
 static uint16_t startAngleQ6;
 bool start;
 static uint16_t diffAngleQ6;
 static uint16_t oldStartAngleQ6 = 0;
 int32_t diffAngleTotalQ6;
 int32_t angleBrutQ6;
 static int32_t oldAngleBrutQ6;
 int32_t angle;
 static uint8_t deltaAnglesQ3[NBMESURESCABINES];
 static uint16_t distances[NBMESURESCABINES];
 static uint8_t m = 0;
 static uint8_t s = 0;
 static uint16_t j = 0;
 static vector<PointPolar> points;
 bool done = false;

 while(serialDataAvail(ld)) {
  current = serialGetchar(ld);

  switch(n) {

   case 0:                                                                             // Début de réception de l'en-tête
    if(current >> 4 == 0xA) {
     checksum = current & 0xF;
     n = 1;
    }
    break;

   case 1:
    if(current >> 4 == 0x5) {
     checksum |= current << 4;
     n = 2;
    } else
     n = 0;
    break;

   case 2:
    sum ^= current;
    startAngleQ6 = current;
    n = 3;
    break;

   case 3:
    sum ^= current;
    startAngleQ6 |= (current & 0x7F) << 8;
    start = current >> 7;                                                              // Fin de réception de l'en-tête
    if(start)
     fprintf(stderr, "Start\n");

    if(init < NBITERATIONSSYNCHRO) {                                                   // Ne pas calculer pendant la synchronisation ou sans les cabines
     init++;
     n = 4;
     break;
    }

    diffAngleQ6 = startAngleQ6 - oldStartAngleQ6;                                      // Calculer l'angle entre deux mesures de référence
    if(oldStartAngleQ6 > startAngleQ6)
     diffAngleQ6 += UNTOURQ6;

    diffAngleTotalQ6 = 0;
    for(uint8_t i = 0; i < NBMESURESCABINES; i++) {

     angleBrutQ6 = (oldStartAngleQ6 + diffAngleTotalQ6 / NBMESURESCABINES) % UNTOURQ6; // Calculer l'angle non compensé
     diffAngleTotalQ6 += diffAngleQ6;

     if(oldAngleBrutQ6 > angleBrutQ6) {                                                // Détection du passage par zéro de l'angle non compensé
      pointsOut = points;
      points.clear();
      done = true;
     }
     oldAngleBrutQ6 = angleBrutQ6;

     if(distances[i]) {                                                                // Si la lecture est valide et si il reste de la place dans les tableaux
      angle = angleBrutQ6 - (deltaAnglesQ3[i] << 3);                                   // Calculer l'angle compensé
      angle = angle * 65536 / UNTOURQ6;                                                // Remise à l'échelle de l'angle
      points.push_back({distances[i], uint16_t(angle)});
     }

    }
    oldStartAngleQ6 = startAngleQ6;

    n = 4;
    break;

   default:                                                                            // Début de réception des cabines
    sum ^= current;
    switch(m) {
     case 0:
      deltaAnglesQ3[s] = (current & 0b11) << 4;
      distances[s] = current >> 2;
      m = 1;
      break;
     case 1:
      distances[s] |= current << 6;
      m = 2;
      break;
     case 2:
      deltaAnglesQ3[s + 1] = (current & 0b11) << 4;
      distances[s + 1] = current >> 2;
      m = 3;
      break;
     case 3:
      distances[s + 1] |= current << 6;
      m = 4;
      break;
     case 4:
      deltaAnglesQ3[s] |= current & 0b1111;
      deltaAnglesQ3[s + 1] |= current >> 4;
      m = 0;
      s += 2;
      if(s == NBMESURESCABINES) {
       if(sum != checksum) {
        if(init == NBITERATIONSSYNCHRO)
         fprintf(stderr, "Checksum\n");
        init = NBITERATIONSSYNCHRO - 1;                                                // Ne pas faire les calculs pour ces cabines
       }
       n = 0;
       s = 0;
       sum = 0;
      }
      break;
    }
    break;

  }
 }

 return done;
}

void extractLines(vector<Point> &pointsIn, vector<vector<Point>> &lines) {
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
    lines.push_back(pointsNoDp);
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
  int x = LIDARX + pointsIn[i].distance * sin16(pointsIn[i].theta) / UN16;
  int y = LIDARY + pointsIn[i].distance * cos16(pointsIn[i].theta) / UN16;

  if(x > ROBOTXMAX || x < ROBOTXMIN ||
     y > ROBOTYMAX || y < ROBOTYMIN)
   pointsOut.push_back(Point(x, y));
 }
}

void robotToMap(vector<Point> &pointsIn, vector<Point> &pointsOut, Point pointOdometry, uint16_t theta) {
 for(int i = 0; i < pointsIn.size(); i++)
  pointsOut.push_back(Point(pointOdometry.x + (pointsIn[i].x * cos16(theta) - pointsIn[i].y * sin16(theta)) / UN16,
                            pointOdometry.y + (pointsIn[i].x * sin16(theta) + pointsIn[i].y * cos16(theta)) / UN16));
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

 int ld = serialOpen("/dev/serial0", 115200);
 if(ld == -1) {
  fprintf(stderr, "Error opening serial port\n");
  return 1;
 }

 serialPutchar(ld, 0xa5);
 serialPutchar(ld, 0x82);
 serialPutchar(ld, 0x05);
 serialPutchar(ld, 0x00);
 serialPutchar(ld, 0x00);
 serialPutchar(ld, 0x00);
 serialPutchar(ld, 0x00);
 serialPutchar(ld, 0x00);
 serialPutchar(ld, 0x22);

 thread imuThr(imuThread);

 Mat image;
 int size = width * height * 3;

 telemetryFrame.header[0] = '$';
 telemetryFrame.header[1] = 'R';
 telemetryFrame.header[2] = ' ';
 telemetryFrame.header[3] = ' ';

 Point pointOdometry = Point(0, 0);
 uint16_t theta;

 Point pointCenter = Point(width / 2, height / 2);
 vector<PointPolar> pointsPolar;
 vector<Point> pointsRobot;
 vector<Point> pointsMap;
 vector<vector<Point>> lines;

 VideoCapture capture;
 capture.open(0);
 capture.set(CAP_PROP_FRAME_WIDTH, width);
 capture.set(CAP_PROP_FRAME_HEIGHT, height);
 capture.set(CAP_PROP_FPS, fps);
 capture.set(CAP_PROP_FORMAT, CV_8UC3);
 while(run) {
  capture.read(image);

  bool updated = readModem(fd, remoteFrame);

  if(readRpLidar(ld, pointsPolar)) {
   pointsRobot.clear();
   pointsMap.clear();
   lines.clear();

   lidarToRobot(pointsPolar, pointsRobot);
   robotToMap(pointsRobot, pointsMap, pointOdometry, theta);
   extractLines(pointsMap, lines);
  }

  char text[80];
  sprintf(text, "%d %d", pointsMap.size(), lines.size());
  putText(image, text, Point(5, 15), FONT_HERSHEY_PLAIN, 1.0, Scalar::all(0), 1);
  putText(image, text, Point(6, 16), FONT_HERSHEY_PLAIN, 1.0, Scalar::all(255), 1);

  for(int i = 0; i < pointsMap.size(); i++)
   line(image, pointCenter, pointCenter + pointsMap[i] / 10, Scalar::all(64), 1, LINE_AA);

  for(int i = 0; i < lines.size(); i++) {
   Point point1 = pointCenter + (lines[i][0] + lines[i][1]) / 20;
   Point point2 = pointCenter + (lines[i][lines[i].size() - 1] + lines[i][lines[i].size() - 2]) / 20;
   line(image, point1, point2, Scalar::all(255), 1, LINE_AA);
  }

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

 serialPutchar(ld, 0xa5);
 serialPutchar(ld, 0x25);
 return 0;
}
