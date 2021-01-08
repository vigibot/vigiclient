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
 RTIMU *imu = RTIMU::createIMU(settings);
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

void watch(Mat &image, double angle, Point center, int diam, Scalar color1, Scalar color2) {
 double angleDeg = angle * 180.0 / M_PI;
 ellipse(image, center, Point(diam, diam), angleDeg, 0.0, 180.0, color1, FILLED, LINE_AA);
 ellipse(image, center, Point(diam, diam), angleDeg, 0.0, -180.0, color2, FILLED, LINE_AA);
}

void autopilot(Mat &image) {
 bool buttonLess = remoteFrame.switchs & 0b00010000;
 bool buttonMore = remoteFrame.switchs & 0b00100000;
 static bool oldButtonLess = false;
 static bool oldButtonMore = false;
 static int select = -1;
#ifdef BODYPAN
 int x = mapInteger(remoteFrame.xy[0][0], -32767, 32767, -180, 180);
 static int oldx = 0;
#endif
 static int timeout = TIMEOUT;
 static int setPoint = 0;
 static int integ = 0;
 static int oldProp = 0;
 static int oldVz = 0;
 double imux = imuData.fusionPose.x() * DIRX + OFFSETX;
 double imuy = imuData.fusionPose.y() * DIRY + OFFSETY;
 double imuTheta = imuData.fusionPose.z() * DIRZ + OFFSETZ;
 int imuThetaDeg = int(imuTheta * 180.0 / M_PI);

 int x1 = MARGIN + DIAM1;
 int x2 = x1 + MARGIN + DIAM1 * 2;
 int x3 = width - MARGIN - DIAM1;
 int y1 = MARGIN + DIAM1;

 watch(image, imux, Point(x1, y1), DIAM1, Scalar(0, 0, 200), Scalar::all(200));
 watch(image, imuy, Point(x2, y1), DIAM1, Scalar(0, 200, 0), Scalar::all(200));

 watch(image, imux * COEF2, Point(x1, y1), DIAM2, Scalar(0, 0, 255), Scalar::all(255));
 watch(image, imuy * COEF2, Point(x2, y1), DIAM2, Scalar(0, 255, 0), Scalar::all(255));

 if(!buttonMore && oldButtonMore) {
  if(select < 2)
   select++;
  else
   select = -1;
 } else if(!buttonLess && oldButtonLess) {
  if(select > -1)
   select--;
  else
   select = 2;
 }
 oldButtonLess = buttonLess;
 oldButtonMore = buttonMore;

 if(select == -1) {
#ifdef BODYPAN
  setPoint = (x + imuThetaDeg) * DIVVZ;
#else
  setPoint = imuThetaDeg * DIVVZ;
#endif
  integ = 0;
  oldProp = 0;
  telemetryFrame.vz = remoteFrame.vz;
  return;
 }

#ifdef BODYPAN
 if(x != oldx ||
    remoteFrame.vx != 0 ||
    remoteFrame.vy != 0 ||
    remoteFrame.vz != 0) {
  timeout = TIMEOUT;
 }
 oldx = x;
#else
 if(remoteFrame.vx != 0 ||
    remoteFrame.vy != 0 ||
    remoteFrame.vz != 0) {
  timeout = TIMEOUT;
 }
#endif

 if(timeout > 0) {
  timeout--;

  if(select == 0)
   setPoint += remoteFrame.vz;
  else {
   if(remoteFrame.vz >= TRIGGERVX && oldVz < TRIGGERVX)
    setPoint += select * 45 * DIVVZ;
   else if(remoteFrame.vz <= -TRIGGERVX && oldVz > -TRIGGERVX)
    setPoint -= select * 45 * DIVVZ;
  }
  oldVz = remoteFrame.vz;

  if(setPoint < -180 * DIVVZ)
   setPoint += 360 * DIVVZ;
  else if(setPoint >= 180 * DIVVZ)
   setPoint -= 360 * DIVVZ;

#ifdef BODYPAN
  int prop = -x + setPoint / DIVVZ - imuThetaDeg;
#else
  int prop = setPoint / DIVVZ - imuThetaDeg;
#endif

  if(prop < -180)
   prop += 360;
  else if(prop >= 180)
   prop -= 360;

  integ += prop;
  if(prop >= 0 && oldProp < 0 ||
     prop <= 0 && oldProp > 0)
   integ = 0;

  int deriv = prop - oldProp;
  oldProp = prop;

  int autovz = prop * KPVZ + integ / KIVZ + deriv * KDVZ;
  autovz = constrain(autovz, -127, 127);
  telemetryFrame.vz = autovz;

 } else {
  integ = 0;
  oldProp = 0;
  telemetryFrame.vz = 0;
 }

 watch(image, -imuTheta, Point(x3, y1), DIAM1, Scalar(200, 0, 0), Scalar::all(200));
#ifdef BODYPAN
 watch(image, -double(-x + setPoint / DIVVZ) / 180.0 * M_PI, Point(x3, y1), DIAM2, Scalar(255, 0, 0), Scalar::all(255));
#else
 watch(image, -double(setPoint / DIVVZ) / 180.0 * M_PI, Point(x3, y1), DIAM2, Scalar(255, 0, 0), Scalar::all(255));
#endif

 if(select)
  circle(image, Point(x3, y1), DIAM1 + select, Scalar::all(255), select, LINE_AA);
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
  fprintf(stderr, "Error opening serial port\n");
  return 1;
 }

 thread imuThr(imuThread);

 Mat image;
 int size = width * height * 3;

 telemetryFrame.header[0] = '$';
 telemetryFrame.header[1] = 'R';
 telemetryFrame.header[2] = ' ';
 telemetryFrame.header[3] = ' ';

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

  autopilot(image);

  if(updated) {
   for(int i = 0; i < NBCOMMANDS; i++) {
    telemetryFrame.xy[i][0] = remoteFrame.xy[i][0];
    telemetryFrame.xy[i][1] = remoteFrame.xy[i][1];
   }
   telemetryFrame.z = remoteFrame.z;
   telemetryFrame.vx = remoteFrame.vx;
   telemetryFrame.vy = remoteFrame.vy;
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

 fprintf(stderr, "Stopping\n");
 return 0;
}
