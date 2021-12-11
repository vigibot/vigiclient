#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/aruco.hpp>
#include <wiringSerial.h>
#include "../common.hpp"
#include "../frame.hpp"
#include "main.hpp"

using namespace std;
using namespace cv;
using namespace aruco;

void signal_callback_handler(int signum) {
 fprintf(stderr, "Caught signal %d\n", signum);
 run = false;
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

 Mat image;
 int size = width * height * 3;

 vector<vector<Point2f>> corners;
 vector<int> ids;
 vector<Vec3d> rvecs;
 vector<Vec3d> tvecs;

 Ptr<Dictionary> dictionary = getPredefinedDictionary(DICT_6X6_50);

 Mat cameraMatrix = (Mat1d(3, 3) << 3.1785284097755920e+02,
                                    0.0,
                                    3.1655754360070006e+02,
                                    0.0,
                                    3.1786886667307317e+02,
                                    2.1825110088267701e+02,
                                    0.0,
                                    0.0,
                                    1.0);

 Mat distCoeffs = (Mat1d(1, 5) << -3.1134857318275400e-01,
                                   1.0697767047253340e-01,
                                  -1.2872923128342298e-04,
                                  -7.6087959013120237e-05,
                                  -1.6961544076584793e-02);

 telemetryFrame.header[0] = '$';
 telemetryFrame.header[1] = 'R';
 telemetryFrame.header[2] = ' ';
 telemetryFrame.header[3] = ' ';

 fprintf(stderr, "Starting capture\n");
 VideoCapture capture;
 capture.open(0);

 if(capture.isOpened()) {
  fprintf(stderr, "Configuring capture\n");
  capture.set(CAP_PROP_FRAME_WIDTH, width);
  capture.set(CAP_PROP_FRAME_HEIGHT, height);
  capture.set(CAP_PROP_FPS, fps);
 } else {
  fprintf(stderr, "Error starting capture\n");
  return 1;
 }

 while(run) {
  capture.read(image);

  bool updated = readModem(fd, remoteFrame);

  detectMarkers(image, dictionary, corners, ids);
  if(ids.size() > 0) {
   estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);

   for(int i = 0; i < ids.size(); i++)
    drawAxis(image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.05);
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

 fprintf(stderr, "Stopping capture\n");
 capture.release();

 fprintf(stderr, "Stopping\n");
 return 0;
}
