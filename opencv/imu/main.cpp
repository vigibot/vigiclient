#include "../../frame.hpp"
#include "main.hpp"

void signal_callback_handler(int signum) {
 fprintf(stderr, "Caught signal %d\n", signum);
 run = false;
}

bool readModem(int fd) {
 uint8_t octet;
 static uint8_t pos = 0;
 uint8_t p;

 while(serialDataAvail(fd)) {
  octet = serialGetchar(fd);

  switch(pos) {

   case 0:
    if(octet == '$')
     pos = 1;
    break;

   case 1:
    if(octet == 'S')
     pos = 2;
    else
     pos = 0;
    break;

   case 2:
   case 3:
    if(octet == ' ')
     pos++;
    else
     pos = 0;
    break;

   default:
    remoteFrame.bytes[pos++] = octet;
    if(pos == REMOTEFRAMESIZE) {
     pos = 0;
     serialFlush(fd);
     return true;
    }

  }
 }

 return false;
}

void writeModem(int fd) {
 for(int i = 0; i < TELEMETRYFRAMESIZE; i++)
  serialPutchar(fd, telemetryFrame.bytes[i]);
}

int mapInteger(int n, int inMin, int inMax, int outMin, int outMax) {
 return (n - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

float mapFloat(float n, float inMin, float inMax, float outMin, float outMax) {
 return (n - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

int constrain(int n, int min, int max) {
 if(n < min)
  n = min;
 else if(n > max)
  n = max;

 return n;
}

void ui(Mat &image, bool &updated) {
 static bool buttonLess = false;
 static bool oldButtonLess = false;
 static bool buttonMore = false;
 static bool oldButtonMore = false;
 static bool buttonOk = false;
 static bool oldButtonOk = false;

 if(updated) {
  buttonLess = remoteFrame.switchs & 0b00010000;
  buttonMore = remoteFrame.switchs & 0b00100000;
  buttonOk = remoteFrame.switchs & 0b10000000;
 }

 //

 oldButtonLess = buttonLess;
 oldButtonMore = buttonMore;
 oldButtonOk = buttonOk;
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

 thread imuThr(imuThread);

 Mat image;
 int size = width * height * 3;

 telemetryFrame.header[0] = '$';
 telemetryFrame.header[1] = 'R';
 telemetryFrame.header[2] = ' ';
 telemetryFrame.header[3] = ' ';

 VideoCapture capture;
 capture.open(0);
 while(run) {
  capture.read(image);

  bool updated = readModem(fd);

  //ui(image, updated);

  double x = -imuData.fusionPose.x();
  double y = -imuData.fusionPose.y();
  double z = imuData.fusionPose.z();

  int xCenter1 = MARGIN + LINELEN;
  int yCenter1 = height - MARGIN - LINELEN;

  int xCenter2 = MARGIN * 2 + LINELEN * 3;
  int yCenter2 = height - MARGIN - LINELEN;

  int x1 = xCenter1 + sin(x - M_PI / 2) * LINELEN;
  int y1 = yCenter1 + cos(x - M_PI / 2) * LINELEN;
  int x2 = xCenter1 + sin(x + M_PI / 2) * LINELEN;
  int y2 = yCenter1 + cos(x + M_PI / 2) * LINELEN;

  int x3 = xCenter2 + sin(y - M_PI / 2) * LINELEN;
  int y3 = yCenter2 + cos(y - M_PI / 2) * LINELEN;
  int x4 = xCenter2 + sin(y + M_PI / 2) * LINELEN;
  int y4 = yCenter2 + cos(y + M_PI / 2) * LINELEN;

  circle(image, Point(xCenter1, yCenter1), LINELEN, Scalar(0, 255, 0), 1, LINE_AA);
  line(image, Point(x1, y1), Point(x2, y2), Scalar::all(255), 1, LINE_AA);

  circle(image, Point(xCenter2, yCenter2), LINELEN, Scalar(0, 0, 255), 1, LINE_AA);
  line(image, Point(x3, y3), Point(x4, y4), Scalar::all(255), 1, LINE_AA);

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

   writeModem(fd);
  }

  fwrite(image.data, size, 1, stdout);
 }

 return 0;
}
