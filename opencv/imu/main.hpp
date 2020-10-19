#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <wiringSerial.h>
#include <thread>
#include <RTIMULib.h>

#define WIDTH 640
#define HEIGHT 480
#define FPS 30

using namespace std;
using namespace cv;

int width;
int height;
int fps;

volatile bool run = true;

RTIMU_DATA imuData;
