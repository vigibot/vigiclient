#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <wiringSerial.h>
#include "../frame.hpp"
#include "main.hpp"

using namespace std;
using namespace cv;

void signal_callback_handler(int signum) {
 fprintf(stderr, "Caught signal %d\n", signum);
 run = false;
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

void autopilot(Mat &image, bool enabled) {
 int id = -1;
 static Feature oldFeature;
 static int circleRadiusInit = -1;
 static int oldPx = 0;
 static int oldPy = 0;
 static int autox = 0;
 static int autoy = 0;
 static int autovy = 0;
 static int autovz = 0;
 static int timeout = TIMEOUT;

 if(!enabled) {
  autox = mapInteger(remoteFrame.xy[0][0], -32767, 32767, -18000, 18000);
  autoy = mapInteger(remoteFrame.xy[0][1], -32767, 32767, -18000, 18000);
  telemetryFrame.xy[0][0] = remoteFrame.xy[0][0];
  telemetryFrame.xy[0][1] = remoteFrame.xy[0][1];
  telemetryFrame.vy = remoteFrame.vy;
  telemetryFrame.vz = remoteFrame.vz;

  int minSqDist = INT_MAX;
  for(int i = 0; i < features.size(); i++) {
   if(features[i].filtered)
    continue;
   Point diff = features[i].circleCenter - Point(width / 2, height / 2);
   int sqDist = diff.x * diff.x + diff.y * diff.y;
   if(sqDist < minSqDist) {
    minSqDist = sqDist;
    id = i;
   }
  }

  if(id == -1)
   return;

  vector<vector<Point>> polygon(1, features[id].polygon);
  drawContours(image, polygon, -1, hueToBgr[colorToHue[features[id].color]], 2, LINE_AA);

  char text[80];
  sprintf(text, "Ready %s %d", COLORS[features[id].color], features[id].circleRadius);
  putText(image, text, Point(5, 15), FONT_HERSHEY_PLAIN, 1.0, Scalar::all(0), 1);
  putText(image, text, Point(6, 16), FONT_HERSHEY_PLAIN, 1.0, Scalar::all(255), 1);

  oldFeature = features[id];
  circleRadiusInit = features[id].circleRadius;
  return;
 } else if(circleRadiusInit == -1)
  return;

 id = -1;
 int minSqDist = INT_MAX;
 for(int i = 0; i < features.size(); i++) {
  if(features[i].color != oldFeature.color)
   continue;
  Point diff = features[i].circleCenter - oldFeature.circleCenter;
  int sqDist = diff.x * diff.x + diff.y * diff.y;
  if(sqDist < minSqDist) {
   minSqDist = sqDist;
   id = i;
  }
 }

 if(id != -1) {
  oldFeature = features[id];

  int px = features[id].circleCenter.x - width / 2;
  int py = features[id].circleCenter.y - height / 2;

  if(abs(px) <= NEUTRAL)
   px = 0;
  if(abs(py) <= NEUTRAL)
   py = 0;

  int dx = px - oldPx;
  int dy = py - oldPy;
  oldPx = px;
  oldPy = py;

#ifdef HEADPAN
  autox += px * KPX + dx * KDX;
  autox = constrain(autox, XMIN, XMAX);
#endif

#ifdef HEADTILT
  autoy -= py * KPY + dy * KDY;
  autoy = constrain(autoy, YMIN, YMAX);
#endif

  autovy = (circleRadiusInit - features[id].circleRadius) * KVY / circleRadiusInit;
  autovy = constrain(autovy, -127, 127);

#ifdef HEADPAN
  autovz = mapInteger(autox, -9000, 9000, -127, 127);
#else
  autovz = px / KPVZ + dx / KDVZ;
#endif
  autovz = constrain(-autovz, -127, 127);

  vector<vector<Point>> polygon(1, features[id].polygon);
  drawContours(image, polygon, -1, hueToBgr[colorToHue[features[id].color]], 2, LINE_AA);

  char text[80];
  sprintf(text, "Tracking %s %d", COLORS[features[id].color], features[id].circleRadius);
  putText(image, text, Point(5, 15), FONT_HERSHEY_PLAIN, 1.0, Scalar::all(0), 1);
  putText(image, text, Point(6, 16), FONT_HERSHEY_PLAIN, 1.0, Scalar::all(255), 1);

  timeout = TIMEOUT;
 } else {
  char text[80];
  sprintf(text, "Waiting %d %s %d", timeout, COLORS[oldFeature.color], circleRadiusInit);
  putText(image, text, Point(5, 15), FONT_HERSHEY_PLAIN, 1.0, Scalar::all(0), 1);
  putText(image, text, Point(6, 16), FONT_HERSHEY_PLAIN, 1.0, Scalar::all(255), 1);
 }

#ifdef HEADPAN
 telemetryFrame.xy[0][0] = mapInteger(autox, -18000, 18000, -32767, 32767);
#endif

#ifdef HEADTILT
 telemetryFrame.xy[0][1] = mapInteger(autoy, -18000, 18000, -32767, 32767);
#endif

 if(timeout > 0) {
  timeout--;

  if(remoteFrame.vy == 0 && abs(autovy) >= VMIN)
   telemetryFrame.vy = autovy;
  else
   telemetryFrame.vy = remoteFrame.vy;

  if(remoteFrame.vz == 0 && abs(autovz) >= VMIN)
   telemetryFrame.vz = autovz;
  else
   telemetryFrame.vz = remoteFrame.vz;

 } else {
  telemetryFrame.vy = remoteFrame.vy;
  telemetryFrame.vz = remoteFrame.vz;
 }
}

void colorsInit() {
 int c = 0;
 for(int i = 0; i < 180; i++) {
  hueToColor[i] = c % NBCOLORS;
  if(i == hues[c])
   c++;
 }

 colorToHue[0] = ((hues[NBCOLORS - 1] + hues[0] + 180) / 2) % 180;
 for(uchar i = 1; i < NBCOLORS; i++)
  colorToHue[i] = (hues[i] + hues[i - 1]) / 2;
}

void bgrInit() {
 for(uchar i = 0; i < 180; i++) {
  Mat imageHsv = Mat(1, 1, CV_8UC3, Scalar(i, 255, 255));
  Mat imageBgr;
  cvtColor(imageHsv, imageBgr, COLOR_HSV2BGR);
  hueToBgr[i] = imageBgr.at<Vec3b>(0, 0);
 }
}

void colorsEngine(Mat &image, uchar &threshold) {
 Mat imageBgr;
 Mat imageHsv;
 Mat imageMasks[NBCOLORS];

 resize(image, imageBgr, Size(width / BINNING, height / BINNING), INTER_LINEAR);
 cvtColor(imageBgr, imageHsv, COLOR_BGR2HSV);

 for(int i = 0; i < NBCOLORS; i++)
  imageMasks[i] = Mat::zeros(imageBgr.size(), CV_8UC1);

 for(int i = 0; i < imageBgr.rows; i++) {
  uchar *inBgr = imageBgr.ptr<uchar>(i);
  uchar *inHsv = imageHsv.ptr<uchar>(i);
  uchar *outs[NBCOLORS];
  for(int j = 0; j < NBCOLORS; j++)
   outs[j] = imageMasks[j].ptr<uchar>(i);

  for(int j = 0; j < imageBgr.cols; j++) {
   uchar b = *inBgr++;
   uchar g = *inBgr++;
   uchar r = *inBgr++;
   uchar max = b;
   uchar min = b;
   uchar color;

   if(g > max)
    max = g;
   else if(g < min)
    min = g;
   if(r > max)
    max = r;
   else if(r < min)
    min = r;

   if(max - min > threshold) {
    color = hueToColor[*(inHsv + j * 3)];
    *(outs[color] + j) = 255;
   }
  }
 }

 features.clear();
 for(int i = 0; i < NBCOLORS; i++) {
  if(blacks[i])
   continue;

  vector<vector<Point>> contours;
  findContours(imageMasks[i], contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

  int n = 0;
  for(int j = 0; j < contours.size(); j++) {
   vector<Point> polygon;
   approxPolyDP(contours[j], polygon, EPSILON, true);

   for(int k = 0; k < polygon.size(); k++) {
    polygon[k].x *= BINNING;
    polygon[k].y *= BINNING;
   }

   float area = float(contourArea(polygon));
   if(polygon.size() >= MINVERTICES && area >= MINAREA) {

    Moments moment = moments(polygon);
    Point2f center = Point2f(moment.m10 / moment.m00, moment.m01 / moment.m00);

    Point circleCenter;
    int circleRadius;

    int xmin = polygon[0].x;
    int ymin = polygon[0].y;
    int xmax = polygon[0].x;
    int ymax = polygon[0].y;
    for(int k = 0; k < polygon.size(); k++) {
     int x = polygon[k].x;
     int y = polygon[k].y;
     if(x < xmin)
      xmin = x;
     else if(x > xmax)
      xmax = x;
     if(y < ymin)
      ymin = y;
     else if(y > ymax)
      ymax = y;
    }
    circleCenter.x = (xmin + xmax) / 2;
    circleCenter.y = (ymin + ymax) / 2;
    int xdiff = xmax - xmin;
    int ydiff = ymax - ymin;
    circleRadius = max(xdiff, ydiff) / 2;

    features.push_back({i, polygon, area, center, circleCenter, circleRadius, false});
   }
  }
 }

 sort(features.begin(), features.end(), [](const Feature &a, const Feature &b) {
  return a.area > b.area;
 });

 for(int i = 0; i < features.size(); i++) {
  for(int j = 0; j < features.size(); j++) {
   if(i != j &&
      features[i].area < features[j].area &&
      abs(features[i].color - features[j].color) < MINCOLORDIST &&
      pointPolygonTest(features[j].polygon, features[i].center, true) > MINPOINTPOLYGONDIST) {
    features[i].filtered = true;
    break;
   }
  }
 }

 //features.erase(remove_if(features.begin(), features.end(), [](Feature feature) {
  //return feature.filtered;
 //}), features.end());
}

bool ui(Mat &image, bool &updated, uchar &threshold) {
 static bool buttonLess = false;
 static bool oldButtonLess = false;
 static bool buttonMore = false;
 static bool oldButtonMore = false;
 static bool buttonOk = false;
 static bool oldButtonOk = false;
 static int buttonOkCount = 0;
 static bool tune = false;
 static uchar select = SELECTCAMERA;
 uchar selectedColor = 255;
 static bool enabled = false;
 static int oldRemoteFramex = 0;
 static int oldRemoteFramey = 0;

 if(updated) {
  buttonLess = remoteFrame.switchs & 0b00010000;
  buttonMore = remoteFrame.switchs & 0b00100000;
  buttonOk = remoteFrame.switchs & 0b10000000;
 }

 if(!tune) {
  if(!buttonMore && oldButtonMore) {
   if(select < SELECTCONFTHRESHOLD + NBCOLORS)
    select++;
   else
    select = 0;
  } else if(!buttonLess && oldButtonLess) {
   if(select > 0)
    select--;
   else
    select = SELECTCONFTHRESHOLD + NBCOLORS;
  }
 }

 if(select <= SELECTCAMERASHORTS) {
  if(!buttonOk && oldButtonOk)
   enabled = !enabled;

 } else if(select == SELECTCONFTHRESHOLD) {
  if(!buttonOk && oldButtonOk)
   tune = !tune;

  if(tune) {
   if(buttonLess && threshold < 255)
    threshold++;
   else if(buttonMore && threshold > 0)
    threshold--;
  }

 } else if(select >= SELECTCONFFIRSTCOLOR) {
  selectedColor = select - SELECTCONFFIRSTCOLOR;
  if(buttonOk) {
   buttonOkCount++;
   if(buttonOkCount == 15)
    blacks[selectedColor] = !blacks[selectedColor];
  } else if(!buttonOk && oldButtonOk) {
   if(buttonOkCount < 15)
    tune = !tune;
   buttonOkCount = 0;
  }

  if(tune) {
   uchar minHue;
   uchar maxHue;
   if(selectedColor == 0)
    minHue = 0;
   else
    minHue = hues[selectedColor - 1] + 1;
   if(selectedColor == NBCOLORS - 1)
    maxHue = 179;
   else
    maxHue = hues[selectedColor + 1] - 1;

   if(buttonLess && hues[selectedColor] > minHue) {
    hues[selectedColor]--;
    colorsInit();
   } else if(buttonMore && hues[selectedColor] < maxHue) {
    hues[selectedColor]++;
    colorsInit();
   }
  }
 }
 oldButtonLess = buttonLess;
 oldButtonMore = buttonMore;
 oldButtonOk = buttonOk;

 if(remoteFrame.xy[0][0] != oldRemoteFramex ||
    remoteFrame.xy[0][1] != oldRemoteFramey)
  enabled = false;
 oldRemoteFramex = remoteFrame.xy[0][0];
 oldRemoteFramey = remoteFrame.xy[0][1];

 if(select == SELECTCAMERASHORTS) {
  int stats[NBCOLORS] = {0};
  for(int i = 0; i < features.size(); i++) {
   if(features[i].filtered)
    continue;

   float s = 0.5 + features[i].area / 4000.0;

   char text[80];
   sprintf(text, "%s%d", SHORTS[features[i].color], stats[features[i].color]++);

   int baseline;
   Size textSize = getTextSize(text, FONT_HERSHEY_PLAIN, s, 1, &baseline);
   Point2f textCenter(-textSize.width / 2, textSize.height / 2);

   putText(image, text, features[i].center + textCenter, FONT_HERSHEY_PLAIN, s, Scalar::all(0), 1);
   putText(image, text, features[i].center + textCenter + Point2f(1.0, 1.0), FONT_HERSHEY_PLAIN, s, Scalar::all(255), 1);
  }

 } else if(select >= SELECTCONFTHRESHOLD) {
  image = Mat::zeros(image.size(), image.type());
  for(int i = 0; i < features.size(); i++) {
   vector<vector<Point>> polygon(1, features[i].polygon);
   drawContours(image, polygon, -1, hueToBgr[colorToHue[features[i].color]], FILLED, LINE_AA);
  }

  char text[80];
  if(select == SELECTCONFTHRESHOLD)
   sprintf(text, "Sensitivity %d", 255 - threshold);
  else
   sprintf(text, "%s %s %d", COLORS[selectedColor],
                             COLORS[(selectedColor + 1) % NBCOLORS],
                             hues[selectedColor]);

  putText(image, text, Point(5, height - 35), FONT_HERSHEY_PLAIN, 1.0, Scalar::all(0), 1 + tune);
  putText(image, text, Point(6, height - 34), FONT_HERSHEY_PLAIN, 1.0, Scalar::all(255), 1 + tune);

  uchar oldHue = 0;
  for(int i = 0; i < 180; i++) {
   int x = i * width / 180;
   if(!blacks[hueToColor[i]])
    rectangle(image, Rect(x, height - 20, width / 180 + 1, 10), hueToBgr[colorToHue[hueToColor[i]]], FILLED);
   rectangle(image, Rect(x, height - 10, width / 180 + 1, 10), hueToBgr[i], FILLED);
   if(hueToColor[i] != oldHue) {
    if(oldHue == selectedColor)
     circle(image, Point(x, height - 25), 3 + tune, Scalar::all(255), FILLED, LINE_AA);
    line(image, Point(x, height - 20), Point(x, height - 11), Scalar::all(0), 1);
   }
   if(!((i + 7) % 15))
    line(image, Point(x, height - 10), Point(x, height), Scalar::all(0), 1);
   oldHue = hueToColor[i];
  }
 }

 return enabled;
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

 Mat image;
 int size = width * height * 3;
 uchar threshold = THRESHOLD;

 telemetryFrame.header[0] = '$';
 telemetryFrame.header[1] = 'R';
 telemetryFrame.header[2] = ' ';
 telemetryFrame.header[3] = ' ';

 colorsInit();
 bgrInit();

 VideoCapture capture;
 capture.open(0);
 capture.set(CAP_PROP_FRAME_WIDTH, width);
 capture.set(CAP_PROP_FRAME_HEIGHT, height);
 capture.set(CAP_PROP_FPS, fps);
 capture.set(CAP_PROP_FORMAT, CV_8UC3);
 while(run) {
  capture.read(image);

  bool updated = readModem(fd, remoteFrame);

  colorsEngine(image, threshold);

  bool enabled = ui(image, updated, threshold);

  autopilot(image, enabled);

  if(updated) {
   for(int i = 1; i < NBCOMMANDS; i++) {
    telemetryFrame.xy[i][0] = remoteFrame.xy[i][0];
    telemetryFrame.xy[i][1] = remoteFrame.xy[i][1];
   }
   telemetryFrame.z = remoteFrame.z;
   telemetryFrame.vx = remoteFrame.vx;
   telemetryFrame.switchs = remoteFrame.switchs;

   writeModem(fd, telemetryFrame);
  }

  fwrite(image.data, size, 1, stdout);
 }

 return 0;
}
