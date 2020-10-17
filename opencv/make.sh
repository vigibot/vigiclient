#!/bin/bash

name=colors

g++ "$name/main.cpp" -o "$name/bin" \
-O2 \
-lopencv_core \
-lopencv_imgcodecs \
-lopencv_imgproc \
-lopencv_videoio \
-lwiringPi
