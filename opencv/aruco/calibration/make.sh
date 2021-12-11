#!/bin/bash

g++ create_board_charuco.cpp -o create_board_charuco -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_aruco -lopencv_imgproc -lopencv_videoio
g++ calibrate_camera_charuco.cpp -o calibrate_camera_charuco -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_aruco -lopencv_imgproc -lopencv_videoio
./create_board_charuco -d=0 -h=5 -w=8 --ml=200 --sl=350 charuco.png
