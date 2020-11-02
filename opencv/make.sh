#!/bin/bash

set -e
set -u

trace() {
 echo "$(date "+%d/%m/%Y %H:%M:%S") $1"
}

trace "Starting C++ compilation"

for d in *
do
 main="$d/main.cpp"

 if [[ ! -d "$d" || ! -f "$main" ]]
 then
  continue
 fi

 bin="$d/bin"
 build=no

 if [ -f "$bin" ]
 then
  for f in common.hpp common.cpp frame.hpp frame.cpp "$d"/*
  do
   if [ "$f" == "$bin" ]
   then
    continue
   fi

   if [ "$f" -nt "$bin" ]
   then
    trace "$f is newer than $bin"
    build=yes
   else
    trace "$f is older than $bin"
   fi
  done
 else
  trace "$bin is not found"
  build=yes
 fi

 if [ $build == no ]
 then
  continue
 fi

 all="$d"/*.cpp
 trace "Compiling $all to $bin"
 g++ common.cpp frame.cpp $all -o "$bin" \
 -O2 \
 -lopencv_core \
 -lopencv_imgcodecs \
 -lopencv_imgproc \
 -lopencv_videoio \
 -lwiringPi \
 -lpthread \
 -lRTIMULib || true
done

trace "Ending C++ compilation"
