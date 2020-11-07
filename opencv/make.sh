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
  for f in make.sh *.cpp *.hpp "$d"/*
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

 cur=*.cpp
 sub="$d"/*.cpp
 trace "Compiling $cur and $sub to $bin"
 g++ $cur $sub -o "$bin" \
 -O2 \
 -lopencv_core \
 -lopencv_imgcodecs \
 -lopencv_imgproc \
 -lopencv_videoio \
 -lwiringPi \
 -lpthread \
 -lRTIMULib && \
 trace "Success to compile $cur and $sub to $bin" || \
 trace "Failed to compile $cur and $sub to $bin"
done

trace "Ending C++ compilation"
