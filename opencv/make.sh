#!/bin/bash

set -e
set -u

trace() {
 echo "$(date "+%d/%m/%Y %H:%M:%S") $1"
}

for d in *
do
 main="$d/main.cpp"

 if [[ ! -d "$d" || ! -f "$main" ]]
 then
  continue
 fi

 bin="$d/bin"
 build="no"

 if [ -f "$bin" ]
 then
  for f in "../frame.hpp" "$d/"*
  do
   if [ "$f" == "$bin" ]
   then
    continue
   fi

   if [ "$f" -nt "$bin" ]
   then
    trace "$f is newer than $bin"
    build="yes"
   else
    trace "$f is older than $bin"
   fi
  done
 else
  trace "$bin is not found"
  build="yes"
 fi

 if [[ "$build" == "no" || ! -f "$main" ]]
 then
  continue
 fi

 trace "Compiling $main to $bin"
 g++ "$main" -o "$bin" \
 -O2 \
 -lopencv_core \
 -lopencv_imgcodecs \
 -lopencv_imgproc \
 -lopencv_videoio \
 -lwiringPi \
 -lpthread \
 -lRTIMULib || true
done
