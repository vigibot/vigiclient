#!/bin/bash

set -e
set -u

BASEDIR=/usr/local/src/vigiclient

while true
do
 wget https://www.vigibot.com/clientrobotpi.js -P $BASEDIR -N
 wget https://www.vigibot.com/trame.js -P $BASEDIR -N
 node $BASEDIR/clientrobotpi.js
 sleep 1
done
