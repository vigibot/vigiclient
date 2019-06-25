#!/bin/bash

set -e
set -u

BASEDIR=/usr/lib/vigiclient

wget https://www.vigibot.com/clientrobotpi.js -P $BASEDIR -N
wget https://www.vigibot.com/trame.js -P $BASEDIR -N
