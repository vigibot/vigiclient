#!/bin/bash

set -e
set -u

BASEDIR=/usr/lib/vigiclient

wget https://www.vigibot.com/vigiclient/clientrobotpi.js -P $BASEDIR -N
wget https://www.vigibot.com/vigiclient/trame.js -P $BASEDIR -N
