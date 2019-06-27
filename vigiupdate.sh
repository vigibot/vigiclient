#!/bin/bash

set -e
set -u

BASEURL=https://www.vigibot.com/vigiclient
BASEDIR=/usr/lib/vigiclient

restart=no

function check() {
 before=$(date -r $BASEDIR/$1 +%s)
 wget $BASEURL/$1 -P $BASEDIR -N
 after=$(date -r $BASEDIR/$1 +%s)

 if [ $before != $after ]
 then
  restart=yes
 fi
}

check clientrobotpi.js
check trame.js

if [ $restart == "yes" ]
then
 systemctl restart vigiclient
fi
