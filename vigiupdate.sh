#!/bin/bash

set -e
set -u

BASEURL=https://www.vigibot.com/vigiclient
BASEDIR=/usr/local/vigiclient

updated=no

function check() {
 before=$(date -r $BASEDIR/$1 +%s || echo 0)
 wget $BASEURL/$1 -P $BASEDIR -N
 after=$(date -r $BASEDIR/$1 +%s)

 if [ $before != $after ]
 then
  updated=yes
 fi
}

check vigiupdate.sh

if [ $updated == "yes" ]
then
 exit 0
fi

check package.json

if [ $updated == "yes" ]
then
 cd $BASEDIR
 rm -rf node_modules
 npm install
 exit 0
fi

check clientrobotpi.js
check trame.js

if [ $updated == "yes" ]
then
 systemctl restart vigiclient
fi
