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
 echo Self-updating script
 exit 0
fi

if pidof -x $0 -o $$ > /dev/null
then
 echo Only one instance is allowed
 exit 1
fi

check package.json

cd $BASEDIR

if [ $updated == "yes" -o ! -d node_modules ]
then
 rm -rf node_modules.old

 if [ -d node_modules ]
 then
  mv node_modules node_modules.old
 fi

 npm install > npm.log 2>&1 && {
  rm -rf node_modules.old
  echo Success >> npm.log
 } || {
  rm -rf node_modules
  mv node_modules.old node_modules && {
   echo Rollback >> npm.log
  } || {
   echo Can't rollback >> npm.log
  }
 }
fi

check clientrobotpi.js
check trame.js

if [ $updated == "yes" ]
then
 systemctl restart vigiclient
fi
