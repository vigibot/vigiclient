#!/bin/bash

set -e
set -u

BASEURL=https://www.vigibot.com/vigiclient
BASEDIR=/usr/local/vigiclient

updated=no

function check() {
 before=$(date -r $1/$2 +%s || echo 0)
 wget $BASEURL/$2 -P $1 -N > /dev/null
 after=$(date -r $1/$2 +%s)

 if [ $before != $after ]
 then
  updated=yes
 fi
}

check $BASEDIR vigiupdate.sh
check /etc/cron.d vigicron

if [ $updated == "yes" ]
then
 echo Self-updating script
 exit 0
fi

check /etc/systemd/system vigiclient.service

if [ $updated == "yes" ]
then
 echo Updating systemd unit file
 sudo reboot
fi

if pidof -x $0 -o $$ > /dev/null
then
 echo Only one instance is allowed from here
 exit 1
fi

check $BASEDIR package.json

cd $BASEDIR

if [ $updated == "yes" -o ! -d node_modules ]
then
 rm -rf node_modules.old package-lock.json

 if [ -d node_modules ]
 then
  mv node_modules node_modules.old
 fi

 npm cache clean --force

 npm install > npm.log 2>&1 && {
  rm -rf node_modules.old
  echo Success >> npm.log
 } || {
  rm -rf node_modules
  mv node_modules.old node_modules && {
   echo Rollback >> npm.log
  } || {
   echo "Can't rollback" >> npm.log
  }
 }
fi

check $BASEDIR clientrobotpi.js
check $BASEDIR trame.js

if [ $updated == "yes" ]
then
 systemctl restart vigiclient
fi
