#!/bin/bash

set -e
set -u

BASEURL=https://www.vigibot.com/vigiclient
BASEDIR=/usr/local/vigiclient

updated=no

trace() {
 echo "$(date "+%d/%m/%Y %H:%M:%S") $1"
}

check() {
 before=$(date -r $1/$2 +%s || echo 0)
 wget $BASEURL/$2 -P $1 -N > /dev/null 2>&1
 after=$(date -r $1/$2 +%s)

 if [ $after -gt $before ]
 then
  trace "$1/$2 is updated"
  updated=yes
 fi
}

check $BASEDIR vigiupdate.sh

if [ $updated == "yes" ]
then
 trace "Exiting"
 exit 0
fi

check /etc/cron.d vigicron

if [ $updated == "yes" ]
then
 trace "Purging updater log"
 rm -f /var/log/vigiupdate.log
 trace "Exiting"
 exit 0
fi

check /etc/systemd/system vigiclient.service

if [ $updated == "yes" ]
then
 trace "Rebooting"
 sudo reboot
fi

if pidof -x $0 -o $$ > /dev/null
then
 trace "Only one instance is allowed from here"
 exit 1
fi

timedatectl status | fgrep "synchronized: yes" > /dev/null || {
 trace "System clock must be synchronized from here"
 exit 1
}

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

 npm install && {
  rm -rf node_modules.old
  trace "Success"
 } || {
  rm -rf node_modules
  mv node_modules.old node_modules && {
   trace "Rollback"
  } || {
   trace "Can't rollback"
  }
 }
fi

check $BASEDIR clientrobotpi.js
check $BASEDIR trame.js

if [ $updated == "yes" ]
then
 trace "Purging client log"
 rm -f /var/log/vigiclient.log
 trace "Restarting vigiclient"
 systemctl restart vigiclient
fi
