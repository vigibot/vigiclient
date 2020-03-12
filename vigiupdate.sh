#!/bin/bash

set -e
set -u

BASEURL=https://www.vigibot.com/vigiclient
BASEDIR=/usr/local/vigiclient

updated=no

trace() {
 echo "$(date "+%d/%m/%Y %H:%M:%S") $1"
}

abnormal() {
 trace "Abnormal script termination"
}

check() {
 before=$(date -r $1/$2 +%s || echo 0)
 wget $BASEURL/$2 -P $1 -N -T 5 -t 5 > /dev/null 2>&1
 after=$(date -r $1/$2 +%s)

 if [ $after -gt $before ]
 then
  trace "$1/$2 is updated"
  updated=yes
 else
  trace "$1/$2 is checked"
 fi
}

trap abnormal EXIT

check $BASEDIR vigiupdate.sh

if [ $updated == "yes" ]
then
 trace "Exiting"
 trap - EXIT
 exit 0
fi

check /etc/cron.d vigicron

if [ $updated == "yes" ]
then
 trace "Purging updater log"
 rm -f /var/log/vigiupdate.log
 trace "Exiting"
 trap - EXIT
 exit 0
fi

check /etc/systemd/system vigiclient.service

if [ $updated == "yes" ]
then
 trace "Rebooting"
 trap - EXIT
 sudo reboot
fi

if pidof -x $0 -o $$ > /dev/null
then
 trace "Only one instance is allowed from here"
 trap - EXIT
 exit 1
fi

timedatectl status | fgrep "synchronized: yes" > /dev/null || {
 trace "System clock must be synchronized from here"
 trap - EXIT
 exit 1
}

check $BASEDIR node_modules.tar.gz
check $BASEDIR package.json

if [ $updated == "yes" -o ! -d node_modules ]
then
 cd $BASEDIR
 rm -rf node_modules package-lock.json
 npm cache clean --force
 tar xvf node_modules.tar.gz
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

trap - EXIT
