#!/bin/bash

set -e
set -u

BASEDIR=/usr/local/vigiclient

if [ $EUID -ne 0 ]
then
 echo "This script must be run as root" 1>&2
 exit 1
fi

sed -i "/bcm2835-v4l2/d" /etc/modules

systemctl stop vigiclient
systemctl disable vigiclient
rm -f /etc/systemd/system/vigiclient.service

rm -rf $BASEDIR

rm -f /boot/robot.json
rm -f /etc/cron.d/vigicron

echo sudo apt remove --purge nodejs npm ffmpeg
