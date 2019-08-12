#!/bin/bash

set -e
set -u

BASEDIR=/usr/local/vigiclient

if [ $EUID -ne 0 ]
then
 echo "This script must be run as root" 1>&2
 exit 1
fi

echo "Disable I2C"
sed -i "s/dtparam=i2c_arm=on/#dtparam=i2c_arm=on/" /boot/config.txt
sed -i "/i2c-dev/d" /etc/modules

echo "Disable camera"
sed -i "/start_x=1/d" /boot/config.txt
sed -i "/gpu_mem=128/d" /boot/config.txt

echo "Disable Video4Linux"
sed -i "/bcm2835-v4l2/d" /etc/modules

echo "Service uninstall"
systemctl stop vigiclient
systemctl disable vigiclient
rm -f /etc/systemd/system/vigiclient.service

echo "Updater uninstall"
rm -f /etc/cron.d/vigicron

echo "Removing the config file"
rm -f /boot/robot.json

echo "Cleaning"
rm -rf $BASEDIR

echo "To uninstall eSpeak, FFmpeg, pigpio and Node.js please do:"
echo "sudo apt remove --purge espeak ffmpeg pigpio npm"
