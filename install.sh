#!/bin/bash

set -e
set -u

BASEDIR=/usr/lib/vigiclient
BASEURL=https://www.vigibot.com/vigiclient

fgrep bcm2835-v4l2 /etc/modules || echo bcm2835-v4l2 >> /etc/modules

apt install -y nodejs npm ffmpeg

rm -rf $BASEDIR
mkdir -p $BASEDIR
cd $BASEDIR

wget $BASEURL/clientrobotpi.js
wget $BASEURL/trame.js

wget $BASEURL/robot.json -P /boot -N
wget $BASEURL/vigiclient.service -P /etc/systemd/system -N

ln -s /bin/cat $BASEDIR/processdiffusion
ln -s $(which ffmpeg || echo ffmpegnotfound) $BASEDIR/processdiffaudio

wget $BASEURL/package.json
npm install

systemctl enable vigiclient
