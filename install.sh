#!/bin/bash

set -e
set -u

BASEDIR=/usr/lib/vigiclient
BASEURL=https://www.vigibot.com

fgrep bcm2835-v4l2 /etc/modules || echo bcm2835-v4l2 >> /etc/modules

apt install -y nodejs npm ffmpeg

mkdir -p $BASEDIR
cd $BASEDIR

npm install socket.io-client stream-split rpio rpio-pwm

rm -f clientrobotp.js
rm -f trame.js
rm -f run.sh

wget $BASEURL/clientrobotpi.js -P $BASEDIR -N
wget $BASEURL/trame.js -P $BASEDIR -N
wget $BASEURL/robot.json -P /boot -N
wget $BASEURL/vigiclient.service -P /etc/systemd/system -N

rm -f processdiffusion
rm -f processdiffaudio

ln -s /bin/cat $BASEDIR/processdiffusion
ln -s $(which ffmpeg || echo ffmpegnotfound) $BASEDIR/processdiffaudio

systemctl enable vigibot
