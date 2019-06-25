#!/bin/bash

set -e
set -u

BASEDIR=/usr/bin/vigiclient
BASEURL=https://www.vigibot.com

fgrep bcm2835-v4l2 /etc/modules || echo bcm2835-v4l2 >> /etc/modules

apt install -y nodejs npm ffmpeg

mkdir -p $BASEDIR
cd $BASEDIR

npm install socket.io-client stream-split rpio rpio-pwm

rm -f clientrobotp.js
rm -f trame.js
rm -f run.sh

wget $BASEURL/clientrobotpi.js
wget $BASEURL/trame.js
wget $BASEURL/run.sh

rm -f processdiffusion
rm -f processdiffaudio

ln -s /bin/cat $BASEDIR/processdiffusion
ln -s $(which ffmpeg || echo ffmpegnotfound) $BASEDIR/processdiffaudio

fgrep run.sh /etc/rc.local || echo "$BASEDIR/run.sh &" >> /etc/rc.local
