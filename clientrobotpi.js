"use strict";

const USER = require("/boot/robot.json");
const SYS = require("./sys.json");

const TRAME = require("./trame.js");

const OS = require("os");
const FS = require("fs");
const IO = require("socket.io-client");
const EXEC = require("child_process").exec;
const RL = require("readline");
const NET = require("net");
const SPLIT = require("stream-split");
const SP = require("serialport");
const GPIO = require("pigpio").Gpio;
const I2C = require("i2c-bus");
const PCA9685 = require("pca9685");
const GPS = require("gps");

const FRAME0 = "$".charCodeAt();
const FRAME1S = "S".charCodeAt();
const FRAME1T = "T".charCodeAt();

const VERSION = Math.trunc(FS.statSync(__filename).mtimeMs);
const PROCESSTIME = Date.now();
const OSTIME = PROCESSTIME - OS.uptime() * 1000;

let sockets = {};
let currentServer = "";

let up = false;
let engine = false;
let upTimeout;

let init = false;
let initVideo = false;
let initUart = false;
let initPca = 0;

let conf = {};
let hard = {};
let tx;
let rx;
let confVideo;
let oldConfVideo;
let cmdDiffusion;
let cmdDiffAudio;

let lastTimestamp = Date.now();
let lastTrame = Date.now();
let latencyAlarm = false;

let floatTargets16 = [];
let floatTargets8 = [];
let floatTargets1 = [];
let floatCommands16 = [];
let floatCommands8 = [];
let floatCommands1 = [];
let margins16 = [];
let margins8 = [];

let oldOutputs = [];
let backslashs = [];

let contrastBoost = false;
let oldContrastBoost = false;

let serial;
let gps;

let i2c;
let gaugeType;

let pca9685Driver = [];
let gpioOutputs = [];

let prevCpus = OS.cpus();
let nbCpus = prevCpus.length;

let voltage = 0;
let battery = 0;
let cpuLoad = 0;
let socTemp = 0;
let link = 0;
let rssi = 0;

if(typeof USER.CMDDIFFUSION === "undefined")
 USER.CMDDIFFUSION = SYS.CMDDIFFUSION;

if(typeof USER.CMDDIFFAUDIO === "undefined")
 USER.CMDDIFFAUDIO = SYS.CMDDIFFAUDIO;

if(typeof USER.CMDTTS === "undefined")
 USER.CMDTTS = SYS.CMDTTS;

USER.SERVEURS.forEach(function(server) {
 sockets[server] = IO.connect(server, {"connect timeout": 1000, transports: ["websocket"], path: "/" + SYS.PORTROBOTS + "/socket.io"});
});

hard.DEBUG = true;
hard.TELEDEBUG = false;

trace("Client start");

i2c = I2C.openSync(1);

try {
 const CW2015WAKEUP = new Buffer.from([0x0a, 0x00]);
 i2c.i2cWriteSync(SYS.CW2015ADDRESS, 2, CW2015WAKEUP);
 gaugeType = "CW2015";
} catch(err) {
 try {
  i2c.readWordSync(SYS.MAX17043ADDRESS, 0x02);
  gaugeType = "MAX17043";
 } catch(err) {
  try {
   i2c.readWordSync(SYS.BQ27441ADDRESS, 0x04);
   gaugeType = "BQ27441";
  } catch(err) {
   i2c.closeSync();
   gaugeType = "";
  }
 }
}

setTimeout(function() {
 if(gaugeType)
  trace(gaugeType + " I2C fuel gauge detected");
 else
  trace("No I2C fuel gauge detected");
}, 1000);

function setInit() {
 init = initUart && initVideo && initPca == hard.PCA9685ADDRESSES.length;
}

function map(n, inMin, inMax, outMin, outMax) {
 return Math.trunc((n - inMin) * (outMax - outMin) / (inMax - inMin) + outMin);
}

function hmsm(date) {
 return ("0" + date.getHours()).slice(-2) + ":" +
        ("0" + date.getMinutes()).slice(-2) + ":" +
        ("0" + date.getSeconds()).slice(-2) + ":" +
        ("00" + date.getMilliseconds()).slice(-3);
}

function trace(message) {
 if(hard.DEBUG) {
  let trace = hmsm(new Date()) + " | " + message;
  FS.appendFile(SYS.FICHIERLOG, trace + "\n", function(err) {
  });
 }

 if(hard.TELEDEBUG) {
  let trace = hmsm(new Date()) + " | " + message;
  USER.SERVEURS.forEach(function(server) {
   sockets[server].emit("serveurrobottrace", message);
  });
 }
}

function traces(id, messages) {
 if(!hard.DEBUG && !hard.TELEDEBUG)
  return;

 let array = messages.split("\n");
 if(!array[array.length - 1])
  array.pop();
 for(let i = 0; i < array.length; i++)
  trace(id + " | " + array[i]);
}

function constrain(n, nMin, nMax) {
 if(n > nMax)
  n = nMax;
 else if(n < nMin)
  n = nMin;

 return n;
}

function sigterm(name, process, callback) {
 trace("Sending the SIGTERM signal to the process " + name);
 let processkill = EXEC("/usr/bin/pkill -15 -f ^" + process);
 processkill.on("close", function(code) {
  callback(code);
 });
}

function exec(name, command, callback) {
 trace("Starting the process " + name);
 trace(command);
 let processus = EXEC(command);
 let stdout = RL.createInterface(processus.stdout);
 let stderr = RL.createInterface(processus.stderr);
 let pid = processus.pid;
 let execTime = Date.now();

 //processus.stdout.on("data", function(data) {
 stdout.on("line", function(data) {
  traces(name + " | " + pid + " | stdout", data);
 });

 //processus.stderr.on("data", function(data) {
 stderr.on("line", function(data) {
  traces(name + " | " + pid + " | stderr", data);
 });

 processus.on("close", function(code) {
  let elapsed = Date.now() - execTime;

  trace("The " + name + " process is stopped after " + elapsed + " milliseconds with the exit code " + code);
  callback(code);
 });
}

function wake(server) {
 if(up)
  return;

 if(!init) {
  trace("This robot is not initialized");
  return;
 }

 if(currentServer) {
  trace("This robot is already in use from the " + currentServer + " server");
  return;
 }

 trace("Robot wake");

 writeOutputs();

 if(hard.SNAPSHOTSINTERVAL) {
  sigterm("Raspistill", "raspistill", function(code) {
   diffusion();
  });
 } else
  diffusion();
 diffAudio();

 currentServer = server;
 up = true;
 engine = true;
}

function sleep() {
 if(!up)
  return;

 trace("Robot sleep");

 for(let i = 0; i < conf.TX.COMMANDES16.length; i++)
  if(hard.COMMANDS16[i].SLEEP)
   floatTargets16[i] = conf.TX.COMMANDES16[i].INIT;

 for(let i = 0; i < conf.TX.COMMANDES8.length; i++)
  if(hard.COMMANDS8[i].SLEEP)
   floatTargets8[i] = conf.TX.COMMANDES8[i].INIT;

 for(let i = 0; i < conf.TX.COMMANDES1.length; i++)
  if(hard.COMMANDS1[i].SLEEP)
   floatTargets1[i] = conf.TX.COMMANDES1[i].INIT;

 sigterm("Diffusion", SYS.PROCESSDIFFUSION, function(code) {
  sigterm("DiffVideo", SYS.PROCESSDIFFVIDEO, function(code) {
  });
 });

 sigterm("DiffAudio", SYS.PROCESSDIFFAUDIO, function(code) {
 });

 exec("v4l2-ctl", SYS.V4L2 + " -c video_bitrate=" + confVideo.BITRATE, function(code) {
 });

 currentServer = "";
 up = false;
}

function configurationVideo(callback) {
 cmdDiffusion = USER.CMDDIFFUSION[confVideo.SOURCE].join("").replace(new RegExp("WIDTH", "g"), confVideo.WIDTH
                                                           ).replace(new RegExp("HEIGHT", "g"), confVideo.HEIGHT
                                                           ).replace(new RegExp("FPS", "g"), confVideo.FPS
                                                           ).replace(new RegExp("BITRATE", "g"), confVideo.BITRATE
                                                           ).replace(new RegExp("ROTATE", "g"), confVideo.ROTATE
                                                           ).replace(new RegExp("PORTTCPVIDEO", "g"), SYS.PORTTCPVIDEO);
 cmdDiffAudio = USER.CMDDIFFAUDIO.join("").replace(new RegExp("RECORDINGDEVICE", "g"), hard.RECORDINGDEVICE
                                         ).replace(new RegExp("PORTTCPAUDIO", "g"), SYS.PORTTCPAUDIO);

 trace("Initializing the Video4Linux configuration");

 let brightness;
 let contrast;
 if(contrastBoost) {
  brightness = confVideo.BRIGHTNESSBOOST;
  contrast = confVideo.CONTRASTBOOST;
 } else {
  brightness = confVideo.BRIGHTNESS;
  contrast = confVideo.CONTRAST;
 }

 exec("v4l2-ctl", SYS.V4L2 + " -v width=" + confVideo.WIDTH +
                                ",height=" + confVideo.HEIGHT +
                                ",pixelformat=4" +
                             " -p " + confVideo.FPS +
                             " -c h264_profile=0" +
                                ",repeat_sequence_header=1" +
                                ",rotate=" + confVideo.ROTATE +
                                ",video_bitrate=" + confVideo.BITRATE +
                                ",brightness=" + brightness +
                                ",contrast=" + contrast, function(code) {
  callback(code);
 });
}

function diffusion() {
 trace("Starting the H.264 video broadcast stream");
 exec("Diffusion", cmdDiffusion, function(code) {
  trace("Stopping the H.264 video broadcast stream");
 });
}

function diffAudio() {
 trace("Starting the audio broadcast stream");
 exec("DiffAudio", cmdDiffAudio, function(code) {
  trace("Stopping the audio broadcast stream");
 });
}

function initOutputs() {
 gpioOutputs.forEach(function(gpios) {
  gpios.forEach(function(gpio) {
   gpio.mode(GPIO.INPUT);
  });
 });

 pca9685Driver = [];
 gpioOutputs = [];

 for(let i = 0; i < hard.PCA9685ADDRESSES.length; i++) {
  pca9685Driver[i] = new PCA9685.Pca9685Driver({
   i2c: i2c,
   address: hard.PCA9685ADDRESSES[i],
   frequency: SYS.PCA9685FREQUENCY
  }, function(err) {
   if(err)
    trace("Error initializing PCA9685 at address " + hard.PCA9685ADDRESSES[i]);
   else {
    trace("PCA9685 initialized at address " + hard.PCA9685ADDRESSES[i]);
    initPca++;
    setInit();
   }
  });
 }

 for(let i = 0; i < hard.OUTPUTS.length; i++) {
  if(hard.OUTPUTS[i].ADRESSE == SYS.UNUSED) {
   gpioOutputs[i] = [];
   for(let j = 0; j < hard.OUTPUTS[i].GPIOS.length; j++)
    gpioOutputs[i][j] = new GPIO(hard.OUTPUTS[i].GPIOS[j], {mode: GPIO.OUTPUT});
   setMotorFrequency(i);
  }
 }

 for(let i = 0; i < conf.TX.COMMANDES16.length; i++) {
  floatTargets16[i] = conf.TX.COMMANDES16[i].INIT;
  floatCommands16[i] = floatTargets16[i];
  margins16[i] = (conf.TX.COMMANDES16[i].ECHELLEMAX - conf.TX.COMMANDES16[i].ECHELLEMIN) / 65535;
 }

 for(let i = 0; i < conf.TX.COMMANDES8.length; i++) {
  floatTargets8[i] = conf.TX.COMMANDES8[i].INIT;
  floatCommands8[i] = floatTargets8[i];
  margins8[i] = (conf.TX.COMMANDES8[i].ECHELLEMAX - conf.TX.COMMANDES8[i].ECHELLEMIN) / 255;
 }

 for(let i = 0; i < conf.TX.COMMANDES1.length; i++) {
  floatTargets1[i] = conf.TX.COMMANDES1[i].INIT;
  floatCommands1[i] = floatTargets1[i];
 }

 for(let i = 0; i < hard.OUTPUTS.length; i++) {
  oldOutputs[i] = 0;
  backslashs[i] = 0;
 }
}

USER.SERVEURS.forEach(function(server, index) {

 sockets[server].on("connect", function() {
  trace("Connected to " + server + "/" + SYS.PORTROBOTS);
  EXEC("hostname -I").stdout.on("data", function(ipPriv) {
   EXEC("iwgetid -r || echo $?").stdout.on("data", function(ssid) {
    sockets[server].emit("serveurrobotlogin", {
     conf: USER,
     version: VERSION,
     processTime: PROCESSTIME,
     osTime: OSTIME,
     ipPriv: ipPriv.trim(),
     ssid: ssid.trim()
    });
   });
  });
 });

 if(index == 0) {
  sockets[server].on("clientsrobotconf", function(data) {
   trace("Receiving robot configuration data from the " + server + " server");

   // Security hardening: even if already done on server side,
   // always filter values integrated in command lines
   const CMDINT = RegExp(/^-?\d{1,10}$/);
   for(let i = 0; i < data.hard.CAMERAS.length; i++) {
    if(!(CMDINT.test(data.hard.CAMERAS[i].SOURCE) &&
         CMDINT.test(data.hard.CAMERAS[i].WIDTH) &&
         CMDINT.test(data.hard.CAMERAS[i].HEIGHT) &&
         CMDINT.test(data.hard.CAMERAS[i].FPS) &&
         CMDINT.test(data.hard.CAMERAS[i].BITRATE) &&
         CMDINT.test(data.hard.CAMERAS[i].ROTATE) &&
         CMDINT.test(data.hard.CAMERAS[i].BRIGHTNESS) &&
         CMDINT.test(data.hard.CAMERAS[i].CONTRAST) &&
         CMDINT.test(data.hard.CAMERAS[i].BRIGHTNESSBOOST) &&
         CMDINT.test(data.hard.CAMERAS[i].CONTRASTBOOST)))
     return;
   }
   if(!(CMDINT.test(data.hard.PLAYBACKDEVICE) &&
        CMDINT.test(data.hard.RECORDINGDEVICE)))
    return;

   conf = data.conf;
   hard = data.hard;

   tx = new TRAME.Tx(conf.TX);
   rx = new TRAME.Rx(conf.TX, conf.RX);

   confVideo = hard.CAMERAS[conf.COMMANDES[conf.DEFAUTCOMMANDE].CAMERA];
   oldConfVideo = confVideo;
   contrastBoost = false;
   oldContrastBoost = false;

   initOutputs();
   if(!up)
    writeOutputs();

   setTimeout(function() {
    if(up) {
     sigterm("Diffusion", SYS.PROCESSDIFFUSION, function(code) {
      sigterm("DiffVideo", SYS.PROCESSDIFFVIDEO, function(code) {
       configurationVideo(function(code) {
        diffusion();
       });
      });
     });
    } else {
     configurationVideo(function(code) {
      initVideo = true;
      setInit();
     });
     setSleepModes();
    }
   }, 100);

   if(!initUart) {
    if(!hard.WRITEUSERDEVICE) {

     if(hard.ENABLEGPS) {
      serial = new SP(hard.SERIALPORT, {
       baudRate: hard.SERIALRATE,
       lock: false,
       parser: new SP.parsers.Readline("\r\n")
      });

      serial.on("open", function() {
       trace("Connected to " + hard.SERIALPORT);

       gps = new GPS;

       serial.on("data", function(data) {
        gps.updatePartial(data);
       });
      });
     }

     initUart = true;
     setInit();
    } else {
     serial = new SP(hard.SERIALPORT, {
      baudRate: hard.SERIALRATE,
      lock: false
     });

     serial.on("open", function() {
      trace("Connected to " + hard.SERIALPORT);

      if(hard.READUSERDEVICE) {
       serial.on("data", function(data) {

        rx.update(data, function() {
         USER.SERVEURS.forEach(function(server) {
          if(currentServer && server != currentServer)
           return;

          setRxValues();
          sockets[server].emit("serveurrobotrx", {
           timestamp: Date.now(),
           data: rx.arrayBuffer
          });
         });
        }, function(err) {
         trace(err);
        });

       });
      }

      initUart = true;
      setInit();
     });
    }
   }
  });
 }

 sockets[server].on("disconnect", function() {
  trace("Disconnected from " + server + "/" + SYS.PORTROBOTS);
  sleep();
 });

 sockets[server].on("connect_error", function(err) {
  //trace("Error connecting to " + server + "/" + SYS.PORTROBOTS);
 });

 sockets[server].on("clientsrobottts", function(data) {
  FS.writeFile("/tmp/tts.txt", data, function(err) {
   if(err)
    trace(err);
   exec("eSpeak", USER.CMDTTS.replace(new RegExp("PLAYBACKDEVICE", "g"), hard.PLAYBACKDEVICE), function(code) {
   });
  });
 });

 sockets[server].on("clientsrobotsys", function(data) {
  switch(data) {
   case "exit":
    trace("Restart the client process");
    process.exit();
    break;
   case "reboot":
    trace("Restart the system");
    EXEC("reboot");
    break;
   case "poweroff":
    trace("Power off the system");
    EXEC("poweroff");
    break;
  }
 });

 sockets[server].on("echo", function(data) {
  sockets[server].emit("echo", {
   serveur: data,
   client: Date.now()
  });
 });

 sockets[server].on("clientsrobottx", function(data) {
  if(currentServer && server != currentServer || !init)
   return;

  if(data.data[0] != FRAME0 ||
     data.data[1] != FRAME1S &&
     data.data[1] != FRAME1T) {
   trace("Reception of a corrupted frame");
   return;
  }

  // Reject bursts
  let now = Date.now();
  if(now - lastTrame < SYS.TXRATE / 2)
   return;
  lastTrame = now;

  lastTimestamp = data.boucleVideoCommande;

  if(hard.WRITEUSERDEVICE)
   serial.write(data.data);

  if(data.data[1] == FRAME1S) {
   for(let i = 0; i < tx.byteLength; i++)
    tx.bytes[i] = data.data[i];

   for(let i = 0; i < conf.TX.COMMANDES16.length; i++)
    floatTargets16[i] = tx.getFloatCommande16(i);

   for(let i = 0; i < conf.TX.COMMANDES8.length; i++)
    floatTargets8[i] = tx.getFloatCommande8(i);

   for(let i = 0; i < conf.TX.COMMANDES1.length; i++)
    floatTargets1[i] = tx.getCommande1(i);

   contrastBoost = tx.getCommande1(hard.CONTRASTBOOSTSWITCH);
   if(contrastBoost != oldContrastBoost) {
    if(contrastBoost) {
     exec("v4l2-ctl", SYS.V4L2 + " -c brightness=" + confVideo.BRIGHTNESSBOOST +
                                    ",contrast=" + confVideo.CONTRASTBOOST, function(code) {
     });
    } else {
     exec("v4l2-ctl", SYS.V4L2 + " -c brightness=" + confVideo.BRIGHTNESS +
                                    ",contrast=" + confVideo.CONTRAST, function(code) {
     });
    }
    oldContrastBoost = contrastBoost;
   }

   confVideo = hard.CAMERAS[tx.choixCameras[0]];
   if(confVideo != oldConfVideo &&
      JSON.stringify(confVideo) != JSON.stringify(oldConfVideo)) {
    if(up) {
     sigterm("Diffusion", SYS.PROCESSDIFFUSION, function(code) {
      sigterm("DiffVideo", SYS.PROCESSDIFFVIDEO, function(code) {
       configurationVideo(function(code) {
        diffusion();
       });
      });
     });
    } else {
     configurationVideo(function(code) {
     });
    }
    oldConfVideo = confVideo;
   }
  } else
   trace("Reception of a text frame");

  wake(server);
  clearTimeout(upTimeout);
  upTimeout = setTimeout(function() {
   sleep();
  }, SYS.UPTIMEOUT);

  if(!hard.READUSERDEVICE) {
   setRxCommandes();
   setRxValues();
   sockets[server].emit("serveurrobotrx", {
    timestamp: now,
    data: rx.arrayBuffer
   });
  }
 });
});

function computeOut(n, value) {
 let out;
 let nbInMax = hard.OUTPUTS[n].INS.length - 1;

 if(value <= hard.OUTPUTS[n].INS[0])
  out = hard.OUTPUTS[n].OUTS[0];
 else if(value > hard.OUTPUTS[n].INS[nbInMax])
  out = hard.OUTPUTS[n].OUTS[nbInMax];
 else {
  for(let i = 0; i < nbInMax; i++) {
   if(value <= hard.OUTPUTS[n].INS[i + 1]) {
    out = map(value, hard.OUTPUTS[n].INS[i], hard.OUTPUTS[n].INS[i + 1], hard.OUTPUTS[n].OUTS[i], hard.OUTPUTS[n].OUTS[i + 1]);
    break;
   }
  }
 }

 return out;
}

function setMotorFrequency(n) {
 if(hard.OUTPUTS[n].ADRESSE == SYS.UNUSED) {
  switch(hard.OUTPUTS[n].TYPE) {
   case "Pwms":
    for(let i = 0; i < gpioOutputs[n].length; i++)
     gpioOutputs[n][i].pwmFrequency(hard.PWMFREQUENCY);
    break;
   case "PwmPwm":
    gpioOutputs[n][0].pwmFrequency(hard.PWMFREQUENCY);
    gpioOutputs[n][1].pwmFrequency(hard.PWMFREQUENCY);
    break;
   case "PwmDir":
    gpioOutputs[n][0].pwmFrequency(hard.PWMFREQUENCY);
    break;
   case "PwmDirDir":
    gpioOutputs[n][0].pwmFrequency(hard.PWMFREQUENCY);
    break;
  }
 }
}

function setGpio(n, pin, etat) {
 let pcaId = hard.OUTPUTS[n].ADRESSE;
 let gpio;

 if(pcaId == SYS.UNUSED) {
  gpio = gpioOutputs[n][pin];
  if(etat == SYS.INPUT)
   gpio.mode(GPIO.INPUT);
  else
   gpio.digitalWrite(etat);
 } else {
  gpio = hard.OUTPUTS[n].GPIOS[pin];
  if(etat)
   pca9685Driver[pcaId].channelOn(gpio);
  else
   pca9685Driver[pcaId].channelOff(gpio);
 }
}

function setGpios(n, value) {
 let etat = computeOut(n, value);

 for(let i = 0; i < hard.OUTPUTS[n].GPIOS.length; i++)
  setGpio(n, i, etat);
}

function setServos(n, value) {
 let pwm = computeOut(n, value);
 let pcaId = hard.OUTPUTS[n].ADRESSE;

 if(pcaId == SYS.UNUSED)
  for(let i = 0; i < hard.OUTPUTS[n].GPIOS.length; i++)
   gpioOutputs[n][i].servoWrite(pwm);
 else
  for(let i = 0; i < hard.OUTPUTS[n].GPIOS.length; i++)
   pca9685Driver[hard.OUTPUTS[n].ADRESSE].setPulseLength(hard.OUTPUTS[n].GPIOS[i], pwm);
}

function setPwm(n, gpio, pwm) {
 let pcaId = hard.OUTPUTS[n].ADRESSE;

 if(pcaId == SYS.UNUSED)
  gpioOutputs[n][gpio].pwmWrite(Math.abs(map(pwm, -100, 100, -255, 255)));
 else
  pca9685Driver[pcaId].setDutyCycle(hard.OUTPUTS[n].GPIOS[gpio], Math.abs(pwm / 100));
}

function setPwms(n, value) {
 let pwm = computeOut(n, value);

 for(let i = 0; i < hard.OUTPUTS[n].GPIOS.length; i++)
  setPwm(n, i, pwm);
}

function setPwmPwm(n, value) {
 let pwm = computeOut(n, value);

 if(pwm > 0) {
  setPwm(n, 0, pwm);
  setGpio(n, 1, 0);
 } else if(pwm < 0) {
  setGpio(n, 0, 0);
  setPwm(n, 1, pwm);
 } else {
  setGpio(n, 0, 1);
  setGpio(n, 1, 1);
 }
}

function setPwmDir(n, value) {
 let pwm = computeOut(n, value);

 if(pwm > 0)
  setGpio(n, 1, 1);
 else
  setGpio(n, 1, 0);
 setPwm(n, 0, pwm);
}

function setPwmDirDir(n, value) {
 let pwm = computeOut(n, value);

 if(pwm > 0) {
  setGpio(n, 1, 1);
  setGpio(n, 2, 0);
 } else if(pwm < 0) {
  setGpio(n, 1, 0);
  setGpio(n, 2, 1);
 } else {
  setGpio(n, 1, 1);
  setGpio(n, 2, 1);
 }
 setPwm(n, 0, pwm);
}

function writeOutputs() {
 for(let i = 0; i < hard.OUTPUTS.length; i++) {

  let output = 0;

  for(let j = 0; j < hard.OUTPUTS[i].COMMANDS16.length; j++)
   output += floatCommands16[hard.OUTPUTS[i].COMMANDS16[j]] * hard.OUTPUTS[i].GAINS16[j];
  for(let j = 0; j < hard.OUTPUTS[i].COMMANDS8.length; j++)
   output += floatCommands8[hard.OUTPUTS[i].COMMANDS8[j]] * hard.OUTPUTS[i].GAINS8[j];
  for(let j = 0; j < hard.OUTPUTS[i].COMMANDS1.length; j++)
   output += floatCommands1[hard.OUTPUTS[i].COMMANDS1[j]] * hard.OUTPUTS[i].GAINS1[j];

  if(output < oldOutputs[i])
   backslashs[i] = -hard.OUTPUTS[i].BACKSLASH;
  else if(output > oldOutputs[i])
   backslashs[i] = hard.OUTPUTS[i].BACKSLASH;

  oldOutputs[i] = output;

  let value = output + backslashs[i];

  switch(hard.OUTPUTS[i].TYPE) {
   case "Gpios":
    setGpios(i, value);
    break;
   case "Servos":
    setServos(i, value);
    break;
   case "Pwms":
    setPwms(i, value);
    break;
   case "PwmPwm":
    setPwmPwm(i, value);
    break;
   case "PwmDir":
    setPwmDir(i, value);
    break;
   case "PwmDirDir":
    setPwmDirDir(i, value);
    break;
  }
 }
}

function setSleepModes() {
 for(let i = 0; i < hard.OUTPUTS.length; i++) {
  let pcaId = hard.OUTPUTS[i].ADRESSE;
  let etat;
  for(let j = 0; j < hard.OUTPUTS[i].SLEEPMODES.length; j++) {
   let sleepMode = hard.OUTPUTS[i].SLEEPMODES[j];
   if(sleepMode == "None")
    continue;
   if(sleepMode == "High")
    etat = 1;
   else if(sleepMode == "Low")
    etat = 0;
   else
    etat = 2;
   setGpio(i, j, etat);
  }
 }
}

setInterval(function() {
 if(!engine)
  return;

 let change = false;
 let predictiveLatency = Date.now() - lastTimestamp;

 if(predictiveLatency < SYS.LATENCEFINALARME && latencyAlarm) {
  trace("Latency of " + predictiveLatency + " ms, return to configured video bitrate");
  exec("v4l2-ctl", SYS.V4L2 + " -c video_bitrate=" + confVideo.BITRATE, function(code) {
  });
  latencyAlarm = false;
 } else if(predictiveLatency > SYS.LATENCEDEBUTALARME && !latencyAlarm) {
  trace("Latency of " + predictiveLatency + " ms, stop the motors and switch to reduced video bitrate");
  exec("v4l2-ctl", SYS.V4L2 + " -c video_bitrate=" + SYS.BITRATEVIDEOFAIBLE, function(code) {
  });
  latencyAlarm = true;
 }

 if(latencyAlarm) {
  for(let i = 0; i < conf.TX.COMMANDES16.length; i++)
   if(hard.COMMANDS16[i].FAILSAFE)
    floatTargets16[i] = conf.TX.COMMANDES16[i].INIT;

  for(let i = 0; i < conf.TX.COMMANDES8.length; i++)
   if(hard.COMMANDS8[i].FAILSAFE)
    floatTargets8[i] = conf.TX.COMMANDES8[i].INIT;

  for(let i = 0; i < conf.TX.COMMANDES1.length; i++)
   if(hard.COMMANDS1[i].FAILSAFE)
    floatTargets1[i] = conf.TX.COMMANDES1[i].INIT;
 }

 for(let i = 0; i < conf.TX.COMMANDES16.length; i++) {
  if(floatCommands16[i] == floatTargets16[i])
   continue;
  change = true;

  let delta;
  let cible = floatTargets16[i];
  let init = conf.TX.COMMANDES16[i].INIT;

  if(Math.abs(cible - init) <= margins16[i])
   delta = hard.COMMANDS16[i].RAMPINIT;
  else if((cible - init) * (floatCommands16[i] - init) < 0) {
   delta = hard.COMMANDS16[i].RAMPDOWN;
   cible = init;
  } else if(Math.abs(cible) - Math.abs(floatCommands16[i]) < 0)
   delta = hard.COMMANDS16[i].RAMPDOWN;
  else
   delta = hard.COMMANDS16[i].RAMPUP;

  if(delta <= 0)
   floatCommands16[i] = cible;
  else if(floatCommands16[i] - cible < -delta)
   floatCommands16[i] += delta;
  else if(floatCommands16[i] - cible > delta)
   floatCommands16[i] -= delta;
  else
   floatCommands16[i] = cible;
 }

 for(let i = 0; i < conf.TX.COMMANDES8.length; i++) {
  if(floatCommands8[i] == floatTargets8[i])
   continue;
  change = true;

  let delta;
  let cible = floatTargets8[i];
  let init = conf.TX.COMMANDES8[i].INIT;

  if(Math.abs(cible - init) <= margins8[i])
   delta = hard.COMMANDS8[i].RAMPINIT;
  else if((cible - init) * (floatCommands8[i] - init) < 0) {
   delta = hard.COMMANDS8[i].RAMPDOWN;
   cible = init;
  } else if(Math.abs(cible) - Math.abs(floatCommands8[i]) < 0)
   delta = hard.COMMANDS8[i].RAMPDOWN;
  else
   delta = hard.COMMANDS8[i].RAMPUP;

  if(delta <= 0)
   floatCommands8[i] = cible;
  else if(floatCommands8[i] - cible < -delta)
   floatCommands8[i] += delta;
  else if(floatCommands8[i] - cible > delta)
   floatCommands8[i] -= delta;
  else
   floatCommands8[i] = cible;
 }

 for(let i = 0; i < conf.TX.COMMANDES1.length; i++) {
  if(floatCommands1[i] == floatTargets1[i])
   continue;
  change = true;

  let delta;
  if(Math.abs(floatTargets1[i] - conf.TX.COMMANDES1[i].INIT) < 1)
   delta = hard.COMMANDS1[i].RAMPINIT;
  else
   delta = hard.COMMANDS1[i].RAMPUP;

  if(delta <= 0)
   floatCommands1[i] = floatTargets1[i];
  else if(floatTargets1[i] - floatCommands1[i] > delta)
   floatCommands1[i] += delta;
  else if(floatTargets1[i] - floatCommands1[i] < -delta)
   floatCommands1[i] -= delta;
  else
   floatCommands1[i] = floatTargets1[i];
 }

 if(change)
  writeOutputs();
 else if(!up) {
  setSleepModes();
  engine = false;
 }
}, SYS.SERVORATE);

setInterval(function() {
 if(!init)
  return;

 let currCpus = OS.cpus();
 let charges = 0;
 let idles = 0;

 for(let i = 0; i < nbCpus; i++) {
  let prevCpu = prevCpus[i];
  let currCpu = currCpus[i];

  charges += currCpu.times.user - prevCpu.times.user;
  charges += currCpu.times.nice - prevCpu.times.nice;
  charges += currCpu.times.sys - prevCpu.times.sys;
  charges += currCpu.times.irq - prevCpu.times.irq;
  idles += currCpu.times.idle - prevCpu.times.idle;
 }
 prevCpus = currCpus;

 cpuLoad = Math.trunc(100 * charges / (charges + idles));
}, SYS.CPURATE);

setInterval(function() {
 if(!init)
  return;

 FS.readFile(SYS.FICHIERTEMPERATURE, function(err, data) {
  socTemp = data / 1000;
 });
}, SYS.TEMPERATURERATE);

setInterval(function() {
 if(!init)
  return;

 const STATS = RL.createInterface(FS.createReadStream(SYS.FICHIERWIFI));

 STATS.on("line", function(ligne) {
  ligne = ligne.split(/\s+/);

  if(ligne[1] == SYS.INTERFACEWIFI + ":") {
   link = ligne[3];
   rssi = ligne[4];
  }
 });
}, SYS.WIFIRATE);

function swapWord(word) {
 return (word & 0xff) << 8 | word >> 8;
}

switch(gaugeType) {
 case "CW2015":
  setInterval(function() {
   if(!init)
    return;
   i2c.readWord(SYS.CW2015ADDRESS, 0x02, function(err, microVolts305) {
    voltage = swapWord(microVolts305) * 305 / 1000000;
    i2c.readWord(SYS.CW2015ADDRESS, 0x04, function(err, pour25600) {
     battery = swapWord(pour25600) / 256;
    });
   });
  }, SYS.GAUGERATE);
  break;

 case "MAX17043":
  setInterval(function() {
   if(!init)
    return;
   i2c.readWord(SYS.MAX17043ADDRESS, 0x02, function(err, volts12800) {
    voltage = swapWord(volts12800) / 12800;
    i2c.readWord(SYS.MAX17043ADDRESS, 0x04, function(err, pour25600) {
     battery = swapWord(pour25600) / 256;
    });
   });
  }, SYS.GAUGERATE);
  break;

 case "BQ27441":
  setInterval(function() {
   if(!init)
    return;
   i2c.readWord(SYS.BQ27441ADDRESS, 0x04, function(err, milliVolts) {
    voltage = milliVolts / 1000;
    i2c.readByte(SYS.BQ27441ADDRESS, 0x1c, function(err, pourcents) {
     battery = pourcents;
    });
   });
  }, SYS.GAUGERATE);
  break;
}

function setRxCommandes() {
 for(let i = 0; i < conf.TX.COMMANDES16.length; i++)
  rx.commandesInt16[i] = tx.computeRawCommande16(i, floatCommands16[i]);
 rx.choixCameras[0] = tx.choixCameras[0];
 for(let i = 0; i < conf.TX.COMMANDES8.length; i++)
  rx.commandesInt8[i] = tx.computeRawCommande8(i, floatCommands8[i]);
 for(let i = 0; i < conf.TX.COMMANDES1.length / 8; i++) {
  let commande1 = 0;
  for(let j = 0; j < 8; j++)
   if(floatCommands1[i * 8 + j] >= 0.5)
    commande1 += 1 << j;
  rx.commandes1[i] = commande1;
 }
}

function setRxValues() {
 if(hard.ENABLEGPS && gps.state.lat !== null) {
  rx.setFloatValeur32(0, gps.state.lat);
  rx.setFloatValeur32(1, gps.state.lon);
 }
 rx.setFloatValeur16(0, voltage);
 rx.setFloatValeur16(1, battery);
 rx.setFloatValeur8(0, cpuLoad);
 rx.setFloatValeur8(1, socTemp);
 rx.setFloatValeur8(2, link);
 rx.setFloatValeur8(3, rssi);
 if(hard.ENABLEGPS) {
  if(typeof gps.state.satsActive !== "undefined")
   rx.setFloatValeur8(4, gps.state.satsActive.length);
  rx.setFloatValeur8(5, gps.state.speed);
  if(gps.state.track !== null)
   rx.setFloatValeur8(6, gps.state.track);
 }
}

setInterval(function() {
 if(up || !init || hard.READUSERDEVICE)
  return;

 setRxCommandes();
 setRxValues();
 USER.SERVEURS.forEach(function(server) {
  sockets[server].emit("serveurrobotrx", {
   timestamp: Date.now(),
   data: rx.arrayBuffer
  });
 });
}, SYS.BEACONRATE);

setInterval(function() {
 if(up || !init || !hard.SNAPSHOTSINTERVAL)
  return;

 let date = new Date();

 if(date.getMinutes() % hard.SNAPSHOTSINTERVAL)
  return;

 let overlay = date.toLocaleDateString() + " " + date.toLocaleTimeString();
 if(hard.EXPOSUREBRACKETING)
  overlay += " HDR " + hard.EXPOSUREBRACKETING;
 let options = "-a 1024 -a '" + overlay + "' -rot " + confVideo.ROTATE;

 if(hard.EXPOSUREBRACKETING) {
  EXEC("raspistill -ev " + -hard.EXPOSUREBRACKETING + " " + options + " -o /tmp/1.jpg", function(err) {
   if(err) {
    trace("Error while capturing the first photo");
    return;
   }
   EXEC("raspistill " + options + " -o /tmp/2.jpg", function(err) {
    if(err) {
     trace("Error while capturing the second photo");
     return;
    }
    EXEC("raspistill -ev " + hard.EXPOSUREBRACKETING + " " + options + " -o /tmp/3.jpg", function(err) {
     if(err) {
      trace("Error while capturing the third photo");
      return;
     }
     EXEC("enfuse -o /tmp/out.jpg /tmp/1.jpg /tmp/2.jpg /tmp/3.jpg", function(err) {
      if(err)
       trace("Error when merging photos");
      else {
       FS.readFile("/tmp/out.jpg", function(err, data) {
        USER.SERVEURS.forEach(function(server) {
         trace("Uploading a photo to the server " + server);
         sockets[server].emit("serveurrobotcapturesenveille", data);
        });
       });
      }
     });
    });
   });
  });
 } else {
  EXEC("raspistill -q 10 " + options + " -o /tmp/out.jpg", function(err) {
   if(err)
    trace("Error while capturing the photo");
   else {
    FS.readFile("/tmp/out.jpg", function(err, data) {
     USER.SERVEURS.forEach(function(server) {
      trace("Uploading a photo to the server " + server);
      sockets[server].emit("serveurrobotcapturesenveille", data);
     });
    });
   }
  });
 }
}, 60000);

NET.createServer(function(socket) {
 const SEPARATEURNALU = new Buffer.from([0, 0, 0, 1]);
 const SPLITTER = new SPLIT(SEPARATEURNALU);

 trace("H.264 video streaming process is connected to tcp://127.0.0.1:" + SYS.PORTTCPVIDEO);

 SPLITTER.on("data", function(data) {

  if(currentServer) {
   sockets[currentServer].emit("serveurrobotvideo", {
    timestamp: Date.now(),
    data: data
   });
  }

 }).on("error", function(err) {
  trace("Error when splitting input stream into H.264 network abstraction layer units");
 });

 socket.pipe(SPLITTER);

 socket.on("end", function() {
  trace("H.264 video streaming process is disconnected from tcp://127.0.0.1:" + SYS.PORTTCPVIDEO);
 });

}).listen(SYS.PORTTCPVIDEO);

NET.createServer(function(socket) {

 trace("The audio streaming process is connected to tcp://127.0.0.1:" + SYS.PORTTCPAUDIO);

 let array = [];
 let i = 0;
 socket.on("data", function(data) {

  array.push(data);
  i++;

  if(i == 20) {
   if(currentServer) {
    sockets[currentServer].emit("serveurrobotaudio", {
     timestamp: Date.now(),
     data: Buffer.concat(array)
    });
   }
   array = [];
   i = 0;
  }

 })

 socket.on("end", function() {
  trace("Audio streaming process is disconnected from tcp://127.0.0.1:" + SYS.PORTTCPAUDIO);
 });

}).listen(SYS.PORTTCPAUDIO);

process.on("uncaughtException", function(err) {
 let i = 0;
 let errors = err.stack.split("\n");

 while(i < errors.length)
  trace(errors[i++]);

 trace("Following this uncaught exception, the Node.js process will be terminated automatically");
 setTimeout(function() {
  process.exit(1);
 }, 1000);
})

trace("Client ready");
