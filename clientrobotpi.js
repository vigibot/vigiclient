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
let serveurCourant = "";

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
let alarmeLatence = false;

let floatCibles16 = [];
let floatCibles8 = [];
let floatCibles1 = [];
let floatCommandes16 = [];
let floatCommandes8 = [];
let floatCommandes1 = [];
let marges16 = [];
let marges8 = [];

let oldOutputs = [];
let backslashs = [];

let boostVideo = false;
let oldBoostVideo = false;

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

USER.SERVEURS.forEach(function(serveur) {
 sockets[serveur] = IO.connect(serveur, {"connect timeout": 1000, transports: ["websocket"], path: "/" + SYS.PORTROBOTS + "/socket.io"});
});

hard.DEBUG = true;
hard.TELEDEBUG = false;

trace("Démarrage du client");

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

function heure(date) {
 return ("0" + date.getHours()).slice(-2) + ":" +
        ("0" + date.getMinutes()).slice(-2) + ":" +
        ("0" + date.getSeconds()).slice(-2) + ":" +
        ("00" + date.getMilliseconds()).slice(-3);
}

function trace(message) {
 if(hard.DEBUG) {
  let trace = heure(new Date()) + " | " + message;
  FS.appendFile(SYS.FICHIERLOG, trace + "\n", function(err) {
  });
 }

 if(hard.TELEDEBUG) {
  let trace = heure(new Date()) + " | " + message;
  USER.SERVEURS.forEach(function(serveur) {
   sockets[serveur].emit("serveurrobottrace", message);
  });
 }
}

function traces(id, messages) {
 if(!hard.DEBUG && !hard.TELEDEBUG)
  return;

 let tableau = messages.split("\n");
 if(!tableau[tableau.length - 1])
  tableau.pop();
 for(let i = 0; i < tableau.length; i++)
  trace(id + " | " + tableau[i]);
}

function constrain(n, nMin, nMax) {
 if(n > nMax)
  n = nMax;
 else if(n < nMin)
  n = nMin;

 return n;
}

function sigterm(nom, processus, callback) {
 trace("Envoi du signal SIGTERM au processus " + nom);
 let processkill = EXEC("/usr/bin/pkill -15 -f ^" + processus);
 processkill.on("close", function(code) {
  callback(code);
 });
}

function exec(nom, commande, callback) {
 trace("Démarrage du processus " + nom);
 trace(commande);
 let processus = EXEC(commande);
 let stdout = RL.createInterface(processus.stdout);
 let stderr = RL.createInterface(processus.stderr);
 let pid = processus.pid;
 let execTime = Date.now();

 //processus.stdout.on("data", function(data) {
 stdout.on("line", function(data) {
  traces(nom + " | " + pid + " | stdout", data);
 });

 //processus.stderr.on("data", function(data) {
 stderr.on("line", function(data) {
  traces(nom + " | " + pid + " | stderr", data);
 });

 processus.on("close", function(code) {
  let elapsed = Date.now() - execTime;

  trace("Le processus " + nom + " c'est arrêté après " + elapsed + " millisecondes avec le code de sortie " + code);
  callback(code);
 });
}

function debout(serveur) {
 if(up)
  return;

 if(!init) {
  trace("Ce robot n'est pas initialisé");
  return;
 }

 if(serveurCourant) {
  trace("Ce robot est déjà utilisé depuis le serveur " + serveurCourant);
  return;
 }

 trace("Sortie de veille du robot");

 writeOutputs();

 if(hard.CAPTURESENVEILLE) {
  sigterm("Raspistill", "raspistill", function(code) {
   diffusion();
  });
 } else
  diffusion();
 diffAudio();

 serveurCourant = serveur;
 up = true;
 engine = true;
}

function dodo() {
 if(!up)
  return;

 trace("Mise en veille du robot");

 for(let i = 0; i < conf.TX.COMMANDES16.length; i++)
  if(hard.MIXAGES16[i].SLEEP)
   floatCibles16[i] = conf.TX.COMMANDES16[i].INIT;

 for(let i = 0; i < conf.TX.COMMANDES8.length; i++)
  if(hard.MIXAGES8[i].SLEEP)
   floatCibles8[i] = conf.TX.COMMANDES8[i].INIT;

 for(let i = 0; i < conf.TX.COMMANDES1.length; i++)
  if(hard.MIXAGES1[i].SLEEP)
   floatCibles1[i] = conf.TX.COMMANDES1[i].INIT;

 sigterm("Diffusion", SYS.PROCESSDIFFUSION, function(code) {
  sigterm("DiffVideo", SYS.PROCESSDIFFVIDEO, function(code) {
  });
 });

 sigterm("DiffAudio", SYS.PROCESSDIFFAUDIO, function(code) {
 });

 exec("v4l2-ctl", SYS.V4L2 + " -c video_bitrate=" + confVideo.BITRATE, function(code) {
 });

 serveurCourant = "";
 up = false;
}

function configurationVideo(callback) {
 cmdDiffusion = USER.CMDDIFFUSION[confVideo.SOURCE].join("").replace(new RegExp("WIDTH", "g"), confVideo.WIDTH
                                                           ).replace(new RegExp("HEIGHT", "g"), confVideo.HEIGHT
                                                           ).replace(new RegExp("FPS", "g"), confVideo.FPS
                                                           ).replace(new RegExp("BITRATE", "g"), confVideo.BITRATE
                                                           ).replace(new RegExp("ROTATION", "g"), confVideo.ROTATION
                                                           ).replace(new RegExp("PORTTCPVIDEO", "g"), SYS.PORTTCPVIDEO);
 cmdDiffAudio = USER.CMDDIFFAUDIO.join("").replace(new RegExp("RECORDINGDEVICE", "g"), hard.RECORDINGDEVICE
                                         ).replace(new RegExp("PORTTCPAUDIO", "g"), SYS.PORTTCPAUDIO);

 trace("Initialisation de la configuration Video4Linux");

 let luminosite;
 let contraste;
 if(boostVideo) {
  luminosite = confVideo.BOOSTVIDEOLUMINOSITE;
  contraste = confVideo.BOOSTVIDEOCONTRASTE;
 } else {
  luminosite = confVideo.LUMINOSITE;
  contraste = confVideo.CONTRASTE;
 }

 exec("v4l2-ctl", SYS.V4L2 + " -v width=" + confVideo.WIDTH +
                                ",height=" + confVideo.HEIGHT +
                                ",pixelformat=4" +
                             " -p " + confVideo.FPS +
                             " -c h264_profile=0" +
                                ",repeat_sequence_header=1" +
                                ",rotate=" + confVideo.ROTATION +
                                ",video_bitrate=" + confVideo.BITRATE +
                                ",brightness=" + luminosite +
                                ",contrast=" + contraste, function(code) {
  callback(code);
 });
}

function diffusion() {
 trace("Démarrage du flux de diffusion vidéo H.264");
 exec("Diffusion", cmdDiffusion, function(code) {
  trace("Arrêt du flux de diffusion vidéo H.264");
 });
}

function diffAudio() {
 trace("Démarrage du flux de diffusion audio");
 exec("DiffAudio", cmdDiffAudio, function(code) {
  trace("Arrêt du flux de diffusion audio");
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
  floatCibles16[i] = conf.TX.COMMANDES16[i].INIT;
  floatCommandes16[i] = floatCibles16[i];
  marges16[i] = (conf.TX.COMMANDES16[i].ECHELLEMAX - conf.TX.COMMANDES16[i].ECHELLEMIN) / 65535;
 }

 for(let i = 0; i < conf.TX.COMMANDES8.length; i++) {
  floatCibles8[i] = conf.TX.COMMANDES8[i].INIT;
  floatCommandes8[i] = floatCibles8[i];
  marges8[i] = (conf.TX.COMMANDES8[i].ECHELLEMAX - conf.TX.COMMANDES8[i].ECHELLEMIN) / 255;
 }

 for(let i = 0; i < conf.TX.COMMANDES1.length; i++) {
  floatCibles1[i] = conf.TX.COMMANDES1[i].INIT;
  floatCommandes1[i] = floatCibles1[i];
 }

 for(let i = 0; i < hard.OUTPUTS.length; i++) {
  oldOutputs[i] = 0;
  backslashs[i] = 0;
 }
}

USER.SERVEURS.forEach(function(serveur, index) {

 sockets[serveur].on("connect", function() {
  trace("Connecté sur " + serveur + "/" + SYS.PORTROBOTS);
  EXEC("hostname -I").stdout.on("data", function(ipPriv) {
   EXEC("iwgetid -r || echo $?").stdout.on("data", function(ssid) {
    sockets[serveur].emit("serveurrobotlogin", {
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
  sockets[serveur].on("clientsrobotconf", function(data) {
   trace("Réception des données de configuration du robot depuis le serveur " + serveur);

   // Security hardening: even if already done on server side,
   // always filter values integrated in command lines
   const CMDINT = RegExp(/^-?\d{1,10}$/);
   for(let i = 0; i < data.hard.CAMERAS.length; i++) {
    if(!(CMDINT.test(data.hard.CAMERAS[i].SOURCE) &&
         CMDINT.test(data.hard.CAMERAS[i].WIDTH) &&
         CMDINT.test(data.hard.CAMERAS[i].HEIGHT) &&
         CMDINT.test(data.hard.CAMERAS[i].FPS) &&
         CMDINT.test(data.hard.CAMERAS[i].BITRATE) &&
         CMDINT.test(data.hard.CAMERAS[i].ROTATION) &&
         CMDINT.test(data.hard.CAMERAS[i].LUMINOSITE) &&
         CMDINT.test(data.hard.CAMERAS[i].CONTRASTE) &&
         CMDINT.test(data.hard.CAMERAS[i].BOOSTVIDEOLUMINOSITE) &&
         CMDINT.test(data.hard.CAMERAS[i].BOOSTVIDEOCONTRASTE)))
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
   boostVideo = false;
   oldBoostVideo = false;

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
    if(!hard.DEVTELECOMMANDE) {

     if(hard.DEVGPS) {
      serial = new SP(hard.DEVROBOT, {
       baudRate: hard.DEVDEBIT,
       lock: false,
       parser: new SP.parsers.Readline("\r\n")
      });

      serial.on("open", function() {
       trace("Connecté sur " + hard.DEVROBOT);

       gps = new GPS;

       serial.on("data", function(data) {
        gps.updatePartial(data);
       });
      });
     }

     initUart = true;
     setInit();
    } else {
     serial = new SP(hard.DEVROBOT, {
      baudRate: hard.DEVDEBIT,
      lock: false
     });

     serial.on("open", function() {
      trace("Connecté sur " + hard.DEVROBOT);

      if(hard.DEVTELEMETRIE) {
       serial.on("data", function(data) {

        rx.update(data, function() {
         USER.SERVEURS.forEach(function(serveur) {
          if(serveurCourant && serveur != serveurCourant)
           return;

          setRxValeurs();
          sockets[serveur].emit("serveurrobotrx", {
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

 sockets[serveur].on("disconnect", function() {
  trace("Déconnecté de " + serveur + "/" + SYS.PORTROBOTS);
  dodo();
 });

 sockets[serveur].on("connect_error", function(err) {
  //trace("Erreur de connexion au serveur " + serveur + "/" + SYS.PORTROBOTS);
 });

 sockets[serveur].on("clientsrobottts", function(data) {
  FS.writeFile("/tmp/tts.txt", data, function(err) {
   if(err)
    trace(err);
   exec("eSpeak", USER.CMDTTS.replace(new RegExp("PLAYBACKDEVICE", "g"), hard.PLAYBACKDEVICE), function(code) {
   });
  });
 });

 sockets[serveur].on("clientsrobotsys", function(data) {
  switch(data) {
   case "exit":
    trace("Fin du processus Node.js");
    process.exit();
    break;
   case "reboot":
    trace("Redémarrage du système");
    EXEC("reboot");
    break;
   case "poweroff":
    trace("Arrêt du système");
    EXEC("poweroff");
    break;
  }
 });

 sockets[serveur].on("echo", function(data) {
  sockets[serveur].emit("echo", {
   serveur: data,
   client: Date.now()
  });
 });

 sockets[serveur].on("clientsrobottx", function(data) {
  if(serveurCourant && serveur != serveurCourant || !init)
   return;

  if(data.data[0] != FRAME0 ||
     data.data[1] != FRAME1S &&
     data.data[1] != FRAME1T) {
   trace("Réception d'une trame corrompue");
   return;
  }

  // Reject bursts
  let now = Date.now();
  if(now - lastTrame < SYS.TXRATE / 2)
   return;
  lastTrame = now;

  lastTimestamp = data.boucleVideoCommande;

  if(hard.DEVTELECOMMANDE)
   serial.write(data.data);

  if(data.data[1] == FRAME1S) {
   for(let i = 0; i < tx.byteLength; i++)
    tx.bytes[i] = data.data[i];

   for(let i = 0; i < conf.TX.COMMANDES16.length; i++)
    floatCibles16[i] = tx.getFloatCommande16(i);

   for(let i = 0; i < conf.TX.COMMANDES8.length; i++)
    floatCibles8[i] = tx.getFloatCommande8(i);

   for(let i = 0; i < conf.TX.COMMANDES1.length; i++)
    floatCibles1[i] = tx.getCommande1(i);

   boostVideo = tx.getCommande1(hard.INTERRUPTEURBOOSTVIDEO);
   if(boostVideo != oldBoostVideo) {
    if(boostVideo) {
     exec("v4l2-ctl", SYS.V4L2 + " -c brightness=" + confVideo.BOOSTVIDEOLUMINOSITE +
                                    ",contrast=" + confVideo.BOOSTVIDEOCONTRASTE, function(code) {
     });
    } else {
     exec("v4l2-ctl", SYS.V4L2 + " -c brightness=" + confVideo.LUMINOSITE +
                                    ",contrast=" + confVideo.CONTRASTE, function(code) {
     });
    }
    oldBoostVideo = boostVideo;
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
   trace("Réception d'une trame texte");

  debout(serveur);
  clearTimeout(upTimeout);
  upTimeout = setTimeout(function() {
   dodo();
  }, SYS.UPTIMEOUT);

  if(!hard.DEVTELEMETRIE) {
   setRxCommandes();
   setRxValeurs();
   sockets[serveur].emit("serveurrobotrx", {
    timestamp: now,
    data: rx.arrayBuffer
   });
  }
 });
});

function computeOut(n, consigne) {
 let out;
 let nbInMax = hard.OUTPUTS[n].INS.length - 1;

 if(consigne <= hard.OUTPUTS[n].INS[0])
  out = hard.OUTPUTS[n].OUTS[0];
 else if(consigne > hard.OUTPUTS[n].INS[nbInMax])
  out = hard.OUTPUTS[n].OUTS[nbInMax];
 else {
  for(let i = 0; i < nbInMax; i++) {
   if(consigne <= hard.OUTPUTS[n].INS[i + 1]) {
    out = map(consigne, hard.OUTPUTS[n].INS[i], hard.OUTPUTS[n].INS[i + 1], hard.OUTPUTS[n].OUTS[i], hard.OUTPUTS[n].OUTS[i + 1]);
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
     gpioOutputs[n][i].pwmFrequency(hard.FREQUENCEPWM);
    break;
   case "PwmPwm":
    gpioOutputs[n][0].pwmFrequency(hard.FREQUENCEPWM);
    gpioOutputs[n][1].pwmFrequency(hard.FREQUENCEPWM);
    break;
   case "PwmDir":
    gpioOutputs[n][0].pwmFrequency(hard.FREQUENCEPWM);
    break;
   case "PwmDirDir":
    gpioOutputs[n][0].pwmFrequency(hard.FREQUENCEPWM);
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

function setGpios(n, consigne) {
 let etat = computeOut(n, consigne);

 for(let i = 0; i < hard.OUTPUTS[n].GPIOS.length; i++)
  setGpio(n, i, etat);
}

function setServos(n, consigne) {
 let pwm = computeOut(n, consigne);
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

function setPwms(n, consigne) {
 let pwm = computeOut(n, consigne);

 for(let i = 0; i < hard.OUTPUTS[n].GPIOS.length; i++)
  setPwm(n, i, pwm);
}

function setPwmPwm(n, consigne) {
 let pwm = computeOut(n, consigne);

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

function setPwmDir(n, consigne) {
 let pwm = computeOut(n, consigne);

 if(pwm > 0)
  setGpio(n, 1, 1);
 else
  setGpio(n, 1, 0);
 setPwm(n, 0, pwm);
}

function setPwmDirDir(n, consigne) {
 let pwm = computeOut(n, consigne);

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

  for(let j = 0; j < conf.TX.COMMANDES16.length; j++)
   output += floatCommandes16[j] * hard.MIXAGES16[j].GAINS[i];
  for(let j = 0; j < conf.TX.COMMANDES8.length; j++)
   output += floatCommandes8[j] * hard.MIXAGES8[j].GAINS[i];
  for(let j = 0; j < conf.TX.COMMANDES1.length; j++)
   output += floatCommandes1[j] * hard.MIXAGES1[j].GAINS[i];

  if(output < oldOutputs[i])
   backslashs[i] = -hard.OUTPUTS[i].BACKSLASH;
  else if(output > oldOutputs[i])
   backslashs[i] = hard.OUTPUTS[i].BACKSLASH;

  oldOutputs[i] = output;

  let consigne = output + backslashs[i];

  switch(hard.OUTPUTS[i].TYPE) {
   case "Gpios":
    setGpios(i, consigne);
    break;
   case "Servos":
    setServos(i, consigne);
    break;
   case "Pwms":
    setPwms(i, consigne);
    break;
   case "PwmPwm":
    setPwmPwm(i, consigne);
    break;
   case "PwmDir":
    setPwmDir(i, consigne);
    break;
   case "PwmDirDir":
    setPwmDirDir(i, consigne);
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
 let latencePredictive = Date.now() - lastTimestamp;

 if(latencePredictive < SYS.LATENCEFINALARME && alarmeLatence) {
  trace("Latence de " + latencePredictive + " ms, retour au débit vidéo configuré");
  exec("v4l2-ctl", SYS.V4L2 + " -c video_bitrate=" + confVideo.BITRATE, function(code) {
  });
  alarmeLatence = false;
 } else if(latencePredictive > SYS.LATENCEDEBUTALARME && !alarmeLatence) {
  trace("Latence de " + latencePredictive + " ms, arrêt des moteurs et passage en débit vidéo réduit");
  exec("v4l2-ctl", SYS.V4L2 + " -c video_bitrate=" + SYS.BITRATEVIDEOFAIBLE, function(code) {
  });
  alarmeLatence = true;
 }

 if(alarmeLatence) {
  for(let i = 0; i < conf.TX.COMMANDES16.length; i++)
   if(hard.MIXAGES16[i].FAILSAFE)
    floatCibles16[i] = conf.TX.COMMANDES16[i].INIT;

  for(let i = 0; i < conf.TX.COMMANDES8.length; i++)
   if(hard.MIXAGES8[i].FAILSAFE)
    floatCibles8[i] = conf.TX.COMMANDES8[i].INIT;

  for(let i = 0; i < conf.TX.COMMANDES1.length; i++)
   if(hard.MIXAGES1[i].FAILSAFE)
    floatCibles1[i] = conf.TX.COMMANDES1[i].INIT;
 }

 for(let i = 0; i < conf.TX.COMMANDES16.length; i++) {
  if(floatCommandes16[i] == floatCibles16[i])
   continue;
  change = true;

  let delta;
  let cible = floatCibles16[i];
  let init = conf.TX.COMMANDES16[i].INIT;

  if(Math.abs(cible - init) <= marges16[i])
   delta = hard.MIXAGES16[i].RAMPINIT;
  else if((cible - init) * (floatCommandes16[i] - init) < 0) {
   delta = hard.MIXAGES16[i].RAMPDOWN;
   cible = init;
  } else if(Math.abs(cible) - Math.abs(floatCommandes16[i]) < 0)
   delta = hard.MIXAGES16[i].RAMPDOWN;
  else
   delta = hard.MIXAGES16[i].RAMPUP;

  if(delta <= 0)
   floatCommandes16[i] = cible;
  else if(floatCommandes16[i] - cible < -delta)
   floatCommandes16[i] += delta;
  else if(floatCommandes16[i] - cible > delta)
   floatCommandes16[i] -= delta;
  else
   floatCommandes16[i] = cible;
 }

 for(let i = 0; i < conf.TX.COMMANDES8.length; i++) {
  if(floatCommandes8[i] == floatCibles8[i])
   continue;
  change = true;

  let delta;
  let cible = floatCibles8[i];
  let init = conf.TX.COMMANDES8[i].INIT;

  if(Math.abs(cible - init) <= marges8[i])
   delta = hard.MIXAGES8[i].RAMPINIT;
  else if((cible - init) * (floatCommandes8[i] - init) < 0) {
   delta = hard.MIXAGES8[i].RAMPDOWN;
   cible = init;
  } else if(Math.abs(cible) - Math.abs(floatCommandes8[i]) < 0)
   delta = hard.MIXAGES8[i].RAMPDOWN;
  else
   delta = hard.MIXAGES8[i].RAMPUP;

  if(delta <= 0)
   floatCommandes8[i] = cible;
  else if(floatCommandes8[i] - cible < -delta)
   floatCommandes8[i] += delta;
  else if(floatCommandes8[i] - cible > delta)
   floatCommandes8[i] -= delta;
  else
   floatCommandes8[i] = cible;
 }

 for(let i = 0; i < conf.TX.COMMANDES1.length; i++) {
  if(floatCommandes1[i] == floatCibles1[i])
   continue;
  change = true;

  let delta;
  if(Math.abs(floatCibles1[i] - conf.TX.COMMANDES1[i].INIT) < 1)
   delta = hard.MIXAGES1[i].RAMPINIT;
  else
   delta = hard.MIXAGES1[i].RAMPUP;

  if(delta <= 0)
   floatCommandes1[i] = floatCibles1[i];
  else if(floatCibles1[i] - floatCommandes1[i] > delta)
   floatCommandes1[i] += delta;
  else if(floatCibles1[i] - floatCommandes1[i] < -delta)
   floatCommandes1[i] -= delta;
  else
   floatCommandes1[i] = floatCibles1[i];
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
  rx.commandesInt16[i] = tx.computeRawCommande16(i, floatCommandes16[i]);
 rx.choixCameras[0] = tx.choixCameras[0];
 for(let i = 0; i < conf.TX.COMMANDES8.length; i++)
  rx.commandesInt8[i] = tx.computeRawCommande8(i, floatCommandes8[i]);
 for(let i = 0; i < conf.TX.COMMANDES1.length / 8; i++) {
  let commande1 = 0;
  for(let j = 0; j < 8; j++)
   if(floatCommandes1[i * 8 + j] >= 0.5)
    commande1 += 1 << j;
  rx.commandes1[i] = commande1;
 }
}

function setRxValeurs() {
 if(hard.DEVGPS && gps.state.lat !== null) {
  rx.setFloatValeur32(0, gps.state.lat);
  rx.setFloatValeur32(1, gps.state.lon);
 }
 rx.setFloatValeur16(0, voltage);
 rx.setFloatValeur16(1, battery);
 rx.setFloatValeur8(0, cpuLoad);
 rx.setFloatValeur8(1, socTemp);
 rx.setFloatValeur8(2, link);
 rx.setFloatValeur8(3, rssi);
 if(hard.DEVGPS) {
  if(typeof gps.state.satsActive !== "undefined")
   rx.setFloatValeur8(4, gps.state.satsActive.length);
  rx.setFloatValeur8(5, gps.state.speed);
  if(gps.state.track !== null)
   rx.setFloatValeur8(6, gps.state.track);
 }
}

setInterval(function() {
 if(up || !init || hard.DEVTELEMETRIE)
  return;

 setRxCommandes();
 setRxValeurs();
 USER.SERVEURS.forEach(function(serveur) {
  sockets[serveur].emit("serveurrobotrx", {
   timestamp: Date.now(),
   data: rx.arrayBuffer
  });
 });
}, SYS.BEACONRATE);

setInterval(function() {
 if(up || !init || !hard.CAPTURESENVEILLE)
  return;

 let date = new Date();

 if(date.getMinutes() % hard.CAPTURESENVEILLE)
  return;

 let overlay = date.toLocaleDateString() + " " + date.toLocaleTimeString();
 if(hard.CAPTURESHDR)
  overlay += " HDR " + hard.CAPTURESHDR;
 let options = "-a 1024 -a '" + overlay + "' -rot " + confVideo.ROTATION;

 if(hard.CAPTURESHDR) {
  EXEC("raspistill -ev " + -hard.CAPTURESHDR + " " + options + " -o /tmp/1.jpg", function(err) {
   if(err) {
    trace("Erreur lors de la capture de la première photo");
    return;
   }
   EXEC("raspistill " + options + " -o /tmp/2.jpg", function(err) {
    if(err) {
     trace("Erreur lors de la capture de la deuxième photo");
     return;
    }
    EXEC("raspistill -ev " + hard.CAPTURESHDR + " " + options + " -o /tmp/3.jpg", function(err) {
     if(err) {
      trace("Erreur lors de la capture de la troisième photo");
      return;
     }
     EXEC("enfuse -o /tmp/out.jpg /tmp/1.jpg /tmp/2.jpg /tmp/3.jpg", function(err) {
      if(err)
       trace("Erreur lors de la fusion des photos");
      else {
       FS.readFile("/tmp/out.jpg", function(err, data) {
        USER.SERVEURS.forEach(function(serveur) {
         trace("Envoi d'une photo sur le serveur " + serveur);
         sockets[serveur].emit("serveurrobotcapturesenveille", data);
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
    trace("Erreur lors de la capture de la photo");
   else {
    FS.readFile("/tmp/out.jpg", function(err, data) {
     USER.SERVEURS.forEach(function(serveur) {
      trace("Envoi d'une photo sur le serveur " + serveur);
      sockets[serveur].emit("serveurrobotcapturesenveille", data);
     });
    });
   }
  });
 }
}, 60000);

NET.createServer(function(socket) {
 const SEPARATEURNALU = new Buffer.from([0, 0, 0, 1]);
 const SPLITTER = new SPLIT(SEPARATEURNALU);

 trace("Le processus de diffusion vidéo H.264 est connecté sur tcp://127.0.0.1:" + SYS.PORTTCPVIDEO);

 SPLITTER.on("data", function(data) {

  if(serveurCourant) {
   sockets[serveurCourant].emit("serveurrobotvideo", {
    timestamp: Date.now(),
    data: data
   });
  }

 }).on("error", function(err) {
  trace("Erreur lors du découpage du flux d'entrée en unités de couche d'abstraction réseau H.264");
 });

 socket.pipe(SPLITTER);

 socket.on("end", function() {
  trace("Le processus de diffusion vidéo H.264 est déconnecté de tcp://127.0.0.1:" + SYS.PORTTCPVIDEO);
 });

}).listen(SYS.PORTTCPVIDEO);

NET.createServer(function(socket) {

 trace("Le processus de diffusion audio est connecté sur tcp://127.0.0.1:" + SYS.PORTTCPAUDIO);

 let array = [];
 let i = 0;
 socket.on("data", function(data) {

  array.push(data);
  i++;

  if(i == 20) {
   if(serveurCourant) {
    sockets[serveurCourant].emit("serveurrobotaudio", {
     timestamp: Date.now(),
     data: Buffer.concat(array)
    });
   }
   array = [];
   i = 0;
  }

 })

 socket.on("end", function() {
  trace("Le processus de diffusion audio est déconnecté de tcp://127.0.0.1:" + SYS.PORTTCPAUDIO);
 });

}).listen(SYS.PORTTCPAUDIO);

process.on("uncaughtException", function(err) {
 let i = 0;
 let erreur = err.stack.split("\n");

 while(i < erreur.length)
  trace(erreur[i++]);

 trace("Suite à cette exception non interceptée, le processus Node.js va être terminé automatiquement");
 setTimeout(function() {
  process.exit(1);
 }, 1000);
})

trace("Client prêt");
