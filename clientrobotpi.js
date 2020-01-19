"use strict";

const CONF = require("/boot/robot.json");

const TRAME = require("./trame.js");

const PORTROBOTS = 8042;
const PORTTCPVIDEO = 8043;
const PORTTCPAUDIO = 8044;

const FICHIERLOG = "/var/log/vigiclient.log";

const INTERFACEWIFI = "wlan0";
const FICHIERSTATS = "/proc/net/wireless";
const STATSRATE = 250;

const PROCESSDIFFUSION = "/usr/local/vigiclient/processdiffusion";
const PROCESSDIFFVIDEO = "/usr/local/vigiclient/processdiffvideo";
const PROCESSDIFFAUDIO = "/usr/local/vigiclient/processdiffaudio";

const CMDINT = RegExp(/^-?\d{1,10}$/);

const CMDDIFFUSION = [
 [
  PROCESSDIFFUSION,
  " /dev/video0",
  " | /bin/nc 127.0.0.1 PORTTCPVIDEO",
  " -w 2"
 ], [
  PROCESSDIFFVIDEO,
  " -loglevel fatal",
  " -f fbdev",
  " -r FPS",
  " -i /dev/fb0",
  " -c:v h264_omx",
  " -profile:v baseline",
  " -b:v BITRATE",
  " -flags:v +global_header",
  " -bsf:v dump_extra",
  " -f rawvideo",
  " tcp://127.0.0.1:PORTTCPVIDEO"
 ]
];

const CMDDIFFAUDIO = [
 PROCESSDIFFAUDIO,
 " -loglevel fatal",
 " -f alsa",
 " -ac 1",
 " -i hw:1,0",
 " -ar 16000",
 " -c:a pcm_s16le",
 " -f s16le",
 " tcp://127.0.0.1:PORTTCPAUDIO"
];

const FRAME0 = "$".charCodeAt();
const FRAME1S = "S".charCodeAt();
const FRAME1T = "T".charCodeAt();
const FRAME1R = "R".charCodeAt();

const V4L2 = "/usr/bin/v4l2-ctl";
const LATENCEFINALARME = 250;
const LATENCEDEBUTALARME = 500;
const BITRATEVIDEOFAIBLE = 100000;
const TXRATE = 50;
const BEACONRATE = 10000;
const CAPTURESENVEILLERATE = 60000;

const SEPARATEURNALU = new Buffer.from([0, 0, 0, 1]);

const CW2015ADDRESS = 0x62;
const CW2015WAKEUP = new Buffer.from([0x0a, 0x00]);
const MAX17043ADDRESS = 0x36;
const BQ27441ADDRESS = 0x55;
const GAUGERATE = 250;

const PCA9685FREQUENCY = 50;
const PIGPIOMOTORFREQUENCY = 100;

const PIGPIO = -1;
const SERVO = 0;
const PCASERVO = 1;
const L9110 = 2;
const L298 = 3;
const PCAL298 = 4;

const OS = require("os");
const FS = require("fs");
const IO = require("socket.io-client");
const EXEC = require("child_process").exec;
const RL = require("readline");
const NET = require("net");
const SPLIT = require("stream-split");
const HTTP = require("http");
const SP = require("serialport");
const GPIO = require("pigpio").Gpio;
const I2C = require("i2c-bus");
const PCA9685 = require("pca9685");

const VERSION = Math.trunc(FS.statSync(__filename).mtimeMs);
const PROCESSTIME = Date.now();
const OSTIME = PROCESSTIME - OS.uptime() * 1000;

let sockets = {};
let serveurCourant = "";

let up = false;
let init = false;
let initVideo = false;
let conf;
let hard;
let tx;
let rx;
let oldCamera;
let confVideo;
let cmdDiffusion;
let cmdDiffAudio;

let lastTimestamp = Date.now();
let latence = 0;
let lastTrame = Date.now();
let alarmeLatence = false;

let oldPositions = [];
let oldVitesses = [];
let moteurs = [];
let oldMoteurs = [];
let rattrapage = [];
let oldTxInterrupteurs = [];

let boostVideo = false;
let oldBoostVideo = false;

let gpiosMoteurs = [];
let gpioInterrupteurs = [];

let serial;

let i2c;
let gaugeType;

let pca9685Driver = [];

if(typeof CONF.CMDDIFFUSION === "undefined")
 CONF.CMDDIFFUSION = CMDDIFFUSION;

if(typeof CONF.CMDDIFFAUDIO === "undefined")
 CONF.CMDDIFFAUDIO = CMDDIFFAUDIO;

CONF.SERVEURS.forEach(function(serveur) {
 sockets[serveur] = IO.connect(serveur, {"connect timeout": 1000, transports: ["websocket"], path: "/" + PORTROBOTS + "/socket.io"});
});

trace("Démarrage du client");

i2c = I2C.openSync(1);

try {
 i2c.i2cWriteSync(CW2015ADDRESS, 2, CW2015WAKEUP);
 gaugeType = "cw2015";
} catch(err) {
 try {
  i2c.readWordSync(MAX17043ADDRESS, 0x02);
  gaugeType = "max17043";
 } catch(err) {
  try {
   i2c.readWordSync(BQ27441ADDRESS, 0x04);
   gaugeType = "bq27441";
  } catch(err) {
   trace("No I2C fuel gauge detected");
   i2c.closeSync();
   gaugeType = "";
  }
 }
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
 let trace = heure(new Date());

 trace += " | " + message;

 FS.appendFile(FICHIERLOG, trace + "\n", function(err) {
 });

 CONF.SERVEURS.forEach(function(serveur) {
  sockets[serveur].emit("serveurrobottrace", message);
 });
}

function traces(id, messages) {
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

function debout() {
 for(let i = 0; i < hard.MOTEURS.length; i++)
  oldMoteurs[i]++;

 for(let i = 0; i < hard.INTERRUPTEURS.length; i++)
  setGpio(i, tx.interrupteurs[0] >> i & 1 ^ hard.INTERRUPTEURS[i].INV);

 if(hard.CAPTURESENVEILLE) {
  sigterm("Raspistill", "raspistill", function(code) {
   diffusion();
  });
 } else
  diffusion();
 diffAudio();

 up = true;
 lastTimestamp = Date.now();
 latence = 0;
}

function dodo() {
 serveurCourant = "";

 for(let i = 0; i < conf.TX.POSITIONS.length; i++)
  tx.positions[i] = (conf.TX.POSITIONS[i] + 180) * 0x10000 / 360;

 for(let i = 0; i < conf.TX.VITESSES.length; i++)
  tx.vitesses[i] = conf.TX.VITESSES[i];

 for(let i = 0; i < hard.MOTEURS.length; i++)
  if(hard.MOTEURS[i].FAILSAFE)
   setConsigneMoteur(i, 0);

 for(let i = 0; i < 8; i++)
  setGpio(i, hard.INTERRUPTEURS[i].INV);

 sigterm("Diffusion", PROCESSDIFFUSION, function(code) {
  sigterm("DiffVideo", PROCESSDIFFVIDEO, function(code) {
  });
 });

 sigterm("DiffAudio", PROCESSDIFFAUDIO, function(code) {
 });

 up = false;
}

function configurationVideo(callback) {
 cmdDiffusion = CONF.CMDDIFFUSION[confVideo.SOURCE].join("").replace("WIDTH", confVideo.WIDTH
                                                           ).replace("HEIGHT", confVideo.HEIGHT
                                                           ).replace(new RegExp("FPS", "g"), confVideo.FPS
                                                           ).replace(new RegExp("BITRATE", "g"), confVideo.BITRATE
                                                           ).replace("ROTATION", confVideo.ROTATION
                                                           ).replace("PORTTCPVIDEO", PORTTCPVIDEO);
 cmdDiffAudio = CONF.CMDDIFFAUDIO.join("").replace("PORTTCPAUDIO", PORTTCPAUDIO);

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

 exec("v4l2-ctl", V4L2 + " -v width=" + confVideo.WIDTH +
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

CONF.SERVEURS.forEach(function(serveur, index) {

 sockets[serveur].on("connect", function() {
  trace("Connecté sur " + serveur + "/" + PORTROBOTS);
  EXEC("hostname -I").stdout.on("data", function(ipPriv) {
   EXEC("iwgetid -r || echo $?").stdout.on("data", function(ssid) {
    sockets[serveur].emit("serveurrobotlogin", {
     conf: CONF,
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

   conf = data.conf;
   hard = data.hard;

   // Security hardening: even if already done on server side,
   // always filter values integrated in command lines
   for(let i = 0; i < hard.CAMERAS.length; i++) {
    if(!(CMDINT.test(hard.CAMERAS[i].SOURCE) &&
         CMDINT.test(hard.CAMERAS[i].WIDTH) &&
         CMDINT.test(hard.CAMERAS[i].HEIGHT) &&
         CMDINT.test(hard.CAMERAS[i].FPS) &&
         CMDINT.test(hard.CAMERAS[i].BITRATE) &&
         CMDINT.test(hard.CAMERAS[i].ROTATION) &&
         CMDINT.test(hard.CAMERAS[i].LUMINOSITE) &&
         CMDINT.test(hard.CAMERAS[i].CONTRASTE) &&
         CMDINT.test(hard.CAMERAS[i].BOOSTVIDEOLUMINOSITE) &&
         CMDINT.test(hard.CAMERAS[i].BOOSTVIDEOCONTRASTE)))
     return;
   }

   tx = new TRAME.Tx(conf.TX);
   rx = new TRAME.Rx(conf.TX, conf.RX);


   for(let i = 0; i < conf.TX.POSITIONS.length; i++)
    oldPositions[i] = tx.positions[i] + 1;

   for(let i = 0; i < conf.TX.VITESSES.length; i++)
    oldVitesses[i] = tx.vitesses[i] + 1;

   for(let i = 0; i < hard.MOTEURS.length; i++) {
    oldMoteurs[i] = 0;
    rattrapage[i] = 0;
   }

   for(let i = 0; i < conf.TX.INTERRUPTEURS.length; i++)
    oldTxInterrupteurs[i] = conf.TX.INTERRUPTEURS[i] + 1;

   oldCamera = conf.COMMANDES[conf.DEFAUTCOMMANDE].CAMERA;
   confVideo = hard.CAMERAS[oldCamera];
   boostVideo = false;
   oldBoostVideo = false;

   gpiosMoteurs.forEach(function(gpios) {
    gpios.forEach(function(gpio) {
     gpio.mode(GPIO.INPUT);
    });
   });

   gpioInterrupteurs.forEach(function(gpio) {
    gpio.mode(GPIO.INPUT);
   });

   pca9685Driver = [];
   gpiosMoteurs = [];
   gpioInterrupteurs = [];

   for(let i = 0; i < hard.PCA9685ADDRESSES.length; i++) {
    pca9685Driver[i] = new PCA9685.Pca9685Driver({
     i2c: i2c,
     address: hard.PCA9685ADDRESSES[i],
     frequency: PCA9685FREQUENCY
    }, function(err) {
     if(err)
      trace("Error initializing PCA9685 at address " + hard.PCA9685ADDRESSES[i]);
     else
      trace("PCA9685 initialized at address " + hard.PCA9685ADDRESSES[i]);
    });
   }

   for(let i = 0; i < hard.MOTEURS.length; i++) {
    if(hard.MOTEURS[i].ADRESSE < 0) {
     gpiosMoteurs[i] = [];
     for(let j = 0; j < hard.MOTEURS[i].PINS.length; j++)
      gpiosMoteurs[i][j] =  new GPIO(hard.MOTEURS[i].PINS[j], {mode: GPIO.OUTPUT});
     setMotorFrequency(i);
    }
   }

   for(let i = 0; i < 8; i++) {
    if(hard.INTERRUPTEURS[i].PIN >= 0) {
     if(hard.INTERRUPTEURS[i].PCA9685 == PIGPIO)
      gpioInterrupteurs[i] = new GPIO(hard.INTERRUPTEURS[i].PIN, {mode: GPIO.OUTPUT});
     setGpio(i, hard.INTERRUPTEURS[i].INV);
    }
   }

   setTimeout(function() {
    if(up) {
     sigterm("Diffusion", PROCESSDIFFUSION, function(code) {
      sigterm("DiffVideo", PROCESSDIFFVIDEO, function(code) {
       configurationVideo(function(code) {
        diffusion();
       });
      });
     });
    } else
     configurationVideo(function(code) {
      initVideo = true;
     });
   }, 100);

   if(!init) {
    serial = new SP(hard.DEVROBOT, {
     baudRate: hard.DEVDEBIT,
     lock: false
    });

    serial.on("open", function() {
     trace("Connecté sur " + hard.DEVROBOT);

     serial.on("data", function(data) {
      if(hard.DEVTELEMETRIE) {
       CONF.SERVEURS.forEach(function(serveur) {
        if(serveurCourant && serveur != serveurCourant)
         return;

        sockets[serveur].emit("serveurrobotrx", {
         timestamp: Date.now(),
         data: data
        });
       });
      }
     });

     init = true;
    });
   }
  });
 }

 sockets[serveur].on("disconnect", function() {
  trace("Déconnecté de " + serveur + "/" + PORTROBOTS);

  if(serveur != serveurCourant)
   return;

  dodo();
 });

 sockets[serveur].on("connect_error", function(err) {
  //trace("Erreur de connexion au serveur " + serveur + "/" + PORTROBOTS);
 });

 sockets[serveur].on("clientsrobotdebout", function() {
  if(!init) {
   trace("Ce robot n'est pas initialisé");
   sockets[serveur].emit("serveurrobotdebout", false);
   return;
  }

  if(!initVideo) {
   trace("La vidéo n'est pas initialisée");
   sockets[serveur].emit("serveurrobotdebout", false);
   return;
  }

  if(serveurCourant) {
   trace("Ce robot est déjà utilisé depuis le serveur " + serveurCourant);
   sockets[serveur].emit("serveurrobotdebout", false);
   return;
  }
  serveurCourant = serveur;

  debout();

  sockets[serveur].emit("serveurrobotdebout", true);
 });

 sockets[serveur].on("clientsrobotdodo", function() {
  if(serveur != serveurCourant)
   return;

  dodo();
 });

 sockets[serveur].on("clientsrobottts", function(data) {
  FS.writeFile("/tmp/tts.txt", data, function(err) {
   if(err)
    trace(err);
   exec("eSpeak", "/usr/bin/espeak -v fr -f /tmp/tts.txt --stdout > /tmp/tts.wav", function(code) {
    exec("Aplay", "/usr/bin/aplay -D plughw:" + hard.PLAYBACKDEVICE + " /tmp/tts.wav", function(code) {
    });
   });
  });
 });

 sockets[serveur].on("clientsrobotexit", function() {
  trace("Redémarrage du robot");
  dodo();
  setTimeout(function() {
   EXEC("reboot");
  }, 1000);
 });

 sockets[serveur].on("echo", function(data) {
  sockets[serveur].emit("echo", {
   serveur: data,
   client: Date.now()
  });
 });

 sockets[serveur].on("clientsrobottx", function(data) {
  if(serveur != serveurCourant)
   return;

  let now = Date.now();

  lastTimestamp = data.boucleVideoCommande;
  latence = now - data.boucleVideoCommande;

  if(data.data[0] != FRAME0 ||
     data.data[1] != FRAME1S) {
   if(data.data[1] == FRAME1T) {
    trace("Réception d'une trame texte");
    serial.write(data.data);
   } else
    trace("Réception d'une trame corrompue");
   return;
  }

  if(now - lastTrame < TXRATE / 2)
   return;

  lastTrame = now;

  for(let i = 0; i < tx.byteLength; i++)
   tx.bytes[i] = data.data[i];

  if(latence > LATENCEDEBUTALARME) {
   //trace("Réception d'une trame avec trop de latence");
   for(let i = 0; i < conf.TX.VITESSES.length; i++)
    tx.vitesses[i] = 0;
  } else
   serial.write(data.data);

  let camera = tx.choixCameras[0];
  if(camera != oldCamera) {
   confVideo = hard.CAMERAS[camera];
   if(up) {
    sigterm("Diffusion", PROCESSDIFFUSION, function(code) {
     sigterm("DiffVideo", PROCESSDIFFVIDEO, function(code) {
      configurationVideo(function(code) {
       diffusion();
      });
     });
    });
   } else
    configurationVideo(function(code) {
    });
   oldCamera = camera;
  }

  for(let i = 0; i < hard.MOTEURS.length; i++)
   setConsigneMoteur(i, 1);

  for(let i = 0; i < conf.TX.INTERRUPTEURS.length; i++) {
   if(tx.interrupteurs[i] != oldTxInterrupteurs[i]) {
    for(let j = 0; j < 8; j++) {
     let etat = tx.interrupteurs[i] >> j & 1 ^ hard.INTERRUPTEURS[i].INV;
     setGpio(i * 8 + j, etat);
     if(i * 8 + j == hard.INTERRUPTEURBOOSTVIDEO)
      boostVideo = etat;
     }
    oldTxInterrupteurs[i] = tx.interrupteurs[i];
   }
  }

  if(boostVideo != oldBoostVideo) {
   if(boostVideo) {
    exec("v4l2-ctl", V4L2 + " -c brightness=" + confVideo.BOOSTVIDEOLUMINOSITE +
                               ",contrast=" + confVideo.BOOSTVIDEOCONTRASTE, function(code) {
    });
   } else {
    exec("v4l2-ctl", V4L2 + " -c brightness=" + confVideo.LUMINOSITE +
                               ",contrast=" + confVideo.CONTRASTE, function(code) {
    });
   }
   oldBoostVideo = boostVideo;
  }

  if(!hard.DEVTELEMETRIE) {
   rx.sync[1] = FRAME1R;
   for(let i = 0; i < conf.TX.POSITIONS.length; i++)
    rx.positions[i] = tx.positions[i];
   rx.choixCameras[0] = tx.choixCameras[0];
   for(let i = 0; i < conf.TX.VITESSES.length; i++)
    rx.vitesses[i] = tx.vitesses[i];
   rx.interrupteurs[0] = tx.interrupteurs[0];

   sockets[serveur].emit("serveurrobotrx", {
    timestamp: now,
    data: rx.arrayBuffer
   });
  }
 });

});

function setPca9685Gpio(pcaId, pin, state) {
 if(state)
  pca9685Driver[pcaId].channelOn(pin);
 else
  pca9685Driver[pcaId].channelOff(pin);
}

function setGpio(n, etat) {
 if(hard.INTERRUPTEURS[n].PIN >= 0) {
  if(hard.INTERRUPTEURS[n].PCA9685 == PIGPIO) {
   if(hard.INTERRUPTEURS[n].MODE == 1 && !etat || // Drain ouvert
      hard.INTERRUPTEURS[n].MODE == 2 && etat)    // Collecteur ouvert
    gpioInterrupteurs[n].mode(GPIO.INPUT);
   else
    gpioInterrupteurs[n].digitalWrite(etat);
  } else
   setPca9685Gpio(hard.INTERRUPTEURS[n].PCA9685, hard.INTERRUPTEURS[n].PIN, etat);
 }
}

function computePwm(n, velocity, min, max) {
 let pwm;
 let pwmNeutre = (min + max) / 2 + hard.MOTEURS[n].OFFSET;

 if(velocity < 0)
  pwm = map(velocity, -hard.MOTEURS[n].COURSE * 0x8000 / 360, 0, min, pwmNeutre + hard.MOTEURS[n].NEUTREAR);
 else if(velocity > 0)
  pwm = map(velocity, 0, hard.MOTEURS[n].COURSE * 0x8000 / 360, pwmNeutre + hard.MOTEURS[n].NEUTREAV, max);
 else
  pwm = pwmNeutre;

 return pwm;
}

function setMotorFrequency(n) {
 switch(hard.MOTEURS[n].TYPE) {
  case L9110:
   gpiosMoteurs[n][0].pwmFrequency(PIGPIOMOTORFREQUENCY);
   gpiosMoteurs[n][1].pwmFrequency(PIGPIOMOTORFREQUENCY);
   break;
  case L298:
   gpiosMoteurs[n][0].pwmFrequency(PIGPIOMOTORFREQUENCY);
   break;
 }
}


function setConsigneMoteur(n, rattrape) {
 oldMoteurs[n] = moteurs[n];
 moteurs[n] = 0;

 for(let i = 0; i < conf.TX.POSITIONS.length; i++)
  moteurs[n] += (tx.positions[i] - 0x8000) * hard.MIXAGES[n][i];

 for(let i = 0; i < conf.TX.VITESSES.length; i++)
  moteurs[n] += tx.vitesses[i] * hard.MIXAGES[n][i + conf.TX.POSITIONS.length] * 0x100;

 if(moteurs[n] != oldMoteurs[n]) {
  if(rattrape) {
   if(moteurs[n] < oldMoteurs[n])
    rattrapage[n] = -hard.MOTEURS[n].RATTRAPAGE * 0x10000 / 360;
   if(moteurs[n] > oldMoteurs[n])
    rattrapage[n] = hard.MOTEURS[n].RATTRAPAGE * 0x10000 / 360;
  } else
   rattrapage[n] = 0;
  let consigne = constrain(moteurs[n] + rattrapage[n] + hard.MOTEURS[n].OFFSET * 0x10000 / 360, -hard.MOTEURS[n].COURSE * 0x8000 / 360,
                                                                                                 hard.MOTEURS[n].COURSE * 0x8000 / 360);
  setMoteur(n, consigne);
 }
}

function setMoteur(n, velocity) {
 switch(hard.MOTEURS[n].TYPE) {
  case PCASERVO:
   pca9685Driver[hard.MOTEURS[n].PCA9685].setPulseLength(hard.MOTEURS[n].PIN, computePwm(n, velocity, hard.MOTEURS[n].PWMMIN, hard.MOTEURS[n].PWMMAX));
   break;
  case SERVO:
   gpiosMoteurs[n][0].servoWrite(computePwm(n, velocity, hard.MOTEURS[n].PWMMIN, hard.MOTEURS[n].PWMMAX));
   break;
  case L9110:
   l9110MotorDrive(n, computePwm(n, velocity, -255, 255));
   break;
  case L298:
   l298MotorDrive(n, computePwm(n, velocity, -255, 255));
   break;
  case PCAL298:
   pca9685MotorDrive(n, computePwm(n, velocity, -100, 100));
   break;
 }
}

function l298MotorDrive(n, velocity) {
 let pwm;

 if(velocity < 0) {
  gpiosMoteurs[n][1].digitalWrite(false);
  gpiosMoteurs[n][2].digitalWrite(true);
  pwm = -velocity;
 } else if(velocity > 0) {
  gpiosMoteurs[n][1].digitalWrite(true);
  gpiosMoteurs[n][2].digitalWrite(false);
  pwm = velocity;
 } else {
  gpiosMoteurs[n][1].digitalWrite(false);
  gpiosMoteurs[n][2].digitalWrite(false);
  pwm = 0;
 }

 gpiosMoteurs[n][0].pwmWrite(pwm);
}

function l9110MotorDrive(n, velocity) {
 if(velocity < 0) {
  gpiosMoteurs[n][0].digitalWrite(false);
  gpiosMoteurs[n][1].pwmWrite(-velocity);
 } else if(velocity > 0) {
  gpiosMoteurs[n][0].pwmWrite(velocity);
  gpiosMoteurs[n][1].digitalWrite(false);
 } else {
  gpiosMoteurs[n][0].digitalWrite(false);
  gpiosMoteurs[n][1].digitalWrite(false);
 }
}

function pca9685MotorDrive(n, velocity) {
 let pcaId = hard.MOTEURS[n].PCA9685;
 let chIn1 = hard.MOTEURS[n].PINS[1];
 let chIn2 = hard.MOTEURS[n].PINS[2];
 let pwm;

 if(velocity < 0) {
  pca9685Driver[pcaId].channelOff(chIn1);
  pca9685Driver[pcaId].channelOn(chIn2);
  pwm = -velocity / 100;
 } else if(velocity > 0) {
  pca9685Driver[pcaId].channelOn(chIn1);
  pca9685Driver[pcaId].channelOff(chIn2);
  pwm = velocity / 100;
 } else {
  pca9685Driver[pcaId].channelOff(chIn1);
  pca9685Driver[pcaId].channelOff(chIn2);
  pwm = 0;
 }

 pca9685Driver[pcaId].setDutyCycle(hard.MOTEURS[n].PINS[0], pwm);
}

function failSafe() {
 trace("Arrêt des moteurs");

 for(let i = 0; i < conf.TX.VITESSES.length; i++)
  tx.vitesses[i] = conf.TX.VITESSES[i];

 for(let i = 0; i < hard.MOTEURS.length; i++)
  if(hard.MOTEURS[i].FAILSAFE)
   setConsigneMoteur(i, 0);
}

setInterval(function() {
 if(!up || !init)
  return;

 let latencePredictive = Math.max(latence, Date.now() - lastTimestamp);

 if(latencePredictive < LATENCEFINALARME && alarmeLatence) {
  trace("Latence de " + latencePredictive + " ms, retour au débit vidéo configuré");
  exec("v4l2-ctl", V4L2 + " -c video_bitrate=" + confVideo.BITRATE, function(code) {
  });
  alarmeLatence = false;
 } else if(latencePredictive > LATENCEDEBUTALARME && !alarmeLatence) {
  failSafe();
  trace("Latence de " + latencePredictive + " ms, passage en débit vidéo réduit");
  exec("v4l2-ctl", V4L2 + " -c video_bitrate=" + BITRATEVIDEOFAIBLE, function(code) {
  });
  alarmeLatence = true;
 }
}, TXRATE);

function swapWord(word) {
 return (word & 0xff) << 8 | word >> 8;
}

if(gaugeType == "cw2015") {
 setInterval(function() {
  if(!init)
   return;

  i2c.readWord(CW2015ADDRESS, 0x02, function(err, microVolts305) {
   rx.setValeur16(0, swapWord(microVolts305) * 305 / 1000000);
   i2c.readWord(CW2015ADDRESS, 0x04, function(err, pour25600) {
    rx.setValeur16(1, swapWord(pour25600) / 256);
   });
  });
 }, GAUGERATE);
}

if(gaugeType == "max17043") {
 setInterval(function() {
  if(!init)
   return;

  i2c.readWord(MAX17043ADDRESS, 0x02, function(err, volts12800) {
   rx.setValeur16(0, swapWord(volts12800) / 12800);
   i2c.readWord(MAX17043ADDRESS, 0x04, function(err, pour25600) {
    rx.setValeur16(1, swapWord(pour25600) / 256);
   });
  });
 }, GAUGERATE);
}

if(gaugeType == "bq27441") {
 setInterval(function() {
  if(!init)
   return;

  i2c.readWord(BQ27441ADDRESS, 0x04, function(err, milliVolts) {
   rx.setValeur16(0, milliVolts / 1000);
   i2c.readByte(BQ27441ADDRESS, 0x1c, function(err, pourcents) {
    rx.setValeur16(1, pourcents);
   });
  });
 }, GAUGERATE);
}

setInterval(function() {
 if(!init)
  return;

 const STATS = RL.createInterface(FS.createReadStream(FICHIERSTATS));

 STATS.on("line", function(ligne) {
  ligne = ligne.split(/\s+/);

  if(ligne[1] == INTERFACEWIFI + ":") {
   rx.setValeur8(0, ligne[3]);
   rx.setValeur8(1, ligne[4]);
  }
 });
}, STATSRATE);

setInterval(function() {
 if(up || !init || hard.DEVTELEMETRIE)
  return;

 CONF.SERVEURS.forEach(function(serveur) {
  sockets[serveur].emit("serveurrobotrx", {
   timestamp: Date.now(),
   data: rx.arrayBuffer
  });
 });
}, BEACONRATE);

setInterval(function() {
 if(up || !init || !initVideo || !hard.CAPTURESENVEILLE)
  return;

 let date = new Date();
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
        CONF.SERVEURS.forEach(function(serveur) {
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
     CONF.SERVEURS.forEach(function(serveur) {
      trace("Envoi d'une photo sur le serveur " + serveur);
      sockets[serveur].emit("serveurrobotcapturesenveille", data);
     });
    });
   }
  });
 }
}, CAPTURESENVEILLERATE);

NET.createServer(function(socket) {
 const SPLITTER = new SPLIT(SEPARATEURNALU);

 trace("Le processus de diffusion vidéo H.264 est connecté sur tcp://127.0.0.1:" + PORTTCPVIDEO);

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
  trace("Le processus de diffusion vidéo H.264 est déconnecté de tcp://127.0.0.1:" + PORTTCPVIDEO);
 });

}).listen(PORTTCPVIDEO);

NET.createServer(function(socket) {

 trace("Le processus de diffusion audio est connecté sur tcp://127.0.0.1:" + PORTTCPAUDIO);

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
  trace("Le processus de diffusion audio est déconnecté de tcp://127.0.0.1:" + PORTTCPAUDIO);
 });

}).listen(PORTTCPAUDIO);

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
