"use strict";

const CONF = require("/boot/robot.json");

const TRAME = require("./trame.js");

const PORTROBOTS = 86;
const PORTTCPVIDEO = 8003;
const PORTTCPAUDIO = 8004;

const FICHIERLOG = "/var/log/vigiclient.log";

const INTERFACEWIFI = "wlan0";
const FICHIERSTATS = "/proc/net/wireless";
const STATSRATE = 250;

const PROCESSDIFFUSION = "/usr/local/vigiclient/processdiffusion";
const PROCESSDIFFAUDIO = "/usr/local/vigiclient/processdiffaudio";

const CMDDIFFUSION = [
 PROCESSDIFFUSION,
 " SOURCEVIDEO",
 " | /bin/nc 127.0.0.1 PORTTCPVIDEO",
 " -w 2"
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

const SEPARATEURNALU = new Buffer.from([0, 0, 0, 1]);

const CW2015ADDRESS = 0x62;
const CW2015WAKEUP = new Buffer.from([0x0a, 0x00]);
const MAX17043ADDRESS = 0x10;
const BQ27441ADDRESS = 0x55;
const GAUGERATE = 250;

const PCA9685FREQUENCY = 50;

const CAPTURESENVEILLERATE = 60000;

const OS = require("os");
const FS = require("fs");
const IO = require("socket.io-client");
const EXEC = require("child_process").exec;
const RL = require("readline");
const NET = require("net");
const SPLIT = require("stream-split");
const HTTP = require("http");
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
let confStatique;
let confDynamique;
let oldIdConfStatique;
let oldIdConfDynamique;
let cmdDiffusion;
let cmdDiffAudio;

let lastTimestamp = Date.now();
let latence = 0;
let lastTrame = Date.now();
let alarmeLatence = false;

let oldOutils = [];
let oldMoteurs = [];
let rattrapage = [];
let oldTxInterrupteurs;

let gpioOutils = [];
let gpioMoteurs = [];
let gpioMoteursA = [];
let gpioMoteursB = [];
let gpioInterrupteurs = [];

let i2c;
let cw2015;
let max17043;
let bq27441;
let gaugeBuffer = new Buffer.alloc(256);

let pca9685Driver = [];

if(typeof CONF.CMDDIFFUSION === "undefined")
 CONF.CMDDIFFUSION = CMDDIFFUSION;

if(typeof CONF.CMDDIFFAUDIO === "undefined")
 CONF.CMDDIFFAUDIO = CMDDIFFAUDIO;

CONF.SERVEURS.forEach(function(serveur) {
 sockets[serveur] = IO.connect(serveur, {"connect timeout": 1000, transports: ["websocket"], path: "/" + PORTROBOTS + "/socket.io"});
});

trace("Initialisation I2C");

i2c = I2C.openSync(1);

try {
 i2c.i2cWriteSync(CW2015ADDRESS, 2, CW2015WAKEUP);
 cw2015 = true;
 max17043 = false;
 bq27441 = false;
} catch(err) {
 try {
  i2c.i2cReadSync(MAX17043ADDRESS, 6, gaugeBuffer);
  cw2015 = false;
  max17043 = true;
  bq27441 = false;
 } catch(err) {
  try {
   i2c.i2cReadSync(BQ27441ADDRESS, 29, gaugeBuffer);
   cw2015 = false;
   max17043 = false;
   bq27441 = true;
  } catch(err) {
   trace("I2C désactivé");
   i2c.closeSync();
   cw2015 = false;
   max17043 = false;
   bq27441 = false;
  }
 }
}

trace("Démarrage du client");

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

function notification(serveur, message, type) {
 sockets[serveur].emit("serveurrobotnotification", {
  message: message,
  type: type
 });
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
 for(let i = 0; i < conf.TX.OUTILS.length; i++)
  oldOutils[i]++;

 for(let i = 0; i < 8; i++)
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

 for(let i = 0; i < hard.OUTILS.length; i++)
  setOutil(i, 0);

 for(let i = 0; i < hard.MOTEURS.length; i++)
  setMotor(i, 0);

 for(let i = 0; i < 8; i++)
  setGpio(i, hard.INTERRUPTEURS[i].INV);

 sigterm("Diffusion", PROCESSDIFFUSION, function(code) {
 });

 sigterm("DiffAudio", PROCESSDIFFAUDIO, function(code) {
 });

 up = false;
}

function confVideo(callback) {
 cmdDiffusion = CONF.CMDDIFFUSION.join("").replace("SOURCEVIDEO", confStatique.SOURCE
                                         ).replace("PORTTCPVIDEO", PORTTCPVIDEO
                                         ).replace("ROTATIONVIDEO", confDynamique.ROTATION
                                         ).replace(new RegExp("BITRATEVIDEO", "g"), confDynamique.BITRATE);
 cmdDiffAudio = CONF.CMDDIFFAUDIO.join("").replace("PORTTCPAUDIO", PORTTCPAUDIO);

 trace("Initialisation de la configuration Video4Linux");

 exec("v4l2-ctl", V4L2 + " -v width=" + confStatique.WIDTH +
                            ",height=" + confStatique.HEIGHT +
                            ",pixelformat=4" +
                         " -p " + confStatique.FPS +
                         " -c h264_profile=0" +
                            ",repeat_sequence_header=1" +
                            ",rotate=" + confDynamique.ROTATION +
                            ",video_bitrate=" + confDynamique.BITRATE +
                            ",brightness=" + confDynamique.LUMINOSITE +
                            ",contrast=" + confDynamique.CONTRASTE, function(code) {
  callback(code);
 });
}

function confDynamiqueVideo() {
 trace("Modification de la configuration Video4Linux");

 exec("v4l2-ctl", V4L2 + " -c h264_profile=0" +
                            ",repeat_sequence_header=1" +
                            ",rotate=" + confDynamique.ROTATION +
                            ",video_bitrate=" + confDynamique.BITRATE +
                            ",brightness=" + confDynamique.LUMINOSITE +
                            ",contrast=" + confDynamique.CONTRASTE, function(code) {
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

CONF.SERVEURS.forEach(function(serveur) {

 sockets[serveur].on("connect", function() {
  trace("Connecté sur " + serveur + "/" + PORTROBOTS);
  sockets[serveur].emit("serveurrobotlogin", {
   conf: CONF,
   version: VERSION,
   processTime: PROCESSTIME,
   osTime: OSTIME
  });
 });

 sockets[serveur].on("clientsrobotconf", function(data) {
  trace("Réception des données de configuration du robot depuis le serveur " + serveur);

  conf = data.conf;
  hard = data.hard;

  tx = new TRAME.Tx(conf.TX);
  rx = new TRAME.Rx(conf.TX, conf.RX);

  for(let i = 0; i < conf.TX.OUTILS.length; i++) {
   oldOutils[i] = tx.outils[i];
   rattrapage[i] = 0;
  }

  for(let i = 0; i < hard.MOTEURS.length; i++)
   oldMoteurs[i] = 0;

  oldTxInterrupteurs = conf.TX.INTERRUPTEURS[0];

  oldCamera = conf.COMMANDES[conf.DEFAUTCOMMANDE].CAMERA;
  oldIdConfStatique = conf.CAMERAS[oldCamera].CONFSTATIQUE;
  oldIdConfDynamique = conf.CAMERAS[oldCamera].CONFDYNAMIQUE;
  confStatique = conf.CONFSSTATIQUE[oldIdConfStatique];
  confDynamique = conf.CONFSDYNAMIQUE[oldIdConfDynamique];

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

  gpioOutils.forEach(function(gpio) {
   gpio.mode(GPIO.INPUT);
  });

  gpioMoteurs.forEach(function(gpio) {
   gpio.mode(GPIO.INPUT);
  });

  gpioMoteursA.forEach(function(gpio) {
   gpio.mode(GPIO.INPUT);
  });

  gpioMoteursB.forEach(function(gpio) {
   gpio.mode(GPIO.INPUT);
  });

  gpioInterrupteurs.forEach(function(gpio) {
   gpio.mode(GPIO.INPUT);
  });

  gpioOutils = [];
  gpioMoteurs = [];
  gpioMoteursA = [];
  gpioMoteursB = [];
  gpioInterrupteurs = [];

  for(let i = 0; i < hard.OUTILS.length; i++)
   if(hard.OUTILS[i].PCA9685 == -1)
    gpioOutils[i] = new GPIO(hard.OUTILS[i].PIN, {mode: GPIO.OUTPUT});

  for(let i = 0; i < hard.MOTEURS.length; i++) {
   if(hard.MOTEURS[i].PCA9685 < 0) {
    if(hard.MOTEURS[i].PIN >= 0)
     gpioMoteurs[i] = new GPIO(hard.MOTEURS[i].PIN, {mode: GPIO.OUTPUT});
    if(hard.MOTEURS[i].PINA >= 0)
     gpioMoteursA[i] = new GPIO(hard.MOTEURS[i].PINA, {mode: GPIO.OUTPUT});
    if(hard.MOTEURS[i].PINB >= 0)
     gpioMoteursB[i] = new GPIO(hard.MOTEURS[i].PINB, {mode: GPIO.OUTPUT});
   }
  }

  for(let i = 0; i < 8; i++) {
   if(hard.INTERRUPTEURS[i].PCA9685 == -1)
    gpioInterrupteurs[i] = new GPIO(hard.INTERRUPTEURS[i].PIN, {mode: GPIO.OUTPUT});
   setGpio(i, hard.INTERRUPTEURS[i].INV);
  }

  setTimeout(function() {
   confVideo(function(code) {
    initVideo = true;
   });
  }, 100);

  init = true;
 });

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
   notification(serveur, "Ce robot n'est pas initialisé", "error");
   sockets[serveur].emit("serveurrobotdebout", false);
   return;
  }

  if(!initVideo) {
   trace("La vidéo n'est pas initialisée");
   notification(serveur, "La vidéo n'est pas initialisée", "error");
   sockets[serveur].emit("serveurrobotdebout", false);
   return;
  }

  if(serveurCourant) {
   trace("Ce robot est déjà utilisé depuis le serveur " + serveurCourant);
   notification(serveur, "Utilisé par " + serveurCourant, "error");
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
   if(data.data[1] == FRAME1T)
    trace("Réception d'une trame texte");
   else
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
  }

  let camera = tx.choixCameras[0];
  if(camera != oldCamera) {
   let idConfStatique = conf.CAMERAS[camera].CONFSTATIQUE;
   let idConfDynamique = conf.CAMERAS[camera].CONFDYNAMIQUE;

   if(idConfStatique != oldIdConfStatique) {
    confStatique = conf.CONFSSTATIQUE[idConfStatique];
    confDynamique = conf.CONFSDYNAMIQUE[idConfDynamique];
    if(up) {
     sigterm("Diffusion", PROCESSDIFFUSION, function(code) {
      confVideo(function(code) {
       diffusion();
      });
     });
    } else {
     confVideo(function(code) {
     });
    }
   } else if(idConfDynamique != oldIdConfDynamique) {
    confDynamique = conf.CONFSDYNAMIQUE[idConfDynamique];
    confDynamiqueVideo();
   }

   oldIdConfStatique = idConfStatique;
   oldIdConfDynamique = idConfDynamique;
   oldCamera = camera;
  }

  if(tx.outils.length == hard.OUTILS.length) {
   let outils = [];

   for(let i = 0; i < hard.OUTILS.length; i++) {
    if(tx.outils[i] == oldOutils[i])
     continue;
    else if(tx.outils[i] < oldOutils[i])
     rattrapage[i] = -hard.OUTILS[i].RATTRAPAGE * 0x10000 / 360;
    else if(tx.outils[i] > oldOutils[i])
     rattrapage[i] = hard.OUTILS[i].RATTRAPAGE * 0x10000 / 360;
    oldOutils[i] = tx.outils[i];

    outils[i] = constrain(tx.outils[i] + rattrapage[i] + hard.OUTILS[i].ANGLEOFFSET * 0x10000 / 360, (-hard.OUTILS[i].COURSE / 2 + 180) * 0x10000 / 360,
                                                                                                     (hard.OUTILS[i].COURSE / 2 + 180) * 0x10000 / 360);

    let pwm = map(outils[i], (-hard.OUTILS[i].COURSE / 2 + 180) * 0x10000 / 360,
                             (hard.OUTILS[i].COURSE / 2 + 180) * 0x10000 / 360, hard.OUTILS[i].PWMMIN, hard.OUTILS[i].PWMMAX);

    setOutil(i, pwm);
   }
  }

  let moteurs = [];

  for(let i = 0; i < hard.MIXAGESMOTEURS.length; i++)
   moteurs[i] = constrain(tx.vitesses[0] * hard.MIXAGESMOTEURS[i][0] +
                          tx.vitesses[1] * hard.MIXAGESMOTEURS[i][1] +
                          tx.vitesses[2] * hard.MIXAGESMOTEURS[i][2], -0x80, 0x80);

  for(let i = 0; i < hard.MOTEURS.length; i++) {
   if(moteurs[i] == oldMoteurs[i])
    continue;
   oldMoteurs[i] = moteurs[i];
   setMotor(i, moteurs[i]);
  }

  if(tx.interrupteurs[0] != oldTxInterrupteurs) {
   for(let i = 0; i < 8; i++)
    setGpio(i, tx.interrupteurs[0] >> i & 1 ^ hard.INTERRUPTEURS[i].INV);
   oldTxInterrupteurs = tx.interrupteurs[0]
  }

  rx.sync[1] = FRAME1R;
  for(let i = 0; i < conf.TX.OUTILS.length; i++)
   rx.outils[i] = tx.outils[i];
  rx.choixCameras[0] = tx.choixCameras[0];
  for(let i = 0; i < conf.TX.VITESSES.length; i++)
   rx.vitesses[i] = tx.vitesses[i];
  rx.interrupteurs[0] = tx.interrupteurs[0];

  sockets[serveur].emit("serveurrobotrx", {
   timestamp: now,
   data: rx.arrayBuffer
  });
 });

});

function setPca9685Gpio(pcaId, pin, value) {
 if(value)
  pca9685Driver[pcaId].channelOn(pin);
 else
  pca9685Driver[pcaId].channelOff(pin);
}

function setGpio(n, etat) {
 if(hard.INTERRUPTEURS[n].PCA9685 == -1) {
  if(hard.INTERRUPTEURS[n].MODE == 1 && !etat || // Drain ouvert
     hard.INTERRUPTEURS[n].MODE == 2 && etat)    // Collecteur ouvert
   gpioInterrupteurs[n].mode(GPIO.INPUT);
  else
   gpioInterrupteurs[n].digitalWrite(etat);
 } else
  setPca9685Gpio(hard.INTERRUPTEURS[n].PCA9685, hard.INTERRUPTEURS[n].PIN, etat);
}

function setOutil(n, pwm) {
 if(hard.OUTILS[n].PCA9685 == -1)
  gpioOutils[n].servoWrite(pwm);
 else
  pca9685Driver[hard.OUTILS[n].PCA9685].setPulseLength(hard.OUTILS[n].PIN, pwm);
}

function setMotor(n, value) {
 let pwm;
 let pwmNeutre = (hard.MOTEURS[n].PWMMAX + hard.MOTEURS[n].PWMMIN) / 2;

 if(value < 0)
  pwm = map(value + hard.MOTEURS[n].NEUTREAR, -0x80 + hard.MOTEURS[n].NEUTREAR, 0, hard.MOTEURS[n].PWMMIN, pwmNeutre);
 else if(value > 0)
  pwm = map(value + hard.MOTEURS[n].NEUTREAV, 0, 0x80 + hard.MOTEURS[n].NEUTREAV, pwmNeutre, hard.MOTEURS[n].PWMMAX);
 else
  pwm = pwmNeutre;

 switch(hard.MOTEURS[n].PCA9685) {
  case -1:
   gpioMoteurs[n].servoWrite(pwm);
   break;
  case -2:
   l298MotorDrive(n, pwm);
   break;
  case -3:
   l9110MotorDrive(n, pwm);
   break;
  default:
   pca9685MotorDrive(n, pwm);
 }
}

function l298MotorDrive(n, value) {
 let pwm;

 if(value < 0) {
  gpioMoteursA[n].digitalWrite(false);
  gpioMoteursB[n].digitalWrite(true);
  pwm = -value;
 } else if(value > 0) {
  gpioMoteursA[n].digitalWrite(true);
  gpioMoteursB[n].digitalWrite(false);
  pwm = value;
 } else {
  gpioMoteursA[n].digitalWrite(false);
  gpioMoteursB[n].digitalWrite(false);
  pwm = 0;
 }

 gpioMoteurs[n].pwmWrite(pwm);
}

function l9110MotorDrive(n, value) {
 if(value < 0) {
  gpioMoteursA[n].digitalWrite(false);
  gpioMoteursB[n].pwmWrite(-value);
 } else if(value > 0) {
  gpioMoteursA[n].pwmWrite(value);
  gpioMoteursB[n].digitalWrite(false);
 } else {
  gpioMoteursA[n].digitalWrite(false);
  gpioMoteursB[n].digitalWrite(false);
 }
}

function pca9685MotorDrive(n, value) {
 let pcaId = hard.MOTEURS[n].PCA9685;
 let pwm;
 let chIn1 = hard.MOTEURS[n].PINA;
 let chIn2 = hard.MOTEURS[n].PINB;
 let chPwm = hard.MOTEURS[n].PIN;

 if(value < 0) {
  pca9685Driver[pcaId].channelOff(chIn1);
  pca9685Driver[pcaId].channelOn(chIn2);
  pwm = -value;
 } else if(value > 0) {
  pca9685Driver[pcaId].channelOn(chIn1);
  pca9685Driver[pcaId].channelOff(chIn2);
  pwm = value;
 } else {
  pca9685Driver[pcaId].channelOff(chIn1);
  pca9685Driver[pcaId].channelOff(chIn2);
  pwm = 0;
 }

 pca9685Driver[pcaId].setDutyCycle(chPwm, pwm);
}

function failSafe() {
 trace("Arrêt des moteurs");
 for(let i = 0; i < hard.MOTEURS.length; i++)
  setMotor(i, 0);
}

setInterval(function() {
 if(!up || !init)
  return;

 let latencePredictive = Math.max(latence, Date.now() - lastTimestamp);

 if(latencePredictive < LATENCEFINALARME && alarmeLatence) {
  trace("Latence de " + latencePredictive + " ms, retour au débit vidéo configuré");
  exec("v4l2-ctl", V4L2 + " -c video_bitrate=" + confDynamique.BITRATE, function(code) {
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

if(cw2015) {
 setInterval(function() {
  if(!init)
   return;

  i2c.i2cRead(CW2015ADDRESS, 256, gaugeBuffer, function() {
   let microVolts = ((gaugeBuffer[247] << 8) + gaugeBuffer[248]) * 305;
   let pour25600 = (gaugeBuffer[249] << 8) + gaugeBuffer[250];

   rx.setValeur16(0, microVolts / 1000000);
   rx.setValeur16(1, pour25600 / 256);
  });
 }, GAUGERATE);
}

if(max17043) {
 setInterval(function() {
  if(!init)
   return;

  i2c.i2cRead(MAX17043ADDRESS, 7, gaugeBuffer, function() {
   let milliVolts = ((gaugeBuffer[3] << 8) + gaugeBuffer[4]) * 5000 / 4096;
   let pour25600 = (gaugeBuffer[5] << 8) + gaugeBuffer[6];

   rx.setValeur16(0, milliVolts / 1000);
   rx.setValeur16(1, pour25600 / 256);
  });
 }, GAUGERATE);
}

if(bq27441) {
 setInterval(function() {
  if(!init)
   return;

  i2c.readWord(BQ27441ADDRESS, 0x04, function(err, milliVolts) {
   rx.setValeur16(0, milliVolts / 1000);
  });

  i2c.readByte(BQ27441ADDRESS, 0x1c, function(err, pourcents) {
   rx.setValeur16(1, pourcents);
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
 if(up || !init)
  return;

 CONF.SERVEURS.forEach(function(serveur) {
  sockets[serveur].emit("serveurrobotrx", {
   timestamp: Date.now(),
   data: rx.arrayBuffer
  });
 });
}, BEACONRATE);

setInterval(function() {
 if(!hard.CAPTURESENVEILLE || up || !init || !initVideo)
  return;

 let date = new Date();
 let overlay = date.toLocaleDateString() + " " + date.toLocaleTimeString();
 if(hard.CAPTURESHDR)
  overlay += " HDR " + hard.CAPTURESHDR;
 let options = "-a 1024 -a '" + overlay + "' -rot " + confDynamique.ROTATION;

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
