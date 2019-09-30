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
const GAUGERATE = 250;

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

let old = [];
let rattrapage = [];
let oldTxInterrupteurs;

let gpioOutils = [];
let gpioMoteurs = [];
let gpioInterrupteurs = [];

let i2c;
let cw2015;
let max17043;
let gaugeBuffer = new Buffer.alloc(256);

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
} catch(err) {
 try {
  i2c.i2cReadSync(MAX17043ADDRESS, 6, gaugeBuffer);
  cw2015 = false;
  max17043 = true;
 } catch(err) {
  trace("I2C désactivé");
  i2c.closeSync();
  cw2015 = false;
  max17043 = false;
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

function sigterm(timeout, nom, process, callback) {
 setTimeout(function() {
  trace("Envoi du signal SIGTERM au processus " + nom);
  let processkill = EXEC("/usr/bin/killall -15 " + process);
  processkill.on("close", function(code) {
   callback(code);
  });
 }, timeout);
}

function exec(timeout, nom, commande, callback) {
 setTimeout(function() {
  trace("Démarrage du processus " + nom);
  trace(commande);
  let process = EXEC(commande);
  let stdout = RL.createInterface(process.stdout);
  let stderr = RL.createInterface(process.stderr);
  let pid = process.pid;
  let execTime = Date.now();

  //process.stdout.on("data", function(data) {
  stdout.on("line", function(data) {
   traces(nom + " | " + pid + " | stdout", data);
  });

  //process.stderr.on("data", function(data) {
  stderr.on("line", function(data) {
   traces(nom + " | " + pid + " | stderr", data);
  });

  process.on("close", function(code) {
   let elapsed = Date.now() - execTime;

   trace("Le processus " + nom + " c'est arrêté après " + elapsed + " millisecondes avec le code de sortie " + code);
   callback(code);
  });

 }, timeout);
}

function dodo() {
 serveurCourant = "";

 gpioOutils.forEach(function(gpio) {
  gpio.mode(GPIO.INPUT);
 });

 for(let i = 0; i < hard.MOTEURS.length; i++)
  gpioMoteurs[i].servoWrite(map(0, -0x80, 0x80, hard.MOTEURS[i].PWMMIN, hard.MOTEURS[i].PWMMAX));

 for(let i = 0; i < 8; i++)
  setGpio(i, hard.INTERRUPTEURS[i].INV);

 sigterm(0, "Diffusion", PROCESSDIFFUSION, function(code) {
 });

 sigterm(0, "DiffAudio", PROCESSDIFFAUDIO, function(code) {
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

 sigterm(0, "Diffusion", PROCESSDIFFUSION, function(code) {
  exec(0, "v4l2-ctl", V4L2 + " -v width=" + confStatique.WIDTH +
                                ",height=" + confStatique.HEIGHT +
                                ",pixelformat=4" +
                             " -p " + confStatique.FPS +
                             " -c h264_profile=0" +
                                ",repeat_sequence_header=1" +
                                ",rotate=" + confDynamique.ROTATION +
                                ",video_bitrate=" + confDynamique.BITRATE +
                                ",brightness=" + confDynamique.LUMINOSITE +
                                ",contrast=" + confDynamique.CONTRASTE, function(code) {
   if(up)
    diffusion();
   callback(code);
  });
 });
}

function confDynamiqueVideo() {
 trace("Modification de la configuration Video4Linux");

 exec(0, "v4l2-ctl", V4L2 + " -c h264_profile=0" +
                               ",repeat_sequence_header=1" +
                               ",rotate=" + confDynamique.ROTATION +
                               ",video_bitrate=" + confDynamique.BITRATE +
                               ",brightness=" + confDynamique.LUMINOSITE +
                               ",contrast=" + confDynamique.CONTRASTE, function(code) {
 });
}

function diffusion() {
 trace("Démarrage du flux de diffusion vidéo H.264");
 exec(0, "Diffusion", cmdDiffusion, function(code) {
  trace("Arrêt du flux de diffusion vidéo H.264");
 });
}

function diffAudio() {
 trace("Démarrage du flux de diffusion audio");
 exec(0, "DiffAudio", cmdDiffAudio, function(code) {
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
   old[i] = tx.outils[i];
   rattrapage[i] = 0;
  }

  oldTxInterrupteurs = conf.TX.INTERRUPTEURS[0];

  oldCamera = conf.COMMANDES[conf.DEFAUTCOMMANDE].CAMERA;
  oldIdConfStatique = conf.CAMERAS[oldCamera].CONFSTATIQUE;
  oldIdConfDynamique = conf.CAMERAS[oldCamera].CONFDYNAMIQUE;
  confStatique = conf.CONFSSTATIQUE[oldIdConfStatique];
  confDynamique = conf.CONFSDYNAMIQUE[oldIdConfDynamique];

  confVideo(function(code) {
   initVideo = true;
  });

  trace("Initialisation des I/O Raspberry PI");

  gpioOutils.forEach(function(gpio) {
   gpio.mode(GPIO.INPUT);
  });

  gpioMoteurs.forEach(function(gpio) {
   gpio.mode(GPIO.INPUT);
  });

  gpioInterrupteurs.forEach(function(gpio) {
   gpio.mode(GPIO.INPUT);
  });

  gpioOutils = [];
  gpioMoteurs = [];
  gpioInterrupteurs = [];

  for(let i = 0; i < hard.OUTILS.length; i++)
   gpioOutils[i] = new GPIO(hard.OUTILS[i].PIN, {mode: GPIO.OUTPUT});

  for(let i = 0; i < hard.MOTEURS.length; i++)
   gpioMoteurs[i] = new GPIO(hard.MOTEURS[i].PIN, {mode: GPIO.OUTPUT});

  for(let i = 0; i < 8; i++) {
   gpioInterrupteurs[i] = new GPIO(hard.INTERRUPTEURS[i].PIN, {mode: GPIO.OUTPUT});
   setGpio(i, hard.INTERRUPTEURS[i].INV);
  }

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
  if(!init || !initVideo) {
   trace("Ce robot n'est pas initialisé");
   notification(serveur, "Ce robot n'est pas initialisé", "error");
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

  gpioOutils.forEach(function(gpio) {
   gpio.mode(GPIO.OUTPUT);
  });

  for(let i = 0; i < 8; i++)
   setGpio(i, tx.interrupteurs[0] >> i & 1 ^ hard.INTERRUPTEURS[i].INV);

  if(conf.CAPTURESENVEILLE) {
   sigterm(0, "Raspistill", "raspistill", function(code) {
    diffusion();
   });
  } else
   diffusion();
  diffAudio();

  sockets[serveur].emit("serveurrobotdebout", true);

  up = true;
  lastTimestamp = Date.now();
  latence = 0;
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
   exec(0, "eSpeak", "/usr/bin/espeak -v fr -f /tmp/tts.txt --stdout > /tmp/tts.wav", function(code) {
    exec(0, "Aplay", "/usr/bin/aplay -D plughw:0 /tmp/tts.wav", function(code) {
    });
   });
  });
 });

 sockets[serveur].on("clientsrobotexit", function() {
  trace("Ordre de terminer le processus Node.js");
  dodo();
  setTimeout(function() {
   process.exit();
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
    confVideo(function(code) {
    });
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
    if(tx.outils[i] < old[i])
     rattrapage[i] = -hard.OUTILS[i].RATTRAPAGE * 0x10000 / 360;
    else if(tx.outils[i] > old[i])
     rattrapage[i] = hard.OUTILS[i].RATTRAPAGE * 0x10000 / 360;

    old[i] = tx.outils[i];

    outils[i] = constrain(tx.outils[i] + rattrapage[i] + hard.OUTILS[i].ANGLEOFFSET * 0x10000 / 360, (-hard.OUTILS[i].COURSE / 2 + 180) * 0x10000 / 360,
                                                                                                     (hard.OUTILS[i].COURSE / 2 + 180) * 0x10000 / 360);

    let pwm = map(outils[i], (-hard.OUTILS[i].COURSE / 2 + 180) * 0x10000 / 360,
                             (hard.OUTILS[i].COURSE / 2 + 180) * 0x10000 / 360, hard.OUTILS[i].PWMMIN, hard.OUTILS[i].PWMMAX);

    gpioOutils[i].servoWrite(pwm);
   }
  }

  let moteurs = [];

  for(let i = 0; i < hard.MIXAGESMOTEURS.length; i++)
   moteurs[i] = constrain(tx.vitesses[0] * hard.MIXAGESMOTEURS[i][0] +
                          tx.vitesses[1] * hard.MIXAGESMOTEURS[i][1] +
                          tx.vitesses[2] * hard.MIXAGESMOTEURS[i][2], -0x80, 0x80);

  for(let i = 0; i < hard.MOTEURS.length; i++) {
   let pwm;
   let pwmNeutre = (hard.MOTEURS[i].PWMMAX + hard.MOTEURS[i].PWMMIN) / 2;

   if(moteurs[i] < 0)
    pwm = map(moteurs[i] + hard.MOTEURS[i].NEUTREAR, -0x80 + hard.MOTEURS[i].NEUTREAR, 0, hard.MOTEURS[i].PWMMIN, pwmNeutre);
   else if(moteurs[i] > 0)
    pwm = map(moteurs[i] + hard.MOTEURS[i].NEUTREAV, 0, 0x80 + hard.MOTEURS[i].NEUTREAV, pwmNeutre, hard.MOTEURS[i].PWMMAX);
   else
    pwm = pwmNeutre;

   gpioMoteurs[i].servoWrite(pwm);
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

function setGpio(n, etat) {
 if(hard.INTERRUPTEURS[n].MODE == 1 && !etat || // Drain ouvert
    hard.INTERRUPTEURS[n].MODE == 2 && etat)    // Collecteur ouvert
  gpioInterrupteurs[n].mode(GPIO.INPUT);
 else
  gpioInterrupteurs[n].digitalWrite(etat);
}

function failSafe() {
 trace("Arrêt des moteurs");

 for(let i = 0; i < hard.MOTEURS.length; i++)
  gpioMoteurs[i].servoWrite(map(0, -0x80, 0x80, hard.MOTEURS[i].PWMMIN, hard.MOTEURS[i].PWMMAX));
}

setInterval(function() {
 if(!up || !init)
  return;

 let latencePredictive = Math.max(latence, Date.now() - lastTimestamp);

 if(latencePredictive < LATENCEFINALARME && alarmeLatence) {
  trace("Latence de " + latencePredictive + " ms, retour au débit vidéo configuré");
  exec(0, "v4l2-ctl", V4L2 + " -c video_bitrate=" + confDynamique.BITRATE, function(code) {
  });
  alarmeLatence = false;
 } else if(latencePredictive > LATENCEDEBUTALARME && !alarmeLatence) {
  failSafe();
  trace("Latence de " + latencePredictive + " ms, passage en débit vidéo réduit");
  exec(0, "v4l2-ctl", V4L2 + " -c video_bitrate=" + BITRATEVIDEOFAIBLE, function(code) {
  });
  alarmeLatence = true;
 }
}, TXRATE);

if(cw2015) {
 setInterval(function() {
  if(!init)
   return;
  i2c.i2cReadSync(CW2015ADDRESS, 256, gaugeBuffer);

  let microVolts = ((gaugeBuffer[247] << 8) + gaugeBuffer[248]) * 305;
  let pour25600 = (gaugeBuffer[249] << 8) + gaugeBuffer[250];

  rx.setValeur16(0, microVolts / 1000000);
  rx.setValeur16(1, pour25600 / 256);
 }, GAUGERATE);
}

if(max17043) {
 setInterval(function() {
  if(!init)
   return;
  i2c.i2cReadSync(MAX17043ADDRESS, 7, gaugeBuffer);

  let milliVolts = ((gaugeBuffer[3] << 8) + gaugeBuffer[4]) * 5000 / 4096;
  let pour25600 = (gaugeBuffer[5] << 8) + gaugeBuffer[6];

  rx.setValeur16(0, milliVolts / 1000);
  rx.setValeur16(1, pour25600 / 256);
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
 if(!conf.CAPTURESENVEILLE || up || !init || !initVideo)
  return;

 let overlay = new Date().toISOString();

 let process = EXEC("raspistill -q 20 -a 1024 -a '" + overlay + "' -rot " + confDynamique.ROTATION + " -o -", {
  encoding: "binary",
  maxBuffer: 10 * 1024 * 1024
 }, function(error, stdout) {
  if(error)
   trace("Erreur lors de la capture d'une photo");
  else {
   CONF.SERVEURS.forEach(function(serveur) {
    trace("Envoi d'une photo sur le serveur " + serveur);
    sockets[serveur].emit("serveurrobotcapturesenveille", new Buffer(stdout, "binary"));
   });
  }
 });
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
