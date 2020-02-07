function constrain(n, nMin, nMax) {
 if(n > nMax)
  n = nMax;
 else if(n < nMin)
  n = nMin;

 return n;
}

function map(n, inMin, inMax, outMin, outMax) {
 return (n - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

class Tx {
 constructor(conftx) {
  let p = 0;
  let nb16 = conftx.POSITIONS.length + conftx.AUTOGOTO.length + conftx.AUTOANGLES.length + conftx.COMMANDES16.length;
  let nb8 = conftx.SYNC.length + conftx.CHOIXCAMERAS.length + conftx.VITESSES.length + conftx.REQUETESMISSION.length +
            conftx.INTERRUPTEURS.length + conftx.COMMANDES8.length + conftx.FIN.length;

  this.conftx = conftx;

  this.byteLength = nb16 * 2 + nb8;

  this.arrayBuffer = new ArrayBuffer(this.byteLength);

  this.sync = new Uint8Array(this.arrayBuffer, p, conftx.SYNC.length);
  p += this.sync.byteLength;
  for(let i = 0; i < conftx.SYNC.length; i++)
   this.sync[i] = conftx.SYNC[i].charCodeAt();

  this.positions = new Uint16Array(this.arrayBuffer, p,  conftx.POSITIONS.length);
  p += this.positions.byteLength;
  for(let i = 0; i < conftx.POSITIONS.length; i++)
   this.positions[i] = (conftx.POSITIONS[i] + 180) * 0x10000 / 360;

  this.autoGoto = new Int16Array(this.arrayBuffer, p, conftx.AUTOGOTO.length);
  p += this.autoGoto.byteLength;
  for(let i = 0; i < conftx.AUTOGOTO.length; i++)
   this.autoGoto[i] = conftx.AUTOGOTO[i];

  this.autoAngles = new Uint16Array(this.arrayBuffer, p, conftx.AUTOANGLES.length);
  p += this.autoAngles.byteLength;
  for(let i = 0; i < conftx.AUTOANGLES.length; i++)
   this.autoAngles[i] = conftx.AUTOANGLES[i];

  this.commandesUint16 = new Uint16Array(this.arrayBuffer, p, conftx.COMMANDES16.length);
  this.commandesInt16 = new Int16Array(this.arrayBuffer, p, conftx.COMMANDES16.length);
  p += this.commandesUint16.byteLength;
  for(let i = 0; i < conftx.COMMANDES16.length; i++)
   this.setCommande16(i, conftx.COMMANDES16[i]);

  this.choixCameras = new Uint8Array(this.arrayBuffer, p, conftx.CHOIXCAMERAS.length);
  p += this.choixCameras.byteLength;
  for(let i = 0; i < conftx.CHOIXCAMERAS.length; i++)
   this.choixCameras[i] = conftx.CHOIXCAMERAS[i];

  this.vitesses = new Int8Array(this.arrayBuffer, p, conftx.VITESSES.length);
  p += this.vitesses.byteLength;
  for(let i = 0; i < conftx.VITESSES.length; i++)
   this.vitesses[i] = conftx.VITESSES[i];

  this.requetesMission = new Uint8Array(this.arrayBuffer, p, conftx.REQUETESMISSION.length);
  p += this.requetesMission.byteLength;
  for(let i = 0; i < conftx.REQUETESMISSION.length; i++)
   this.requetesMission[i] = conftx.REQUETESMISSION[i];

  this.interrupteurs = new Uint8Array(this.arrayBuffer, p, conftx.INTERRUPTEURS.length);
  p += this.interrupteurs.byteLength;
  for(let i = 0; i < conftx.INTERRUPTEURS.length; i++) {
   this.interrupteurs[i] = 0;
   for(let j = 0; j < 8; j++)
    if(conftx.INTERRUPTEURS[i].substring(j, j + 1) == "1")
     this.interrupteurs[i] += 1 << j;
  }

  this.commandesUint8 = new Uint8Array(this.arrayBuffer, p, conftx.COMMANDES8.length);
  this.commandesInt8 = new Int8Array(this.arrayBuffer, p, conftx.COMMANDES8.length);
  p += this.commandesUint8.byteLength;
  for(let i = 0; i < conftx.COMMANDES8.length; i++)
   this.setCommande8(i, conftx.COMMANDES8[i]);

  this.fin = new Uint8Array(this.arrayBuffer, p, conftx.FIN.length);
  p += this.fin.byteLength;
  for(let i = 0; i < conftx.FIN.length; i++)
   this.fin[i] = conftx.FIN[i];

  this.bytes = new Uint8Array(this.arrayBuffer);
 }

 setCommande16(id, commande) {
  let min;
  let max;

  if(this.conftx.COMMANDES16[id].SIGNE) {
   min = -32768;
   max = 32767;
  } else {
   min = 0;
   max = 65535;
  }
  commande = constrain(commande, this.conftx.COMMANDES16[id].MIN, this.conftx.COMMANDES16[id].MAX);
  commande = map(commande, this.conftx.COMMANDES16[id].MIN, this.conftx.COMMANDES16[id].MAX, min, max);

  if(this.conftx.COMMANDES16[id].SIGNE)
   this.commandesInt16[id] = commande;
  else
   this.commandesUint16[id] = commande;
 }

 setCommande8(id, commande) {
  let min;
  let max;

  if(this.conftx.COMMANDES8[id].SIGNE) {
   min = -128;
   max = 127;
  } else {
   min = 0;
   max = 255;
  }
  commande = constrain(commande, this.conftx.COMMANDES8[id].MIN, this.conftx.COMMANDES8[id].MAX);
  commande = map(commande, this.conftx.COMMANDES8[id].MIN, this.conftx.COMMANDES8[id].MAX, min, max);

  if(this.conftx.COMMANDES16[id].SIGNE)
   this.commandesInt8[id] = commande;
  else
   this.commandesUint8[id] = commande;
 }

 getCommande16(id) {
  let commande;

  if(this.conftx.COMMANDES16[id].SIGNE)
   commande = map(this.commandesInt16[id], -32768, 32767, this.conftx.COMMANDES16[id].MIN, this.conftx.COMMANDES16[id].MAX);
  else
   commande = map(this.commandesUint16[id], 0, 65535, this.conftx.COMMANDES16[id].MIN, this.conftx.COMMANDES16[id].MAX);
  commande = commande.toFixed(this.conftx.COMMANDES16[id].NBCHIFFRES);

  return commande;
 }

 getCommande8(id) {
  let min;
  let max;
  let commande;

  if(this.conftx.COMMANDES8[id].SIGNE)
   commande = map(this.commandesInt8[id], -128, 127, this.conftx.COMMANDES8[id].MIN, this.conftx.COMMANDES8[id].MAX);
  else
   commande = map(this.commandesUint8[id], 0, 255, this.conftx.COMMANDES8[id].MIN, this.conftx.COMMANDES8[id].MAX);
  commande = commande.toFixed(this.conftx.COMMANDES8[id].NBCHIFFRES);

  return commande;
 }

 getTexteCommande16(id) {
  return this.getCommande16(id) + this.conftx.COMMANDES16[id].UNITE;
 }

 getTexteCommande8(id) {
  return this.getCommande8(id) + this.conftx.COMMANDES8[id].UNITE;
 }
}

class Rx {
 constructor(conftx, confrx) {
  this.conftx = conftx;
  this.confrx = confrx;

  let p = 0;
  let nb16 = conftx.POSITIONS.length + conftx.AUTOGOTO.length + conftx.AUTOANGLES.length + conftx.COMMANDES16.length +
             confrx.ODOMETRIES.length + confrx.ANGLES.length + confrx.CIBLES.length + confrx.RESULTATSMISSION.length +
            (confrx.NBCORRECTEURS + confrx.NBCLUSTERS) * 4 + confrx.NBPOINTSLIDAR2D * 2 + confrx.VALEURS16.length;
  let nb8 = confrx.SYNC.length + conftx.CHOIXCAMERAS.length + conftx.VITESSES.length + conftx.REQUETESMISSION.length + conftx.INTERRUPTEURS.length +
            conftx.COMMANDES8.length + confrx.NBCORRECTEURS + confrx.NBCLUSTERS + confrx.VALEURS8.length + confrx.FIN.length;

  this.byteLength = nb16 * 2 + nb8;

  this.arrayBuffer = new ArrayBuffer(this.byteLength);

  this.sync = new Uint8Array(this.arrayBuffer, p, confrx.SYNC.length);
  p += this.sync.byteLength;
  for(let i = 0; i < confrx.SYNC.length; i++)
   this.sync[i] = confrx.SYNC[i].charCodeAt();

  this.positions = new Uint16Array(this.arrayBuffer, p,  conftx.POSITIONS.length);
  p += this.positions.byteLength;
  for(let i = 0; i < conftx.POSITIONS.length; i++)
   this.positions[i] = (conftx.POSITIONS[i] + 180) * 0x10000 / 360;

  this.autoGoto = new Int16Array(this.arrayBuffer, p, conftx.AUTOGOTO.length);
  p += this.autoGoto.byteLength;
  for(let i = 0; i < conftx.AUTOGOTO.length; i++)
   this.autoGoto[i] = conftx.AUTOGOTO[i];

  this.autoAngles = new Uint16Array(this.arrayBuffer, p, conftx.AUTOANGLES.length);
  p += this.autoAngles.byteLength;
  for(let i = 0; i < conftx.AUTOANGLES.length; i++)
   this.autoAngles[i] = conftx.AUTOANGLES[i];

  this.commandesUint16 = new Uint16Array(this.arrayBuffer, p, conftx.COMMANDES16.length);
  this.commandesInt16 = new Int16Array(this.arrayBuffer, p, conftx.COMMANDES16.length);
  p += this.commandesUint16.byteLength;
  for(let i = 0; i < conftx.COMMANDES16.length; i++)
   this.setCommande16(i, conftx.COMMANDES16[i]);

  this.odometries = new Int16Array(this.arrayBuffer, p, confrx.ODOMETRIES.length);
  p += this.odometries.byteLength;
  for(let i = 0; i < confrx.ODOMETRIES.length; i++)
   this.odometries[i] = confrx.ODOMETRIES[i];

  this.angles = new Uint16Array(this.arrayBuffer, p, confrx.ANGLES.length);
  p += this.angles.byteLength;
  for(let i = 0; i < confrx.ANGLES.length; i++)
   this.angles[i] = confrx.ANGLES[i];

  this.cibles = new Int16Array(this.arrayBuffer, p, confrx.CIBLES.length);
  p += this.cibles.byteLength;
  for(let i = 0; i < confrx.CIBLES.length; i++)
   this.cibles[i] = confrx.CIBLES[i];

  this.resultatsMission = new Uint16Array(this.arrayBuffer, p, confrx.RESULTATSMISSION.length);
  p += this.resultatsMission.byteLength;
  for(let i = 0; i < confrx.RESULTATSMISSION.length; i++)
   this.resultatsMission[i] = confrx.RESULTATSMISSION[i];

  this.correcteurs = new Int16Array(this.arrayBuffer, p, confrx.NBCORRECTEURS * 4);
  p += this.correcteurs.byteLength;
  for(let i = 0; i < confrx.NBCORRECTEURS * 4; i++)
   this.correcteurs[i] = -32768;

  this.clusters = new Int16Array(this.arrayBuffer, p, confrx.NBCLUSTERS * 4);
  p += this.clusters.byteLength;
  for(let i = 0; i < confrx.NBCLUSTERS * 4; i++)
   this.clusters[i] = -32768;

  this.pointsLidar2D = new Int16Array(this.arrayBuffer, p, confrx.NBPOINTSLIDAR2D * 2);
  p += this.pointsLidar2D.byteLength;
  for(let i = 0; i < confrx.NBPOINTSLIDAR2D * 2; i++)
   this.pointsLidar2D[i] = -32768;

  this.valeursUint16 = new Uint16Array(this.arrayBuffer, p, confrx.VALEURS16.length);
  this.valeursInt16 = new Int16Array(this.arrayBuffer, p, confrx.VALEURS16.length);
  p += this.valeursUint16.byteLength;
  for(let i = 0; i < confrx.VALEURS16.length; i++)
   this.setValeur16(i, confrx.VALEURS16[i]);

  this.choixCameras = new Uint8Array(this.arrayBuffer, p, conftx.CHOIXCAMERAS.length);
  p += this.choixCameras.byteLength;
  for(let i = 0; i < conftx.CHOIXCAMERAS.length; i++)
   this.choixCameras[i] = conftx.CHOIXCAMERAS[i];

  this.vitesses = new Int8Array(this.arrayBuffer, p, conftx.VITESSES.length);
  p += this.vitesses.byteLength;
  for(let i = 0; i < conftx.VITESSES.length; i++)
   this.vitesses[i] = conftx.VITESSES[i];

  this.requetesMission = new Uint8Array(this.arrayBuffer, p, conftx.REQUETESMISSION.length);
  p += this.requetesMission.byteLength;
  for(let i = 0; i < conftx.REQUETESMISSION.length; i++)
   this.requetesMission[i] = conftx.REQUETESMISSION[i];

  this.interrupteurs = new Uint8Array(this.arrayBuffer, p, conftx.INTERRUPTEURS.length);
  p += this.interrupteurs.byteLength;
  for(let i = 0; i < conftx.INTERRUPTEURS.length; i++) {
   this.interrupteurs[i] = 0;
   for(let j = 0; j < 8; j++)
    if(conftx.INTERRUPTEURS[i].substring(j, j + 1) == "1")
     this.interrupteurs[i] += 1 << j;
  }

  this.commandesUint8 = new Uint8Array(this.arrayBuffer, p, conftx.COMMANDES8.length);
  this.commandesInt8 = new Int8Array(this.arrayBuffer, p, conftx.COMMANDES8.length);
  p += this.commandesUint8.byteLength;
  for(let i = 0; i < conftx.COMMANDES8.length; i++)
   this.setCommande8(i, conftx.COMMANDES8[i]);

  this.idCorrecteurs = new Uint8Array(this.arrayBuffer, p, confrx.NBCORRECTEURS);
  p += this.idCorrecteurs.byteLength;
  for(let i = 0; i < conftx.NBCORRECTEURS; i++)
   this.idCorrecteurs[i] = 0;

  this.idClusters = new Uint8Array(this.arrayBuffer, p, confrx.NBCLUSTERS);
  p += this.idClusters.byteLength;
  for(let i = 0; i < conftx.NBCLUSTERS; i++)
   this.idClusters[i] = 0;

  this.valeursUint8 = new Uint8Array(this.arrayBuffer, p, confrx.VALEURS8.length);
  this.valeursInt8 = new Int8Array(this.arrayBuffer, p, confrx.VALEURS8.length);
  p += this.valeursUint8.byteLength;
  for(let i = 0; i < confrx.VALEURS8.length; i++)
   this.setValeur8(i, confrx.VALEURS8[i]);

  this.fin = new Uint8Array(this.arrayBuffer, p, confrx.FIN.length);
  p += this.fin.byteLength;
  for(let i = 0; i < confrx.FIN.length; i++)
   this.fin[i] = confrx.FIN[i];

  this.bytes = new Uint8Array(this.arrayBuffer);
 }

 constrain(n, nMin, nMax) {
  if(n > nMax)
   n = nMax;
  else if(n < nMin)
   n = nMin;

  return n;
 }

 map(n, inMin, inMax, outMin, outMax) {
  return (n - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
 }

 setCommande16(id, commande) {
  let min;
  let max;

  if(this.conftx.COMMANDES16[id].SIGNE) {
   min = -32768;
   max = 32767;
  } else {
   min = 0;
   max = 65535;
  }
  commande = constrain(commande, this.conftx.COMMANDES16[id].MIN, this.conftx.COMMANDES16[id].MAX);
  commande = map(commande, this.conftx.COMMANDES16[id].MIN, this.conftx.COMMANDES16[id].MAX, min, max);

  if(this.conftx.COMMANDES16[id].SIGNE)
   this.commandesInt16[id] = commande;
  else
   this.commandesUint16[id] = commande;
 }

 setCommande8(id, commande) {
  let min;
  let max;

  if(this.conftx.COMMANDES8[id].SIGNE) {
   min = -128;
   max = 127;
  } else {
   min = 0;
   max = 255;
  }
  commande = constrain(commande, this.conftx.COMMANDES8[id].MIN, this.conftx.COMMANDES8[id].MAX);
  commande = map(commande, this.conftx.COMMANDES8[id].MIN, this.conftx.COMMANDES8[id].MAX, min, max);

  if(this.conftx.COMMANDES16[id].SIGNE)
   this.commandesInt8[id] = commande;
  else
   this.commandesUint8[id] = commande;
 }

 getCommande16(id) {
  let commande;

  if(this.conftx.COMMANDES16[id].SIGNE)
   commande = map(this.commandesInt16[id], -32768, 32767, this.conftx.COMMANDES16[id].MIN, this.conftx.COMMANDES16[id].MAX);
  else
   commande = map(this.commandesUint16[id], 0, 65535, this.conftx.COMMANDES16[id].MIN, this.conftx.COMMANDES16[id].MAX);
  commande = commande.toFixed(this.conftx.COMMANDES16[id].NBCHIFFRES);

  return commande;
 }

 getCommande8(id) {
  let min;
  let max;
  let commande;

  if(this.conftx.COMMANDES8[id].SIGNE)
   commande = map(this.commandesInt8[id], -128, 127, this.conftx.COMMANDES8[id].MIN, this.conftx.COMMANDES8[id].MAX);
  else
   commande = map(this.commandesUint8[id], 0, 255, this.conftx.COMMANDES8[id].MIN, this.conftx.COMMANDES8[id].MAX);
  commande = commande.toFixed(this.conftx.COMMANDES8[id].NBCHIFFRES);

  return commande;
 }

 getTexteCommande16(id) {
  return this.getCommande16(id) + this.conftx.COMMANDES16[id].UNITE;
 }

 getTexteCommande8(id) {
  return this.getCommande8(id) + this.conftx.COMMANDES8[id].UNITE;
 }

 setValeur16(id, valeur) {
  let min;
  let max;

  if(this.confrx.VALEURS16[id].SIGNE) {
   min = -32768;
   max = 32767;
  } else {
   min = 0;
   max = 65535;
  }
  valeur = constrain(valeur, this.confrx.VALEURS16[id].MIN, this.confrx.VALEURS16[id].MAX);
  valeur = map(valeur, this.confrx.VALEURS16[id].MIN, this.confrx.VALEURS16[id].MAX, min, max);

  if(this.confrx.VALEURS16[id].SIGNE)
   this.valeursInt16[id] = valeur;
  else
   this.valeursUint16[id] = valeur;
 }

 setValeur8(id, valeur) {
  let min;
  let max;

  if(this.confrx.VALEURS8[id].SIGNE) {
   min = -128;
   max = 127;
  } else {
   min = 0;
   max = 255;
  }
  valeur = constrain(valeur, this.confrx.VALEURS8[id].MIN, this.confrx.VALEURS8[id].MAX);
  valeur = map(valeur, this.confrx.VALEURS8[id].MIN, this.confrx.VALEURS8[id].MAX, min, max);

  if(this.confrx.VALEURS8[id].SIGNE)
   this.valeursInt8[id] = valeur;
  else
   this.valeursUint8[id] = valeur;
 }

 getValeur16(id) {
  let valeur;

  if(this.confrx.VALEURS16[id].SIGNE)
   valeur = map(this.valeursInt16[id], -32768, 32767, this.confrx.VALEURS16[id].MIN, this.confrx.VALEURS16[id].MAX);
  else
   valeur = map(this.valeursUint16[id], 0, 65535, this.confrx.VALEURS16[id].MIN, this.confrx.VALEURS16[id].MAX);
  valeur = valeur.toFixed(this.confrx.VALEURS16[id].NBCHIFFRES);

  return valeur;
 }

 getValeur8(id) {
  let min;
  let max;
  let valeur;

  if(this.confrx.VALEURS8[id].SIGNE)
   valeur = map(this.valeursInt8[id], -128, 127, this.confrx.VALEURS8[id].MIN, this.confrx.VALEURS8[id].MAX);
  else
   valeur = map(this.valeursUint8[id], 0, 255, this.confrx.VALEURS8[id].MIN, this.confrx.VALEURS8[id].MAX);
  valeur = valeur.toFixed(this.confrx.VALEURS8[id].NBCHIFFRES);

  return valeur;
 }

 getTexte16(id) {
  return this.getValeur16(id) + this.confrx.VALEURS16[id].UNITE;
 }

 getTexte8(id) {
  return this.getValeur8(id) + this.confrx.VALEURS8[id].UNITE;
 }
}

module.exports = {Tx, Rx};
