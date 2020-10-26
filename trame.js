const FRAME0 = "$".charCodeAt();
const FRAME1R = "R".charCodeAt();

function constrain(n, nMin, nMax) {
 if(n > nMax)
  n = nMax;
 else if(n < nMin)
  n = nMin;

 return n;
}

function mapFloat(n, inMin, inMax, outMin, outMax) {
 return (n - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

function mapTrunc(n, inMin, inMax, outMin, outMax) {
 return Math.trunc(mapFloat(n, inMin, inMax, outMin, outMax));
}

class Tx {
 constructor(conftx) {
  this.conftx = conftx;

  let nb32 = conftx.COMMANDS32.length;
  let nb16 = conftx.COMMANDS16.length;
  let nb8 = conftx.SYNC.length + conftx.CAMERACHOICES.length + conftx.COMMANDS8.length;

  this.byteLength = nb32 * 4 + nb16 * 2 + nb8 + Math.ceil(conftx.COMMANDS1.length / 8);
  this.arrayBuffer = new ArrayBuffer(this.byteLength);

  let p = 0;
  this.sync = new Uint8Array(this.arrayBuffer, p, conftx.SYNC.length);
  p += this.sync.byteLength;
  for(let i = 0; i < conftx.SYNC.length; i++)
   this.sync[i] = conftx.SYNC[i].charCodeAt();

  this.commandsUint32 = new Uint32Array(this.arrayBuffer, p, conftx.COMMANDS32.length);
  this.commandsInt32 = new Int32Array(this.arrayBuffer, p, conftx.COMMANDS32.length);
  p += this.commandsUint32.byteLength;
  for(let i = 0; i < conftx.COMMANDS32.length; i++)
   this.setFloatCommand32(i, conftx.COMMANDS32[i].INIT);

  this.commandsUint16 = new Uint16Array(this.arrayBuffer, p, conftx.COMMANDS16.length);
  this.commandsInt16 = new Int16Array(this.arrayBuffer, p, conftx.COMMANDS16.length);
  p += this.commandsUint16.byteLength;
  for(let i = 0; i < conftx.COMMANDS16.length; i++)
   this.setFloatCommand16(i, conftx.COMMANDS16[i].INIT);

  this.cameraChoices = new Uint8Array(this.arrayBuffer, p, conftx.CAMERACHOICES.length);
  p += this.cameraChoices.byteLength;
  for(let i = 0; i < conftx.CAMERACHOICES.length; i++)
   this.cameraChoices[i] = conftx.CAMERACHOICES[i];

  this.commandsUint8 = new Uint8Array(this.arrayBuffer, p, conftx.COMMANDS8.length);
  this.commandsInt8 = new Int8Array(this.arrayBuffer, p, conftx.COMMANDS8.length);
  p += this.commandsUint8.byteLength;
  for(let i = 0; i < conftx.COMMANDS8.length; i++)
   this.setFloatCommand8(i, conftx.COMMANDS8[i].INIT);

  this.commands1 = new Uint8Array(this.arrayBuffer, p, Math.ceil(conftx.COMMANDS1.length / 8));
  p += this.commands1.byteLength;
  for(let i = 0; i < conftx.COMMANDS1.length; i++)
   this.setCommand1(i, conftx.COMMANDS1[i].INIT);

  this.bytes = new Uint8Array(this.arrayBuffer);
 }

 setCommand32(id, value) {
  if(this.conftx.COMMANDS32[id].SIGNED)
   this.commandsInt32[id] = value;
  else
   this.commandsUint32[id] = value;
 }

 setCommand16(id, value) {
  if(this.conftx.COMMANDS16[id].SIGNED)
   this.commandsInt16[id] = value;
  else
   this.commandsUint16[id] = value;
 }

 setCommand8(id, value) {
  if(this.conftx.COMMANDS8[id].SIGNED)
   this.commandsInt8[id] = value;
  else
   this.commandsUint8[id] = value;
 }

 setCommand1(id, value) {
  let pos = Math.trunc(id / 8);

  if(value)
   this.commands1[pos] |= 1 << id % 8;
  else
   this.commands1[pos] &= ~(1 << id % 8);
 }

 getCommand32(id) {
  let value;

  if(this.conftx.COMMANDS32[id].SIGNED)
   value = this.commandsInt32[id];
  else
   value = this.commandsUint32[id];

  return value;
 }

 getCommand16(id) {
  let value;

  if(this.conftx.COMMANDS16[id].SIGNED)
   value = this.commandsInt16[id];
  else
   value = this.commandsUint16[id];

  return value;
 }

 getCommand8(id) {
  let value;

  if(this.conftx.COMMANDS8[id].SIGNED)
   value = this.commandsInt8[id];
  else
   value = this.commandsUint8[id];

  return value;
 }

 getCommand1(id) {
  let pos = Math.trunc(id / 8);

  return this.commands1[pos] >> id % 8 & 1;
 }

 computeRawCommand32(id, value) {
  let min;
  let max;

  if(this.conftx.COMMANDS32[id].SIGNED) {
   min = -2147483647;
   max = 2147483647;
  } else {
   min = 0;
   max = 4294967295;
  }
  value = constrain(value, this.conftx.COMMANDS32[id].SCALEMIN, this.conftx.COMMANDS32[id].SCALEMAX);
  value = mapTrunc(value, this.conftx.COMMANDS32[id].SCALEMIN, this.conftx.COMMANDS32[id].SCALEMAX, min, max);

  return value;
 }

 computeRawCommand16(id, value) {
  let min;
  let max;

  if(this.conftx.COMMANDS16[id].SIGNED) {
   min = -32767;
   max = 32767;
  } else {
   min = 0;
   max = 65535;
  }
  value = constrain(value, this.conftx.COMMANDS16[id].SCALEMIN, this.conftx.COMMANDS16[id].SCALEMAX);
  value = mapTrunc(value, this.conftx.COMMANDS16[id].SCALEMIN, this.conftx.COMMANDS16[id].SCALEMAX, min, max);

  return value;
 }

 computeRawCommand8(id, value) {
  let min;
  let max;

  if(this.conftx.COMMANDS8[id].SIGNED) {
   min = -127;
   max = 127;
  } else {
   min = 0;
   max = 255;
  }
  value = constrain(value, this.conftx.COMMANDS8[id].SCALEMIN, this.conftx.COMMANDS8[id].SCALEMAX);
  value = mapTrunc(value, this.conftx.COMMANDS8[id].SCALEMIN, this.conftx.COMMANDS8[id].SCALEMAX, min, max);

  return value;
 }

 setFloatCommand32(id, value) {
  value = this.computeRawCommand32(id, value);
  this.setCommand32(id, value);
 }

 setFloatCommand16(id, value) {
  value = this.computeRawCommand16(id, value);
  this.setCommand16(id, value);
 }

 setFloatCommand8(id, value) {
  value = this.computeRawCommand8(id, value);
  this.setCommand8(id, value);
 }

 getFloatCommand32(id) {
  let value;

  if(this.conftx.COMMANDS32[id].SIGNED)
   value = mapFloat(this.commandsInt32[id], -2147483648, 2147483648, this.conftx.COMMANDS32[id].SCALEMIN, this.conftx.COMMANDS32[id].SCALEMAX);
  else
   value = mapFloat(this.commandsUint32[id], 0, 4294967295, this.conftx.COMMANDS32[id].SCALEMIN, this.conftx.COMMANDS32[id].SCALEMAX);

  return value;
 }

 getFloatCommand16(id) {
  let value;

  if(this.conftx.COMMANDS16[id].SIGNED)
   value = mapFloat(this.commandsInt16[id], -32768, 32768, this.conftx.COMMANDS16[id].SCALEMIN, this.conftx.COMMANDS16[id].SCALEMAX);
  else
   value = mapFloat(this.commandsUint16[id], 0, 65535, this.conftx.COMMANDS16[id].SCALEMIN, this.conftx.COMMANDS16[id].SCALEMAX);

  return value;
 }

 getFloatCommand8(id) {
  let value;

  if(this.conftx.COMMANDS8[id].SIGNED)
   value = mapFloat(this.commandsInt8[id], -128, 128, this.conftx.COMMANDS8[id].SCALEMIN, this.conftx.COMMANDS8[id].SCALEMAX);
  else
   value = mapFloat(this.commandsUint8[id], 0, 255, this.conftx.COMMANDS8[id].SCALEMIN, this.conftx.COMMANDS8[id].SCALEMAX);

  return value;
 }
}

class Rx {
 constructor(conftx, confrx) {
  this.conftx = conftx;
  this.confrx = confrx;
  this.pos = 0;

  let nb32 = conftx.COMMANDS32.length + confrx.VALUES32.length;
  let nb16 = conftx.COMMANDS16.length + confrx.VALUES16.length;
  let nb8 = confrx.SYNC.length + conftx.CAMERACHOICES.length +
            conftx.COMMANDS8.length + confrx.VALUES8.length;

  this.byteLength = nb32 * 4 + nb16 * 2 + nb8 + Math.ceil(conftx.COMMANDS1.length / 8);
  this.arrayBuffer = new ArrayBuffer(this.byteLength);

  let p = 0;
  this.sync = new Uint8Array(this.arrayBuffer, p, confrx.SYNC.length);
  p += this.sync.byteLength;
  for(let i = 0; i < confrx.SYNC.length; i++)
   this.sync[i] = confrx.SYNC[i].charCodeAt();

  this.commandsUint32 = new Uint32Array(this.arrayBuffer, p, conftx.COMMANDS32.length);
  this.commandsInt32 = new Int32Array(this.arrayBuffer, p, conftx.COMMANDS32.length);
  p += this.commandsUint32.byteLength;
  for(let i = 0; i < conftx.COMMANDS32.length; i++)
   this.setFloatCommand32(i, conftx.COMMANDS32[i].INIT);

  this.valuesUint32 = new Uint32Array(this.arrayBuffer, p, confrx.VALUES32.length);
  this.valuesInt32 = new Int32Array(this.arrayBuffer, p, confrx.VALUES32.length);
  p += this.valuesUint32.byteLength;
  for(let i = 0; i < confrx.VALUES32.length; i++)
   this.setFloatValue32(i, confrx.VALUES32[i].INIT);

  this.commandsUint16 = new Uint16Array(this.arrayBuffer, p, conftx.COMMANDS16.length);
  this.commandsInt16 = new Int16Array(this.arrayBuffer, p, conftx.COMMANDS16.length);
  p += this.commandsUint16.byteLength;
  for(let i = 0; i < conftx.COMMANDS16.length; i++)
   this.setFloatCommand16(i, conftx.COMMANDS16[i].INIT);

  this.valuesUint16 = new Uint16Array(this.arrayBuffer, p, confrx.VALUES16.length);
  this.valuesInt16 = new Int16Array(this.arrayBuffer, p, confrx.VALUES16.length);
  p += this.valuesUint16.byteLength;
  for(let i = 0; i < confrx.VALUES16.length; i++)
   this.setFloatValue16(i, confrx.VALUES16[i].INIT);

  this.cameraChoices = new Uint8Array(this.arrayBuffer, p, conftx.CAMERACHOICES.length);
  p += this.cameraChoices.byteLength;
  for(let i = 0; i < conftx.CAMERACHOICES.length; i++)
   this.cameraChoices[i] = conftx.CAMERACHOICES[i];

  this.commandsUint8 = new Uint8Array(this.arrayBuffer, p, conftx.COMMANDS8.length);
  this.commandsInt8 = new Int8Array(this.arrayBuffer, p, conftx.COMMANDS8.length);
  p += this.commandsUint8.byteLength;
  for(let i = 0; i < conftx.COMMANDS8.length; i++)
   this.setFloatCommand8(i, conftx.COMMANDS8[i].INIT);

  this.commands1 = new Uint8Array(this.arrayBuffer, p, Math.ceil(conftx.COMMANDS1.length / 8));
  p += this.commands1.byteLength;
  for(let i = 0; i < conftx.COMMANDS1.length; i++)
   this.setCommand1(i, conftx.COMMANDS1[i].INIT);

  this.valuesUint8 = new Uint8Array(this.arrayBuffer, p, confrx.VALUES8.length);
  this.valuesInt8 = new Int8Array(this.arrayBuffer, p, confrx.VALUES8.length);
  p += this.valuesUint8.byteLength;
  for(let i = 0; i < confrx.VALUES8.length; i++)
   this.setFloatValue8(i, confrx.VALUES8[i].INIT);

  this.bytes = new Uint8Array(this.arrayBuffer);
 }

 update(data, success, err) {
  let i = 0;
  while(i < data.byteLength) {

   switch(this.pos) {
    case 0:
     if(data[i] == FRAME0)
      this.pos++;
     else
      err("Premier octet de la trame télémétrique invalide");
     break;

    case 1:
     if(data[i] == FRAME1R)
      this.pos++;
     else {
      this.pos = 0;
      err("Second octet de la trame télémétrique invalide");
     }
     break;

    default:
     this.bytes[this.pos++] = data[i];
     if(this.pos == this.byteLength) {
      success();
      this.pos = 0;
     }
     break;
   }

   i++;
  }
 }

 setCommand32(id, value) {
  if(this.conftx.COMMANDS32[id].SIGNED)
   this.commandsInt32[id] = value;
  else
   this.commandsUint32[id] = value;
 }

 setCommand16(id, value) {
  if(this.conftx.COMMANDS16[id].SIGNED)
   this.commandsInt16[id] = value;
  else
   this.commandsUint16[id] = value;
 }

 setCommand8(id, value) {
  if(this.conftx.COMMANDS8[id].SIGNED)
   this.commandsInt8[id] = value;
  else
   this.commandsUint8[id] = value;
 }

 setCommand1(id, value) {
  let pos = Math.trunc(id / 8);

  if(value)
   this.commands1[pos] |= 1 << id % 8;
  else
   this.commands1[pos] &= ~(1 << id % 8);
 }

 getCommand32(id) {
  let value;

  if(this.conftx.COMMANDS32[id].SIGNED)
   value = this.commandsInt32[id];
  else
   value = this.commandsUint32[id];

  return value;
 }

 getCommand16(id) {
  let value;

  if(this.conftx.COMMANDS16[id].SIGNED)
   value = this.commandsInt16[id];
  else
   value = this.commandsUint16[id];

  return value;
 }

 getCommand8(id) {
  let value;

  if(this.conftx.COMMANDS8[id].SIGNED)
   value = this.commandsInt8[id];
  else
   value = this.commandsUint8[id];

  return value;
 }

 getCommand1(id) {
  let pos = Math.trunc(id / 8);

  return this.commands1[pos] >> id % 8 & 1;
 }

 computeRawCommand32(id, value) {
  let min;
  let max;

  if(this.conftx.COMMANDS32[id].SIGNED) {
   min = -2147483647;
   max = 2147483647;
  } else {
   min = 0;
   max = 4294967295;
  }
  value = constrain(value, this.conftx.COMMANDS32[id].SCALEMIN, this.conftx.COMMANDS32[id].SCALEMAX);
  value = mapTrunc(value, this.conftx.COMMANDS32[id].SCALEMIN, this.conftx.COMMANDS32[id].SCALEMAX, min, max);

  return value;
 }

 computeRawCommand16(id, value) {
  let min;
  let max;

  if(this.conftx.COMMANDS16[id].SIGNED) {
   min = -32767;
   max = 32767;
  } else {
   min = 0;
   max = 65535;
  }
  value = constrain(value, this.conftx.COMMANDS16[id].SCALEMIN, this.conftx.COMMANDS16[id].SCALEMAX);
  value = mapTrunc(value, this.conftx.COMMANDS16[id].SCALEMIN, this.conftx.COMMANDS16[id].SCALEMAX, min, max);

  return value;
 }

 computeRawCommand8(id, value) {
  let min;
  let max;

  if(this.conftx.COMMANDS8[id].SIGNED) {
   min = -127;
   max = 127;
  } else {
   min = 0;
   max = 255;
  }
  value = constrain(value, this.conftx.COMMANDS8[id].SCALEMIN, this.conftx.COMMANDS8[id].SCALEMAX);
  value = mapTrunc(value, this.conftx.COMMANDS8[id].SCALEMIN, this.conftx.COMMANDS8[id].SCALEMAX, min, max);

  return value;
 }

 setFloatCommand32(id, value) {
  value = this.computeRawCommand32(id, value);
  this.setCommand32(id, value);
 }

 setFloatCommand16(id, value) {
  value = this.computeRawCommand16(id, value);
  this.setCommand16(id, value);
 }

 setFloatCommand8(id, value) {
  value = this.computeRawCommand8(id, value);
  this.setCommand8(id, value);
 }

 getFloatCommand32(id) {
  let value;

  if(this.conftx.COMMANDS32[id].SIGNED)
   value = mapFloat(this.commandsInt32[id], -2147483648, 2147483648, this.conftx.COMMANDS32[id].SCALEMIN, this.conftx.COMMANDS32[id].SCALEMAX);
  else
   value = mapFloat(this.commandsUint32[id], 0, 4294967295, this.conftx.COMMANDS32[id].SCALEMIN, this.conftx.COMMANDS32[id].SCALEMAX);

  return value;
 }

 getFloatCommand16(id) {
  let value;

  if(this.conftx.COMMANDS16[id].SIGNED)
   value = mapFloat(this.commandsInt16[id], -32768, 32768, this.conftx.COMMANDS16[id].SCALEMIN, this.conftx.COMMANDS16[id].SCALEMAX);
  else
   value = mapFloat(this.commandsUint16[id], 0, 65535, this.conftx.COMMANDS16[id].SCALEMIN, this.conftx.COMMANDS16[id].SCALEMAX);

  return value;
 }

 getFloatCommand8(id) {
  let value;

  if(this.conftx.COMMANDS8[id].SIGNED)
   value = mapFloat(this.commandsInt8[id], -128, 128, this.conftx.COMMANDS8[id].SCALEMIN, this.conftx.COMMANDS8[id].SCALEMAX);
  else
   value = mapFloat(this.commandsUint8[id], 0, 255, this.conftx.COMMANDS8[id].SCALEMIN, this.conftx.COMMANDS8[id].SCALEMAX);

  return value;
 }

 getTextCommand32(id) {
  return this.getFloatCommand32(id).toFixed(this.conftx.COMMANDS32[id].NBDIGITS) + this.conftx.COMMANDS32[id].UNIT;
 }

 getTextCommand16(id) {
  return this.getFloatCommand16(id).toFixed(this.conftx.COMMANDS16[id].NBDIGITS) + this.conftx.COMMANDS16[id].UNIT;
 }

 getTextCommand8(id) {
  return this.getFloatCommand8(id).toFixed(this.conftx.COMMANDS8[id].NBDIGITS) + this.conftx.COMMANDS8[id].UNIT;
 }

 getTextCommand1(id) {
  if(this.getCommand1(id))
   return "On";
  else
   return "Off";
 }

 setValue32(id, value) {
  if(this.confrx.VALUES32[id].SIGNED)
   this.valuesInt32[id] = value;
  else
   this.valuesUint32[id] = value;
 }

 setValue16(id, value) {
  if(this.confrx.VALUES16[id].SIGNED)
   this.valuesInt16[id] = value;
  else
   this.valuesUint16[id] = value;
 }

 setValue8(id, value) {
  if(this.confrx.VALUES8[id].SIGNED)
   this.valuesInt8[id] = value;
  else
   this.valuesUint8[id] = value;
 }

 getValue32(id) {
  let value;

  if(this.confrx.VALUES32[id].SIGNED)
   value = this.valuesInt32[id];
  else
   value = this.valuesUint32[id];

  return value;
 }

 getValue16(id) {
  let value;

  if(this.confrx.VALUES16[id].SIGNED)
   value = this.valuesInt16[id];
  else
   value = this.valuesUint16[id];

  return value;
 }

 getValue8(id) {
  let value;

  if(this.confrx.VALUES8[id].SIGNED)
   value = this.valuesInt8[id];
  else
   value = this.valuesUint8[id];

  return value;
 }

 computeRawValue32(id, value) {
  let min;
  let max;

  if(this.confrx.VALUES32[id].SIGNED) {
   min = -2147483647;
   max = 2147483647;
  } else {
   min = 0;
   max = 4294967295;
  }
  value = constrain(value, this.confrx.VALUES32[id].SCALEMIN, this.confrx.VALUES32[id].SCALEMAX);
  value = mapTrunc(value, this.confrx.VALUES32[id].SCALEMIN, this.confrx.VALUES32[id].SCALEMAX, min, max);

  return value;
 }

 computeRawValue16(id, value) {
  let min;
  let max;

  if(this.confrx.VALUES16[id].SIGNED) {
   min = -32767;
   max = 32767;
  } else {
   min = 0;
   max = 65535;
  }
  value = constrain(value, this.confrx.VALUES16[id].SCALEMIN, this.confrx.VALUES16[id].SCALEMAX);
  value = mapTrunc(value, this.confrx.VALUES16[id].SCALEMIN, this.confrx.VALUES16[id].SCALEMAX, min, max);

  return value;
 }

 computeRawValue8(id, value) {
  let min;
  let max;

  if(this.confrx.VALUES8[id].SIGNED) {
   min = -127;
   max = 127;
  } else {
   min = 0;
   max = 255;
  }
  value = constrain(value, this.confrx.VALUES8[id].SCALEMIN, this.confrx.VALUES8[id].SCALEMAX);
  value = mapTrunc(value, this.confrx.VALUES8[id].SCALEMIN, this.confrx.VALUES8[id].SCALEMAX, min, max);

  return value;
 }

 setFloatValue32(id, value) {
  value = this.computeRawValue32(id, value);
  this.setValue32(id, value);
 }

 setFloatValue16(id, value) {
  value = this.computeRawValue16(id, value);
  this.setValue16(id, value);
 }

 setFloatValue8(id, value) {
  value = this.computeRawValue8(id, value);
  this.setValue8(id, value);
 }

 getFloatValue32(id) {
  let value;

  if(this.confrx.VALUES32[id].SIGNED)
   value = mapFloat(this.valuesInt32[id], -2147483648, 2147483648, this.confrx.VALUES32[id].SCALEMIN, this.confrx.VALUES32[id].SCALEMAX);
  else
   value = mapFloat(this.valuesUint32[id], 0, 4294967295, this.confrx.VALUES32[id].SCALEMIN, this.confrx.VALUES32[id].SCALEMAX);

  return value;
 }

 getFloatValue16(id) {
  let value;

  if(this.confrx.VALUES16[id].SIGNED)
   value = mapFloat(this.valuesInt16[id], -32768, 32768, this.confrx.VALUES16[id].SCALEMIN, this.confrx.VALUES16[id].SCALEMAX);
  else
   value = mapFloat(this.valuesUint16[id], 0, 65535, this.confrx.VALUES16[id].SCALEMIN, this.confrx.VALUES16[id].SCALEMAX);

  return value;
 }

 getFloatValue8(id) {
  let value;

  if(this.confrx.VALUES8[id].SIGNED)
   value = mapFloat(this.valuesInt8[id], -128, 128, this.confrx.VALUES8[id].SCALEMIN, this.confrx.VALUES8[id].SCALEMAX);
  else
   value = mapFloat(this.valuesUint8[id], 0, 255, this.confrx.VALUES8[id].SCALEMIN, this.confrx.VALUES8[id].SCALEMAX);

  return value;
 }

 getTextValue32(id) {
  return this.getFloatValue32(id).toFixed(this.confrx.VALUES32[id].NBDIGITS) + this.confrx.VALUES32[id].UNIT;
 }

 getTextValue16(id) {
  return this.getFloatValue16(id).toFixed(this.confrx.VALUES16[id].NBDIGITS) + this.confrx.VALUES16[id].UNIT;
 }

 getTextValue8(id) {
  return this.getFloatValue8(id).toFixed(this.confrx.VALUES8[id].NBDIGITS) + this.confrx.VALUES8[id].UNIT;
 }
}

module.exports = {Tx, Rx};
