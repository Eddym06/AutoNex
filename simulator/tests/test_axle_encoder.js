/* Prueba pequeña del encoder de eje; no ejecuta el simulador. */
const assert = require('node:assert/strict');
const WRO = require('../web/rng.js');
require('../web/sensors.js');
const encoder = new WRO.sensors.Encoders({ motorOutputCountsPerRev: 440, motorGearTeeth: 10,
  diffGearTeeth: 20, noiseDeg: 0 }, WRO.rng(1));
const sample = encoder.sampleAxle({ encL: 90, encR: 270 });
assert(Math.abs(sample - 180) <= 360 / 880);
const backward = encoder.sampleAxle({ encL: -90, encR: -270 });
assert(Math.abs(backward + 180) <= 360 / 880);
assert.throws(() => new WRO.sensors.Encoders({ diffGearTeeth: 0 }, WRO.rng(1)), /inválida/);
console.log('PASS: encoder simulado de eje y reducción externa');
