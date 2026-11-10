/* trace_map v2 en una mision simulada con el controlador C real: acumulacion, espacio libre, objetos con color y posicion,
 * y plaza de parqueo, medidos contra la geometria verdadera (usada solo para evaluar). */
const fs = require('node:fs'), path = require('node:path'), assert = require('node:assert/strict');
const web = path.join(__dirname, '../web');
const W = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));

(async () => {
  const core = await W.core.load(fs.readFileSync(path.join(web, 'wro_core.wasm')));
  const field = W.field.randomLayout(2, { direction: 'cw', signs: 3, parking: true, startInLot: true });
  const sim = new W.sim.Simulation(core, { field, seed: 2 });
  sim.run(20);
  const first = core.traceMap();
  assert.equal(first.size, 160);
  assert.equal(first.cellCm, 2.5);
  assert(first.scans > 0 && first.occupied > 0, 'sin barridos acumulados');
  sim.run(120);
  const m = core.traceMap();
  assert(m.scans > first.scans);
  const count = bit => m.cells.reduce((n, v) => n + !!(v & bit), 0);
  assert.equal(count(1), m.occupied, 'el contador de celdas ocupadas no cuadra con la rejilla');
  const free = m.cells.reduce((n, v, i) => n + (!(v & 1) && m.miss[i] >= 2), 0);
  assert(free > 5000, 'sin espacio libre trazado');

  // letreros: cada uno de la verdad debe estar como pilar a menos de 3 cm
  const pillars = core.traceObjects().filter(o => o.kind === 2);
  let errMax = 0;
  for (const s of field.signs) {
    const best = Math.min(...pillars.map(o => Math.hypot(o.x - s.x, o.z - s.z)));
    assert(best < 3, `letrero en (${s.x},${s.z}) sin objeto cercano (${best.toFixed(1)} cm)`);
    errMax = Math.max(errMax, best);
  }
  // plaza
  const spot = core.traceSpot(), lot = field.lot;
  assert.equal(spot.status, 2, 'plaza no confirmada');
  const truth = lot.axis === 'x' ? { x: (lot.a0 + lot.a1) / 2, z: lot.lateralCenter } : { x: lot.lateralCenter, z: (lot.a0 + lot.a1) / 2 };
  const spotErr = Math.hypot(spot.x - truth.x, spot.z - truth.z);
  assert(spotErr < 2.5, `plaza a ${spotErr.toFixed(2)} cm de la verdad`);
  assert(Math.abs(spot.length - lot.length) < 4, `largo de la plaza ${spot.length} frente a ${lot.length}`);
  console.log(JSON.stringify({ seconds: +sim.t.toFixed(1), scans: m.scans, occupied: m.occupied, freeCells: free, cleared: m.cleared,
    objects: core.traceObjects().length, signErrMaxCm: +errMax.toFixed(2), spotErrCm: +spotErr.toFixed(2), ramBytes: m.bytes }));
  console.log('PASS: trace_map v2 (rejilla, espacio libre, objetos, plaza) en una mision simulada');
})().catch(e => { console.error(e); process.exit(1); });
