/* Mide los objetos y la plaza del mapa persistente (trace_map v2) contra la geometria verdadera del simulador.
 *   node simulator/tools/measure_trace_objects.js [semillas=6] [segundos=140] [letreros=5]
 * DIR=cw|ccw limita el sentido. La verdad se usa solo despues, para evaluar; el controlador no la ve. */
'use strict';
const fs = require('node:fs'), path = require('node:path');
const web = path.join(__dirname, '../web');
const W = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));
const KIND = { 1: 'pared', 2: 'pilar', 3: 'barra' }, COLOR = { 0: '-', 1: 'rojo', 2: 'verde', 3: 'magenta' };

(async () => {
  const core = await W.core.load(fs.readFileSync(path.join(web, 'wro_core.wasm')));
  const seeds = Number(process.argv[2] || 6), seconds = Number(process.argv[3] || 140), signs = Number(process.argv[4] || 5);
  const dirs = process.env.DIR ? [process.env.DIR] : ['ccw', 'cw'];
  const rows = [];
  for (const direction of dirs) for (let seed = 1; seed <= seeds; seed++) {
    const field = W.field.randomLayout(seed, { direction, signs, parking: true, startInLot: true });
    const sim = new W.sim.Simulation(core, { field, seed });
    sim.run(seconds);
    const objs = core.traceObjects(), spot = core.traceSpot(), tm = core.traceMap();
    const pillars = objs.filter(o => o.kind === 2);
    const truth = field.signs.map(s => ({ x: s.x, z: s.z, color: s.color === 'red' ? 1 : s.color === 'green' ? 2 : 0 }));
    let found = 0, colorOk = 0; const errs = [];
    const used = new Set();
    for (const t of truth) {
      let best = -1, bd = 10;
      pillars.forEach((o, i) => { const d = Math.hypot(o.x - t.x, o.z - t.z); if (!used.has(i) && d < bd) { bd = d; best = i; } });
      if (best >= 0) { used.add(best); found++; errs.push(bd); if (pillars[best].color === t.color) colorOk++; }
    }
    const lot = field.lot;
    const truthSpot = lot.axis === 'x' ? { x: (lot.a0 + lot.a1) / 2, z: lot.lateralCenter } : { x: lot.lateralCenter, z: (lot.a0 + lot.a1) / 2 };
    const spotErr = spot.status === 2 ? Math.hypot(spot.x - truthSpot.x, spot.z - truthSpot.z) : null;
    // limpio de la caja: mapa de verdad pintado en celdas para medir falsos objetos
    const falseObjects = pillars.length - found;
    rows.push({ direction, seed, t: +sim.t.toFixed(0), objects: objs.length, signs: truth.length, found, colorOk,
      meanErr: errs.length ? +(errs.reduce((a, b) => a + b, 0) / errs.length).toFixed(2) : null, maxErr: errs.length ? +Math.max(...errs).toFixed(2) : null,
      falseObjects, spotStatus: spot.status, spotErr: spotErr === null ? null : +spotErr.toFixed(2), spotLen: +spot.length.toFixed(1),
      bars: objs.filter(o => o.kind === 3).length, cleared: tm.cleared, occupied: tm.occupied });
    console.log(JSON.stringify(rows[rows.length - 1]));
  }
  const sum = (a) => a.reduce((x, y) => x + y, 0);
  const total = sum(rows.map(r => r.signs)), found = sum(rows.map(r => r.found)), colorOk = sum(rows.map(r => r.colorOk));
  const spots = rows.filter(r => r.spotStatus === 2);
  console.log(`letreros: ${found}/${total} encontrados, ${colorOk}/${found} con el color correcto, falsos objetos: ${sum(rows.map(r => r.falseObjects))}`);
  console.log(`plaza confirmada en ${spots.length}/${rows.length} rondas; error medio ${spots.length ? (sum(spots.map(r => r.spotErr)) / spots.length).toFixed(2) : '-'} cm, maximo ${spots.length ? Math.max(...spots.map(r => r.spotErr)).toFixed(2) : '-'} cm`);
})().catch(e => { console.error(e); process.exit(1); });
