/* Exporta de una ronda simulada (con y sin IT) los datos con que se dibujan los graficos y las animaciones del README:
 * pista, letreros, traza del robot (t, x, z, velocidad, fase, IT activo), trayectoria que descubre IT y mapa que construye trace_map.
 *   node simulator/tools/export_readme_data.js [semilla=5] [ccw|cw] [salida.json] */
'use strict';
const fs = require('node:fs'), path = require('node:path');
const web = path.join(__dirname, '../web'), W = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));

async function run(core, seed, direction, it) {
  const field = W.field.randomLayout(seed, { direction, signs: 3, parking: true, startInLot: true });
  const robot = Object.assign({}, W.robot.PROFILES.largo26d, { maxAccel: 350, speedTauS: 0.08, speedGain: 1 });
  const sim = new W.sim.Simulation(core, { field, seed, robot, ctrlConfig: it ? 'it_enabled 1' : '' });
  const trace = [], snaps = {};
  let line = null, lastLap = 0;
  while (sim.t < 240 && !sim.finished) {
    sim.run(0.05);
    const c = sim.cmd;
    if (!c) continue;
    const p = sim.robot.pose();
    const st = it ? core.itState(false) : null;
    trace.push([+sim.t.toFixed(2), +p.x.toFixed(1), +p.z.toFixed(1), +(p.yaw * 180 / Math.PI).toFixed(1), +sim.robot.v.toFixed(1), c.phase, Math.floor(c.corners / 4) + 1, st && st.active ? 1 : 0]);
    if (it && !line && st.valid) line = core.itState(true).line.map(q => [+q.x.toFixed(1), +q.z.toFixed(1), +q.v.toFixed(0)]);
    const lap = Math.floor(c.corners / 4) + 1;
    if (lap !== lastLap) { lastLap = lap; if (lap === 2) snaps.map1 = core.traceMap(); }
  }
  const m = snaps.map1 || core.traceMap();
  const r = sim.run(0.1);
  return {
    seed, direction, it, end: r.endReason, points: r.points, time: sim.t, contacts: r.contacts.sign + r.contacts.wall, trace, line,
    field: { signs: field.signs.map(q => ({ color: q.color, x: q.x, z: q.z, w: q.w, d: q.d, rot: q.rot || 0 })), walls: field.walls.map(w => ({ x: w.x, z: w.z, w: w.w, d: w.d })),
             lot: field.lot ? { limiters: field.lot.limiters.map(b => ({ x: b.x, z: b.z, w: b.w, d: b.d, rot: b.rot || 0 })) } : null },
    map: { size: m.size, cellCm: m.cellCm, originCm: m.originCm, cells: Buffer.from(m.cells).toString('base64'), objects: core.traceObjects(), spot: core.traceSpot() }
  };
}

(async () => {
  const seed = Number(process.argv[2] || 5), dir = process.argv[3] || 'ccw', out = process.argv[4] || path.join(__dirname, '../mediciones/readme_ronda.json');
  const core = await W.core.load(fs.readFileSync(path.join(web, 'wro_core.wasm')));
  const base = await run(core, seed, dir, false), it = await run(core, seed, dir, true);
  fs.writeFileSync(out, JSON.stringify({ base, it }));
  console.log(`base: ${base.end} ${base.points} pts ${base.time.toFixed(0)} s | IT: ${it.end} ${it.points} pts ${it.time.toFixed(0)} s | ${out}`);
})().catch(e => { console.error(e); process.exit(1); });
