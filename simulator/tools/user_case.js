/* Corre un plano JSON (el de la interfaz) como la interfaz: IT + cajon + chasis largo26d, y vuelca la trayectoria verdadera.
 *   node simulator/tools/user_case.js plano.json [--it 0|1] [--startlot 0|1] [--out traza.json] [--limit 240]
 * Salida: resumen y, con --out, la traza (t, fase, x, z, yaw, v, orden) cada 0,05 s. Sirve para ver el parqueo y las vueltas de IT. */
'use strict';
const fs = require('node:fs'), path = require('node:path');
const web = path.join(__dirname, '../web'), W = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));
const fl = (n, d) => { const i = process.argv.indexOf(n); return i >= 0 ? process.argv[i + 1] : d; };
(async () => {
  const core = await W.core.load(fs.readFileSync(path.join(web, 'wro_core.wasm')));
  const field = W.field.parse(fs.readFileSync(process.argv[2], 'utf8'));
  if (fl('--startlot', '1') === '1') field.startInLot = true;
  const it = fl('--it', '1') === '1';
  const robot = Object.assign({}, W.robot.PROFILES.largo26d, { speedTauS: 0.08, speedGain: 1 });
  const sim = new W.sim.Simulation(core, { field, seed: field.seed || 1, robot, ctrlConfig: (fl('--cfg', '') + (it ? '\nit_enabled 1' : '')).trim() });
  const rows = []; let next = 0, phase = '', lastCorners = -1; const marks = [];
  const limit = Number(fl('--limit', 240));
  while (sim.t < limit && !sim.finished) {
    sim.tick();
    const c = sim.cmd; if (!c) continue;
    if (c.phase !== phase) { marks.push(`${c.phase}@${sim.t.toFixed(1)}`); phase = c.phase; }
    if (c.corners !== lastCorners) { marks.push(`esq${c.corners}@${sim.t.toFixed(1)}`); lastCorners = c.corners; }
    if (sim.t >= next) { next += 0.05; const p = sim.robot.pose(); const st = it ? core.itState(false) : {};
      rows.push([+sim.t.toFixed(2), c.phase, +p.x.toFixed(2), +p.z.toFixed(2), +(p.yaw * 180 / Math.PI).toFixed(1), +sim.robot.v.toFixed(1), +(c.speed || 0).toFixed(1), +(c.steer || 0).toFixed(1), st.active ? 1 : 0, +(st.e || 0).toFixed(1), +(st.vPlan || 0).toFixed(0), st.idx || 0]); }
  }
  const r = sim.run(0.1);
  console.log(JSON.stringify({ end: r.endReason, points: r.points, t: +sim.t.toFixed(1), contacts: r.contacts, parking: r.parking, marks }, null, 1));
  if (fl('--out')) fs.writeFileSync(fl('--out'), JSON.stringify(rows));
  if (fl('--plan') && it) fs.writeFileSync(fl('--plan'), JSON.stringify(core.itState(true).line));
})().catch(e => { console.error(e); process.exit(1); });
