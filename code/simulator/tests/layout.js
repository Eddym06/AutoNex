/* Reproduce un plano exportado (Plano <-> JSON): node simulator/tests/layout.js plano.json [tFrom] [tTo] [step] [seed]
 * Sin tFrom/tTo hace la ronda completa y escribe el resultado del juez. */
const fs = require('node:fs'), path = require('node:path');
const web = path.join(__dirname, '../web');
const WRO = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));
(async () => {
  const core = await WRO.core.load(fs.readFileSync(path.join(web, 'wro_core.wasm')));
  const field = WRO.field.parse(fs.readFileSync(process.argv[2], 'utf8'));
  const tFrom = Number(process.argv[3] || 0), tTo = Number(process.argv[4] || 200), step = Number(process.argv[5] || 0.5);
  const seed = Number(process.argv[6] || field.seed || 1);
  const sim = new WRO.sim.Simulation(core, { field, seed, ctrlConfig: (process.env.CTRL || '').split(';').join('\n') });
  let last = -1;
  while (sim.t < tTo && !sim.finished) {
    sim.tick();
    if (process.argv[3] !== undefined && sim.t >= tFrom && sim.t - last >= step) {
      last = sim.t;
      const c = sim.cmd || {}, p = sim.robot.pose();
      console.log(`t=${sim.t.toFixed(1)} pos=(${p.x.toFixed(0)},${p.z.toFixed(0)}) yaw=${WRO.geometry.wrapDeg(p.yaw * 180 / Math.PI).toFixed(0)} ${c.phase}/${c.action} st=${(c.steer || 0).toFixed(0)} v=${sim.robot.v.toFixed(0)} cor=${c.corners} esc=${c.escaping ? c.escapePhase : '-'}`);
    }
  }
  const r = sim.result();
  console.log(JSON.stringify({ end: r.endReason, pts: r.points, sections: r.sections, viol: r.violations, contacts: r.contacts, t: +(r.elapsed || r.t).toFixed(0) }));
})();
