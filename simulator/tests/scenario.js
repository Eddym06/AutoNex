/* node simulator/tests/scenario.js '<json signs>' [ccw|cw] [tFrom] [tTo] [step]
 * Ejemplo: node simulator/tests/scenario.js '[{"color":"red","x":100,"z":0}]' ccw 14 24 */
const fs = require('node:fs'), path = require('node:path');
const web = path.join(__dirname, '../web');
const WRO = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));
(async () => {
  const core = await WRO.core.load(fs.readFileSync(path.join(web, 'wro_core.wasm')));
  const signs = JSON.parse(process.argv[2] || '[]');
  const direction = process.argv[3] === 'cw' ? 'cw' : 'ccw';
  const tFrom = Number(process.argv[4] || 0), tTo = Number(process.argv[5] || 60), step = Number(process.argv[6] || 0.4);
  const field = WRO.field.create({ direction, signs, start: { along: 20 } });
  const sim = new WRO.sim.Simulation(core, { field, seed: 7, ctrlConfig: (process.env.CTRL || '').split(';').join('\n') });
  let last = -1;
  while (sim.t < tTo && !sim.finished) {
    sim.tick();
    if (sim.t >= tFrom && sim.t - last >= step) {
      last = sim.t;
      const c = sim.cmd || {}, p = sim.robot.pose();
      const d = sim.snapshot.debug || {};
      console.log(`t=${sim.t.toFixed(1)} pos=(${p.x.toFixed(0)},${p.z.toFixed(0)}) yaw=${(WRO.geometry.wrapDeg(p.yaw * 180 / Math.PI)).toFixed(0)} ${c.phase}/${c.action} st=${(c.steer || 0).toFixed(0)} v=${sim.robot.v.toFixed(0)} cor=${c.corners} lane=${(c.lane || 0).toFixed(0)} pass=${(c.pass || 0).toFixed(0)} esc=${c.escaping ? c.escapePhase : '-'} curt=${d.curtains || 0} col=${(d.colored || []).map(o => (o.color === 1 ? 'r' : 'g') + (o.confirmed ? '*' : '?') + o.x.toFixed(0) + ',' + o.forward.toFixed(0)).join(' ')}`);
    }
  }
  const r = sim.result();
  console.log(JSON.stringify({ end: r.endReason, sections: r.sections, viol: r.violations, contacts: r.contacts }));
})();
