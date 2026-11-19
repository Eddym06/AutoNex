/* Depuracion de una ronda: node simulator/tests/inspect.js <seed> <ccw|cw> [signs] [tFrom] [tTo] */
const fs = require('node:fs'), path = require('node:path');
const web = path.join(__dirname, '../web');
const WRO = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));
(async () => {
  const core = await WRO.core.load(fs.readFileSync(path.join(web, 'wro_core.wasm')));
  const seed = Number(process.argv[2] || 1), direction = process.argv[3] === 'cw' ? 'cw' : 'ccw';
  const signs = Number(process.argv[4] || 8);
  const tFrom = Number(process.argv[5] || 0), tTo = Number(process.argv[6] || 200);
  const seatModel = process.env.SEAT_ALONG ? { seatAlong: process.env.SEAT_ALONG.split(',').map(Number), seatLateral: (process.env.SEAT_LAT || '-0.2,0,0.2').split(',').map(Number) } : null;
  const field = WRO.field.randomLayout(seed, { direction, signs, parking: false, seatModel });
  console.log('start', JSON.stringify(WRO.field.startPose(field)));
  console.log('signs', field.signs.map(s => `${s.color[0]}(${s.x.toFixed(0)},${s.z.toFixed(0)})`).join(' '));
  const sim = new WRO.sim.Simulation(core, { field, seed, ctrlConfig: (process.env.CTRL || '').split(';').join('\n') });
  let last = -1;
  while (sim.t < tTo && !sim.finished) {
    sim.tick();
    if (sim.t >= tFrom && sim.t - last >= 0.4) {
      last = sim.t;
      const c = sim.cmd || {}, p = sim.robot.pose();
      const d = sim.snapshot.debug || {};
      console.log(`t=${sim.t.toFixed(1)} pos=(${p.x.toFixed(0)},${p.z.toFixed(0)}) yaw=${(WRO.geometry.wrapDeg(p.yaw * 180 / Math.PI)).toFixed(0)} ${c.phase}/${c.action} st=${(c.steer || 0).toFixed(0)} v=${sim.robot.v.toFixed(0)} corners=${c.corners} F=${c.front>1e6?'-':c.front.toFixed(0)} L=${c.left>1e6?'-':c.left.toFixed(0)} R=${c.right>1e6?'-':c.right.toFixed(0)} crs=${(c.course||0).toFixed(0)} lane=${(c.lane || 0).toFixed(0)} pass=${(c.pass || 0).toFixed(0)} esc=${c.escaping ? c.escapePhase : '-'} curt=${d.curtains || 0} col=${(d.colored || []).map(o => (o.color === 1 ? 'r' : 'g') + (o.confirmed ? '*' : '?') + o.x.toFixed(0) + ',' + o.forward.toFixed(0)).join(' ')}`);
    }
  }
  const r = sim.result();
  console.log(JSON.stringify(r.violations), r.endReason, 'points', r.points);
  console.log('passes', JSON.stringify(sim.judge.passes));
  console.log('ctrl lastSign', sim.cmd && sim.cmd.lastSign);
  console.log(sim.log.events.slice(-12));
})();
