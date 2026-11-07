/* Maniobra de parqueo: pose estimada por el LiDAR frente a la verdad del simulador, en el marco del cajon.
 * node simulator/tests/parktrace.js plano.json tFrom tTo step */
const fs = require('node:fs'), path = require('node:path');
const web = path.join(__dirname, '../web');
const WRO = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));
(async () => {
  const core = await WRO.core.load(fs.readFileSync(path.join(web, 'wro_core.wasm')));
  const field = WRO.field.parse(fs.readFileSync(process.argv[2], 'utf8'));
  const a = Number(process.argv[3] || 0), b = Number(process.argv[4] || 30), step = Number(process.argv[5] || 0.5);
  const sim = new WRO.sim.Simulation(core, { field, seed: field.seed || 1, ctrlConfig: (process.env.CTRL || '').split(';').join('\n') });
  const lot = field.lot, ccw = field.direction !== "cw", mirror = ccw ? 1 : -1;
  const centre = (lot.a0 + lot.a1) / 2;
  let last = -1;
  while (sim.t < b && !sim.finished) {
    sim.tick();
    if (sim.t >= a && sim.t - last >= step) {
      last = sim.t; const c = sim.cmd || {}, p = sim.robot.pose();
      /* verdad en el marco del cajon (solo la recta sur, sentido antihorario: u = x, w = z + 150) */
      const u = (ccw ? 1 : -1) * (p.x - centre), w = p.z - lot.wallCoord;
      const yawDeg = p.yaw * 180 / Math.PI;
      let th = ccw ? 90 - yawDeg : -(-90 - yawDeg) * -1; th = ccw ? (90 - yawDeg) : (yawDeg + 90);
      th = ((th + 540) % 360) - 180;
      const e = c.park || {};
      console.log(`t=${sim.t.toFixed(1)} ${c.phase}/${c.action} v=${sim.robot.v.toFixed(0)} verdad(u=${u.toFixed(1)}, w=${w.toFixed(1)}, th=${th.toFixed(0)}) estima(u=${(e.u || 0).toFixed(1)}, w=${(e.w || 0).toFixed(1)}, th=${(e.theta || 0).toFixed(0)}) lost=${e.lost} mov=${e.move} cmdYaw=${(c.yawRate||0).toFixed(0)} speed=${(c.speed||0).toFixed(0)}`);
    }
  }
  const r = sim.result();
  console.log(JSON.stringify({ end: r.endReason, pts: r.points, parking: r.parking, viol: r.violations.map(v => v.kind) }));
})();
