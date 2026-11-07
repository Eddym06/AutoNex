/* Prueba manual: node simulator/tests/smoke.js [ccw|cw] [segundos] */
const fs = require('node:fs'), path = require('node:path');
const web = path.join(__dirname, '../web');
const WRO = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));

(async () => {
  const core = await WRO.core.load(fs.readFileSync(path.join(web, 'wro_core.wasm')));
  const direction = process.argv[2] === 'cw' ? 'cw' : 'ccw';
  const seconds = Number(process.argv[3] || 60);
  const field = WRO.field.create({ direction });
  const sim = new WRO.sim.Simulation(core, { field, seed: 3 });
  const t0 = Date.now();
  let lastPrint = 0;
  while (sim.t < seconds && !sim.finished) {
    sim.tick();
    if (sim.t - lastPrint >= 2) {
      lastPrint = sim.t;
      const c = sim.cmd || {};
      const p = sim.robot.pose();
      console.log(`t=${sim.t.toFixed(1)} pos=(${p.x.toFixed(0)},${p.z.toFixed(0)}) yaw=${(p.yaw * 180 / Math.PI).toFixed(0)} phase=${c.phase} act=${c.action} corners=${c.corners} dir=${c.direction} front=${(c.front || 0).toFixed(0)} L=${(c.left || 0).toFixed(0)} R=${(c.right || 0).toFixed(0)} course=${(c.course || 0).toFixed(0)}`);
    }
  }
  console.log(JSON.stringify(sim.result(), (k, v) => (k === 'command' || k === 'stats') ? undefined : v));
  console.log('events:', sim.log.events.slice(0, 40));
  console.log('cpu ms', Date.now() - t0);
})();
