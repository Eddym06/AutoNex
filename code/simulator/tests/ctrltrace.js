/* Estado del controlador de un plano: node simulator/tests/ctrltrace.js plano.json tFrom tTo step */
const fs = require('node:fs'), path = require('node:path');
const web = path.join(__dirname, '../web');
const WRO = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));
(async () => {
  const core = await WRO.core.load(fs.readFileSync(path.join(web, 'wro_core.wasm')));
  const field = WRO.field.parse(fs.readFileSync(process.argv[2], 'utf8'));
  const a = Number(process.argv[3] || 0), b = Number(process.argv[4] || 30), step = Number(process.argv[5] || 0.5);
  const sim = new WRO.sim.Simulation(core, { field, seed: field.seed || 1, ctrlConfig: (process.env.CTRL || '').split(';').join('\n') });
  let last = -1;
  while (sim.t < b && !sim.finished) {
    sim.tick();
    if (sim.t >= a && sim.t - last >= step) {
      last = sim.t; const c = sim.cmd || {}, p = sim.robot.pose();
      const f = x => (x == null ? '?' : Number(x).toFixed(0));
      console.log(`t=${sim.t.toFixed(1)} (${f(p.x)},${f(p.z)}) ${c.action} st=${f(c.steer)} gyro=${f(c.yaw)} course=${f(c.course)} front=${f(c.front)} L=${f(c.left)} R=${f(c.right)} cor=${c.corners} dir=${c.direction} free=${f(c.freeCm)} pass=${f(c.pass)}`);
    }
  }
})();
