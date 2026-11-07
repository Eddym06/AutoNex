/* node simulator/tests/planat2.js <seed> <ccw|cw> <signs> <T> : corredor y ruta de una ronda aleatoria en el instante T */
const fs = require('node:fs'), path = require('node:path');
const web = path.join(__dirname, '../web');
const WRO = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));
(async () => {
  const core = await WRO.core.load(fs.readFileSync(path.join(web, 'wro_core.wasm')));
  const seed = Number(process.argv[2]), direction = process.argv[3], n = Number(process.argv[4]), T = Number(process.argv[5]);
  const field = WRO.field.randomLayout(seed, { direction, signs: n, parking: false });
  const sim = new WRO.sim.Simulation(core, { field, seed, ctrlConfig: (process.env.CTRL || '').split(';').join('\n') });
  while (sim.t < T && !sim.finished) sim.tick();
  const c = sim.cmd, p = sim.robot.pose(), d = core.debug();
  console.log(`t=${sim.t.toFixed(2)} pos=(${p.x.toFixed(1)},${p.z.toFixed(1)}) yaw=${(p.yaw * 180 / Math.PI).toFixed(1)} action=${c.action} steer=${c.steer.toFixed(1)} pass=${c.pass} ref=${d.reference.toFixed(1)} lane=${d.lane} planPts=${d.planPoints}`);
  console.log('colored', JSON.stringify(d.colored.map(o => [o.color, +o.x.toFixed(1), +o.forward.toFixed(1), o.confirmed])));
  console.log('plan (x,fwd):', d.plan.filter((_, i) => i % 3 === 0).map(q => `(${q[0].toFixed(0)},${q[1].toFixed(0)})`).join(' '));
  console.log('route (x,fwd):', d.route.filter((_, i) => i % 2 === 0).map(q => `(${q[0].toFixed(0)},${q[1].toFixed(0)})`).join(' '));
})();
