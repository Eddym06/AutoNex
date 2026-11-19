/* node simulator/tests/planat.js '<signs>' T [ccw|cw] : corredor y accion del planificador en el instante T */
const fs = require('node:fs'), path = require('node:path');
const web = path.join(__dirname, '../web');
const WRO = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));
(async () => {
  const core = await WRO.core.load(fs.readFileSync(path.join(web, 'wro_core.wasm')));
  const isFile = (process.argv[2] || '').endsWith('.json'); const signs = isFile ? [] : JSON.parse(process.argv[2] || '[]'), T = Number(process.argv[3] || 20);
  const field = isFile ? WRO.field.parse(fs.readFileSync(process.argv[2], 'utf8')) : WRO.field.create({ signs, direction: process.argv[4] === 'cw' ? 'cw' : 'ccw', start: { along: 20 } });
  const sim = new WRO.sim.Simulation(core, { field, seed: isFile ? (field.seed || 1) : 7, ctrlConfig: (process.env.CTRL || '').split(';').join('\n') });
  while (sim.t < T && !sim.finished) sim.tick();
  const c = sim.cmd, p = sim.robot.pose(), d = core.debug();
  console.log(`t=${sim.t.toFixed(2)} pos=(${p.x.toFixed(1)},${p.z.toFixed(1)}) yaw=${(p.yaw * 180 / Math.PI).toFixed(1)} action=${c.action} steer=${c.steer} pass=${c.pass} ref=${d.reference.toFixed(1)} lane=${d.lane}`);
  console.log('colored', JSON.stringify(d.colored.map(o => [o.color, +o.x.toFixed(1), +o.forward.toFixed(1), o.confirmed])));
  console.log('plan (x,fwd):', d.plan.filter((_, i) => i % 3 === 0).map(q => `(${q[0].toFixed(0)},${q[1].toFixed(0)})`).join(' '));
  console.log('route (x,fwd):', d.route.filter((_, i) => i % 3 === 0).map(q => `(${q[0].toFixed(0)},${q[1].toFixed(0)})`).join(' '));
})();
