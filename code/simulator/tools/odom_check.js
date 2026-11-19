/* Compara la odometria que calcula el firmware con la pose verdadera del simulador.
 *   node simulator/tools/odom_check.js [segundos] [semilla] [--cfg "clave valor;clave valor"] [--signs N]
 * El marco de la odometria es el del giroscopio: origen donde arranca el eje, y hacia el rumbo inicial. */
const fs = require('node:fs'), path = require('node:path');
const web = path.join(__dirname, '../web');
const WRO = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));
const F = WRO.field;
const args = process.argv.slice(2);
const flag = (name, def) => { const i = args.indexOf(name); return i >= 0 ? args[i + 1] : def; };
const secs = Number(args[0] && !args[0].startsWith('--') ? args[0] : 120);
const seed = Number(args[1] && !args[1].startsWith('--') ? args[1] : 1);
const cfg = (flag('--cfg', '') || '').split(';').join('\n');
const signs = Number(flag('--signs', 0));

(async () => {
  const core = await WRO.core.load(fs.readFileSync(path.join(web, 'wro_core.wasm')));
  const field = F.randomLayout(seed, { direction: seed % 2 ? 'ccw' : 'cw', signs, parking: false });
  const sim = new WRO.sim.Simulation(core, { field, seed, ctrlConfig: cfg });
  const start = sim.robot.pose();
  const toOdom = p => {
    const dx = p.x - start.x, dz = p.z - start.z;
    return { x: dx * Math.cos(start.yaw) - dz * Math.sin(start.yaw), y: dx * Math.sin(start.yaw) + dz * Math.cos(start.yaw) };
  };
  let worst = 0, last = null, rows = [];
  while (sim.t < secs && !sim.finished) {
    sim.run(0.5);
    const c = sim.cmd;
    if (!c || !c.odom || !c.odom.inUse) continue;
    const t = toOdom(sim.robot.pose());
    const err = Math.hypot(c.odom.x - t.x, c.odom.y - t.y);
    worst = Math.max(worst, err);
    last = { t: sim.t, err, path: c.odom.path, truePath: sim.robot.travelled, slips: c.odom.slips, slipCm: c.odom.slipCm };
    if (Math.round(sim.t * 2) % 20 === 0) rows.push(last);
  }
  for (const r of rows) console.log(`t=${r.t.toFixed(0)} s  error ${r.err.toFixed(1)} cm  camino odom ${r.path.toFixed(0)} cm (real ${r.truePath.toFixed(0)})  patinajes ${r.slips}`);
  if (!last) { console.log('la odometria no se uso (sin muestras de encoders)'); process.exit(1); }
  const scale = last.path / Math.max(1, last.truePath);
  console.log(`error maximo ${worst.toFixed(1)} cm, error final ${last.err.toFixed(1)} cm sobre ${last.truePath.toFixed(0)} cm recorridos (${(100 * last.err / last.truePath).toFixed(2)} %), escala de camino ${scale.toFixed(3)}`);
})();
