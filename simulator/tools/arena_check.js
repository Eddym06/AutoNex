/* Error de la pose en el campo (odometria corregida con el LiDAR) frente a la verdad del simulador.
 *   node simulator/tools/arena_check.js [segundos] [semilla] [--cfg "clave valor;clave valor"] [--signs N] [--plan archivo.json] [--profile nombre]
 * El marco del campo es el del simulador (origen en el centro, x este, z norte, rumbo horario desde el norte). */
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
const plan = flag('--plan', null);
const profile = flag('--profile', null);
if (profile && !WRO.robot.PROFILES[profile]) throw new Error(`Perfil desconocido: ${profile}`);
const wrap = a => { while (a > 180) a -= 360; while (a <= -180) a += 360; return a; };

(async () => {
  const core = await WRO.core.load(fs.readFileSync(path.join(web, 'wro_core.wasm')));
  const field = plan ? F.parse(fs.readFileSync(plan, 'utf8')) : F.randomLayout(seed, { direction: seed % 2 ? 'ccw' : 'cw', signs, parking: false });
  const sim = new WRO.sim.Simulation(core, { field, seed, ctrlConfig: cfg, ...(profile ? { robot: WRO.robot.PROFILES[profile] } : {}) });
  let worst = 0, worstT = 0, worstTt = 0, n = 0, sum = 0, first = null, last = null; const ets = [], errors = []; let samples = 0;
  while (sim.t < secs && !sim.finished) {
    sim.run(0.5);
    const c = sim.cmd;
    ++samples;
    if (!c || !c.arena || !c.arena.valid) continue;
    const p = sim.robot.pose();
    const e = Math.hypot(c.arena.x - p.x, c.arena.z - p.z);
    const et = Math.abs(wrap(c.arena.theta - p.yaw * 180 / Math.PI));
    if (!first) first = { t: sim.t, e, et };
    worst = Math.max(worst, e); if (et > worstT) { worstT = et; worstTt = sim.t; } ets.push(et); errors.push(e); sum += e; ++n;
    last = { t: sim.t, e, et, inl: c.arena.inliers, used: c.arena.used, upd: c.arena.updates };
    if (Math.round(sim.t * 2) % 40 === 0) console.log(`t=${sim.t.toFixed(0)} s  error ${e.toFixed(2)} cm / ${et.toFixed(2)} grados  respaldo ${c.arena.inliers}/${c.arena.used}  correcciones ${c.arena.updates}`);
  }
  if (!n) { console.log('el localizador nunca adquirio pose'); process.exit(1); }
  errors.sort((a, b) => a - b);
  const p = (xs, q) => xs[Math.ceil(xs.length * q) - 1];
  console.log(`perfil ${profile || 'por defecto'}; pose disponible ${n}/${samples} muestras; posicion p50 ${p(errors, 0.5).toFixed(2)} cm, p95 ${p(errors, 0.95).toFixed(2)} cm`);
  console.log(`primera pose en t=${first.t.toFixed(1)} s (error ${first.e.toFixed(2)} cm). Error medio ${(sum / n).toFixed(2)} cm, maximo ${worst.toFixed(2)} cm; rumbo: mediana ${ets.sort((a, b) => a - b)[Math.floor(n / 2)].toFixed(2)} grados, p95 ${ets[Math.floor(n * 0.95)].toFixed(2)}, maximo ${worstT.toFixed(2)} (t=${worstTt.toFixed(1)} s) sobre ${n} medidas`);
})();
