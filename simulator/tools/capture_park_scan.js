/* Guarda un barrido real del simulador en la fase PARKING como cabecera C para medir park_localize_track en el P4 (perf_bench.c):
 *   node simulator/tools/capture_park_scan.js semilla direccion salida.h */
'use strict';
const fs = require('node:fs'), path = require('node:path');
const web = path.join(__dirname, '../web'), W = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));
(async () => {
  const [seed, direction, out] = [Number(process.argv[2] || 3), process.argv[3] || 'ccw', process.argv[4]];
  const core = await W.core.load(fs.readFileSync(path.join(web, 'wro_core.wasm')));
  const field = W.field.randomLayout(seed, { direction, signs: 5, parking: true, startInLot: true, feasible: true, allowGates: false });
  const robot = Object.assign({}, W.robot.PROFILES.largo26d, { speedTauS: 0.08, speedGain: 1 });
  const sim = new W.sim.Simulation(core, { field, seed, robot, ctrlConfig: 'it_enabled 1' });
  let t0 = null;
  while (sim.t < 200 && !sim.finished) {
    sim.run(0.1);
    const c = sim.cmd; if (!c) continue;
    if (c.phase === 'PARKING' && t0 === null) t0 = sim.t;
    if (t0 !== null && sim.t - t0 > 3.0) break;
  }
  const pts = sim.snapshot.scan, c = sim.cmd;
  const lines = pts.map(p => `{${p.angle.toFixed(3)}f, ${p.distance.toFixed(2)}f, ${p.valid ? 1 : 0}, ${p.blind ? 1 : 0}}`);
  const txt = `/* Barrido de ${direction} ${seed} en PARKING (t=${sim.t.toFixed(1)} s), generado por simulator/tools/capture_park_scan.js */\n` +
    `#define PERF_PARK_N ${pts.length}\n#define PERF_PARK_MIRROR ${direction === 'ccw' ? -1 : 1}\n#define PERF_PARK_LOT_LEN ${Number(field.lot.length).toFixed(1)}f\n` +
    `#define PERF_PARK_U ${c.park.u.toFixed(2)}f\n#define PERF_PARK_W ${c.park.w.toFixed(2)}f\n#define PERF_PARK_TH ${c.park.theta.toFixed(2)}f\n` +
    `static const struct { float a, d; unsigned char v, b; } perf_park_scan[PERF_PARK_N] = {\n${lines.join(',\n')}\n};\n`;
  fs.writeFileSync(out, txt);
  console.log('puntos', pts.length, 'pose', JSON.stringify(c.park), 'lote', field.lot.length, 'fase', c.phase);
})();
