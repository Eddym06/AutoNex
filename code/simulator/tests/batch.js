/* Lote de rondas: node simulator/tests/batch.js [desde] [hasta] [ccw|cw|both] [signs] */
const fs = require('node:fs'), path = require('node:path');
const web = path.join(__dirname, '../web');
const WRO = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));

(async () => {
  const bytes = fs.readFileSync(path.join(web, 'wro_core.wasm'));
  const from = Number(process.argv[2] || 1), to = Number(process.argv[3] || 5);
  const dirs = process.argv[4] === 'cw' ? ['cw'] : process.argv[4] === 'ccw' ? ['ccw'] : ['ccw', 'cw'];
  const signs = Number(process.argv[5] || 8);
  const rows = [];
  for (let seed = from; seed <= to; seed++) for (const direction of dirs) {
    const core = await WRO.core.load(bytes);
    const seatModel = process.env.SEAT_ALONG ? { seatAlong: process.env.SEAT_ALONG.split(',').map(Number), seatLateral: (process.env.SEAT_LAT || '-0.2,0,0.2').split(',').map(Number) } : null;
    const park = process.env.PARK === '1';     // con cajon: arranca dentro, sale, hace tres vueltas y aparca
    const field = WRO.field.randomLayout(seed, { direction, signs, parking: park, startInLot: park, seatModel });
    const sim = new WRO.sim.Simulation(core, { field, seed, ctrlConfig: (process.env.CTRL || '').replace(/;/g, '\n') });
    const t0 = Date.now();
    const r = sim.run(200);
    rows.push({ seed, direction, pts: r.points, sec: r.sections, end: r.endReason, t: +(r.elapsed || r.t).toFixed(0), parking: r.parking, walls: r.contacts.wall, signHits: r.contacts.sign, viol: r.violations.map(v => v.kind + (v.sign ? '@' + v.sign + ' robot@' + v.robot + ' lat ' + v.lateral : '')).join(','), flipped: r.flipped, last: r.lastSignLap2, cpu: Date.now() - t0 });
    console.log(JSON.stringify(rows[rows.length - 1]));
  }
  const ok = rows.filter(r => r.end === 'finished' && !r.viol).length;
  console.log(`${ok}/${rows.length} rondas completas sin infraccion; media ${(rows.reduce((a, r) => a + r.pts, 0) / rows.length).toFixed(1)} puntos`);
})();
