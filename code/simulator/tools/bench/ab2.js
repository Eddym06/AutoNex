const fs = require('fs');
const R = (process.env.WEB || require('path').join(__dirname, '..', '..', 'web')) + '/';   // WEB=carpeta con una copia congelada de simulator/web
const WRO = require(R + 'sim.js'); require(R + 'core.js');
const F = WRO.field;
const bytes = fs.readFileSync(R + 'wro_core.wasm');
const tag = process.argv[2], jobs = process.argv.slice(3);
const cfg = (process.env.CFG || '').split(';').join('\n');
const signs = Number(process.env.SIGNS || 7);
(async () => {
  for (const j of jobs) {
    const [sd, dir] = j.split(':');
    const core = await WRO.core.load(bytes);
    const field = F.randomLayout(+sd, { direction: dir, signs });
    const sim = new WRO.sim.Simulation(core, { field, seed: +sd, ctrlConfig: cfg });
    const r = sim.run(200);
    console.log(JSON.stringify({ mem: tag, seed: +sd, dir, end: r.endReason, pts: r.points, sec: r.sections,
      viol: r.violations.length, wrong: r.violations.filter(v => v.kind === 'wrong-side').length,
      contact: r.contacts.sign + r.contacts.wall, el: r.elapsed, park: r.parking }));
  }
})();
