/* Compara dos fuentes de la pose del robot en el cajon con la verdad del simulador, en las mismas rondas:
 *   - LiDAR local (park_localize, lo que usa el parqueo): cmd.park {u, w}
 *   - plaza de trace_map + pose del campo (arena_localizer): u = signo*(x_robot - x_plaza), w = z_robot - (z_plaza - 10)
 *   node simulator/tools/measure_parking_sources.js [semilla_ini=21] [semilla_fin=28] [salida.json]
 * Modo ronda (sin letreros) para que todas las rondas lleguen a aparcar. La verdad no entra en el controlador. */
'use strict';
const fs = require('node:fs'), path = require('node:path');
const web = path.join(__dirname, '../web'), W = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));
function stats(v) {
  if (!v.length) return null;
  const s = [...v].sort((a, b) => a - b), q = f => s[Math.min(s.length - 1, Math.ceil(s.length * f) - 1)];
  const m = v.reduce((a, b) => a + b, 0) / v.length;
  return { n: v.length, mean: +m.toFixed(2), rms: +Math.sqrt(v.reduce((a, b) => a + b * b, 0) / v.length).toFixed(2), p50: +q(.5).toFixed(2), p95: +q(.95).toFixed(2), max: +s[s.length - 1].toFixed(2) };
}
(async () => {
  const core = await W.core.load(fs.readFileSync(path.join(web, 'wro_core.wasm')));
  const a = Number(process.argv[2] || 21), b = Number(process.argv[3] || 28);
  const rows = [], allLidar = [], allMap = [], allSearch = [], biasL = [], biasM = [];
  const meanSigned = v => v.length ? +(v.reduce((x, y) => x + y, 0) / v.length).toFixed(2) : null;
  let skipped = 0;
  for (const direction of ['ccw', 'cw']) for (let seed = a; seed <= b; seed++) {
    const field = W.field.randomLayout(seed, { direction, signs: 0, parking: true, startInLot: true });
    const lot = field.lot;
    if (lot.axis !== 'x' || lot.wallCoord > 0) { skipped++; console.error(`${direction} ${seed}: cajon en eje ${lot.axis}, pared ${lot.wallCoord}: no comparado`); continue; }
    const ccw = field.direction !== 'cw', centre = (lot.a0 + lot.a1) / 2, sgn = ccw ? 1 : -1;
    const sim = new W.sim.Simulation(core, { field, seed, robot: W.robot.PROFILES.largo26 });
    const eL = [], eM = [], dL = [], dM = [], eS = [];
    let spotFirst = null, searchStart = null, acq = null, lastSample = -1;
    while (sim.t < 180 && !sim.finished) {
      sim.tick();
      const ph = sim.cmd ? sim.cmd.phase : '';
      if (ph === 'PARK_SEARCH' && searchStart === null) { searchStart = sim.t; const sp = core.traceSpot(); spotFirst = sp.status === 2 ? 'ya confirmada al empezar la busqueda' : 'no confirmada al empezar la busqueda'; }
      if (ph !== 'PARKING' && ph !== 'PARK_SEARCH') continue;
      if (sim.t - lastSample < 0.25) continue;
      lastSample = sim.t;
      const p = sim.robot.pose(), c = sim.cmd, sp = core.traceSpot(), ar = c.arena;
      const uT = sgn * (p.x - centre), wT = p.z - lot.wallCoord;
      if (ph === 'PARKING' && c.park) {
        const du = c.park.u - uT, dw = c.park.w - wT;
        eL.push(Math.hypot(du, dw)); dL.push(du, dw);
        if (acq === null) acq = { lidar: +Math.hypot(du, dw).toFixed(2) };
      }
      if (ar && ar.valid && sp.status === 2) {
        const uM = sgn * (ar.x - sp.x), wM = ar.z - (sp.z - 10);
        const du = uM - uT, dw = wM - wT, e = Math.hypot(du, dw);
        /* Pareado: en PARKING se compara con las MISMAS muestras que el LiDAR local; la busqueda se informa aparte. */
        if (ph === 'PARKING' && c.park) { eM.push(e); dM.push(du, dw); if (acq && acq.map === undefined) acq.map = +e.toFixed(2); }
        else if (ph === 'PARK_SEARCH') eS.push(e);
      }
    }
    const r = sim.result();
    rows.push({ direction, seed, parking: r.parking, end: r.endReason, spotFirst, searchStart: searchStart && +searchStart.toFixed(1),
      lidar: stats(eL), map: stats(eM), acquisition: acq });
    allLidar.push(...eL); allMap.push(...eM); allSearch.push(...eS);
    /* sesgo con signo (u, w): la media de la diferencia, no del modulo */
    biasL.push(...dL); biasM.push(...dM);
    console.error(`${direction} ${seed}: parqueo=${r.parking} fin=${r.endReason} | LiDAR ${eL.length} muestras p95 ${stats(eL) ? stats(eL).p95 : '-'} | mapa ${eM.length} muestras p95 ${stats(eM) ? stats(eM).p95 : '-'} | ${spotFirst}`);
  }
  const out = { rondas: rows.length, no_comparadas: skipped,
    parqueo_completo: `${rows.filter(r => r.parking === 'full').length}/${rows.length}`,
    plaza_ya_confirmada_al_empezar_busqueda: `${rows.filter(r => r.spotFirst && r.spotFirst.startsWith('ya')).length}/${rows.length}`,
    error_pose_en_cajon_cm_pareado_en_PARKING: { lidar_local: stats(allLidar), plaza_del_mapa_mas_pose_del_campo: stats(allMap) },
    error_plaza_del_mapa_durante_PARK_SEARCH_cm: stats(allSearch),
    sesgo_con_signo_cm_media_de_u_y_w_mezclados: { lidar_local: meanSigned(biasL), mapa: meanSigned(biasM) } };
  console.log(JSON.stringify(out, null, 2));
  if (process.argv[4]) fs.writeFileSync(process.argv[4], JSON.stringify({ out, rows }, null, 2) + '\n');
})().catch(e => { console.error(e); process.exit(1); });
