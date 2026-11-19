/* Trabajador de depuracion del simulador: ejecuta EXACTAMENTE el mismo simulador (firmware C en WebAssembly,
 * sensores, robot y juez) y escribe lo que pasa en JSON. Lo usa tools/depurar.py; tambien sirve a mano.
 *
 *   node simulator/tools/debug_trace.js trace plano.json [--seconds 190] [--every 0.1] [--ctrl "clave valor;clave valor"] [--lenient]
 *   node simulator/tools/debug_trace.js snap  plano.json T [--reach 45]          mapa local y planificador en el instante T
 *
 * trace: una linea JSON por muestra ("kind":"sample"), una cabecera ("header") y un final ("end").
 * snap : un unico JSON con la pose, la telemetria, el mapa que ve el planificador (ASCII), la ruta, el
 *        corredor y lo que la camara y el LiDAR devuelven en ese instante.
 * Todo lo "verdad" (posicion real, letreros) sale marcado como `truth`: el robot no lo ve; es para depurar. */
const fs = require('node:fs'), path = require('node:path');
const web = path.join(__dirname, '../web');
const WRO = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));
const F = WRO.field, G = WRO.geometry;

function args() {
  const a = process.argv.slice(2), mode = a[0], file = a[1], opt = { seconds: 190, every: 0.1, reach: 45, ctrl: '' }, pos = [];
  for (let i = 2; i < a.length; i++) {
    if (a[i] === '--seconds') opt.seconds = Number(a[++i]);
    else if (a[i] === '--every') opt.every = Number(a[++i]);
    else if (a[i] === '--reach') opt.reach = Number(a[++i]);
    else if (a[i] === '--ctrl') opt.ctrl = a[++i];
    else if (a[i] === '--lenient') opt.lenient = true;
    else pos.push(a[i]);
  }
  return { mode, file, opt, pos };
}
const r1 = v => (Number.isFinite(v) ? Math.round(v * 10) / 10 : null);
const deg = v => ((v * 180 / Math.PI + 540) % 360) - 180;

/* Posicion de un letrero respecto al robot (verdad): adelante y derecha del eje motriz. */
function relative(pose, x, z) {
  const dx = x - pose.x, dz = z - pose.z, s = Math.sin(pose.yaw), c = Math.cos(pose.yaw);
  return { forward: dx * s + dz * c, right: dx * c - dz * s };
}

function laneFrame(field, pose) {
  /* Posicion en el marco del cajon (u a lo largo de la pared, w desde la pared) si hay cajon. */
  const lot = field.lot;
  if (!lot) return null;
  const ccw = field.direction !== 'cw', centre = (lot.a0 + lot.a1) / 2;
  const u = (ccw ? 1 : -1) * ((lot.axis === 'x' ? pose.x : pose.z) - centre);
  const w = Math.abs((lot.axis === 'x' ? pose.z : pose.x) - lot.wallCoord);
  return { u: r1(u), w: r1(w) };
}

async function build(file, opt) {
  const core = await WRO.core.load(fs.readFileSync(path.join(web, 'wro_core.wasm')));
  const field = F.parse(fs.readFileSync(file, 'utf8'));
  core.ctrlDefaults();
  /* Igual que la interfaz: el cuadro de texto lleva el volcado de la configuracion por defecto. */
  const ctrl = core.ctrlConfigDump() + '\n' + (opt.ctrl || '').split(';').join('\n');
  const sim = new WRO.sim.Simulation(core, { field, seed: field.seed || 1, ctrlConfig: ctrl });
  if (opt.lenient) {
    /* Modo indulgente: las infracciones se anotan pero la ronda sigue, para ver TODOS los problemas del plano
     * y no solo el primero. Solo el tiempo (180 s) termina la ronda. */
    const end = sim.judge.end.bind(sim.judge);
    sim.judge.end = (t, reason) => { if (reason === 'time' || reason === 'finished') end(t, reason); };
  }
  return { core, field, sim };
}

function sample(sim, field, prevContacts) {
  const c = sim.cmd || {}, p = sim.robot.pose(), j = sim.judge;
  const signs = field.signs.map((s, i) => {
    const rel = relative(p, s.x, s.z);
    return { i, color: s.color, dist: r1(Math.hypot(s.x - p.x, s.z - p.z)), forward: r1(rel.forward), right: r1(rel.right),
             moved: Math.hypot(s.x - s.seat.x, s.z - s.seat.z) > 0.5 };
  }).sort((a, b) => a.dist - b.dist);
  const cont = j.contacts, delta = { wall: cont.wall - prevContacts.wall, sign: cont.sign - prevContacts.sign, limiter: cont.limiter - prevContacts.limiter };
  return {
    kind: 'sample', t: r1(sim.t), x: r1(p.x), z: r1(p.z), yaw: r1(deg(p.yaw)), v: r1(sim.robot.v),
    section: F.sectionAt(field, p.x, p.z), lot: laneFrame(field, p),
    phase: c.phase || 'IDLE', action: c.action || '', steer: r1(c.steer), speed: r1(c.speed), yawRate: r1(c.yawRate),
    corners: c.corners, laps: c.laps, direction: c.direction, course: r1(c.course), gyro: r1(c.yaw),
    front: c.front > 1e6 ? null : r1(c.front), left: c.left > 1e6 ? null : r1(c.left), right: c.right > 1e6 ? null : r1(c.right),
    lane: r1(c.lane), pass: r1(c.pass), freeCm: r1(c.freeCm), escaping: !!c.escaping, escPhase: c.escapePhase, escapes: c.escapes,
    park: c.park ? { u: r1(c.park.u), w: r1(c.park.w), theta: r1(c.park.theta), lost: c.park.lost, move: c.park.move } : null,
    sidestep: c.sidestep || 0, pivoting: !!c.pivoting, lapMem: c.lapMem, odom: c.odom, arena: c.arena, net: j.net, maxNet: j.maxNet, signs: signs.slice(0, 3), contactsDelta: delta,
    contacts: { wall: cont.wall, sign: cont.sign, limiter: cont.limiter },
    movedSigns: field.signs.filter(s => Math.hypot(s.x - s.seat.x, s.z - s.seat.z) > 0.5).length,
    ended: j.ended ? j.ended.reason : null
  };
}

async function trace(file, opt) {
  const { field, sim } = await build(file, opt);
  const out = line => process.stdout.write(JSON.stringify(line) + '\n');
  out({ kind: 'header', direction: field.direction, startInLot: !!field.startInLot, startOnTrack: !!field.startOnTrack,
        lot: field.lot ? { a0: r1(field.lot.a0), a1: r1(field.lot.a1), wall: field.lot.wallCoord, axis: field.lot.axis } : null,
        signs: field.signs.map((s, i) => ({ i, color: s.color, x: r1(s.seat.x), z: r1(s.seat.z), section: F.sectionAt(field, s.seat.x, s.seat.z) })),
        robot: sim.robot.cfg, start: F.startPose(field), layoutWarnings: F.checkLayout(field, { bodyWidth: sim.robot.cfg.bodyWidth }) });
  let next = 0, prev = { wall: 0, sign: 0, limiter: 0 };
  while (!sim.finished && sim.t < opt.seconds) {
    sim.tick();
    const cont = sim.judge.contacts;
    const touched = cont.wall !== prev.wall || cont.sign !== prev.sign || cont.limiter !== prev.limiter;
    if (sim.t >= next || touched) {
      out(sample(sim, field, prev));
      prev = { wall: cont.wall, sign: cont.sign, limiter: cont.limiter };
      if (sim.t >= next) next += opt.every;
    }
  }
  const r = sim.result();
  out({ kind: 'end', t: r1(sim.t), endReason: r.endReason, points: r.points, sections: r.sections, laps: r.laps, parking: r.parking,
        violations: r.violations, contacts: r.contacts, signMoved: r.signMoved, elapsed: r1(r.elapsed) });
}

async function snap(file, T, opt) {
  const { core, field, sim } = await build(file, opt);
  while (!sim.finished && sim.t < T) sim.tick();
  const c = sim.cmd || {}, p = sim.robot.pose(), d = core.debug();
  const map = core.map(), N = Math.round(Math.sqrt(map.length)), cell = core.mapCellCm(), C = N / 2;
  const rows = [], cols = Math.round(opt.reach / cell), up = Math.round(opt.reach / cell), down = Math.round(15 / cell);
  for (let fr = up; fr >= -down; fr--) {
    let row = (Math.round(fr * cell) + '').padStart(4) + ' ';
    for (let xr = -cols; xr <= cols; xr++) {
      const v = map[(C - fr) * N + C + xr];
      row += (xr === 0 && fr === 0) ? 'R' : (v & 2) ? ((v & 4) ? 'V' : '#') : (v & 1) ? '.' : ' ';
    }
    rows.push(row);
  }
  const pts = (arr, n) => arr.filter((_, i) => i % Math.max(1, Math.floor(arr.length / n)) === 0).map(q => [r1(q[0]), r1(q[1])]);
  const scan = sim.snapshot.scan || [];
  console.log(JSON.stringify({
    kind: 'snap', t: r1(sim.t), truth: { x: r1(p.x), z: r1(p.z), yaw: r1(deg(p.yaw)), v: r1(sim.robot.v) },
    command: { phase: c.phase, action: c.action, steer: r1(c.steer), speed: r1(c.speed), yawRate: r1(c.yawRate), pass: r1(c.pass), freeCm: r1(c.freeCm), lane: r1(c.lane), escaping: !!c.escaping, escPhase: c.escapePhase },
    reference: r1(d.reference), curtains: d.curtains,
    colored: d.colored.map(o => ({ color: o.color === 1 ? 'red' : 'green', x: r1(o.x), forward: r1(o.forward), confirmed: !!o.confirmed })),
    plan: pts(d.plan, 12), route: pts(d.route, 8), map: rows,
    legend: '# ocupado, V ocupado por vision, . libre, espacio = desconocido, R = eje motriz del robot; arriba = delante, derecha = derecha del robot; una columna = 2,5 cm',
    camera: sim.snapshot.vision ? { action: sim.snapshot.vision.action, objects: sim.snapshot.vision.objects.map(o => ({ color: o.color, distance: r1(o.distance), flags: o.flags })) } : null,
    lidarPoints: scan.filter(q => q.valid).length, lidarBlind: scan.filter(q => q.blind).length,
    truthSigns: field.signs.map((s, i) => { const rel = relative(p, s.x, s.z); return { i, color: s.color, forward: r1(rel.forward), right: r1(rel.right) }; })
      .filter(s => Math.abs(s.forward) < 80 && Math.abs(s.right) < 80)
  }));
}

(async () => {
  const { mode, file, opt, pos } = args();
  if (!file || !fs.existsSync(file)) { console.error('uso: debug_trace.js trace|snap plano.json [T] [opciones]'); process.exit(2); }
  if (mode === 'trace') await trace(file, opt);
  else if (mode === 'snap') await snap(file, Number(pos[0] || 10), opt);
  else { console.error('modo desconocido: ' + mode); process.exit(2); }
})().catch(e => { console.error(e); process.exit(1); });
