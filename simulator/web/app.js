/* Interfaz del simulador. Solo dibuja y recoge ordenes: la fisica, los sensores y
 * el juez viven en los otros modulos, y todo lo que decide el robot es el firmware
 * C compilado a WebAssembly. */
(function () {
  'use strict';
  const WRO = window.WRO, $ = id => document.getElementById(id);
  const F = WRO.field, G = WRO.geometry, DEG = Math.PI / 180;
  const PRESETS = ['empty', 'slalom5', 'gate30', 'gate20', 'pocket', 'reported', 'colors', 'swapped'];
  const PRESET_NAMES = { empty: 'Pista vacía', slalom5: 'Eslalon de 5', gate30: 'Puerta de 30 cm', gate20: 'Puerta de 20 cm',
    pocket: 'Bolsillo sin salida', reported: 'Mapa reportado (7)', colors: 'Rojo y verde', swapped: 'Colores cruzados' };
  const COLORS = { red: '#ee2737', green: '#44d62c', magenta: '#ff00ff', orange: '#ff6600', blue: '#0033ff' };

  let core = null, sim = null, field = null, playing = false, last = 0, hoverSeat = null, overlays = true;
  let simBudget = 0, speedWindowStart = 0, speedWindowSim = 0, actualSpeed = 0;
  let generatedNotice = '';

  /* ---------------------------------------------------------------- arranque */
  async function boot() {
    const bytes = Uint8Array.from(atob(window.WASM_B64), c => c.charCodeAt(0));
    core = await WRO.core.load(bytes);
    for (const p of PRESETS) $('preset').add(new Option(PRESET_NAMES[p], p));
    $('preset').add(new Option('Aleatoria (semilla)', 'random'));
    $('preset').value = 'random';
    core.visionDefaults(); core.visionConfig(WRO.sim.DEFAULT_VISION_TEXT);
    $('visionText').value = core.visionConfigDump();
    core.ctrlDefaults();
    $('ctrlText').value = core.ctrlConfigDump();
    bind();
    newField();
    requestAnimationFrame(loop);
  }

  /* ------------------------------------------------------------------ pista */
  function newField() {
    const dir = $('direction').value, preset = $('preset').value;
    const lot = $('withLot').checked, inLot = lot && $('startInLot').checked, onTrack = lot && !inLot && $('startOnTrack').checked;
    /* Sin `robotLength`: el cajon y la pose de arranque los ajusta la simulacion al
     * chasis que corre (field.fitToRobot), como manda el reglamento (1,5 x el robot). */
    if (preset === 'random') field = F.randomLayout(Number($('seed').value) || 1, { direction: dir, signs: Number($('signCount').value), parking: lot, startInLot: inLot, startOnTrack: onTrack, feasible: true, allowGates: false });
    else {
      const base = F.preset(preset);
      field = F.create({ signs: base.signs.map(s => ({ color: s.color, x: s.seat.x, z: s.seat.z })), direction: dir,
                        parking: lot ? { section: 0, along: 0 } : null, startInLot: inLot, startOnTrack: onTrack });
    }
    const requested = Number($('signCount').value);
    generatedNotice = preset === 'random' && field.signs.length < requested
      ? `El filtro generó ${field.signs.length} de ${requested} letreros solicitados; no garantiza una trayectoria.` : '';
    startSim();
  }
  function startSim() {
    const num = id => Number($(id).value);
    const noise = num('rLidarNoise'), camNoise = num('rCamNoise');
    const lines = [$('ctrlText').value, 'planner.speed_cm_s ' + num('rSpeed'),
      'planner.pass_side_red ' + $('redSide').value, 'planner.pass_side_green ' + $('greenSide').value].join('\n');
    const vision = [$('visionText').value, 'pass_side_red ' + $('redSide').value, 'pass_side_green ' + $('greenSide').value].join('\n');
    const gyro = $('rGyroBias').value === '' ? null : num('rGyroBias');
    const camFps = num('rCamFps') || WRO.sensors.DEFAULT_CAMERA.fps;
    const passSide = $('judgeFollows').checked ? { red: Number($('redSide').value), green: Number($('greenSide').value) } : { red: 1, green: -1 };
    try {
      sim = new WRO.sim.Simulation(core, {
        field, seed: Number($('seed').value) || 1, ctrlConfig: lines, visionConfig: vision,
        lidar: { sigmaNearCm: 0.5 * noise, sigmaFarCm: 2 * noise, angleJitterDeg: 0.15 * noise, dropout: noise > 0 ? 0.015 : 0 },
        camera: { noiseSigma: 3.5 * camNoise, gain: num('rLight'), latencyMs: num('rLatency'), fps: camFps, bitErrorRate: num('rBitErr') / 100 },
        imu: { biasDps: gyro }, robot: { speedGain: num('rSpeedGain'), steerTrimDeg: num('rTrim') }, judge: { passSide }
      });
      setStatus('listo', '');
    } catch (e) { setStatus('error: ' + e.message, 'bad'); sim = null; }
    playing = false; simBudget = 0; speedWindowStart = 0; actualSpeed = 0; $('play').textContent = '▶ Simular';
    draw();
  }

  /* ------------------------------------------------------------------ bucle */
  function loop(now) {
    const dt = Math.min(0.1, (now - last) / 1000 || 0);
    last = now;
    if (playing && sim && !sim.finished) {
      const speed = Number($('speed').value);
      simBudget = Math.min(simBudget + dt * speed, Math.max(0.5, speed * 0.25));
      const t0 = performance.now();
      while (simBudget >= sim.dt && !sim.finished && performance.now() - t0 < 26) { sim.tick(); simBudget -= sim.dt; }
      if (!speedWindowStart) { speedWindowStart = now; speedWindowSim = sim.t; }
      if (now - speedWindowStart >= 1000) {
        actualSpeed = (sim.t - speedWindowSim) / ((now - speedWindowStart) / 1000);
        speedWindowStart = now; speedWindowSim = sim.t;
      }
    }
    if (sim && sim.finished && playing) {
      playing = false; $('play').textContent = '▶ Simular';
    }
    draw();
    requestAnimationFrame(loop);
  }

  function setStatus(text, cls) { const s = $('status'); s.textContent = text; s.className = 'tag ' + (cls || ''); }

  /* ----------------------------------------------------------------- dibujo */
  function worldTransform(c) { const S = c.width / 340; return { S, px: x => c.width / 2 + x * S, py: z => c.height / 2 - z * S }; }
  function robotToWorld(p, x, f) { const s = Math.sin(p.yaw), c = Math.cos(p.yaw); return [p.x + x * c + f * s, p.z - x * s + f * c]; }

  function drawWorld() {
    const c = $('world'), g = c.getContext('2d'), T = worldTransform(c), S = T.S;
    g.clearRect(0, 0, c.width, c.height);
    g.fillStyle = '#d9d7cf'; g.fillRect(T.px(-160), T.py(160), 320 * S, 320 * S);
    const [ihx, ihz] = field.inner;
    g.fillStyle = '#ececE6'; g.fillRect(T.px(-150), T.py(150), 300 * S, 300 * S);
    g.fillStyle = '#6a7380'; g.fillRect(T.px(-ihx), T.py(ihz), 2 * ihx * S, 2 * ihz * S);
    for (const ln of field.lines) { g.save(); g.translate(T.px(ln.x), T.py(ln.z)); g.rotate(ln.rot || 0); g.fillStyle = COLORS[ln.kind]; g.fillRect(-ln.w / 2 * S, -ln.d / 2 * S, ln.w * S, ln.d * S); g.restore(); }
    g.fillStyle = '#10141b';
    for (const w of field.walls) g.fillRect(T.px(w.x - w.w / 2), T.py(w.z + w.d / 2), w.w * S, w.d * S);
    if (field.lot) {
      g.fillStyle = 'rgba(255,0,255,.12)';
      const l = field.lot, lo = Math.min(l.wallCoord, l.innerCoord), hi = Math.max(l.wallCoord, l.innerCoord);
      if (l.axis === 'x') g.fillRect(T.px(l.a0), T.py(hi), (l.a1 - l.a0) * S, (hi - lo) * S); else g.fillRect(T.px(lo), T.py(l.a1), (hi - lo) * S, (l.a1 - l.a0) * S);
      g.fillStyle = COLORS.magenta;
      for (const b of l.limiters) g.fillRect(T.px(b.x - b.w / 2), T.py(b.z + b.d / 2), b.w * S, b.d * S);
    }
    /* asientos y letreros */
    g.lineWidth = 1;
    for (const s of F.seats(field)) {
      g.strokeStyle = hoverSeat === s ? '#1f5fae' : 'rgba(90,100,120,.35)';
      g.beginPath(); g.arc(T.px(s.x), T.py(s.z), 4.25 * S, 0, 7); g.stroke();
    }
    for (const s of field.signs) {
      g.save(); g.translate(T.px(s.x), T.py(s.z)); g.rotate(s.rot || 0);
      g.fillStyle = COLORS[s.color]; g.fillRect(-2.5 * S, -2.5 * S, 5 * S, 5 * S);
      g.strokeStyle = '#111'; g.strokeRect(-2.5 * S, -2.5 * S, 5 * S, 5 * S); g.restore();
    }
    if (!sim) return;
    const p = sim.robot.pose();
    if ($('showTrace').checked && sim.log.poses.length > 1) {
      g.strokeStyle = 'rgba(30,110,200,.8)'; g.lineWidth = 1.5; g.beginPath();
      sim.log.poses.forEach((q, i) => i ? g.lineTo(T.px(q[1]), T.py(q[2])) : g.moveTo(T.px(q[1]), T.py(q[2]))); g.stroke();
    }
    const dbg = sim.snapshot.debug;
    if ($('showRace') && $('showRace').checked && sim.cmd && sim.cmd.race && sim.cmd.race.valid) {
      /* La linea de carrera del robot (tras la primera vuelta), coloreada por la velocidad que pide. */
      if (sim.raceKey !== sim.cmd.race.n + ':' + sim.cmd.race.length) { sim.raceKey = sim.cmd.race.n + ':' + sim.cmd.race.length; sim.racePts = core.raceLine(); }
      const rl = sim.racePts || [], vmin = 18, vmax = 36;
      g.lineWidth = 3; g.lineCap = 'round';
      for (let i = 0; i < rl.length; i++) {
        const a = rl[i], b = rl[(i + 1) % rl.length], t = Math.max(0, Math.min(1, (a.v - vmin) / (vmax - vmin)));
        g.strokeStyle = `hsl(${Math.round(20 + 100 * t)}, 85%, 45%)`;
        g.beginPath(); g.moveTo(T.px(a.x), T.py(a.z)); g.lineTo(T.px(b.x), T.py(b.z)); g.stroke();
      }
      const k = sim.cmd.race.idx;
      if (sim.cmd.race.active && k >= 0 && rl[k]) { g.fillStyle = '#ffd24d'; g.strokeStyle = '#222'; g.lineWidth = 1; g.beginPath(); g.arc(T.px(rl[k].x), T.py(rl[k].z), 4, 0, 7); g.fill(); g.stroke(); }
    }
    if ($('showLidar').checked && sim.snapshot.scan.length) {
      g.fillStyle = 'rgba(0,170,210,.9)';
      for (const q of sim.snapshot.scan) {
        if (!q.valid) continue;
        const a = q.angle * DEG, [wx, wz] = robotToWorld(p, sim.lidar.cfg.mountLateral + Math.sin(a) * q.distance, sim.lidar.cfg.mountForward + Math.cos(a) * q.distance);
        g.fillRect(T.px(wx) - 1, T.py(wz) - 1, 2, 2);
      }
    }
    if ($('showPlan').checked && dbg) {
      const poly = (pts, color, dash) => {
        if (pts.length < 2) return;
        g.strokeStyle = color; g.lineWidth = 2; g.setLineDash(dash || []); g.beginPath();
        pts.forEach(([x, f], i) => { const [wx, wz] = robotToWorld(p, x, f); i ? g.lineTo(T.px(wx), T.py(wz)) : g.moveTo(T.px(wx), T.py(wz)); }); g.stroke(); g.setLineDash([]);
      };
      poly([[0, 0]].concat(dbg.plan), '#3a8bff', [5, 4]);
      poly([[0, 0]].concat(dbg.route), '#ffb020');
    }
    if ($('showCurtain').checked && dbg) {
      for (const o of dbg.colored) {
        const side = o.color === 1 ? Number($('redSide').value) : Number($('greenSide').value);
        if (!side) continue;
        const ref = dbg.reference * DEG, dir = side > 0 ? -1 : 1;
        const [x0, z0] = robotToWorld(p, o.x, o.forward), [x1, z1] = robotToWorld(p, o.x + dir * Math.cos(ref) * 80, o.forward - dir * Math.sin(ref) * 80);
        g.strokeStyle = o.color === 1 ? 'rgba(238,39,55,.55)' : 'rgba(68,214,44,.55)'; g.lineWidth = 3;
        g.beginPath(); g.moveTo(T.px(x0), T.py(z0)); g.lineTo(T.px(x1), T.py(z1)); g.stroke();
      }
    }
    /* robot: el eje trasero es la pose */
    g.save(); g.translate(T.px(p.x), T.py(p.z)); g.rotate(p.yaw);
    const rb = sim.robot.cfg;
    g.fillStyle = sim.robot.v === 0 && sim.cmd && sim.cmd.done ? '#7aa7d9' : '#4da3ff'; g.fillRect(-rb.bodyWidth / 2 * S, -(rb.bodyLength - rb.rearOverhang) * S, rb.bodyWidth * S, rb.bodyLength * S);
    g.strokeStyle = '#fff'; g.lineWidth = 1; g.strokeRect(-rb.bodyWidth / 2 * S, -(rb.bodyLength - rb.rearOverhang) * S, rb.bodyWidth * S, rb.bodyLength * S);
    g.fillStyle = '#fff'; g.beginPath(); g.moveTo(0, -(rb.bodyLength - rb.rearOverhang) * S - 4); g.lineTo(-3, -(rb.bodyLength - rb.rearOverhang) * S + 2); g.lineTo(3, -(rb.bodyLength - rb.rearOverhang) * S + 2); g.fill();
    g.strokeStyle = '#102236'; g.lineWidth = 3;
    if (rb.drive === 'differential') {
      /* ruedas motrices atras (en el eje) y una rueda libre delante */
      for (const sx of [-1, 1]) { g.beginPath(); g.moveTo(sx * rb.trackWidth / 2 * S, -3.2 * S); g.lineTo(sx * rb.trackWidth / 2 * S, 3.2 * S); g.stroke(); }
      g.fillStyle = '#102236'; g.beginPath(); g.arc(0, -rb.wheelbase * S, 1.4 * S, 0, 7); g.fill();
    } else {
      for (const sx of [-1, 1]) { g.save(); g.translate(sx * rb.bodyWidth / 2 * S, -rb.wheelbase * S); g.rotate(sim.robot.steer * DEG); g.beginPath(); g.moveTo(0, -3); g.lineTo(0, 3); g.stroke(); g.restore(); }
    }
    g.restore();
    if ($('showPose') && $('showPose').checked && sim.cmd && sim.cmd.arena && sim.cmd.arena.valid) {
      /* Contorno amarillo: la pose que cree el robot (odometria corregida con el LiDAR). */
      const a = sim.cmd.arena;
      g.save(); g.translate(T.px(a.x), T.py(a.z)); g.rotate(a.theta * DEG);
      g.strokeStyle = '#ffd24d'; g.lineWidth = 1.5; g.setLineDash([4, 3]);
      g.strokeRect(-rb.bodyWidth / 2 * S, -(rb.bodyLength - rb.rearOverhang) * S, rb.bodyWidth * S, rb.bodyLength * S);
      g.setLineDash([]); g.restore();
    }
    if ($('showPose') && $('showPose').checked && sim.cmd && sim.cmd.odom && sim.cmd.odom.inUse && sim.start) {
      /* Cruz magenta: solo encoders y giroscopio (sin LiDAR), llevada del marco del giroscopio al del campo. */
      const o = sim.cmd.odom, y0 = sim.start.yaw;
      const wx = sim.start.x + o.x * Math.cos(y0) + o.y * Math.sin(y0), wz = sim.start.z - o.x * Math.sin(y0) + o.y * Math.cos(y0);
      g.strokeStyle = '#ff5cf0'; g.lineWidth = 1.5; g.beginPath();
      g.moveTo(T.px(wx) - 4, T.py(wz) - 4); g.lineTo(T.px(wx) + 4, T.py(wz) + 4); g.moveTo(T.px(wx) + 4, T.py(wz) - 4); g.lineTo(T.px(wx) - 4, T.py(wz) + 4); g.stroke();
    }
    g.fillStyle = '#56dcff'; const [lx, lz] = robotToWorld(p, 0, sim.lidar.cfg.mountForward); g.beginPath(); g.arc(T.px(lx), T.py(lz), 3, 0, 7); g.fill();
  }

  function drawLocal() {
    const c = $('local'), g = c.getContext('2d'), s = c.width / 300;
    g.clearRect(0, 0, c.width, c.height);
    if (!sim || !sim.cmd) return;
    const cell = core.mapCellCm(), n = core.mapSize(), map = core.map(), k = 1.1 * s;
    const cx = c.width / 2, cy = c.height * 0.72;
    const X = x => cx + x * k, Y = f => cy - f * k;
    g.strokeStyle = '#223048'; g.lineWidth = 1;
    for (let r = 50; r <= 150; r += 50) { g.beginPath(); g.arc(X(0), Y(0), r * k, 0, 7); g.stroke(); }
    for (let row = 0; row < n; row++) for (let col = 0; col < n; col++) {
      const v = map[row * n + col];
      if (!(v & 2)) continue;
      const x = (col - n / 2) * cell, f = (n / 2 - row) * cell;
      g.fillStyle = v & 4 ? '#ffd24a' : '#ff9a3c'; g.fillRect(X(x) - cell * k / 2, Y(f) - cell * k / 2, cell * k, cell * k);
    }
    const dbg = sim.snapshot.debug;
    if (dbg) {
      const line = (pts, color, w, dash) => { if (pts.length < 1) return; g.strokeStyle = color; g.lineWidth = w; g.setLineDash(dash || []); g.beginPath(); g.moveTo(X(0), Y(0)); for (const [x, f] of pts) g.lineTo(X(x), Y(f)); g.stroke(); g.setLineDash([]); };
      line(dbg.plan, '#3a8bff', 2, [4, 3]); line(dbg.route, '#ffd479', 3); line(dbg.probe, 'rgba(255,174,61,.6)', 1.5, [3, 3]);
      for (const o of dbg.colored) { g.fillStyle = o.color === 1 ? COLORS.red : COLORS.green; g.beginPath(); g.arc(X(o.x), Y(o.forward), 4, 0, 7); g.fill(); g.strokeStyle = o.confirmed ? '#fff' : '#f55'; g.lineWidth = 1.5; g.stroke(); }
    }
    { const rb = sim.robot.cfg; g.fillStyle = '#4da3ff'; g.fillRect(X(-rb.bodyWidth / 2), Y(rb.bodyLength - rb.rearOverhang), rb.bodyWidth * k, rb.bodyLength * k); }
    /* campo de la camara: +-34 grados */
    g.strokeStyle = 'rgba(255,255,255,.3)'; g.beginPath(); g.moveTo(X(0), Y(6)); g.lineTo(X(-67), Y(106)); g.moveTo(X(0), Y(6)); g.lineTo(X(67), Y(106)); g.stroke();
  }

  function drawCamera() {
    const c = $('camera'), g = c.getContext('2d');
    if (!sim || !sim.snapshot.camera) { g.fillStyle = '#000'; g.fillRect(0, 0, 160, 120); return; }
    g.putImageData(new ImageData(sim.snapshot.camera, 160, 120), 0, 0);
    g.strokeStyle = 'rgba(255,255,255,.35)'; g.lineWidth = .6; g.strokeRect(0, 20, 160, 90);
    const v = sim.snapshot.vision;
    if (v) for (const o of v.objects) {
      g.strokeStyle = o.color === 'red' ? '#ff8c8c' : o.color === 'green' ? '#8bffac' : '#ff9cf5'; g.lineWidth = 1;
      g.strokeRect(o.minX, o.minY, o.maxX - o.minX + 1, o.maxY - o.minY + 1);
    }
    $('cameraInfo').textContent = v ? 'S3 propone ' + v.action + ' · ' + v.objects.map(o => o.color + ' ' + o.distance.toFixed(0) + ' cm' + (o.flags & 1 ? ' (cortado)' : '')).join(', ') +
      (v.orangeRow ? ' · línea naranja fila ' + v.orangeRow : '') + (v.blueRow ? ' · línea azul fila ' + v.blueRow : '') : '';
  }

  function kv(el, rows) { el.innerHTML = rows.map(([k, v, cls]) => `<b>${k}</b><span class="${cls || ''}">${v}</span>`).join(''); }
  /* Odometria y pose en el campo, con el error contra la verdad del simulador (en el robot real no existe). */
  function poseRows(c, p) {
    const f = (v, d = 0) => (Number.isFinite(v) && Math.abs(v) < 1e8) ? v.toFixed(d) : '—';
    const rows = [];
    if (c.odom) {
      const o = c.odom;
      let err = '';
      if (o.inUse && sim.start) {
        const dx = p.x - sim.start.x, dz = p.z - sim.start.z, y0 = sim.start.yaw;
        const tx = dx * Math.cos(y0) - dz * Math.sin(y0), ty = dx * Math.sin(y0) + dz * Math.cos(y0);
        err = ` · error ${f(Math.hypot(o.x - tx, o.y - ty), 1)} cm`;
      }
      rows.push(['Odometría (encoders)', o.inUse ? `${f(o.path)} cm · ${f(o.v, 1)} cm/s · ${o.slips} patinaje(s)${err}` : 'sin encoders: usa la velocidad ordenada', o.inUse ? '' : 'warn']);
    }
    if (c.arena) {
      const a = c.arena;
      let err = '';
      if (a.valid) {
        let dth = a.theta - p.yaw * 180 / Math.PI; while (dth > 180) dth -= 360; while (dth <= -180) dth += 360;
        err = ` · error ${f(Math.hypot(a.x - p.x, a.z - p.z), 1)} cm / ${f(Math.abs(dth), 1)}°`;
      }
      rows.push(['Pose en el campo (LiDAR)', a.valid ? `x ${f(a.x)} · z ${f(a.z)} · ${f(a.theta)}° · ${a.inliers}/${a.used} puntos${err}` : 'buscando (sección de salida)', a.valid ? 'good' : 'warn']);
    }
    if (c.hug) {
      const h = c.hug;
      rows.push(['Pegado a la pared interior', h.active ? `desplazamiento ${f(h.cmd, 1)} cm (objetivo ${f(h.target, 1)} cm)` : (h.hold > 0 ? 'centrado: pilar que no esta en el mapa' : 'centrado (primera vuelta, esquina, letrero o sin usar)'), h.active ? 'good' : '']);
    }
    if (c.race) {
      const r = c.race;
      let txt = 'se calcula al terminar la primera vuelta', cls = '';
      if (r.valid) {
        txt = `${f(r.n)} estaciones · ${f(r.length)} cm · vuelta estimada ${f(r.time, 1)} s (centro ${f(r.timeCenter, 1)} s) · ${f(r.signs)} letreros`;
        if (r.active) txt += ` · velocidad ${f(r.vCmd, 0)}/${f(r.vLine, 0)} cm/s · a ${f(r.dist, 1)} cm de la línea${r.grip < 0.99 ? ' · agarre ' + f(100 * r.grip) + ' %' : ''}${r.hold > 0 ? ' · pilar desconocido: velocidad base' : ''}`;
        else txt += ' · fuera de la línea (modo reactivo)';
        cls = r.active ? 'good' : 'warn';
      } else if (r.tried) { txt = 'no hay paso para la línea con estos letreros: modo reactivo'; cls = 'warn'; }
      rows.push(['Línea de carrera', txt, cls]);
    }
    if (c.lapMem && c.arena && c.arena.valid) rows.push(['Letreros recordados', `${f(c.lapMem.err)} en el mapa · ${f(c.lapMem.inject)} reinyecciones (${f(c.lapMem.anchored)} ancladas al LiDAR)`]);
    return rows;
  }
  function drawPanels() {
    if (!sim) return;
    const c = sim.cmd, r = sim.result(), p = sim.robot.pose(), j = sim.judge;
    const f = (v, d = 0) => (Number.isFinite(v) && Math.abs(v) < 1e8) ? v.toFixed(d) : '—';
    kv($('telemetry'), c ? [
      ['Fase', c.phase], ['Orden', c.action], ['Volante / velocidad', `${f(c.steer, 1)}° · ${f(c.speed, 0)} cm/s`],
      ['Esquinas / vueltas', `${c.corners} · ${c.laps}`], ['Sentido', c.direction > 0 ? 'horario' : c.direction < 0 ? 'antihorario' : 'por aprender'],
      ['Rumbo / recorrido', `${f(c.yaw)}° / ${f(c.course)}°`], ['Muro delante', f(c.front) + ' cm'], ['Izq / der', `${f(c.left)} / ${f(c.right)} cm`],
      ['Carril / paso previsto', `${f(c.lane)} / ${c.pass < 0 ? 'bloqueado' : f(c.pass)} cm`], ['Escape', c.escaping ? 'fase ' + c.escapePhase + ' (' + c.escapes + ')' : '—'],
      ['Último letrero', c.lastSign === 1 ? 'rojo' : c.lastSign === 2 ? 'verde' : '—'],
      ...poseRows(c, p), ['Parqueo (pose en el cajón)', c.phase === 'PARKING' || c.phase === 'PARK_EXIT' ? `u ${f(c.park.u, 1)} · w ${f(c.park.w, 1)} · ${f(c.park.theta)}°` : '—'], ['Cámara', c.cameraFresh ? `${c.cameraHits} anclada(s), ${c.cameraUnmatched} sin LiDAR` : 'sin datos frescos', c.cameraFresh ? '' : 'warn']
    ] : [['Estado', 'esperando el arranque']]);
    kv($('judge'), [
      ['Tiempo', `${f(r.elapsed == null ? Math.max(0, sim.t - sim.options.startDelay) : r.elapsed, 1)} / 180 s`], ['Puntos', f(r.points)],
      ['Secciones', `${r.sections} / 24`], ['Vueltas', r.laps], ['Final en la salida', r.inStartSection ? 'sí' : 'no', r.inStartSection ? 'good' : ''],
      ['Parqueo', r.parking === 'full' ? 'completo (15)' : r.parking === 'partial' ? 'parcial (7)' : sim.field.lot ? 'no' : 'sin cajón', r.parking === 'full' ? 'good' : ''],
      ['Arranque en el cajón', sim.field.startInLot ? 'sí (7)' : 'no'],
      ['Contactos', `pared ${r.contacts.wall} · letrero ${r.contacts.sign}`], ['Distancia', f(r.travelled) + ' cm'],
      ['Termina por', r.endReason, r.endReason === 'finished' ? 'good' : r.endReason === 'running' ? '' : 'bad']
    ]);
    const warn = F.checkLayout(field, { bodyWidth: sim.robot.cfg.bodyWidth }).map(w => '⚠ ' + w.text);
    $('violations').textContent = r.violations.map(v => `${v.t.toFixed(1)} s: ${v.text}`).concat(warn, generatedNotice ? [generatedNotice] : []).join(' · ');
    $('log').innerHTML = sim.log.events.slice(-10).map(e => `${e.t.toFixed(1)} s · ${e.text}`).join('<br>');
    if (sim.finished) setStatus('terminada: ' + r.endReason + ' · ' + r.points + ' puntos', r.endReason === 'finished' ? 'good' : 'bad');
    else if (playing) setStatus('simulando ' + sim.t.toFixed(1) + ' s · ' + actualSpeed.toFixed(2) + '× real', '');
  }

  function drawSequences() {
    const box = $('seqBox');
    if (!window.SEQ || box.dataset.done) return;
    box.dataset.done = '1'; box.style.display = '';
    const q = window.SEQ;
    box.querySelector('summary').textContent = 'Pruebas de visión con imágenes sintéticas: ' + q.summary;
    box.querySelector('div').innerHTML = '<table style="border-collapse:collapse;width:100%;font-size:12px"><tr><th align=left>Escenario</th><th align=left>Cuadros</th><th align=left>Resultado</th></tr>' +
      q.scenarios.map(s => `<tr><td>${s.name}</td><td>${s.frames}</td><td class="${s.ok ? 'good' : 'bad'}">${s.result}</td></tr>`).join('') + '</table>' +
      (q.failures.length ? '<ul class="bad">' + q.failures.map(x => '<li>' + x + '</li>').join('') + '</ul>' : '');
  }
  function drawTraceMap() {
    const canvas=$('traceMap'), g=canvas.getContext('2d');
    g.clearRect(0,0,canvas.width,canvas.height);
    if (!sim || !sim.cmd) { $('traceInfo').textContent='Esperando simulación.'; return; }
    const m=core.traceMap(), scale=canvas.width/m.size;
    for(let row=0;row<m.size;row++) for(let col=0;col<m.size;col++) {
      const v=m.cells[row*m.size+col]; if(!v)continue;
      g.fillStyle=(v&8)?'#ff72ed':(v&2)?'#ee2737':(v&4)?'#44d62c':'#9eb0c2';
      g.fillRect(col*scale,canvas.height-(row+1)*scale,Math.ceil(scale),Math.ceil(scale));
    }
    const toPx=(x,z)=>[(x-m.originCm)/m.cellCm*scale, canvas.height-(z-m.originCm)/m.cellCm*scale];
    /* Objetos del trace_map (sin las paredes del campo): circulo con el color confirmado y el id. */
    g.font='10px sans-serif';g.lineWidth=2;
    for(const o of core.traceObjects()){
      if(o.kind===1)continue;
      const [px,py]=toPx(o.x,o.z), r=Math.max(5,o.length/m.cellCm*scale/2+2);
      g.strokeStyle=o.color===1?'#ee2737':o.color===2?'#44d62c':o.color===3?'#ff72ed':'#ffffff';
      g.beginPath();g.arc(px,py,r,0,7);g.stroke();
      g.fillStyle='#ffffff';g.fillText('#'+o.id,px+r+2,py+3);
    }
    /* Plaza de parqueo del mapa: rectangulo entre los dos topes (confirmada) o aviso en el primer tope (hipotesis). */
    const sp=core.traceSpot();
    if(sp.status===2){
      const hl=sp.length/2, hw=sp.width/2, alongX=sp.axis===0?0:1, [x0,z0]=sp.axis===0?[sp.x-hw,sp.z-hl]:[sp.x-hl,sp.z-hw];
      const [ax,ay]=toPx(x0,z0), [bx,by]=toPx(x0+(sp.axis===0?2*hw:2*hl),z0+(sp.axis===0?2*hl:2*hw));
      g.strokeStyle='#ffd24c';g.setLineDash([6,4]);g.strokeRect(Math.min(ax,bx),Math.min(ay,by),Math.abs(bx-ax),Math.abs(by-ay));g.setLineDash([]);
      g.fillStyle='#ffd24c';g.fillText('plaza',Math.min(ax,bx),Math.min(ay,by)-3);
    } else if(sp.status===1){
      const [px,py]=toPx(sp.bars[0][0],sp.bars[0][1]);g.strokeStyle='#ffd24c';g.beginPath();g.arc(px,py,8,0,7);g.stroke();
    }
    const a=sim.cmd.arena;
    if(a&&a.valid){ const px=(a.x-m.originCm)/m.cellCm*scale, py=canvas.height-(a.z-m.originCm)/m.cellCm*scale;
      g.fillStyle='#57b4ff';g.beginPath();g.arc(px,py,3,0,7);g.fill(); }
    const parking=core.traceParking();
    $('traceParkingInfo').textContent=!parking.seen?'Candidato de parqueo: esperando cámara.':parking.status===1?'Candidato de parqueo experimental; aún no confirmado.':parking.memory?'Último candidato guardado: requiere volver a verificar.':parking.status===2?'Candidato de parqueo: evidencia ambigua.':'Candidato de parqueo: esperando evidencia LiDAR.';
    if(parking.status===1||parking.memory) for(const p of (parking.status===1?parking.points:parking.memory)){
      const px=(p.x-m.originCm)/m.cellCm*scale,py=canvas.height-(p.z-m.originCm)/m.cellCm*scale;
      g.strokeStyle='#ffd24c';g.lineWidth=2;g.beginPath();g.arc(px,py,5,0,7);g.stroke();
    }
    $('traceInfo').textContent=m.scans ? `${m.laps<1?'construyendo la 1.ª vuelta':m.laps+' vuelta(s) acumuladas'} · ${core.traceObjects().filter(o=>o.kind!==1).length} objetos · ${m.occupied} celdas LiDAR · ${m.scans} barridos · ${m.bytes} bytes en RAM · ${m.clipped} retornos fuera · desconocido en oscuro` : `Esperando pose global válida · ${m.bytes} bytes reservados en RAM`;
  }
  function draw() { drawWorld(); drawLocal(); drawTraceMap(); drawCamera(); drawPanels(); drawSequences(); }

  /* --------------------------------------------------------------- controles */
  $('downloadTrace').onclick = () => {
    if (!sim) return;
    const m=core.traceMap(), blob=new Blob([m.cells],{type:'application/octet-stream'}), link=document.createElement('a');
    link.href=URL.createObjectURL(blob);link.download='trace_map_160x160_2.5cm.bin';link.click();URL.revokeObjectURL(link.href);
  };
  function nearestSeat(e) {
    const c = $('world'), r = c.getBoundingClientRect(), T = worldTransform(c);
    const x = ((e.clientX - r.left) * c.width / r.width - c.width / 2) / T.S, z = (c.height / 2 - (e.clientY - r.top) * c.height / r.height) / T.S;
    let best = null, bd = 9;
    for (const s of F.seats(field)) { const d = Math.hypot(s.x - x, s.z - z); if (d < bd) { bd = d; best = s; } }
    return best;
  }
  function cycleSeat(seat) {
    generatedNotice = '';
    const i = field.signs.findIndex(s => Math.hypot(s.seat.x - seat.x, s.seat.z - seat.z) < 1);
    const list = field.signs.map(s => ({ color: s.color, x: s.x, z: s.z, rot: s.rot, seatX: s.seat.x, seatZ: s.seat.z }));
    if (i < 0) list.push({ color: 'red', x: seat.x, z: seat.z });
    else if (list[i].color === 'red') list[i].color = 'green';
    else list.splice(i, 1);
    field = F.create({ signs: list, direction: field.direction, inner: field.inner, wallT: field.wallT, seatModel: field.seatModel,
                      robotBody: field.robotBody, seed: field.seed, start: field.start, parking: field.parking,
                      startInLot: field.startInLot, startOnTrack: field.startOnTrack });
    $('preset').value = 'random'; startSim();
  }
  function bind() {
    $('play').onclick = () => { if (!sim) return startSim(); if (sim.finished) startSim(); playing = !playing; simBudget = 0; speedWindowStart = 0; actualSpeed = 0; $('play').textContent = playing ? '⏸ Pausar' : '▶ Simular'; };
    $('restart').onclick = startSim;
    $('preset').onchange = newField; $('direction').onchange = newField;
    $('withLot').onchange = () => { if (!$('withLot').checked) { $('startInLot').checked = false; $('startOnTrack').checked = false; } newField(); };
    $('startInLot').onchange = () => { if ($('startInLot').checked) { $('withLot').checked = true; $('startOnTrack').checked = false; } newField(); };
    $('startOnTrack').onchange = () => { if ($('startOnTrack').checked) { $('withLot').checked = true; $('startInLot').checked = false; } newField(); };
    $('random').onclick = () => { $('seed').value = Math.floor(Math.random() * 9000) + 1; $('preset').value = 'random'; newField(); };
    $('seed').onchange = () => { $('preset').value = 'random'; newField(); };
    $('signCount').onchange = () => { $('preset').value = 'random'; newField(); };
    for (const id of ['redSide', 'greenSide', 'judgeFollows', 'rSpeed', 'rLidarNoise', 'rCamNoise', 'rGyroBias', 'rSpeedGain', 'rTrim', 'rLight', 'rLatency', 'rCamFps', 'rBitErr']) $(id).onchange = startSim;
    $('applyCtrl').onclick = startSim; $('applyVision').onclick = startSim;
    $('defaultsCtrl').onclick = () => { core.ctrlDefaults(); $('ctrlText').value = core.ctrlConfigDump(); startSim(); };
    $('defaultsVision').onclick = () => { core.visionDefaults(); $('visionText').value = core.visionConfigDump(); startSim(); };
    const w = $('world');
    w.onmousemove = e => { hoverSeat = nearestSeat(e); };
    w.onmouseleave = () => { hoverSeat = null; };
    w.onclick = e => { const s = nearestSeat(e); if (s) cycleSeat(s); };
    $('ioBtn').onclick = () => { $('ioText').value = F.toJSON(field); $('io').style.display = 'block'; $('ioStatus').textContent = field.signs.length + ' letrero(s)'; };
    $('ioClose').onclick = () => { $('io').style.display = 'none'; };
    $('ioCopy').onclick = () => { $('ioText').select(); document.execCommand('copy'); $('ioStatus').textContent = 'copiado'; };
    $('ioApply').onclick = () => { try { field = F.parse($('ioText').value); $('direction').value = field.direction;
      $('withLot').checked = !!field.lot; $('startInLot').checked = !!field.startInLot; $('startOnTrack').checked = !!field.startOnTrack; $('preset').value = 'random'; startSim(); $('ioStatus').textContent = 'aplicado'; } catch (e) { $('ioStatus').textContent = 'Error: ' + e.message; } };
    window.WROSim = { get sim() { return sim; }, get core() { return core; } };
  }
  boot().catch(e => { setStatus('no se pudo cargar el firmware: ' + e.message, 'bad'); console.error(e); });
})();
