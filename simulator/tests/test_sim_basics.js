/* Pruebas del mundo simulado: geometria, pista, sensores, dinamica y juez.
 * node simulator/tests/test_sim_basics.js */
const fs = require('node:fs'), path = require('node:path'), assert = require('node:assert/strict');
const web = path.join(__dirname, '../web');
const WRO = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));
const { geometry: G, field: F, sensors: S, robot: R, judge: J } = WRO;
const near = (a, b, tol, msg) => assert(Math.abs(a - b) <= tol, `${msg || ''} ${a} != ${b} (+-${tol})`);

/* --- geometria ---------------------------------------------------------- */
{
  const box = { x: 0, z: 50, w: 10, d: 4, rot: 0 };
  near(G.rayBox(0, 0, 0, 1, box).t, 48, 1e-9, 'rayo frontal');
  assert.equal(G.rayBox(20, 0, 0, 1, box), null, 'rayo que pasa de largo');
  assert.equal(G.rayBox(0, 0, 0, 1, { x: 0, z: -50, w: 10, d: 4, rot: 0 }), null, 'caja a la espalda');
  const turned = { x: 0, z: 50, w: 10, d: 10, rot: Math.PI / 4 };           // rombo
  near(G.rayBox(0, 0, 0, 1, turned).t, 50 - 5 * Math.SQRT2, 1e-9, 'caja girada');
  assert(G.overlapBoxes({ x: 0, z: 0, w: 8, d: 12, rot: 0 }, { x: 5, z: 0, w: 5, d: 5, rot: 0 }));
  assert(!G.overlapBoxes({ x: 0, z: 0, w: 8, d: 12, rot: 0 }, { x: 8, z: 0, w: 5, d: 5, rot: 0 }));
  const push = G.separation({ x: 0, z: 0, w: 8, d: 12, rot: 0 }, { x: 5, z: 0, w: 5, d: 5, rot: 0 });
  near(push[0], 1.5, 1e-9, 'empuje minimo'); near(push[1], 0, 1e-9);
}

/* --- pista -------------------------------------------------------------- */
{
  const f = F.create({});
  /* Carril de 100 cm: las caras interiores de los muros estan a +-150 y +-50. */
  const south = f.walls.find(w => w.z < -140), inner = f.walls.find(w => Math.abs(w.z + 50) < 2 && w.w > 90);
  near(south.z + south.d / 2, -150, 1e-9, 'cara interior del muro exterior');
  near(inner.z - inner.d / 2, -50, 1e-9, 'cara del muro interior');
  assert.equal(F.sectionAt(f, 0, -100), 0); assert.equal(F.sectionAt(f, 100, -100), 1);
  assert.equal(F.sectionAt(f, 100, 0), 2); assert.equal(F.sectionAt(f, 0, 0), -1);
  /* Seis asientos por recta, cuatro rectas. */
  assert.equal(F.seats(f).length, 4 * 6);
  /* El orden de marcha cambia con el sentido. */
  assert.equal(F.orderIndex(1, 'ccw'), 1); assert.equal(F.orderIndex(1, 'cw'), 7);
  /* Plano en texto: ida y vuelta identica, y rechaza en vez de corregir. */
  const r = F.randomLayout(7, { direction: 'cw', signs: 6 });
  assert.equal(F.toJSON(F.parse(F.toJSON(r))), F.toJSON(r));
  const custom = F.create({ wallT: 3, seatModel: { seatAlong: [0.25, 0.75], seatLateral: [-0.1, 0.1] }, robotBody: { length: 22, rear: 5 } });
  const customCopy = F.parse(F.toJSON(custom));
  assert.equal(customCopy.wallT, 3);
  assert.deepEqual(customCopy.seatModel, custom.seatModel);
  assert.deepEqual(customCopy.robotBody, custom.robotBody);
  assert.throws(() => F.parse('{"signs":[{"color":"red","x":0,"z":0}]}'), /muro interior/);
  assert.throws(() => F.parse('{"signs":[{"color":"blue","x":0,"z":-100}]}'), /color/);
  assert.throws(() => F.parse('nada'), /JSON/);
  /* Colores oficiales del reglamento. */
  assert.deepEqual(F.RULES.colors.red, [238, 39, 55]); assert.deepEqual(F.RULES.colors.green, [68, 214, 44]);
  /* Parqueo (reglamento 2026): 20 cm de ancho y 1,5 veces el largo del robot entre dos topes magenta de 20 x 2 x 10. */
  const p = F.create({ parking: { section: 4, along: 0, robotLength: 17 } });
  near(p.lot.length, 25.5, 1e-9); assert.equal(p.lot.limiters.length, 2);
  assert(p.lot.limiters.every(b => Math.max(b.w, b.d) === 20 && Math.min(b.w, b.d) === 2 && b.h === 10));
  /* Lineas del suelo: ocho de 2 cm (4 naranjas, 4 azules), en V desde el vertice del muro
   * interior, a 30 grados de cada muro, hasta el muro exterior. */
  assert.equal(f.lines.length, 8);
  assert.equal(f.lines.filter(l => l.kind === 'orange').length, 4);
  assert.equal(f.lines.filter(l => l.kind === 'blue').length, 4);
  const ne = f.lines.filter(l => l.x > 50 && l.z > 50);
  assert.equal(ne.length, 2);
  for (const l of ne) {
    const dx = Math.sin(l.rot), dz = Math.cos(l.rot);
    const vx = l.x - dx * l.d / 2, vz = l.z - dz * l.d / 2;
    near(vx, 50, 1e-6, 'sale del vertice (x)'); near(vz, 50, 1e-6, 'sale del vertice (z)');
    const ang = Math.min(Math.atan2(Math.abs(dz), Math.abs(dx)), Math.atan2(Math.abs(dx), Math.abs(dz))) * 180 / Math.PI;
    near(ang, 30, 1e-6, 'a 30 grados del muro');
    near(Math.max(vx + dx * l.d, vz + dz * l.d), 150, 1e-6, 'llega al muro exterior');
  }
  /* El cajon va en la seccion de arranque y el robot puede empezar dentro, paralelo y centrado. */
  const L0 = F.randomLayout(3, { signs: 4, parking: true, startInLot: true });
  assert.equal(L0.lot.section, 0);
  const sp = F.startPose(L0);
  near(sp.z, L0.lot.wallCoord + 12.5, 1e-6);   /* pegado al lado del carril: 12,5 cm de la pared */ assert(sp.x > L0.lot.a0 && sp.x < L0.lot.a1);
  /* Los diseños aleatorios son pasables: nunca dos letreros con lados incompatibles seguidos. */
  for (let seed = 1; seed <= 30; seed++) {
    const L = F.randomLayout(seed, { direction: seed % 2 ? 'ccw' : 'cw', signs: 9 });
    assert(L.signs.length <= 14); assert(L.signs.every(s => F.sectionAt(L, s.x, s.z) >= 0));
  }
}

/* --- avisos del plano (checkLayout) ---------------------------------------- */
{
  /* Verde en el asiento interior y rojo exterior 40 cm despues: 60 cm de cambio de lado en 40 cm de recta. */
  const bad = F.create({ signs: [{ color: 'green', x: 80, z: -10 }, { color: 'red', x: 120, z: 30 }], direction: 'ccw' });
  const w = F.checkLayout(bad);
  assert.equal(w.length, 1); assert.equal(w[0].kind, 'cambio-imposible'); assert(w[0].shift >= 55 && w[0].along === 40);
  /* El mismo par con otro color es pasable. */
  assert.equal(F.checkLayout(F.create({ signs: [{ color: 'green', x: 80, z: -10 }, { color: 'green', x: 120, z: 30 }], direction: 'ccw' })).length, 0);
  /* Saliendo del cajon, un letrero pegado a la salida que exige el lado interior no se puede alcanzar. */
  const lot = F.create({ signs: [{ color: 'green', x: 30, z: -100 }], direction: 'ccw', parking: { section: 0, along: 5, robotLength: 17 }, robotBody: { length: 17, rear: 7 }, startInLot: true });
  assert(F.checkLayout(lot).some(x => x.from === 'salida del cajon'));
  /* La entrada es el primer extremo en el sentido de marcha, en ambas orientaciones. */
  for (const direction of ['ccw', 'cw']) for (const index of [0, 2, 4, 6]) {
    const f = F.create({ direction }), info = F.straightInfo(f, index), t = F.travelVector(index, direction);
    const right = [t[1], -t[0]], axisX = info.axis === 'x';
    const entry = (axisX ? t[0] : t[1]) > 0 ? info.lo : info.hi;
    const x = (axisX ? entry + 5 * t[0] : info.center) + right[0] * 30;
    const z = (axisX ? info.center : entry + 5 * t[1]) + right[1] * 30;
    const test = F.create({ direction, signs: [{ color: 'red', x, z }] });
    assert(F.checkLayout(test).some(w => w.section === index && w.from === 'entrada de la recta'), `${direction} recta ${index}`);
  }
  /* El generador aleatorio nunca produce estos casos. */
  for (let seed = 1; seed <= 60; seed++) for (const direction of ['ccw', 'cw']) {
    const L = F.randomLayout(seed, { direction, signs: 7, parking: seed % 2 === 0, startInLot: seed % 2 === 0, feasible: true, allowGates: false });
    assert.equal(F.checkLayout(L).length, 0, 'aleatorio ' + seed + ' ' + direction);
  }
}

/* --- LiDAR -------------------------------------------------------------- */
{
  const field = F.create({ signs: [{ color: 'red', x: 0, z: -60 }] });
  const rng = WRO.rng(5);
  const lidar = new S.Lidar({ sigmaNearCm: 0, sigmaFarCm: 0, dropout: 0, angleJitterDeg: 0 }, rng);
  const boxes = field.walls.concat(field.signs);
  const pose = { x: 0, z: -100, yaw: 0 };
  const hit = lidar.measure(200, pose, boxes);      // rayo hacia delante (angulo 0)
  const mount = lidar.cfg.mountForward;
  near(hit.angle, 0, 1e-9); assert(hit.valid);
  near(hit.distance, 40 - mount - 2.5, 0.01, 'pilar a 40 cm del eje: el LiDAR ve su cara a 40 - montaje - 2,5');
  /* Zona ciega: un objeto a menos de 5 cm no devuelve distancia y se marca. */
  const close = lidar.measure(200, { x: 0, z: -60 - mount - 2.5 - 3, yaw: 0 }, boxes);
  assert(!close.valid && close.blind, 'zona ciega');
  /* Mascara propia: los rayos de la espalda no ven el campo. */
  assert(!lidar.measure(0, pose, boxes).valid && !lidar.measure(399, pose, boxes).valid);
  /* Barrido continuo: el robot gira mientras mide y la pared se curva. */
  const walls = F.create({}).walls, still = new S.Lidar({ sigmaNearCm: 0, sigmaFarCm: 0, dropout: 0, angleJitterDeg: 0 }, rng);
  const turning = new S.Lidar({ sigmaNearCm: 0, sigmaFarCm: 0, dropout: 0, angleJitterDeg: 0 }, rng);
  let a = null, b = null;
  for (let t = 0.01; t <= 0.1001; t += 0.01) {
    a = still.advance(t, () => ({ x: 0, z: -100, yaw: 0 }), walls) || a;
    b = turning.advance(t, tt => ({ x: 0, z: -100, yaw: tt * 3 }), walls) || b;     // 3 rad/s: deliberadamente rapido
  }
  assert(a && b && a.points.length === 400);
  let moved = 0;
  for (let i = 0; i < 400; i++) if (a.points[i].valid && b.points[i].valid && Math.abs(a.points[i].distance - b.points[i].distance) > 1) moved++;
  assert(moved > 100, 'el giro durante el barrido debe deformar la vuelta (' + moved + ')');
  /* Ruido gaussiano acotado: sigma 0,5 cm cerca. */
  const noisy = new S.Lidar({ sigmaNearCm: 0.5, dropout: 0, angleJitterDeg: 0 }, WRO.rng(9));
  let sum = 0, sum2 = 0, n = 0;
  for (let i = 150; i < 250; i++) { const p = noisy.measure(i, { x: 0, z: -100, yaw: 0 }, walls); if (p.valid) { const e = p.distance - lidar.measure(i, { x: 0, z: -100, yaw: 0 }, walls).distance; sum += e; sum2 += e * e; n++; } }
  assert(n > 60 && Math.abs(sum / n) < 0.3 && Math.sqrt(sum2 / n) < 1.5, 'ruido del LiDAR fuera de ficha');
}

/* --- camara ------------------------------------------------------------- */
{
  const field = F.create({ signs: [{ color: 'red', x: 0, z: -60 }, { color: 'green', x: 25, z: -75 }] });
  const rng = WRO.rng(3), cam = Object.assign({}, S.DEFAULT_CAMERA, { noiseSigma: 0, vignette: 0 });
  const img = S.renderCamera(field, field.walls.concat(field.signs), { x: 0, z: -100, yaw: Math.PI / 2 * 0 }, cam, rng, {});
  assert.equal(img.rgb565.length, 160 * 120 * 2);
  /* El pilar rojo esta a 40 cm del eje, 34 de la lente: ancho = 118,6 * 5 / (34 cos(pitch)). */
  const px = (x, y) => { const o = (y * 160 + x) * 2, v = (img.rgb565[o] << 8) | img.rgb565[o + 1]; return [((v >> 11) & 31) * 255 / 31, ((v >> 5) & 63) * 255 / 63, (v & 31) * 255 / 31]; };
  const isRed = c => c[0] > 180 && c[1] < 90 && c[2] < 110;
  let minX = 999, maxX = -1;
  for (let x = 0; x < 160; x++) if (isRed(px(x, 46))) { minX = Math.min(minX, x); maxX = Math.max(maxX, x); }
  assert(maxX > minX, 'el pilar rojo debe verse');
  const width = maxX - minX + 1, depth = 40 - cam.mountForward, pitch = Math.atan((60 - 30) / 118.6);
  near(width, 118.6 * 5 / (depth * Math.cos(pitch) + 0), 3, 'ancho aparente del pilar');
  assert(isRed(px(80, 46)) || isRed(px(79, 46)), 'el pilar esta centrado');
  /* El suelo es blanco y las paredes negras; el cielo sobre el horizonte es el fondo. */
  const floor = px(20, 110), bg = px(20, 3);
  assert(floor[0] > 180 && floor[1] > 180, 'suelo blanco'); assert(bg[0] < 140, 'fondo');
  /* Cambiar la luz cambia el brillo (exposicion), no la forma. */
  const dark = S.renderCamera(field, field.walls.concat(field.signs), { x: 0, z: -100, yaw: 0 }, cam, WRO.rng(3), { light: 0.5 });
  assert(dark.rgba[(110 * 160 + 20) * 4] < img.rgba[(110 * 160 + 20) * 4] * 0.7, 'menos luz oscurece');
}

/* --- robot -------------------------------------------------------------- */
{
  /* Traccion diferencial: dos ruedas motrices atras y una libre delante; el eje motriz es la pose. */
  const r = new R.Robot({ drive: 'differential', wheelbase: 6, bodyWidth: 13, bodyLength: 17, rearOverhang: 7, speedGain: 1, wheelMismatch: 0 });
  assert.equal(r.cfg.drive, 'differential');
  assert(r.cfg.bodyWidth === 13 && r.cfg.bodyLength === 17 && r.cfg.rearOverhang === 7 && r.cfg.wheelbase === 6);
  r.reset({ x: 0, z: 0, yaw: 0 });
  r.cmd = { steer: 0, speed: 20 };
  for (let i = 0; i < 300; i++) r.step(0.01, { solids: [], signs: [] });
  near(r.z, 3 * 20 - 0.09 * 20, 6, 'recorre ~58 cm en 3 s con retardo de motor'); near(r.x, 0, 1e-6);
  /* El volante virtual del planificador: giro de 30 grados y batalla de 6 cm = radio de 10,4 cm. */
  r.reset({ x: 0, z: 0, yaw: 0 }); r.cmd = { steer: 30, speed: 20 };
  let guard = 0; while (r.yaw < Math.PI / 2 && guard++ < 2000) r.step(0.01, { solids: [], signs: [] });
  near(r.x, 10.4, 3, 'radio de giro con el volante virtual');
  /* Giro en el sitio: velocidad 0 y 90 grados/s. */
  r.reset({ x: 0, z: 0, yaw: 0 }); r.cmd = { steer: 0, speed: 0, yawRate: 90 };
  for (let i = 0; i < 100; i++) r.step(0.01, { solids: [], signs: [] });
  near(r.yaw * 180 / Math.PI, 90 * (1 - 0.09), 12, 'gira ~80 grados en un segundo sin desplazarse');
  near(Math.hypot(r.x, r.z), 0, 0.5, 'el eje motriz no se mueve al girar en el sitio');
  /* Un desajuste de motores desvia el avance recto (lo corrige el planificador, no la mecanica). */
  const bad = new R.Robot({ drive: 'differential', wheelbase: 6, bodyWidth: 13, bodyLength: 17, rearOverhang: 7, speedGain: 1, wheelMismatch: 0.06 });
  bad.reset({ x: 0, z: 0, yaw: 0 }); bad.cmd = { steer: 0, speed: 20 };
  for (let i = 0; i < 300; i++) bad.step(0.01, { solids: [], signs: [] });
  assert(Math.abs(bad.x) > 5, 'motores desiguales: el robot se desvia ' + bad.x);
  /* Modo bicicleta (servo): se mantiene para quien lo use. */
  const b = new R.Robot({ drive: 'bicycle', wheelbase: 12, steerTrimDeg: 0, speedGain: 1, maxSteerDeg: 30 });
  b.reset({ x: 0, z: 0, yaw: 0 }); b.cmd = { steer: 30, speed: 20 };
  for (let i = 0; i < 100; i++) b.step(0.01, { solids: [], signs: [] });
  guard = 0; while (b.yaw < Math.PI / 2 && guard++ < 2000) b.step(0.01, { solids: [], signs: [] });
  near(b.x, 20.8, 4, 'radio de giro con servo');
  /* El motor se cala contra una pared y no la atraviesa. */
  r.reset({ x: 0, z: 0, yaw: 0 }); r.cmd = { steer: 0, speed: 30 };
  const wall = { x: 0, z: 40, w: 100, d: 2, rot: 0, h: 10, kind: 'wall' };
  let hits = 0; for (let i = 0; i < 500; i++) hits += r.step(0.01, { solids: [wall], signs: [] }).length;
  assert(hits > 0 && r.z + (r.cfg.bodyLength - r.cfg.rearOverhang) < 40.5, 'choca y se detiene');
  /* Empuja un letrero. */
  r.reset({ x: 0, z: 0, yaw: 0 }); r.cmd = { steer: 0, speed: 20 };
  const sign = { x: 0, z: 30, w: 5, d: 5, rot: 0, h: 10, kind: 'red' };
  for (let i = 0; i < 300; i++) r.step(0.01, { solids: [], signs: [sign] });
  assert(sign.z > 30.5, 'el letrero se desplaza');
}

/* --- juez ---------------------------------------------------------------- */
{
  /* Rojo por la derecha: un robot que pasa por la izquierda termina la ronda. */
  const mk = (color, lateral) => {
    const field = F.create({ signs: [{ color, x: 0, z: -100 }], direction: 'ccw' });
    const j = new J.Judge(field, {});
    j.start(0);
    const robot = new R.Robot({}); robot.reset({ x: -40, z: -100 - lateral, yaw: Math.PI / 2 });
    for (let i = 0; i < 600 && !j.ended; i++) {
      robot.x += 0.5;                                     // 50 cm/s sin colisiones
      j.update(i / 100, { x: robot.x, z: robot.z, yaw: robot.yaw, v: 50 }, robot.body(), [], false);
    }
    return j;
  };
  /* Viajando hacia +x, la derecha es -z: lateral positivo = derecha. */
  assert(!mk('red', 9).ended || mk('red', 9).ended.reason !== 'wrong-side', 'rojo por la derecha: bien');
  assert.equal(mk('red', -9).ended.reason, 'wrong-side');
  assert.equal(mk('green', 9).ended.reason, 'wrong-side');
  assert(!mk('green', -9).ended || mk('green', -9).ended.reason !== 'wrong-side');
  /* La regla es un dato: cruzada, lo que era error es lo correcto. */
  const field = F.create({ signs: [{ color: 'red', x: 0, z: -100 }] });
  const swapped = new J.Judge(field, { passSide: { red: -1, green: 1 } });
  swapped.start(0);
  const rb = new R.Robot({}); rb.reset({ x: -40, z: -109, yaw: Math.PI / 2 }); // por la derecha del rojo
  for (let i = 0; i < 400 && !swapped.ended; i++) { rb.x += 0.5; swapped.update(i / 100, { x: rb.x, z: rb.z, yaw: rb.yaw, v: 50 }, rb.body(), [], false); }
  assert.equal(swapped.ended.reason, 'wrong-side');
  /* Un letrero fuera de su circulo de 85 mm termina la ronda. */
  const f2 = F.create({ signs: [{ color: 'green', x: 0, z: -100 }] });
  const j2 = new J.Judge(f2, {}); j2.start(0);
  f2.signs[0].x += 0.5; j2.update(0.01, { x: -80, z: -100, yaw: 0, v: 10 }, { x: -80, z: -100, w: 8, d: 12, rot: 0 }, [], false);
  assert(!j2.ended, 'medio centimetro sigue dentro del circulo');
  f2.signs[0].x += 3; j2.update(0.02, { x: -80, z: -100, yaw: 0, v: 10 }, { x: -80, z: -100, w: 8, d: 12, rot: 0 }, [], false);
  assert.equal(j2.ended.reason, 'moved');
  /* 180 segundos. */
  const j3 = new J.Judge(F.create({}), {}); j3.start(0);
  j3.update(180.1, { x: 0, z: -100, yaw: 0, v: 20 }, { x: 0, z: -100, w: 8, d: 12, rot: 0 }, [], false);
  assert.equal(j3.ended.reason, 'time');
  /* El timeout de parada usa el tiempo, no el número de llamadas. */
  const stalled = new J.Judge(F.create({}), { stallSeconds: 1 }); stalled.start(0);
  const stopBody = { x: 0, z: -100, w: 8, d: 12, rot: 0 };
  for (const t of [0.1, 0.35, 0.6, 0.85, 1.2]) stalled.update(t, { x: 0, z: -100, yaw: 0, v: 0 }, stopBody, [], false);
  assert.equal(stalled.ended.reason, 'stalled');
}

/* --- reglamento 2026: puntuacion y parqueo ------------------------------- */
{
  const mkJudge = (field, opt) => { const j = new J.Judge(field, opt || {}); j.start(0); return j; };
  /* Sin regla de inversion por defecto: el ultimo letrero rojo no cambia el sentido. */
  assert.equal(mkJudge(F.create({}), {}).opt.reverseRule, false);
  /* Parqueo paralelo dentro del cajon = completo; torcido o a medias = parcial. */
  const f = F.create({ parking: { section: 0, along: 0, robotLength: 17 } });
  const j = mkJudge(f); j.maxNet = 24;
  const lot = f.lot, cx = 0, cz = lot.lateralCenter;
  const body = (yawDeg, x) => ({ x: x == null ? cx : x, z: cz, w: 13, d: 17, rot: yawDeg * Math.PI / 180 });
  assert.equal(j.parkingResult(body(90)), 'full');
  assert.equal(j.parkingResult(body(90 + 25)), 'partial', 'torcido 25 grados: la rueda delantera queda a mas de 2 cm de la pared');
  assert.equal(j.parkingResult(body(90, lot.a0 - 20)), null, 'fuera del cajon');
  assert.equal(j.parkingResult(body(90, lot.a1 - 6)), 'partial', 'sobresale del cajon');
  /* El chasis cubre el cajon y ninguna de sus esquinas esta dentro. */
  assert.equal(j.parkingResult({ x: 0, z: cz, w: 40, d: 40, rot: 0 }), 'partial');
  /* Puntos: 24 secciones + 3 vueltas + 3 por parar + 10 letreros sin mover + 15 parqueo; +7 si arranco en el cajon. */
  assert.equal(j.report(body(90)).points, 24 + 3 + 3 + 10 + 15);
  const f2 = F.create({ parking: { section: 0, along: 0, robotLength: 17 }, startInLot: true });
  const j2 = mkJudge(f2); j2.maxNet = 24;
  assert.equal(j2.report(body(90)).points, 24 + 3 + 3 + 10 + 15 + 7);
}

/* --- el giroscopio deriva ------------------------------------------------ */
{
  const g = new S.Gyro({ biasDps: 0.5, noiseDps: 0, biasWalkDpsPerSqrtS: 0, scaleError: 0 }, WRO.rng(1));
  let yaw = 0;
  for (let i = 0; i < 6000; i++) yaw += g.sample(0, 0.01) * 0.01;      // un minuto quieto
  near(yaw, 30, 1e-6, 'un sesgo de 0,5 dps son 30 grados por minuto sin corregir');
}

console.log('PASS: geometria, pista WRO, LiDAR (zona ciega, barrido continuo, ruido), camara, dinamica del carrito, juez y giroscopio');
