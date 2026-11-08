/* Rondas completas contra el firmware REAL (WebAssembly) y el juez del reglamento.
 * node simulator/tests/test_missions.js */
const fs = require('node:fs'), path = require('node:path'), assert = require('node:assert/strict');
const web = path.join(__dirname, '../web');
const WRO = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));
const F = WRO.field;
const bytes = fs.readFileSync(path.join(web, 'wro_core.wasm'));

async function run(name, options, seconds, check) {
  const core = await WRO.core.load(bytes);
  /* Casos históricos del chasis diferencial; Ackermann requiere sus propios escenarios. */
  const legacyRobot = { drive: 'differential', wheelbase: 6, bodyWidth: 13, bodyLength: 17, rearOverhang: 7, trackWidth: 11.5, wheelDiameterL: 5.5, wheelDiameterR: 5.5 };
  const sim = new WRO.sim.Simulation(core, Object.assign({}, options, { robot: Object.assign(legacyRobot, options.robot || {}) }));
  const t0 = Date.now();
  const r = sim.run(seconds);
  try { check(r, sim); } catch (e) {
    e.message = `${name}: ${e.message}\n  resultado: ${JSON.stringify(Object.assign({}, r, { command: undefined, stats: undefined }))}`;
    throw e;
  }
  console.log(`  ok  ${name} (${r.points} puntos, ${(Date.now() - t0) / 1000 | 0} s de CPU)`);
}
const full = (r, label) => {
  assert.equal(r.endReason, 'finished', label + ': termina');
  assert(r.allLaps && r.inStartSection, label + ': tres vueltas y parada dentro de la seccion de salida');
  assert.equal(r.violations.length, 0, label + ': sin infracciones');
  assert.equal(r.contacts.wall + r.contacts.sign, 0, label + ': sin contactos');
};
const clean = r => { assert.equal(r.violations.length, 0, 'sin infracciones'); assert.equal(r.contacts.wall + r.contacts.sign, 0, 'sin contactos'); };

(async () => {
  console.log('Pista vacia');
  for (const direction of ['ccw', 'cw'])
    await run('3 vueltas ' + (direction === 'ccw' ? 'antihorario' : 'horario') + ' y parada en la salida', { field: F.create({ direction }), seed: 3 }, 200, r => {
      full(r, direction); assert(r.points >= 40, 'reglamento 2026: 24 + 3 + 3 + 10 puntos'); assert(r.elapsed < 175, 'cabe en los 180 s (' + r.elapsed + ')');
      assert.equal(r.laps, 3);
    });

  console.log('El lado de cada color es configuracion');
  const east = [{ color: 'red', x: 100, z: 0 }];
  await run('rojo se rebasa por la derecha', { field: F.create({ signs: east }), seed: 4 }, 38, r => { clean(r); assert(r.sections >= 4, 'avanza'); });
  await run('verde se rebasa por la izquierda', { field: F.create({ signs: [{ color: 'green', x: 100, z: 0 }] }), seed: 4 }, 38, r => { clean(r); assert(r.sections >= 4); });
  /* Sede con la regla invertida: S3, P4 y juez cambian por datos, sin tocar el codigo. */
  const swappedCtrl = 'planner.pass_side_red -1\nplanner.pass_side_green 1';
  await run('regla invertida (rojo izquierda): se obedece', {
    field: F.create({ signs: east }), seed: 4, ctrlConfig: swappedCtrl, visionConfig: WRO.sim.DEFAULT_VISION_TEXT + '\npass_side_red -1\npass_side_green 1',
    judge: { passSide: { red: -1, green: 1 } }
  }, 38, r => { clean(r); assert(r.sections >= 4); });
  await run('regla invertida con el juez del reglamento normal: seria infraccion', {
    field: F.create({ signs: east }), seed: 4, ctrlConfig: swappedCtrl, visionConfig: WRO.sim.DEFAULT_VISION_TEXT + '\npass_side_red -1'
  }, 38, r => assert.equal(r.endReason, 'wrong-side'));

  console.log('Paso entre letreros');
  await run('puerta de 45 cm (rojo a la izquierda, verde a la derecha)', {
    field: F.create({ signs: [{ color: 'red', x: 75, z: 0 }, { color: 'green', x: 125, z: 0 }] }), seed: 5 }, 40, r => { clean(r); assert(r.sections >= 4); });
  await run('eslalon: verde y luego rojo, ambos en el centro del carril', {
    field: F.create({ signs: [{ color: 'green', x: 100, z: -30 }, { color: 'red', x: 100, z: 25 }] }), seed: 6 }, 45, r => { clean(r); assert(r.sections >= 4); });

  console.log('Sensores imperfectos');
  await run('sesgo del giroscopio de 1,5 grados por segundo', { field: F.create({ direction: 'ccw' }), seed: 7, imu: { biasDps: 1.5 } }, 200, r => full(r, 'giro'));
  await run('un cuadro de cada cinco llega corrupto y la camara tarda 150 ms', {
    field: F.create({ signs: [{ color: 'green', x: 100, z: -10 }] }), seed: 8, camera: { bitErrorRate: 0.2, latencyMs: 150, dropRate: 0.1 } }, 40, r => {
    clean(r); assert(r.stats.framesRejected > 0, 'el CRC rechaza tramas corruptas'); assert(r.sections >= 4); });
  await run('sin camara (LiDAR solo) la pista vacia se completa', { field: F.create({ direction: 'cw' }), seed: 9, camera: { dropRate: 1 } }, 200, r => full(r, 'sin camara'));

  console.log('Ultima vuelta (reglamento 2026: sin inversion)');
  /* En 2026 el ultimo letrero de la 2.a vuelta no cambia nada: las tres vueltas van en el mismo sentido. */
  await run('rojo como ultimo letrero: sigue en el mismo sentido', {
    field: F.create({ signs: [{ color: 'red', x: -120, z: -30 }], direction: 'ccw' }), seed: 10 }, 200, r => {
    full(r, 'sin inversion'); assert(!r.flipped); });
  /* La regla de 2024 sigue disponible como configuracion (robot y juez): para quien la necesite. */
  await run('regla 2024 (opcional): rojo como ultimo letrero = media vuelta', {
    field: F.create({ signs: [{ color: 'red', x: -120, z: -30 }], direction: 'ccw' }), seed: 10,
    ctrlConfig: 'turnaround_enabled 1', judge: { reverseRule: true } }, 200, r => {
    full(r, 'media vuelta'); assert(r.flipped && r.lastSignLap2 === 'red', 'el juez tambien exige girar'); });

  console.log('Parqueo en paralelo (reglamento 2026)');
  /* Arranca dentro del cajon, sale solo, hace tres vueltas y vuelve a aparcar: 62 puntos. */
  for (const direction of ['ccw', 'cw'])
    await run('salir del cajon, 3 vueltas y aparcar (' + (direction === 'ccw' ? 'antihorario' : 'horario') + ')', {
      field: F.randomLayout(5, { direction, signs: 0, parking: true, startInLot: true }), seed: 5 }, 200, r => {
      assert.equal(r.violations.length, 0); assert.equal(r.parking, 'full', 'aparcado dentro y en paralelo');
      assert.equal(r.points, 62, 'maximo del reto con obstaculos 2026'); assert.equal(r.contacts.limiter, 0); });
  /* Lo que hace la interfaz: el cuadro de texto lleva el volcado de los valores por defecto
   * (start_in_lot false) y no debe tapar lo que dice la pista. Fallo real: el robot no salia del cajon. */
  {
    const probe = await WRO.core.load(bytes);
    probe.ctrlDefaults();
    const dump = probe.ctrlConfigDump();
    assert(/start_in_lot false/.test(dump));
    await run('salir del cajon con el volcado de configuracion de la interfaz', {
      field: F.randomLayout(1, { direction: 'ccw', signs: 6, parking: true, startInLot: true, feasible: true, allowGates: false }), seed: 1,
      ctrlConfig: dump + '\nplanner.speed_cm_s 20' }, 20, r => {
      assert(r.sections >= 1, 'sale del cajon y avanza'); assert.notEqual(r.endReason, 'stalled'); assert.equal(r.violations.length, 0); });
  }
  /* Demostracion: empieza en el carril con las vueltas dadas y solo busca el cajon y aparca. */
  for (const direction of ['ccw', 'cw'])
    await run('demostracion de parqueo desde la pista (' + (direction === 'ccw' ? 'antihorario' : 'horario') + ')', {
      field: F.randomLayout(2, { direction, signs: 0, parking: true, startOnTrack: true }), seed: 2 }, 90, r => {
      assert.equal(r.violations.length, 0); assert.equal(r.parking, 'full'); assert(r.elapsed < 60, 'aparca pronto'); });
  await run('con letreros: 3 vueltas y aparcar', {
    /* Plano pasable: el sorteo con `feasible` no deja cambios de lado que ningun chasis
     * puede trazar (el mismo criterio que la interfaz y el banco). */
    field: F.randomLayout(4, { direction: 'ccw', signs: 4, parking: true, startInLot: true, feasible: true, allowGates: false }), seed: 4 }, 200, r => {
    assert.equal(r.violations.length, 0); assert.equal(r.parking, 'full'); assert(r.points >= 52); });
  await run('con cajon pero arrancando fuera de el: sin los 7 puntos de salida', {
    field: F.randomLayout(5, { direction: 'ccw', signs: 0, parking: true, startInLot: false }), seed: 5 }, 200, r => {
    assert.equal(r.violations.length, 0); assert.equal(r.parking, 'full'); assert.equal(r.points, 55); });

  console.log('PASS: rondas completas, regla de color como dato, letreros, sensores imperfectos, reglamento 2026 y parqueo');
})().catch(e => { console.error(e.message || e); process.exit(1); });
