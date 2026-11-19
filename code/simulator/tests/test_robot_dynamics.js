/* Prueba del perfil con dinamica (largo26d): servo MG90S y neumaticos que saturan con el peso detras.
 *   node simulator/tests/test_robot_dynamics.js
 * Comprueba: (1) a baja velocidad coincide con el modelo cinematico; (2) el giro estacionario se queda por debajo del cinematico
 * cuando la aceleracion lateral se acerca a mu*g (subviraje: el eje delantero va ligero); (3) la aceleracion lateral nunca pasa de
 * mu*g; (4) el retardo y la velocidad del servo; (5) el tope de traccion al acelerar y al frenar (solo el eje trasero frena). */
'use strict';
const path = require('node:path');
const W = require(path.join(__dirname, '../web/robot.js'));
const P = W.robot.PROFILES;
let fails = 0;
const ok = (c, m) => { if (!c) { fails++; console.log('FALLA:', m); } else console.log('ok   :', m); };
const world = { solids: [], signs: [] };

function circle(profile, steer, speed, T = 6) {
  const r = new W.robot.Robot(profile);
  r.reset({ x: 0, z: 0, yaw: 0 });
  r.v = speed; r.steer = steer;                       /* arranca ya a velocidad y con el volante puesto */
  const dt = 0.005; let yr = 0, lat = 0, n = 0;
  for (let t = 0; t < T; t += dt) {
    r.cmd = { steer, speed: speed / profile.speedGain };
    r.v = Math.max(r.v, 0); r.step(dt, world);
    if (t > T * 0.6) { yr += r.yawRate; n++; lat += Math.abs(r.yawRate) * r.v; }
  }
  return { yawDps: yr / n / (Math.PI / 180), latAcc: lat / n, v: r.v, slip: r.slipAngleDeg };
}

const base = Object.assign({}, W.robot.DEFAULT_ROBOT, P.largo26d);
const kin = Object.assign({}, W.robot.DEFAULT_ROBOT, P.largo26);
/* (1) baja velocidad: giro cinematico */
{
  const a = circle(base, 15, 25), b = circle(kin, 15, 25);
  ok(Math.abs(a.yawDps - b.yawDps) / Math.abs(b.yawDps) < 0.06, `25 cm/s, 15 grados: giro ${a.yawDps.toFixed(1)} frente a cinematico ${b.yawDps.toFixed(1)} grados/s`);
}
/* (2) y (3) mas velocidad: subviraje y tope de agarre */
const mu = base.tire.mu, lim = mu * 981;
let prev = null;
for (const v of [60, 100, 140, 180]) {
  const a = circle(base, 25, v), b = circle(kin, 25, v);
  const ratio = a.yawDps / b.yawDps;
  console.log(`  v=${v} cm/s volante 25: giro ${a.yawDps.toFixed(1)} (cinematico ${b.yawDps.toFixed(1)}), a_lat ${a.latAcc.toFixed(0)} cm/s2 (limite ${lim.toFixed(0)}), deriva trasera ${a.slip.toFixed(1)} grados`);
  ok(a.latAcc <= lim * 1.05, `v=${v}: aceleracion lateral ${a.latAcc.toFixed(0)} <= mu*g ${lim.toFixed(0)}`);
  if (prev) ok(ratio <= prev + 0.02, `v=${v}: el coche gira cada vez menos de lo que pide el volante (${ratio.toFixed(2)} <= ${prev.toFixed(2)})`);
  prev = ratio;
}
/* (4) servo: retardo y velocidad */
{
  const r = new W.robot.Robot(base); r.reset({ x: 0, z: 0, yaw: 0 }); r.v = 0;
  const dt = 0.001; let tDelay = null, t60 = null, t0 = 0;
  for (let t = 0; t < 0.6; t += dt) {
    r.cmd = { steer: 30, speed: 0 }; r.step(dt, world);
    if (tDelay === null && r.steer > 0.5) tDelay = t;
    if (t60 === null && r.steer >= 29) t60 = t;
  }
  console.log(`  servo: empieza a moverse a ${(tDelay * 1000).toFixed(0)} ms, llega a 29 grados a ${(t60 * 1000).toFixed(0)} ms`);
  ok(tDelay >= 0.018, 'el servo respeta el retardo de la orden');
  ok(t60 > 0.08 && t60 < 0.25, 'recorrer 30 grados tarda como un MG90S cargado');
}
/* (5) traccion: arrancar a fondo y frenar, con y sin tope */
{
  const run = (prof, sp0, sp1) => {
    const r = new W.robot.Robot(prof); r.reset({ x: 0, z: 0, yaw: 0 }); r.v = sp0;
    let vmax = 0, amax = 0, aminv = 0, pv = r.v, lim = 0;
    for (let t = 0; t < 1.5; t += 0.005) {
      r.cmd = { steer: 0, speed: sp1 }; r.step(0.005, world);
      const a = (r.v - pv) / 0.005; pv = r.v; amax = Math.max(amax, a); aminv = Math.min(aminv, a); if (r.tractionLimited) lim++;
    }
    return { amax, amin: aminv, lim };
  };
  const dyn = Object.assign({}, base, { maxAccel: 3000, speedTauS: 0.03 });   /* un motor que empuja mas de lo que el suelo aguanta */
  const acc = run(dyn, 0, 300), dec = run(dyn, 150, 0);
  const capAcc = base.tire.mu * 981 * base.tire.wr / (1 - base.tire.mu * base.tire.h / base.wheelbase);
  const capDec = base.tire.mu * 981 * base.tire.wr / (1 + base.tire.mu * base.tire.h / base.wheelbase);
  console.log(`  aceleracion maxima ${acc.amax.toFixed(0)} (tope ${capAcc.toFixed(0)}), frenada maxima ${(-dec.amin).toFixed(0)} (tope ${capDec.toFixed(0)}) cm/s2`);
  ok(acc.amax <= capAcc * 1.05, 'la aceleracion respeta el tope de traccion');
  ok(-dec.amin <= capDec * 1.05, 'la frenada respeta el tope del eje trasero');
}
console.log(fails ? `FALLAN ${fails}` : 'PASS');
process.exit(fails ? 1 : 0);
