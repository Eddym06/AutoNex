/* El juez: aplica el reglamento WRO con la VERDAD del simulador (posicion real del
 * robot y de los letreros), que el controlador nunca ve. Mide lo mismo que mediria
 * un juez en la pista:
 *   - secciones recorridas en el sentido del reto (1 punto cada una, 24 en tres vueltas);
 *   - paso de cada letrero por su lado (rojo derecha, verde izquierda): la ronda
 *     termina si el robot cruza COMPLETAMENTE la linea radial por el lado malo;
 *   - letrero movido fuera de su circulo de 85 mm: termina la ronda;
 *   - parada en la seccion de salida y parqueo en paralelo (completo / parcial);
 *   - 7 puntos por arrancar dentro del cajon y completar una vuelta;
 *   - 180 s por ronda.
 * Reglamento 2026: las tres vueltas van en el mismo sentido. La regla de 2024 (el ultimo
 * letrero de la segunda vuelta, si es rojo, invierte la tercera) sigue disponible con
 * la opcion `reverseRule`, para quien la necesite.
 * El lado por color es un parametro: una regla sorpresa o una sede distinta lo
 * cambian y el juez debe cambiar con el robot. */
(function (root) {
  'use strict';
  const WRO = root.WRO = root.WRO || {};
  const G = WRO.geometry || require('./geometry.js').geometry;
  const F = WRO.field || require('./field.js').field;

  class Judge {
    constructor(field, options) {
      const o = Object.assign({ passSide: { red: 1, green: -1 }, roundSeconds: F.RULES.roundSeconds, laps: 3, reverseRule: false, wheelbase: 6, parallelToleranceCm: 2,
                                stallSeconds: 8, finishHoldSeconds: 15 }, options || {});
      this.field = field;
      this.opt = o;
      this.reset();
    }
    reset() {
      this.started = false; this.t0 = 0; this.ended = null;
      this.direction = this.field.direction;
      this.initialDirection = this.field.direction;
      this.lastOrder = null; this.net = 0; this.maxNet = 0; this.backSteps = 0; this.flipNet = null;
      this.sectionTrace = [];
      this.signState = new Map();
      this.lastSignLap2 = null; this.finalDirection = this.field.direction; this.flipped = false;
      this.violations = []; this.contacts = { wall: 0, limiter: 0, sign: 0 };
      this.stall = 0; this.stoppedSince = null; this.maxTravelNow = 0;
      this.signMoved = false;
      this.parkingTouched = false;
      this.lapTimes = [];
      /* Demostracion del parqueo: las tres vueltas se dan por hechas. */
      if (this.field.startOnTrack) { this.net = this.maxNet = this.opt.laps * 8; this.lastOrder = null; }
    }
    start(t) { this.started = true; this.t0 = t; }
    elapsed(t) { return t - this.t0; }

    end(t, reason) {
      if (this.ended) return;
      this.ended = { t, reason, elapsed: t - this.t0 };
    }

    /* Un paso: `robot` = {x, z, yaw, v}; `body` = caja del chasis; events = contactos. */
    update(t, robot, body, events, done) {
      if (!this.started || this.ended) return;
      for (const e of events) {
        if (e.kind === 'wall') this.contacts.wall++;
        else if (e.kind === 'limiter') { this.contacts.limiter++; this.parkingTouched = true;
          this.violations.push({ t, kind: 'limiter', text: 'toco un tope del parqueo' }); this.end(t, 'limiter'); }
        else if (e.kind === 'sign') this.contacts.sign++;
      }
      if (this.ended) return;
      if (this.elapsed(t) >= this.opt.roundSeconds) { this.end(t, 'time'); return; }
      this.trackSections(t, body);
      this.checkSigns(t, robot, body);
      if (this.ended) return;
      /* Robot parado sin haber terminado: haria falta reparar (mitad de puntos). */
      if (Math.abs(robot.v) < 0.5) this.stall += 1 / 100; else this.stall = 0;
      if (!done && this.stall > this.opt.stallSeconds) { this.violations.push({ t, kind: 'stalled', text: 'el robot se quedo parado' }); this.end(t, 'stalled'); }
      if (done) {
        if (this.stoppedSince == null) this.stoppedSince = t;
        if (t - this.stoppedSince >= this.opt.finishHoldSeconds) this.end(t, 'finished');
      }
    }

    trackSections(t, body) {
      const sec = F.sectionAt(this.field, body.x, body.z);
      if (sec < 0) return;
      const order = F.orderIndex(sec, this.direction);
      if (this.lastOrder === null) { this.lastOrder = order; this.sectionTrace.push({ t, sec }); return; }
      if (order === this.lastOrder) return;
      const delta = (order - this.lastOrder + 8) % 8;
      this.lastOrder = order;
      this.sectionTrace.push({ t, sec });
      if (delta === 1) {
        this.net++;
        if (this.net > this.maxNet) {
          this.maxNet = this.net;
          if (this.maxNet % 8 === 0) this.lapTimes.push(t - this.t0);
          if (this.maxNet === 16 && this.opt.reverseRule) this.decideFinalDirection(t);
        }
      } else if (delta === 7) {
        this.net--;
        this.backSteps++;
        const reference = this.flipNet == null ? this.maxNet : this.flipNet;
        if (this.net < reference - 2) {
          this.violations.push({ t, kind: 'opposite', text: 'condujo en sentido contrario mas de dos secciones' });
          this.end(t, 'opposite');
        }
      }
    }

    /* El ultimo letrero de la segunda vuelta manda: verde sigue, rojo da media vuelta. */
    decideFinalDirection(t) {
      if (this.lastSignLap2 === 'red') {
        this.finalDirection = this.direction === 'ccw' ? 'cw' : 'ccw';
        this.flipped = true;
        this.direction = this.finalDirection;
        this.flipNet = this.net;
        const sec = this.sectionTrace.length ? this.sectionTrace[this.sectionTrace.length - 1].sec : 0;
        this.lastOrder = F.orderIndex(sec, this.direction);
      }
      this.decisionTime = t;
    }

    checkSigns(t, robot, body) {
      const lapIndex = Math.floor(this.maxNet / 8);
      const corners = G.corners(body);
      for (const sign of this.field.signs) {
        /* Letrero movido: su huella debe seguir dentro del circulo de 85 mm. */
        const R = F.RULES.circleDiameter / 2;
        if (!this.signMoved) {
          const out = G.corners(sign).some(([x, z]) => Math.hypot(x - sign.seat.x, z - sign.seat.z) > R + 1e-6);
          if (out) {
            this.signMoved = true;
            this.violations.push({ t, kind: 'moved', text: 'movio un letrero fuera de su circulo' });
            if (this.maxNet < this.opt.laps * 8 || true) this.end(t, 'moved');
            return;
          }
        }
        if (this.maxNet >= this.opt.laps * 8) continue;    // tras la ultima vuelta los letreros ya no obligan
        const sec = sign.section == null ? F.sectionAt(this.field, sign.seat.x, sign.seat.z) : sign.section;
        const T = F.travelVector(sec, this.direction);
        if (!T[0] && !T[1]) continue;
        const R2 = [T[1], -T[0]];                          // derecha del avance
        let sMin = Infinity;
        for (const [x, z] of corners) sMin = Math.min(sMin, (x - sign.seat.x) * T[0] + (z - sign.seat.z) * T[1]);
        const along = (body.x - sign.seat.x) * T[0] + (body.z - sign.seat.z) * T[1];
        const lateral = (body.x - sign.seat.x) * R2[0] + (body.z - sign.seat.z) * R2[1];
        const st = this.signState.get(sign) || { prev: null, lap: -1 };
        /* Sin muestra anterior no hay cruce: un robot que ya arranca pasado el letrero
         * lo cruzara al final de la vuelta, no ahora. */
        const crossing = st.prev !== null && st.prev <= 0 && sMin > 0 && Math.abs(along) < 25 && Math.abs(lateral) < 60 && st.lap !== lapIndex;
        st.prev = sMin;
        if (crossing) {
          st.lap = lapIndex;
          const wanted = this.opt.passSide[sign.color];
          const ok = wanted === 0 || (wanted > 0 ? lateral >= 0 : lateral <= 0);
          this.passes = this.passes || [];
          this.passes.push({ t: +t.toFixed(2), lap: lapIndex, color: sign.color, ok, at: [+sign.seat.x.toFixed(0), +sign.seat.z.toFixed(0)] });
          if (lapIndex === 1) this.lastSignLap2 = sign.color;
          if (!ok) {
            this.violations.push({ t, kind: 'wrong-side', color: sign.color, sign: [+sign.seat.x.toFixed(1), +sign.seat.z.toFixed(1)], robot: [+body.x.toFixed(1), +body.z.toFixed(1)], lateral: +lateral.toFixed(1), text: 'paso un letrero ' + sign.color + ' por el lado equivocado' });
            this.end(t, 'wrong-side');
          }
        }
        this.signState.set(sign, st);
      }
    }

    /* Parqueo: 'full' = completamente dentro del cajon y paralelo (las dos ruedas de un
     * mismo lado con diferencia de distancias a la pared <= 2 cm, aproximada por
     * sen(angulo) x batalla <= 2 cm);
     * 'partial' = solo parte dentro, o dentro pero no paralelo. */
    parkingResult(body) {
      const lot = this.field.lot;
      if (!lot) return null;
      const inside = ([x, z]) => {
        const along = lot.axis === 'x' ? x : z, lateral = lot.axis === 'x' ? z : x;
        const lo = Math.min(lot.wallCoord, lot.innerCoord), hi = Math.max(lot.wallCoord, lot.innerCoord);
        return along >= lot.a0 && along <= lot.a1 && lateral >= lo && lateral <= hi;
      };
      const pts = G.corners(body), n = pts.filter(inside).length;
      if (n === 0) return null;
      const yaw = body.rot || 0;
      const sinAngle = Math.abs(lot.axis === 'x' ? Math.cos(yaw) : Math.sin(yaw));
      const parallel = sinAngle * this.opt.wheelbase <= this.opt.parallelToleranceCm;
      return n === 4 && parallel ? 'full' : 'partial';
    }

    startSectionResult(body) {
      return G.corners(body).every(([x, z]) => F.sectionAt(this.field, x, z) === 0);
    }

    report(body) {
      const lapsDone = Math.floor(this.maxNet / 8);
      const allLaps = this.maxNet >= this.opt.laps * 8;
      const parking = this.parkingTouched ? null : this.parkingResult(body);
      const inStart = allLaps && this.startSectionResult(body);
      let points = Math.min(24, this.maxNet) + Math.min(this.opt.laps, lapsDone);
      if (allLaps && inStart) points += 3;
      if (lapsDone >= 1) {
        const unmoved = !this.signMoved;
        if (!allLaps) points += unmoved ? 4 : 2;
        else points += unmoved ? 10 : 8;
        if (this.field.startInLot) points += 7;        // arrancar dentro del cajon y completar una vuelta
      }
      if (allLaps && parking === 'full') points += 15;
      else if (allLaps && parking === 'partial') points += 7;
      return {
        points, sections: Math.min(24, this.maxNet), laps: lapsDone, allLaps, inStartSection: inStart, parking,
        endReason: this.ended ? this.ended.reason : 'running', elapsed: this.ended ? this.ended.elapsed : null,
        violations: this.violations.slice(), contacts: Object.assign({}, this.contacts), signMoved: this.signMoved,
        lastSignLap2: this.lastSignLap2, flipped: this.flipped, backSteps: this.backSteps, lapTimes: this.lapTimes.slice()
      };
    }
  }

  WRO.judge = { Judge };
  if (typeof module !== 'undefined') module.exports = WRO;
})(typeof window !== 'undefined' ? window : globalThis);
