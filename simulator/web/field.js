/* La pista de Future Engineers (reglamento WRO 2026), en cm.
 *
 *  - Tapete de 320 x 320; pista interior de 300 x 300; paredes negras de 10 cm.
 *  - Reto con obstaculos: carril de 100 cm, muro interior cuadrado de 100 cm.
 *  - Cuatro rectas y cuatro esquinas por vuelta (ocho secciones).
 *  - Letreros de 5 x 5 x 10 cm (rojo / verde) en asientos de 5 x 5 cm.
 *  - Parqueo: dos topes magenta de 20 x 2 x 10 cm, 20 cm de ancho y 1,5 veces el
 *    largo del robot, junto a la pared exterior de la seccion de arranque.
 *  - Lineas naranja y azul de 2 cm en el suelo: una pareja en V por esquina, que sale
 *    del vertice del muro interior a 30 grados de cada muro hasta el muro exterior.
 *
 * Lo que el reglamento no fija (grosor de las paredes, posicion exacta de los
 * asientos y de las lineas) es parametro: ver `options`. */
(function (root) {
  'use strict';
  const WRO = root.WRO = root.WRO || {};
  const G = WRO.geometry || (typeof require !== 'undefined' ? require('./geometry.js').geometry : null);
  const RNG = WRO.rng || (typeof require !== 'undefined' ? require('./rng.js').rng : null);
  const MAX_LAYOUT_SLOPE = 0.45;

  const RULES = {
    outerHalf: 150, signW: 5, signD: 5, signH: 10, wallH: 10,
    limiterLength: 20, limiterThickness: 2, limiterH: 10, lotWidth: 20, lotLengthFactor: 1.5,
    circleDiameter: 8.5, lineWidth: 2, laneWidth: 100, roundSeconds: 180, maxSigns: 7,
    colors: {
      red: [238, 39, 55], green: [68, 214, 44], magenta: [255, 0, 255],
      orange: [255, 102, 0], blue: [0, 51, 255], wall: [12, 12, 14], floor: [236, 236, 232]
    }
  };

  /* Secciones en el orden del sentido antihorario, empezando por la recta sur. */
  const SECTION_NAMES = ['S', 'SE', 'E', 'NE', 'N', 'NW', 'W', 'SW'];

  function sectionAt(field, x, z) {
    const [ihx, ihz] = field.inner;
    const ax = Math.abs(x) > ihx, az = Math.abs(z) > ihz;
    if (!ax && !az) return -1;
    if (ax && az) return x > 0 ? (z < 0 ? 1 : 3) : (z < 0 ? 7 : 5);
    if (az) return z < 0 ? 0 : 4;
    return x > 0 ? 2 : 6;
  }

  /* Indice de la seccion en el orden de marcha. */
  function orderIndex(index, direction) {
    return direction === 'cw' ? (8 - index) % 8 : index;
  }

  /* Vector unitario de avance en una recta, segun el sentido. */
  function travelVector(index, direction) {
    const ccw = [[1, 0], [0, 0], [0, 1], [0, 0], [-1, 0], [0, 0], [0, -1], [0, 0]][index];
    const s = direction === 'cw' ? -1 : 1;
    return [ccw[0] * s, ccw[1] * s];
  }

  /* Geometria de una recta: eje, centro del carril, extremos. */
  function straightInfo(field, index) {
    const [ihx, ihz] = field.inner, H = RULES.outerHalf;
    switch (index) {
      case 0: return { axis: 'x', center: -(H + ihz) / 2, lo: -ihx, hi: ihx, inner: -ihz, outer: -H, width: H - ihz };
      case 4: return { axis: 'x', center: (H + ihz) / 2, lo: -ihx, hi: ihx, inner: ihz, outer: H, width: H - ihz };
      case 2: return { axis: 'z', center: (H + ihx) / 2, lo: -ihz, hi: ihz, inner: ihx, outer: H, width: H - ihx };
      case 6: return { axis: 'z', center: -(H + ihx) / 2, lo: -ihz, hi: ihz, inner: -ihx, outer: -H, width: H - ihx };
      default: return null;
    }
  }

  /* Asientos: 2 a lo largo x 3 a lo ancho por recta (el reglamento dice seis por
   * recta: cuatro cruces en T y dos en X). El reglamento publicado no da las cotas
   * exactas de los asientos, asi que son parametros (`seatAlong`, `seatLateral`,
   * fracciones del largo de la recta y del ancho del carril): MIDELAS en la pista
   * real y cambialas aqui. Los valores por defecto dejan 27,5 cm entre un letrero y
   * la pared (cabe un robot de 20 cm) y 40 cm de recta antes del primer asiento. */
  const SEAT_DEFAULTS = { seatAlong: [0.4, 0.8], seatLateral: [-0.2, 0, 0.2] };
  function seats(field) {
    const along = (field.seatModel && field.seatModel.seatAlong) || SEAT_DEFAULTS.seatAlong;
    const lateralF = (field.seatModel && field.seatModel.seatLateral) || SEAT_DEFAULTS.seatLateral;
    const out = [];
    for (const index of [0, 2, 4, 6]) {
      const s = straightInfo(field, index);
      const len = s.hi - s.lo;
      /* La fraccion se mide en el sentido de la marcha. */
      const T = travelVector(index, field.direction);
      const forwardPositive = (s.axis === 'x' ? T[0] : T[1]) > 0;
      for (let a = 0; a < along.length; a++) {
        for (let l = 0; l < lateralF.length; l++) {
          const alongCoord = forwardPositive ? s.lo + len * along[a] : s.hi - len * along[a];
          const lateral = s.center + lateralF[l] * s.width;
          const x = s.axis === 'x' ? alongCoord : lateral, z = s.axis === 'x' ? lateral : alongCoord;
          out.push({ section: index, alongIndex: a, lateralIndex: l, x, z });
        }
      }
    }
    return out;
  }

  function wallBoxes(field) {
    const t = field.wallT, H = RULES.outerHalf, [ihx, ihz] = field.inner, h = RULES.wallH, out = [];
    const add = (x, z, w, d) => out.push({ x, z, w, d, rot: 0, h, kind: 'wall' });
    add(0, -H - t / 2, 2 * H + 2 * t, t); add(0, H + t / 2, 2 * H + 2 * t, t);
    add(-H - t / 2, 0, t, 2 * H); add(H + t / 2, 0, t, 2 * H);
    /* El muro interior mira al carril con su cara en +-ih y se extiende hacia dentro. */
    add(0, -ihz + t / 2, 2 * ihx, t); add(0, ihz - t / 2, 2 * ihx, t);
    add(-ihx + t / 2, 0, t, 2 * ihz); add(ihx - t / 2, 0, t, 2 * ihz);
    return out;
  }

  /* Ocho lineas de 2 cm: en cada esquina una naranja y una azul que salen del vertice
   * del muro interior en V y llegan al muro exterior; cada una forma 30 grados con el
   * muro que prolonga. En las esquinas de la izquierda la empinada es azul y la tendida
   * naranja; en las de la derecha, al reves (tal como las muestra el tapete oficial). */
  function cornerLines(field) {
    const [ihx, ihz] = field.inner, H = RULES.outerHalf, w = RULES.lineWidth, out = [];
    const a = 30 * Math.PI / 180;
    const line = (kind, vx, vz, dx, dz, length) => out.push({
      kind, x: vx + dx * length / 2, z: vz + dz * length / 2, w, d: length, rot: Math.atan2(dx, dz)
    });
    for (const sx of [-1, 1]) for (const sz of [-1, 1]) {
      const vx = sx * ihx, vz = sz * ihz;
      const shallow = sx < 0 ? 'orange' : 'blue', steep = sx < 0 ? 'blue' : 'orange';
      /* tendida: sigue la direccion del muro horizontal, abriendose 30 grados hacia el muro exterior */
      line(shallow, vx, vz, sx * Math.cos(a), sz * Math.sin(a), (H - ihx) / Math.cos(a));
      line(steep, vx, vz, sx * Math.sin(a), sz * Math.cos(a), (H - ihz) / Math.cos(a));
    }
    return out;
  }

  function makeSign(color, seat, rng, jitterDeg) {
    const rot = (rng ? rng.range(-jitterDeg, jitterDeg) : 0) * Math.PI / 180;
    return { color, kind: color, seat: { x: seat.x, z: seat.z }, x: seat.x, z: seat.z, rot,
             w: RULES.signW, d: RULES.signD, h: RULES.signH, section: seat.section };
  }

  function lotGeometry(field, parking, robotLength) {
    const s = straightInfo(field, parking.section);
    const length = RULES.lotLengthFactor * (parking.robotLength || robotLength || 22);
    const sign = s.outer > 0 ? 1 : -1;               // el muro exterior esta en +H o -H
    const wallCoord = s.outer;
    const innerCoord = wallCoord - sign * RULES.lotWidth; // borde del parqueo hacia el carril
    const lateralCenter = (wallCoord + innerCoord) / 2;
    const a0 = parking.along - length / 2, a1 = parking.along + length / 2;
    const t = RULES.limiterThickness, out = [];
    for (const a of [a0 - t / 2, a1 + t / 2]) {
      const x = s.axis === 'x' ? a : lateralCenter, z = s.axis === 'x' ? lateralCenter : a;
      out.push({ x, z, w: s.axis === 'x' ? t : RULES.limiterLength, d: s.axis === 'x' ? RULES.limiterLength : t,
                 rot: 0, h: RULES.limiterH, kind: 'limiter' });
    }
    return { section: parking.section, axis: s.axis, a0, a1, length, wallCoord, innerCoord, lateralCenter, lotWidth: RULES.lotWidth, limiters: out };
  }

  function startPose(field) {
    const s = straightInfo(field, 0), st = field.start || {};
    /* Arrancar dentro del cajon (7 puntos si se completa una vuelta): el robot queda
     * paralelo a la pared, centrado entre los topes, con el cuerpo (no el eje) en el centro. */
    /* Demostracion: en el carril, 65 cm antes del cajon y a 40 cm de la pared, paralelo a ella. */
    if (field.startOnTrack && field.lot && field.lot.section === 0) {
      const ccwT = field.direction !== 'cw', lot = field.lot;
      const centreT = (lot.a0 + lot.a1) / 2, lo = Math.max(-s.hi, s.lo + 6);
      const along = ccwT ? Math.max(s.lo + 6, centreT - 65) : Math.min(s.hi - 6, centreT + 65);
      return { x: along, z: lot.wallCoord + (lot.innerCoord - lot.wallCoord) * (40 / lot.lotWidth),
               yaw: ccwT ? Math.PI / 2 : -Math.PI / 2 };
    }
    if (field.startInLot && field.lot && field.lot.section === 0) {
      const ccwL = field.direction !== 'cw', lot = field.lot, rb = field.robotBody || { length: 22, rear: 4 };
      const centreAlong = (lot.a0 + lot.a1) / 2, bodyShift = rb.length / 2 - rb.rear;
      /* pegado al lado del carril (12,5 cm de la pared): centrado en el cajon no puede girar para salir */
      const lateral = lot.wallCoord + (lot.innerCoord - lot.wallCoord) * (12.5 / lot.lotWidth);
      return { x: centreAlong - (ccwL ? 1 : -1) * bodyShift, z: lateral,
               yaw: ccwL ? Math.PI / 2 : -Math.PI / 2 };
    }
    const along = st.along == null ? 20 : st.along, lateral = st.lateral || 0;
    const ccw = field.direction !== 'cw';
    const x = ccw ? s.lo + along : s.hi - along;
    const z = s.center + (ccw ? -lateral : lateral);
    const yaw = (st.yawErrorDeg || 0) * Math.PI / 180 + (ccw ? Math.PI / 2 : -Math.PI / 2);
    return { x, z, yaw };
  }

  function create(options) {
    const o = options || {};
    const field = {
      inner: o.inner ? [o.inner[0], o.inner[1]] : [50, 50],
      wallT: o.wallT == null ? 2 : o.wallT,
      direction: o.direction === 'cw' ? 'cw' : 'ccw',
      start: Object.assign({ along: 20, lateral: 0, yawErrorDeg: 0 }, o.start || {}),
      seed: o.seed == null ? 1 : o.seed,
      signs: [], parking: null, lot: null, walls: [], lines: [], clutter: o.clutter || [],
      seatModel: o.seatModel || null, startInLot: !!o.startInLot, startOnTrack: !!o.startOnTrack && !o.startInLot,
      robotBody: o.robotBody || null
    };
    field.walls = wallBoxes(field);
    field.lines = o.lines === false ? [] : cornerLines(field);
    const rng = RNG(field.seed);
    const jitter = o.signRotJitterDeg == null ? 2 : o.signRotJitterDeg;
    for (const s of (o.signs || [])) {
      const sign = makeSign(s.color, { x: s.x, z: s.z, section: sectionAt(field, s.x, s.z) }, rng, 0);
      sign.rot = (s.rot || 0);
      sign.seat = { x: s.seatX == null ? s.x : s.seatX, z: s.seatZ == null ? s.z : s.seatZ };
      field.signs.push(sign);
    }
    if (o.parking) {
      field.parking = Object.assign({ section: 0, along: 0, robotLength: o.robotBody ? o.robotBody.length : 22 }, o.parking);
      field.lot = lotGeometry(field, field.parking);
    }
    field.signJitterDeg = jitter;
    return field;
  }

  function seatsPerRowAlong(field) {
    return ((field.seatModel && field.seatModel.seatAlong) || SEAT_DEFAULTS.seatAlong).length;
  }

  /* Letreros aleatorios en asientos validos. */
  function randomLayout(seed, options) {
    const o = Object.assign({ signs: 8, parking: true, direction: 'ccw', inner: [50, 50] }, options || {});
    const rng = RNG(seed);
    const base = create({ inner: o.inner, direction: o.direction, seed, seatModel: o.seatModel });
    const all = seats(base);
    const parking = o.parking ? { section: 0, along: 0 } : null;     // el cajon va siempre en la seccion de arranque
    if (parking) {
      const s = straightInfo(base, parking.section);
      const len = s.hi - s.lo;
      parking.along = s.lo + len * rng.range(0.3, 0.7);
      parking.robotLength = o.robotLength || (o.robotBody && o.robotBody.length) || 22;
    }
    /* El robot arranca donde el sorteo lo deja; un letrero a menos de ~75 cm por
     * delante en su recta no deja carrera para ponerse al lado que toca (el radio
     * de giro del carrito es 21 cm), asi que ese asiento no se sortea. */
    const start = startPose(Object.assign({}, base, { start: { along: 0, lateral: 0 } }));
    const startAlong = o.startAlong == null ? 60 : o.startAlong;
    const candidates = all.filter(seat => {
      if (o.feasible === true && seat.section === 0) {
        const ahead = (seat.x - start.x) * (o.direction === 'cw' ? -1 : 1);
        if (ahead > -20 - 0 && ahead < startAlong + 90) return false;
      }
      if (!parking || seat.section !== parking.section) return true;
      /* En la recta del parqueo no hay letreros cerca de la pared exterior. */
      const s = straightInfo(base, seat.section);
      const lateral = s.axis === 'x' ? seat.z : seat.x;
      return Math.abs(lateral - s.outer) > s.width * 0.6;
    });
    const picked = [], rows = {};
    let guard = 0;
    while (picked.length < Math.min(o.signs, RULES.maxSigns * 2) && guard++ < 500) {
      const seat = candidates[Math.floor(rng() * candidates.length)];
      if (picked.includes(seat)) continue;
      const key = seat.section + ':' + seat.alongIndex;
      /* Dos por fila como mucho y separados (asientos 0 y 2): entre dos letreros
       * pegados quedan 15 cm, que no es un paso para un robot de verdad. */
      const row = rows[key] || [];
      /* Una fila con dos letreros (rojo a la izquierda, verde a la derecha) solo tiene
       * paso en el sentido de marcha original: si el ultimo letrero de la 2.a vuelta
       * es rojo y se da media vuelta, esa misma fila queda con el verde a la
       * izquierda, que no tiene trayectoria. Por defecto una fila lleva uno solo. */
      const rowLimit = o.allowGates === false ? 1 : 2;
      if (row.length >= rowLimit || (row.length === 1 && Math.abs(row[0] - seat.lateralIndex) < 2)) continue;
      row.push(seat.lateralIndex);
      rows[key] = row;
      picked.push(seat);
    }
    /* Un layout que el reglamento sortea tiene que poder pasarse: el robot no se
     * teletransporta de un lado del carril al otro entre dos letreros seguidos. Se
     * descartan los letreros cuyo lado exigido queda mas lejos de lo que un giro de
     * radio 21 cm recorre en la distancia que los separa (pendiente <= 0,45).
     * Es el mismo criterio con el que un juez humano reconoceria un layout "injusto". */
    if (o.feasible === true) {
      const colorOf = seat => seat.__color;
      picked.forEach(seat => { seat.__color = rng() < 0.5 ? 'red' : 'green'; });
      for (let changed = true; changed;) {
        changed = false;
        /* En los dos sentidos: si el ultimo letrero de la 2.a vuelta es rojo, la 3.a se
         * conduce al reves y los mismos letreros piden el lado contrario. */
        for (const [index, flip] of [0, 2, 4, 6].flatMap(i => [[i, 1], [i, -1]])) {
          const T0 = travelVector(index, o.direction), T = [T0[0] * flip, T0[1] * flip], right = [T[1], -T[0]];
          const info = straightInfo(base, index), laneC = info.axis === 'x' ? [0, info.center] : [info.center, 0];
          const row = picked.filter(seat => seat.section === index).sort((a, b) =>
            ((a.x - b.x) * T[0] + (a.z - b.z) * T[1]));
          let prevV = null, prevAlong = null;
          for (const seat of row) {
            const along = (seat.x - (T[0] ? (T[0] > 0 ? info.lo : info.hi) : 0)) * T[0] + (seat.z - (T[1] ? (T[1] > 0 ? info.lo : info.hi) : 0)) * T[1];
            const v = (seat.x - laneC[0]) * right[0] + (seat.z - laneC[1]) * right[1];
            const need = colorOf(seat) === 'red' ? v + 12 : v - 12;       // a la derecha / izquierda del letrero (letrero 2,5 + medio chasis 6,5 + 3)
            const lo = colorOf(seat) === 'red' ? need : -1e9, hi = colorOf(seat) === 'red' ? 1e9 : need;
            let shift;
            if (prevV === null) shift = Math.max(0, lo - 6, -6 - hi) / (along + 10);   // el color solo se ve a ~50 cm: poco margen tras la esquina
            else shift = Math.max(0, lo - prevV.hi, prevV.lo - hi) / Math.max(1, along - prevAlong);
            if (shift > MAX_LAYOUT_SLOPE) { picked.splice(picked.indexOf(seat), 1); changed = true; break; }
            prevV = { lo, hi }; prevAlong = along;
          }
          if (changed) break;
        }
      }
    }
    /* El ultimo letrero de la segunda vuelta decide el sentido de la tercera. Si la
     * ultima fila tuviera dos, el "ultimo" seria ambiguo para cualquiera: aqui la
     * ultima fila lleva a lo sumo uno. */
    const lastStraight = o.direction === 'cw' ? 2 : 6;
    const lastKey = lastStraight + ':' + (seatsPerRowAlong(base) - 1);
    const lastRow = picked.filter(seat => seat.section + ':' + seat.alongIndex === lastKey);
    if (o.reverseRule && lastRow.length > 1) picked.splice(picked.indexOf(lastRow[1]), 1);
    const signs = picked.map(seat => ({ color: seat.__color || (rng() < 0.5 ? 'red' : 'green'), x: seat.x, z: seat.z,
                                         rot: rng.range(-2, 2) * Math.PI / 180, seat }));
    /* Dos letreros en la misma fila solo tienen solucion si el de la izquierda es
     * rojo (se rebasa por su derecha) y el de la derecha verde (por su izquierda):
     * el robot pasa ENTRE los dos. Al reves, hay que pasar por fuera de los dos a la
     * vez y no existe trayectoria. */
    const byRow = {};
    for (const sg of signs) (byRow[sg.seat.section + ':' + sg.seat.alongIndex] = byRow[sg.seat.section + ':' + sg.seat.alongIndex] || []).push(sg);
    for (const row of Object.values(byRow)) {
      if (o.feasible !== true || row.length !== 2) continue;
      const T = travelVector(row[0].seat.section, o.direction), right = [T[1], -T[0]];
      row.sort((a, b) => (a.x * right[0] + a.z * right[1]) - (b.x * right[0] + b.z * right[1]));
      row[0].color = 'red';
      row[1].color = 'green';
    }
    for (const sg of signs) delete sg.seat;
    return create({ inner: o.inner, direction: o.direction, seed, signs, parking, seatModel: o.seatModel,
                    startInLot: !!o.startInLot, startOnTrack: !!o.startOnTrack, robotBody: o.robotBody,
                    start: { along: rng.range(10, 60), lateral: rng.range(-15, 15), yawErrorDeg: rng.range(-3, 3) } });
  }

  /* Plano compartible: JSON de una linea. parse() RECHAZA en vez de corregir, para
   * que el plano cargado sea identico al pegado. */
  function toJSON(field) {
    return JSON.stringify({
      v: 3, inner: field.inner, wallT: field.wallT, seatModel: field.seatModel, robotBody: field.robotBody,
      direction: field.direction, seed: field.seed, start: field.start,
      signs: field.signs.map(s => ({ color: s.color, x: +s.x.toFixed(2), z: +s.z.toFixed(2), rot: +(s.rot || 0).toFixed(4), seatX: s.seat.x, seatZ: s.seat.z })),
      parking: field.parking ? { section: field.parking.section, along: +field.parking.along.toFixed(2),
                                 robotLength: field.parking.robotLength } : null,
      startInLot: !!field.startInLot, startOnTrack: !!field.startOnTrack
    });
  }

  function parse(text) {
    const raw = String(text == null ? '' : text).trim();
    if (!raw) throw new Error('no hay texto que cargar');
    let data;
    try { data = JSON.parse(raw); } catch (e) { throw new Error('no es JSON valido (' + e.message + ')'); }
    if (Array.isArray(data)) data = { signs: data };
    const list = data.signs || data.blocks;
    if (!data || typeof data !== 'object' || !Array.isArray(list)) throw new Error('falta la lista signs');
    const inner = data.inner ? data.inner.map(Number) : [50, 50];
    if (inner.length !== 2 || inner.some(v => !Number.isFinite(v) || v < 20 || v > 100)) throw new Error('inner: dos medidas entre 20 y 100 cm');
    const probe = create({ inner });
    const signs = list.map((b, i) => {
      const n = i + 1, x = Number(b && b.x), z = Number(b && b.z);
      const color = b && b.color;
      if (!Number.isFinite(x) || !Number.isFinite(z) || Math.abs(x) > 148 || Math.abs(z) > 148) throw new Error('letrero ' + n + ': x,z deben estar dentro del plano (-148 a 148 cm)');
      if (color !== 'red' && color !== 'green') throw new Error('letrero ' + n + ': el color debe ser "red" o "green"');
      if (sectionAt(probe, x, z) < 0) throw new Error('letrero ' + n + ': esta dentro del muro interior');
      const rot = b.rot == null ? 0 : Number(b.rot);
      if (!Number.isFinite(rot)) throw new Error('letrero ' + n + ': rot invalido');
      const seatX = b.seatX == null ? x : Number(b.seatX), seatZ = b.seatZ == null ? z : Number(b.seatZ);
      if (!Number.isFinite(seatX) || !Number.isFinite(seatZ) || Math.abs(seatX) > 150 || Math.abs(seatZ) > 150)
        throw new Error('letrero ' + n + ': asiento invalido');
      return { color, x, z, rot, seatX, seatZ };
    });
    if (signs.length > 2 * RULES.maxSigns) throw new Error('el reglamento permite como mucho ' + RULES.maxSigns + ' letreros de cada color');
    const direction = data.direction === 'cw' ? 'cw' : 'ccw';
    let parking = null;
    if (data.parking) {
      const p = data.parking, s = Number(p.section), a = Number(p.along);
      if (![0, 2, 4, 6].includes(s) || !Number.isFinite(a) || Math.abs(a) > 48) throw new Error('parking: seccion 0/2/4/6 y along entre -48 y 48');
      parking = { section: s, along: a, robotLength: Number(p.robotLength) || 17 };
    }
    const wallT = data.wallT == null ? 2 : Number(data.wallT);
    if (!Number.isFinite(wallT) || wallT <= 0 || wallT > 20) throw new Error('wallT: grosor invalido');
    const seatModel = data.seatModel || null;
    if (seatModel && (!Array.isArray(seatModel.seatAlong) || !seatModel.seatAlong.length ||
        !Array.isArray(seatModel.seatLateral) || !seatModel.seatLateral.length ||
        seatModel.seatAlong.some(v => !Number.isFinite(v) || v < 0 || v > 1) ||
        seatModel.seatLateral.some(v => !Number.isFinite(v) || Math.abs(v) > 0.5))) throw new Error('seatModel invalido');
    const robotBody = data.robotBody || null;
    if (robotBody && (!(robotBody.length > 0) || !(robotBody.rear >= 0) || robotBody.rear > robotBody.length)) throw new Error('robotBody invalido');
    return create({ inner, wallT, seatModel, robotBody, direction, signs, parking, seed: data.seed == null ? 1 : data.seed, start: data.start,
                   startInLot: !!data.startInLot, startOnTrack: !!data.startOnTrack });
  }

  /* Planos de referencia: los casos que cada regresion del planificador pidio. Las
   * posiciones estan en la recta sur (z = -100), en sentido antihorario. */
  function preset(name) {
    const S = (x, z, color) => ({ x, z, color });
    const presets = {
      empty: [],
      slalom5: [-40, -20, 0, 20, 40].map((x, i) => S(x, -100, i % 2 ? 'green' : 'red')),
      gate30: [S(-15, -100, 'red'), S(15, -100, 'green')],
      gate20: [S(-10, -100, 'red'), S(10, -100, 'green')],
      pocket: [S(-78, -100, 'red'), S(-73, -111, 'red'), S(-73, -89, 'red'), S(-67, -122, 'red'), S(-67, -78, 'red')],
      reported: [S(-24, -98, 'green'), S(-13, -79, 'red'), S(87, -66, 'green'), S(101, -43, 'green'),
                 S(-19, -126, 'green'), S(101, -115, 'green'), S(126, 97, 'green')],
      /* rojo en el carril de la derecha y verde en el de la izquierda: sin conflicto */
      colors: [S(-25, -100, 'red'), S(25, -100, 'green')],
      swapped: [S(-25, -100, 'green'), S(25, -100, 'red')]
    };
    if (!presets[name]) throw new Error('plano desconocido: ' + name);
    return create({ signs: presets[name], direction: 'ccw' });
  }


  /* Avisos sobre un plano: letreros seguidos que ningun chasis puede pasar. Para cada recta se ordenan los letreros en
   * el sentido de la marcha; cada uno exige que el CENTRO del robot quede a un lado (rojo: a su derecha; verde: a su
   * izquierda) a letrero/2 + medio chasis + 1 cm. Entre dos exigencias consecutivas el robot solo puede desplazarse
   * lateralmente `maxSlope` cm por cada cm de recta. Es una aproximacion conservadora, no prueba de trayectoria.
   * Si arranca dentro del cajon, la salida deja al robot a 30 cm de la
   * pared exterior: cuenta como la primera exigencia de esa recta. Es geometria sobre el plano, no mira el firmware. */
  function checkLayout(field, options) {
    const o = Object.assign({ bodyWidth: 13, maxSlope: MAX_LAYOUT_SLOPE, signWidth: 5 }, options || {});
    const ccw = field.direction !== 'cw', warnings = [];
    const need = o.signWidth / 2 + o.bodyWidth / 2 + 1;
    const ranges = { 0: 0, 2: 0, 4: 0, 6: 0 };
    for (const index of [0, 2, 4, 6]) {
      const T = travelVector(index, field.direction), right = [T[1], -T[0]], info = straightInfo(field, index);
      const axisX = info.axis === 'x', laneC = axisX ? [0, info.center] : [info.center, 0];
      const alongOf = p => (axisX ? p.x : p.z) * (axisX ? T[0] : T[1]);
      const latOf = p => (p.x - laneC[0]) * right[0] + (p.z - laneC[1]) * right[1];
      const list = field.signs.map((s, i) => ({ s, i })).filter(e => sectionAt(field, e.s.seat.x, e.s.seat.z) === index)
        .sort((a, b) => alongOf(a.s.seat) - alongOf(b.s.seat));
      const sign = axisX ? T[0] : T[1];
      const entry = { along: (sign > 0 ? info.lo : info.hi) * sign, lo: -15, hi: 15, label: 'entrada de la recta' };
      const signMarks = list.map(e => {
        const v = latOf(e.s.seat), red = e.s.color === 'red';
        return { along: alongOf(e.s.seat), lo: red ? v + need : -1e9, hi: red ? 1e9 : v - need, label: 'letrero #' + e.i + ' (' + e.s.color + ')', sign: e.i };
      });
      const chains = [[entry].concat(signMarks)];
      /* Arranque en el cajon: sale por el lado de la pared exterior (a la derecha en sentido antihorario) y solo
       * le afectan los letreros que quedan por delante. */
      if (index === 0 && field.startInLot && field.lot) {
        const outerRight = ccw ? 1 : -1, start = startPose(field), a0 = alongOf(start);
        chains.push([{ along: a0, lo: outerRight * 20 - 6, hi: outerRight * 20 + 6, label: 'salida del cajon' }].concat(signMarks.filter(m => m.along > a0)));
      }
      for (const marks of chains) {
        marks.sort((a, b) => a.along - b.along);
        for (let k = 1; k < marks.length; k++) {
          const a = marks[k - 1], b = marks[k];
          const gap = Math.max(0, b.lo - a.hi, a.lo - b.hi);
          const along = Math.max(1, b.along - a.along);
          if (gap > 0 && gap / along > o.maxSlope && !warnings.some(w => w.from === a.label && w.to === b.label))
            warnings.push({ kind: 'cambio-imposible', section: index, from: a.label, to: b.label, signs: [a.sign, b.sign].filter(x => x != null),
                            shift: Math.round(gap), along: Math.round(along),
                            text: 'Recta ' + SECTION_NAMES[index] + ': de ' + a.label + ' a ' + b.label + ' el robot debe desplazarse ' + Math.round(gap) + ' cm de lado en solo ' + Math.round(along) + ' cm de recta (limite razonable: ' + o.maxSlope + ' cm por cm).' });
        }
      }
    }
    return warnings;
  }

  WRO.field = { RULES, SECTION_NAMES, sectionAt, orderIndex, travelVector, straightInfo, seats, create,
                randomLayout, toJSON, parse, preset, startPose, lotGeometry, checkLayout };
  if (typeof module !== 'undefined') module.exports = WRO;
})(typeof window !== 'undefined' ? window : globalThis);
