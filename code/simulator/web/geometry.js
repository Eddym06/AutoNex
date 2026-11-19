/* Geometria 2D del simulador. Unidades: cm. Plano (x, z): x a la derecha del
 * mapa, z "norte". El rumbo (yaw) se mide como una brujula: 0 = +z, positivo
 * hacia +x (sentido horario visto desde arriba). */
(function (root) {
  'use strict';
  const WRO = root.WRO = root.WRO || {};

  const TAU = Math.PI * 2;
  const wrapPi = a => { a = (a + Math.PI) % TAU; if (a < 0) a += TAU; return a - Math.PI; };
  const wrapDeg = a => { a = (a + 180) % 360; if (a < 0) a += 360; return a - 180; };

  /* Una caja orientada: centro (x, z), anchura w a lo largo de su eje "derecha",
   * profundidad d a lo largo de su eje "adelante", giro rot (rumbo de su eje
   * adelante) y altura h. */
  function boxAxes(b) {
    const s = Math.sin(b.rot || 0), c = Math.cos(b.rot || 0);
    return { right: [c, -s], forward: [s, c] };
  }

  /* Distancia a lo largo del rayo hasta la caja (Infinity si no la toca). Si el
   * origen esta dentro, 0. Devuelve tambien la normal de la cara de entrada. */
  function rayBox(ox, oz, dx, dz, b) {
    const a = boxAxes(b);
    const rx = ox - b.x, rz = oz - b.z;
    const lo = rx * a.right[0] + rz * a.right[1], lz = rx * a.forward[0] + rz * a.forward[1];
    const ld = dx * a.right[0] + dz * a.right[1], lf = dx * a.forward[0] + dz * a.forward[1];
    let tmin = -Infinity, tmax = Infinity, axis = -1, sign = 1;
    const slabs = [[lo, ld, b.w / 2], [lz, lf, b.d / 2]];
    for (let i = 0; i < 2; i++) {
      const [p, d, h] = slabs[i];
      if (Math.abs(d) < 1e-12) { if (p < -h || p > h) return null; continue; }
      let t0 = (-h - p) / d, t1 = (h - p) / d, s = -1;
      if (t0 > t1) { const t = t0; t0 = t1; t1 = t; s = 1; }
      if (t0 > tmin) { tmin = t0; axis = i; sign = s; }
      if (t1 < tmax) tmax = t1;
      if (tmin > tmax) return null;
    }
    if (tmax < 0) return null;
    if (tmin < 0) return { t: 0, nx: 0, nz: 0 };
    const n = axis === 0 ? a.right : a.forward;
    return { t: tmin, nx: n[0] * sign, nz: n[1] * sign };
  }

  /* Dos cajas orientadas, test del eje de separacion. */
  function overlapBoxes(a, b) {
    const A = boxAxes(a), B = boxAxes(b);
    const dx = b.x - a.x, dz = b.z - a.z;
    for (const ax of [A.right, A.forward, B.right, B.forward]) {
      const sep = Math.abs(dx * ax[0] + dz * ax[1]);
      const ra = a.w / 2 * Math.abs(A.right[0] * ax[0] + A.right[1] * ax[1]) +
                 a.d / 2 * Math.abs(A.forward[0] * ax[0] + A.forward[1] * ax[1]);
      const rb = b.w / 2 * Math.abs(B.right[0] * ax[0] + B.right[1] * ax[1]) +
                 b.d / 2 * Math.abs(B.forward[0] * ax[0] + B.forward[1] * ax[1]);
      if (sep > ra + rb) return false;
    }
    return true;
  }

  /* Vector de traslacion minimo para sacar `b` de `a` (para empujar un pilar). */
  function separation(a, b) {
    const A = boxAxes(a), B = boxAxes(b);
    const dx = b.x - a.x, dz = b.z - a.z;
    let best = Infinity, out = null;
    for (const ax of [A.right, A.forward, B.right, B.forward]) {
      const proj = dx * ax[0] + dz * ax[1];
      const ra = a.w / 2 * Math.abs(A.right[0] * ax[0] + A.right[1] * ax[1]) +
                 a.d / 2 * Math.abs(A.forward[0] * ax[0] + A.forward[1] * ax[1]);
      const rb = b.w / 2 * Math.abs(B.right[0] * ax[0] + B.right[1] * ax[1]) +
                 b.d / 2 * Math.abs(B.forward[0] * ax[0] + B.forward[1] * ax[1]);
      const overlap = ra + rb - Math.abs(proj);
      if (overlap < 0) return null;
      if (overlap < best) { best = overlap; const s = proj >= 0 ? 1 : -1; out = [ax[0] * s * overlap, ax[1] * s * overlap]; }
    }
    return out;
  }

  /* Cuatro esquinas de una caja, en orden. */
  function corners(b) {
    const a = boxAxes(b), hw = b.w / 2, hd = b.d / 2, out = [];
    for (const [sx, sz] of [[-1, -1], [1, -1], [1, 1], [-1, 1]])
      out.push([b.x + a.right[0] * hw * sx + a.forward[0] * hd * sz, b.z + a.right[1] * hw * sx + a.forward[1] * hd * sz]);
    return out;
  }

  WRO.geometry = { TAU, wrapPi, wrapDeg, boxAxes, rayBox, overlapBoxes, separation, corners };
  if (typeof module !== 'undefined') module.exports = WRO;
})(typeof window !== 'undefined' ? window : globalThis);
