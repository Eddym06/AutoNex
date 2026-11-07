/* Aleatoriedad reproducible: el mismo `seed` da la misma pista, el mismo ruido
 * del LiDAR y de la camara, y por tanto el mismo recorrido. Un caso que falla se
 * comparte con su semilla. */
(function (root) {
  'use strict';
  const WRO = root.WRO = root.WRO || {};

  function rng(seed) {
    let s = (seed >>> 0) || 1;
    const next = () => {
      s = (s + 0x6D2B79F5) >>> 0;
      let t = s;
      t = Math.imul(t ^ (t >>> 15), t | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
    let spare = null;
    next.gauss = () => {
      if (spare !== null) { const v = spare; spare = null; return v; }
      let u = 0, v = 0;
      while (u === 0) u = next();
      v = next();
      const m = Math.sqrt(-2 * Math.log(u));
      spare = m * Math.sin(2 * Math.PI * v);
      return m * Math.cos(2 * Math.PI * v);
    };
    next.range = (a, b) => a + (b - a) * next();
    next.int = (a, b) => Math.floor(next.range(a, b + 1));
    next.pick = list => list[Math.floor(next() * list.length)];
    return next;
  }

  WRO.rng = rng;
  if (typeof module !== 'undefined') module.exports = WRO;
})(typeof window !== 'undefined' ? window : globalThis);
