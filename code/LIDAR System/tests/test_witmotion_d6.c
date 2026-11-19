/*
  Prueba en PC del parser del WitMotion D6 / COIN-D6 (mismo algoritmo que
  sketch_oct14a.ino). Usa un paquete real capturado en COM9 a 230400 baudios.

  Compilar y ejecutar (MSYS2 / MinGW):
    gcc -Wall -o test_witmotion_d6 test_witmotion_d6.c && ./test_witmotion_d6
*/
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define PREAMBLE 10
#define MAX_RANGE_MM 12000

static int checksum_ok(const uint8_t* p, uint8_t lsn) {
  uint16_t cs = 0x55AA;
  cs ^= (uint16_t)(p[2] | (p[3] << 8));
  cs ^= (uint16_t)(p[4] | (p[5] << 8));
  cs ^= (uint16_t)(p[6] | (p[7] << 8));
  for (uint8_t i = 0; i < lsn; i++) {
    uint8_t b = PREAMBLE + 3 * i;
    cs ^= p[b];
    cs ^= (uint16_t)((p[b + 2] << 8) | p[b + 1]);
  }
  return cs == (uint16_t)(p[8] | (p[9] << 8));
}

static int decode(const uint8_t* p, uint8_t lsn, int32_t* angles, uint16_t* dists, int max) {
  int32_t sa = ((int32_t)((p[4] | (p[5] << 8)) >> 1) * 100) / 64;
  int32_t ea = ((int32_t)((p[6] | (p[7] << 8)) >> 1) * 100) / 64;
  if (ea < sa) ea += 36000;
  int32_t step = (lsn > 1) ? (ea - sa) / (lsn - 1) : 0;
  int n = 0;
  for (uint8_t i = 0; i < lsn && n < max; i++) {
    uint8_t b = PREAMBLE + 3 * i;
    uint16_t dist = (uint16_t)p[b + 2] * 64 + (p[b + 1] >> 2);
    if (dist == 0 || dist > MAX_RANGE_MM) continue;
    angles[n] = (sa + step * i) % 36000;
    dists[n] = dist;
    n++;
  }
  return n;
}

int main(void) {
  /* Paquete real de 85 bytes (LSN=25) capturado en COM9 */
  const uint8_t pkt[85] = {
    0xAA,0x55,0x00,0x19,0xF9,0x2C,0xC9,0x37,0xDB,0x48,0xD8,0xC4,0x01,0x3C,0x8D,0x01,
    0xCC,0x00,0x00,0x70,0x00,0x00,0x9C,0x00,0x00,0xCC,0x00,0x00,0xA4,0x00,0x00,0xCC,
    0x00,0x00,0xE8,0xD8,0x01,0xC8,0x00,0x00,0x5C,0x00,0x00,0x28,0x00,0x00,0x2C,0x00,
    0x00,0x2C,0x00,0x00,0xE0,0x00,0x00,0x78,0x3E,0x0E,0x1C,0x03,0x0F,0x3C,0xC3,0x0F,
    0x3C,0x87,0x10,0x30,0x57,0x11,0xDC,0x2A,0x12,0x08,0x02,0x00,0x28,0x22,0x03,0x60,
    0x16,0x03,0xE0,0x1E,0x03
  };
  int fails = 0;

  int ok = checksum_ok(pkt, pkt[3]);
  printf("Checksum: %s\n", ok ? "OK" : "FALLA");
  if (!ok) fails++;

  /* Un byte alterado debe fallar el checksum */
  uint8_t bad[85];
  memcpy(bad, pkt, sizeof(pkt));
  bad[20] ^= 0x01;
  if (checksum_ok(bad, bad[3])) { printf("Checksum no detecta corrupción: FALLA\n"); fails++; }
  else printf("Corrupción detectada: OK\n");

  int32_t ang[25];
  uint16_t dist[25];
  int n = decode(pkt, pkt[3], ang, dist, 25);
  printf("Puntos validos: %d (de 25)\n", n);
  for (int i = 0; i < 5 && i < n; i++)
    printf("  %2d: angulo=%.2f deg  dist=%u mm\n", i, ang[i] / 100.0, dist[i]);

  /* Primer punto válido de la captura: dist 113 mm, FSA ~89.94 deg */
  if (n < 1 || dist[0] != 113) { printf("Distancia esperada 113 mm: FALLA\n"); fails++; }
  if (n < 1 || ang[0] < 8990 || ang[0] > 8995) { printf("Angulo inicial esperado ~89.94: FALLA\n"); fails++; }

  printf(fails ? "RESULTADO: FALLA (%d)\n" : "RESULTADO: OK\n", fails);
  return fails ? 1 : 0;
}
