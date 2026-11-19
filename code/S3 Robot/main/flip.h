#ifndef FLIP_H
#define FLIP_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* Voltea un cuadro RGB565 (2 bytes por pixel) en el sitio. flip_x: espejo izquierda-derecha;
 * flip_y: arriba-abajo; los dos a la vez = giro de 180 grados. Los bytes de cada pixel no se cruzan. */
static inline void flip_rgb565(uint8_t *buf, int w, int h, int flip_x, int flip_y)
{
    if (flip_x)
        for (int y = 0; y < h; ++y) {
            uint8_t *row = buf + (size_t)y * w * 2;
            for (int i = 0, j = w - 1; i < j; ++i, --j) {
                uint8_t t[2];
                memcpy(t, row + 2 * i, 2);
                memcpy(row + 2 * i, row + 2 * j, 2);
                memcpy(row + 2 * j, t, 2);
            }
        }
    if (flip_y)
        for (int y = 0; y < h / 2; ++y) {
            uint8_t *a = buf + (size_t)y * w * 2, *b = buf + (size_t)(h - 1 - y) * w * 2;
            for (int i = 0; i < w * 2; ++i) { const uint8_t t = a[i]; a[i] = b[i]; b[i] = t; }
        }
}

#endif
