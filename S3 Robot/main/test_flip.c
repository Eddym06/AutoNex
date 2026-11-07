#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "flip.h"

static void run(int fx, int fy, const uint8_t *expect)
{
    uint8_t f[12] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};   /* 3x2 pixeles: [p0 p1 p2 / p3 p4 p5] */
    flip_rgb565(f, 3, 2, fx, fy);
    assert(memcmp(f, expect, 12) == 0);
}

int main(void)
{
    run(0, 0, (const uint8_t[]){1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12});
    run(1, 0, (const uint8_t[]){5, 6, 3, 4, 1, 2, 11, 12, 9, 10, 7, 8});      /* espejo: cada fila al reves */
    run(0, 1, (const uint8_t[]){7, 8, 9, 10, 11, 12, 1, 2, 3, 4, 5, 6});      /* arriba-abajo: filas intercambiadas */
    run(1, 1, (const uint8_t[]){11, 12, 9, 10, 7, 8, 5, 6, 3, 4, 1, 2});      /* ambos = giro de 180 */
    puts("flip OK");
    return 0;
}
