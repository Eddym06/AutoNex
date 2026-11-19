#ifndef HW_MAPSTORE_H
#define HW_MAPSTORE_H

#include <stdbool.h>
#include <stdint.h>
#include "trace_map.h"

/* Guardado del trace_map en la particion "tracemap" de la flash (2 ranuras de 64 KB que se alternan).
 *
 * El mapa vivo esta en RAM (el controlador lo actualiza en cada barrido, sin tocar la flash). La flash solo guarda una
 * copia al TERMINAR la mision (DONE/FAULT, motores parados): borrar y escribir flash detiene la ejecucion desde flash
 * (el bucle de control de 1 ms), asi que jamas se hace en marcha. La copia dura entre arranques; no se vuelca sola al
 * mapa vivo, porque los letreros cambian de sitio entre rondas. */

/* Busca la particion, lee cual fue la ultima copia y lanza una tarea de baja prioridad que guarda `map` cada vez que
 * `finished()` pasa a verdadero (una vez por mision). `map` no debe cambiar mientras finished() sea verdadero. */
void hw_mapstore_init(const trace_map_t *map, bool (*finished)(void));
bool hw_mapstore_save(const trace_map_t *m);                  /* escribe la copia (lenta: ~100-300 ms); solo con el robot parado */
bool hw_mapstore_load(trace_map_t *m);                        /* copia la ultima guardada al mapa m; false si no hay */
uint32_t hw_mapstore_sequence(void);                          /* numero de la ultima copia (0 = ninguna) */

#endif
