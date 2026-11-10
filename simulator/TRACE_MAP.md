# trace_map: mapa persistente de la pista

Código: `LIDAR System/main/trace_map.c/.h`. Pruebas: `LIDAR System/tests/test_trace_map.c` (campo sintético),
`simulator/tests/test_trace_map.js` (misión simulada) y `simulator/tools/measure_trace_objects.js` (error contra la verdad).

> Medido en simulador y en un campo sintético. **No está probado con el LiDAR real.** El mapa se apoya en la pose del localizador del
> campo (`arena_localizer`, error de ajuste ≤ 3,5 cm) y no se actualiza sin pose válida.

## Qué guarda

Tres capas, en el marco del campo (centro 0, 400 × 400 cm, celdas de 2,5 cm):

1. **Rejilla**: por celda, un byte (retorno LiDAR, rojo/verde confirmados, magenta candidato, repetición hasta 15) y otro con los rayos que la
   **atravesaron**. Da espacio ocupado, **espacio libre conocido** y desconocido. Una celda ocupada que se atraviesa más veces de las que se vio
   se borra (algo se movió, un fantasma): `cleared_cells`.
2. **Objetos** (`trace_map_objects`): las celdas estables se agrupan (componentes conexas) y cada grupo es un objeto con `id` estable, `kind`
   (pared, pilar, barra), `color` (rojo, verde, magenta), posición `x,z`, largo, ancho, eje, celdas y confianza. Las paredes del campo
   (cuadrado exterior de 300 cm y bloque central de 100 cm) se restan con un modelo del campo, de modo que lo que queda son **letreros, topes y
   cualquier cosa inesperada**.
3. **Plaza de parqueo** (`trace_map_spot`): el par de barras perpendiculares a la pared exterior separadas lo que mide el cajón (largo + 2 cm de tope)
   se reconoce solo con geometría. `CONFIRMED` con centro, largo, ancho, eje y posición de los dos topes; `HYPOTHESIS` con un tope magenta;
   el magenta de ambos topes suma confianza. Una plaza confirmada se conserva 300 barridos aunque los topes queden fuera de vista.

## Vueltas: se construye en la primera y se refina en cada una

`wro_controller` llama a `trace_map_lap_complete` cada 4 esquinas. La **1.ª vuelta construye** el mapa (`laps_mapped` pasa de 0 a 1,
`trace_map_is_built`). En cada vuelta siguiente el mismo mapa se **refina**: cada pasada suma retornos (sube la confianza), borra lo que se
movió (`cleared_cells`) y cada objeto que sigue en su sitio suma una vuelta en `laps_seen`. Al terminar la misión (DONE/FAULT) el controlador
congela el mapa para poder copiarlo.

## Promedio de las pasadas (superficie)

Cada celda acumula un contador de retornos de 8 bits (hits, 1 por barrido); el de 4 bits de cells se satura en la primera pasada y no sirve para promediar. En 	race_map_refresh se calcula la **superficie**: una celda ocupada que tiene al menos el 30 % del peso máximo de su entorno de 5 × 5. Es la cresta del histograma de retornos: las pasadas con distinto error de pose se promedian en vez de dejar una mancha. Los objetos se buscan sobre la superficie; el centro de un letrero se corrige con la huella completa porque la superficie solo conserva la cara que ve el LiDAR (ponderar por hits empeoró el centro, de 0,7 a 1,2 cm).

**Límite que no se arregla promediando:** el error de la pose del campo se repite casi igual en cada vuelta (correlación 0,983 entre la vuelta 1 y la 3, RMS 1,6 cm; solo 0,3 cm cambian de una vuelta a otra). Promediar vueltas quita la parte aleatoria, no esa. Ver MEDICION_TRACE_VUELTAS.md.

## Dónde vive en el P4 (ESP32-P4 Waveshare)

- **Mapa vivo: RAM**, dentro del controlador (88 KB). Se actualiza en cada barrido sin tocar la flash; leerlo cuesta lo mismo que cualquier variable.
- **Copia persistente: partición `tracemap` de la flash** (`p4_firmware/partitions.csv`, 128 KB = 2 ranuras de 64 KB que se alternan, así un corte
  de luz escribiendo deja la anterior intacta). Código: `p4_firmware/main/hw_mapstore.c`.
- **Cuándo se escribe:** solo al terminar la misión. Borrar/escribir flash detiene la ejecución desde flash (el bucle de 1 ms), por eso nunca se
  hace en marcha. La copia en PSRAM (`s_buf`) no ocupa RAM interna, y una tarea de prioridad 1 hace el trabajo (~100-300 ms con ~9 KB).
- **Arranque:** se registra cuál fue la última copia (`hw_mapstore_load` la devuelve). **No se vuelca sola al mapa vivo**: los letreros cambian de
  sitio entre rondas y un mapa viejo sería un obstáculo falso.
- Tras actualizar hay que regenerar `sdkconfig` (borrar el existente o `idf.py reconfigure`) y flashear **la tabla de particiones** (`idf.py flash` lo hace).

## API

```c
trace_map_update(&map, points, n, &local_map, &arena_pose, lidar_forward_cm);  // cada barrido (0,13 ms en PC)
trace_map_refresh(&map);                                                       // cada ~10 barridos (0,05 ms en PC)
const trace_object_t *o = trace_map_objects(&map, &count);
const trace_spot_t   *s = trace_map_spot(&map);
trace_map_is_free(&map, x, z);  trace_map_clearance_cm(&map, x, z, max_cm);  trace_map_cell_at(&map, x, z);
size_t n = trace_map_save(&map, buf, cap);  trace_map_load(&map, buf, n);      // CRC16, ~9 KB
```

El controlador lo actualiza solo (`wro_controller_scan`), con el espaciado entre topes sacado de `park_lot_length_cm` (o 1,5 × largo del chasis) más 2 cm. Desde
el simulador: `core.traceObjects()`, `core.traceSpot()`, `core.traceMap()`.

Configuración opcional en `trace_map_t` (0 = valor de la sede): `field_outer_half_cm` (150), `field_inner_half_cm` (50), `wall_band_cm` (6),
`spot_spacing_cm` (41).

## Resultados

Campo sintético (2 vueltas con ruido de ±1 cm en la pose): letreros a 0,5 y 0,7 cm, colores correctos, plaza confirmada en (0,0; −140,0) con error 0,
largo 38 frente a 39, borrado del letrero retirado, guardado y carga idénticos, CRC detecta un byte corrupto.

Misión simulada con el controlador real (5 semillas por sentido, 3 letreros, hasta 140 s), contra la verdad del simulador:

| Medida | Resultado |
| --- | --- |
| Letreros encontrados como pilar | 26 de 30 (los 4 que faltan son de rondas que acabaron en <12 s) |
| Error de posición de los letreros | media 0,3-1,0 cm, máximo 2,2 cm |
| Color correcto | 21 de 26; el resto sin color porque la fusión no los había confirmado |
| Falsos objetos | 1 en 30 |
| Plaza confirmada | 8 de 10 rondas; error medio 0,52 cm, máximo 1,02 cm |

Coste: update 0,13 ms y refresh 0,05 ms por llamada en un PC (la P4 será unas 5-10 veces más lenta: ~1 ms por barrido a 10 Hz); RAM 88.228 bytes.

## Qué no hace (todavía)

- No lo consulta el planificador ni el parqueo: es la fuente de datos para ello (plaza, obstáculos, espacio libre).
- No distingue letreros pegados a otro obstáculo (se fusionan) ni tiene modelo de objetos móviles.
- Asume el campo de la sede (paredes paralelas a los ejes). Otra sede exige `trace_map_t.field_*`.
- `trace_parking.c` (estimador antiguo con una detección de cámara) se conserva por compatibilidad; la plaza del mapa lo sustituye.
- La interfaz del simulador dibuja objetos (círculo con su color e id) y la plaza (rectángulo amarillo), y muestra las vueltas acumuladas.
- El guardado en flash (`hw_mapstore.c`) está escrito pero **no se ha compilado ni probado en el P4**: no hay ESP-IDF en este entorno.
