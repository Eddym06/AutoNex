# trace_map: precisión en la 1.ª vuelta y en la 3.ª, y comparación con el parqueo

Solo simulador (perfil `largo26`, 3 letreros, cajón de 39 cm). **Nada medido con el LiDAR real.**
Reproducir: `node simulator/tools/measure_trace_laps.js 10 3 simulator/mediciones/trace_map_laps.json` y
`node simulator/tools/measure_parking_sources.js 21 32 simulator/mediciones/parking_sources.json`.

## 1. Vuelta 1 (construcción) frente a vuelta 3 (refinada)

20 rondas (semillas 1-10, ambos sentidos). La instantánea se toma justo al cerrar la vuelta (`laps_mapped` = 1 y = 3).

**Sesgo de supervivencia que hay que tener presente:** 7 de las 20 rondas terminan antes por `wrong-side` (el robot pasa un letrero por el lado
que no toca; es un fallo del paso de letreros, no del mapa). 6 no llegan a cerrar la vuelta 1 y 7 no llegan a la 3. La comparación justa es
**pareada**: las 13 rondas que llegaron a las dos instantáneas.

| Medida (13 rondas pareadas) | Vuelta 1 (~39 s) | Vuelta 3 (~112 s) |
| --- | --- | --- |
| Letreros encontrados | 31 de 39 (79 %) | 34 de 39 (87 %) |
| Color correcto | 30 de 31 | 34 de 34 |
| Falsos objetos | 0 | 0 |
| Error de posición del letrero (cm), media / RMS / p95 / máx | 0,66 / 0,73 / 1,25 / 1,25 | 0,72 / 0,83 / 1,47 / 1,60 |
| Plaza confirmada | 13 de 13 | 13 de 13 |
| Error de posición de la plaza (cm), media / p50 / p95 / máx | 1,15 / 1,12 / 1,74 / 1,74 | 1,11 / 0,84 / 1,96 / 1,96 |
| Distancia de las celdas ocupadas al sólido verdadero (cm), media / RMS / p95 / máx | 1,64 / 2,33 / 4,25 / 13,75 | 2,00 / 2,82 / 6,75 / 11,75 |
| Cobertura de las paredes verdaderas (±3,75 cm) | 100 % | 100 % |

Lectura sin adornos:

- **La vuelta 3 mejora la completitud y el color** (más letreros vistos, todos con color), **no la precisión geométrica**.
- El error de posición de los letreros no baja: sube un poco (0,66 → 0,72 cm de media) y el p95 también (1,25 → 1,47). Con 31-34 muestras
  esa diferencia no es significativa, pero tampoco hay mejora.
- La plaza queda igual: la media casi no cambia (1,15 → 1,11 cm), el p95 empeora un poco (1,74 → 1,96 cm).
- **El mapa se ensancha con las vueltas**: las celdas ocupadas se alejan más del sólido verdadero (media 1,64 → 2,00 cm, p95 4,25 → 6,75 cm),
  porque cada pasada suma retornos con su propio error de pose (≤ 3,5 cm) y nada los promedia: se acumulan como una mancha. Es un defecto
  real del refinado actual, no un artefacto de la medida.
- Los objetos no se ven peor porque se extraen de celdas con ≥ 2 retornos y se calcula su centro, que absorbe ese ensanchamiento.

Los números de la vuelta 1 con las 14 rondas que la alcanzan (no pareadas) son casi idénticos (33 de 42 letreros, error medio 0,69 cm,
plaza 14 de 14 con 1,10 cm).

## 2. ¿Debe el parqueo usar la plaza del mapa?

Se compararon, en las mismas rondas y los mismos instantes, las dos fuentes de la pose del robot en el cajón frente a la verdad del simulador:
el LiDAR local (`park_localize`, lo que usa hoy el parqueo) y la plaza de `trace_map` combinada con la pose del campo (`arena_localizer`).
Modo ronda sin letreros, semillas 21-32 en los dos sentidos (24 rondas, semillas que no se usaron para ajustar nada).

Error de la pose del robot en el cajón (módulo de u y w, cm), **mismas 1752 muestras en `PARKING`**:

| Fuente | Media | RMS | p50 | p95 | Máx |
| --- | --- | --- | --- | --- | --- |
| LiDAR local (`park_localize`, el actual) | 0,66 | 0,74 | 0,62 | 1,24 | 2,57 |
| Plaza de `trace_map` + pose del campo | 0,64 | 0,74 | 0,59 | 1,33 | 1,86 |

- Sesgo con signo (media de la diferencia): LiDAR local −0,02 cm, mapa +0,12 cm. Ninguno está descentrado de forma apreciable.
- Durante `PARK_SEARCH` la plaza del mapa da 0,60 cm de media (p95 1,28) con 367 muestras: ya sirve antes de que el parqueo adquiera el cajón.
- La plaza estaba **confirmada al empezar la búsqueda en 24 de 24 rondas**.
- Parqueo completo: 22 de 24 (cw 21 acaba en `limiter`, cw 26 en `stalled`). En esas dos rondas el error de pose de las dos fuentes es el
  mismo que en las demás (p95 ≈ 1,2-1,3 cm): **el fallo no viene de la localización**, viene de la planificación y la ejecución de las
  maniobras (contacto con el tope y avance corto, ya anotado en `PENDIENTES_PARQUEO.md`).

**Conclusión:** las dos fuentes son equivalentes en precisión (diferencia de 0,02 cm en la media, dentro del ruido; el mapa es algo peor en el
p95 y algo mejor en el máximo). Cambiar la localización del parqueo al mapa **no ganaría nada medible** y metería en la cadena de control el error
de la pose global. Por eso **no se cambia el parqueo**. El mapa queda como segunda fuente independiente: sirve para detectar que el LiDAR local
se ha desviado (comparar las dos y avisar si discrepan más de ~3 cm) y como respaldo si el LiDAR pierde el cajón. Ese uso hay que decidirlo con
datos del chasis real, no con simulador: aquí no hay ninguna ronda en la que el LiDAR local falle y el mapa acierte.

## 3. Cosas que esta medición no cubre

- No hay LiDAR real: el ruido, los reflejos y la pose real del chasis pueden cambiar todo.
- La comparación de pose usa rondas sin letreros para que todas lleguen a aparcar; con letreros hay rondas que acaban antes por otras causas.
- Las dos fuentes salen del mismo LiDAR, así que sus errores no son independientes del todo.

## 4. Corrección: promedio de las pasadas (después de la sección 1)

**Por qué la 3.ª vuelta no bajaba el error** (diagnose_pose_repeat.js, 16 rondas, 252 casillas de 25 cm visitadas en la vuelta 1 y en la 3):
el error de la pose del campo (arena_localizer) tiene RMS 1,62 cm en la vuelta 1 y 1,61 cm en la 3, y la **correlación entre las dos es 0,983**: el error
es casi idéntico en el mismo sitio en cada vuelta (diferencia RMS 0,3 cm). Es sistemático, no ruido: promediar vueltas no lo quita, solo quita esos 0,3 cm.
Lo que sí crecía era la **mancha**: el contador de 4 bits se saturaba en la primera pasada y toda celda tocada valía igual.

**Qué se cambió:** contador de retornos de 8 bits por celda (hits) y una superficie = cresta del histograma (≥ 30 % del peso máximo en 5 × 5).
Los objetos se buscan en la superficie. Memoria del mapa: 59 KB → 88 KB.

Mismas 13 rondas pareadas, antes → después:

| Medida | Antes v1 → v3 | Después v1 → v3 |
| --- | --- | --- |
| Celdas al sólido real, media (cm) | 1,64 → 2,00 | 0,89 → 0,94 |
| Celdas al sólido real, p95 (cm) | 4,25 → 6,75 | 3,75 → 3,75 |
| Celdas de pared/objeto en el mapa | 30.759 → 34.458 | 17.824 → 17.808 |
| Letreros encontrados | 31/39 → 34/39 | 39/39 → 39/39 |
| Color correcto | 30/31 → 34/34 | 38/39 → 39/39 |
| Error de posición del letrero, media (cm) | 0,66 → 0,72 | 0,93 → 0,87 |
| Error de posición de la plaza, media (cm) | 1,15 → 1,11 | 0,67 → 0,88 |
| Falsos objetos | 0 → 0 | 0 → 0 |

Lectura sin adornos:
- **La mancha ya no crece con las vueltas** (p95 3,75 cm en las dos) y hay la mitad de celdas: eso es lo que se pidió.
- **Se encuentran los 39 letreros** en vez de 31-34, porque la superficie separa los letreros de la mancha de la pared.
- **El error de posición del letrero no mejora, empeora algo en la media** (0,66 → 0,93 en la vuelta 1): ahora entran los 8 letreros difíciles que antes
  no se encontraban, y el p95 sube a 2,4-2,8 cm por ellos. En los mismos letreros que antes, 0,7-0,9 cm.
- **La vuelta 3 sigue sin ser más precisa que la 1** (plaza 0,67 → 0,88 cm, sin mejora): el error dominante es el sistemático de la pose,
  y esto no lo arregla el mapa. Reducirlo exige mejorar rena_localizer (el modelo del campo que usa para ajustar).
- Ponderar el centro del letrero por retornos lo empeoró (1,2 cm frente a 0,7): el LiDAR solo ve una cara y esa cara concentra los retornos.
  Por eso el centro se corrige con la huella completa (diagnose_sign_center.js).
- Tiempo de las mediciones: 31 s en paralelo (22 núcleos) frente a ~10 min en serie, con una línea de log por ronda (simulator/tools/par.js).