# Planificador y controlador — ESP32-P4

> **Actualización para 12-móvil:** la descripción diferencial, el parqueo y las
> cifras históricas que siguen no validan el chasis Ackermann. La adaptación
> está en curso; consulta [ACKERMANN.md](../ACKERMANN.md).

Código C portable (sin dependencias de hardware) que decide el movimiento: recibe la tabla de la cámara del S3, el barrido del **WitMotion D6** y el giroscopio, y devuelve velocidad y giro. El robot es **diferencial**: la orden lleva `speed_cm_s` y `yaw_rate_dps` (el «volante» del planificador es virtual: giro = velocidad × tan(volante) / batalla); un robot con servo usa `steer_deg`. El mapa local se rehace en cada barrido, centrado en el robot. **Los encoders ópticos de las ruedas alimentan una odometría y un localizador del campo** (ver «Odometría y mapa del campo»); sin encoders el firmware vuelve al método antiguo (velocidad ordenada × tiempo). Los apartados de abajo sobre «sin odometría» describen ese modo de respaldo.

Este mismo código corre en el simulador (compilado a WebAssembly), así que lo que se prueba allí es lo que se carga en el P4.

## Flujo

1. `camera_table_receiver` resincroniza tramas `A5 5A` del S3, valida versión y CRC16 y convierte centro de imagen a bearing con el campo de visión configurado. La hora de recepción es la del reloj del P4.
2. **Tiempo real, sin datos viejos.** El barrido del D6 dura 100 ms y el último punto es el más reciente; la orden, además, tarda en hacer efecto (cálculo + servo). El controlador (`motion_comp`, activo por defecto) mueve cada retorno al instante en que la orden actuará: avanza el punto por velocidad ordenada × edad y lo gira por la velocidad angular del giroscopio × edad, con `latency_s` (0,10 s) de margen de actuación. Sin esto el planificador decidía sobre una escena de hace 100–200 ms (2–4 cm a 20 cm/s, el margen de un paso estrecho). Corrige también el avance durante el barrido, no solo el giro.
3. `imu_yaw` integra el giroscopio (sesgo medido con el robot quieto), y el controlador **reancla el rumbo contra las paredes** para que la deriva no se acumule. El magnetómetro es opcional y está desactivado.
4. `local_obstacle_map` (planificador):
   - proyecta los rayos en una rejilla de 160 × 160 celdas de **2,5 cm**, corrigiendo el movimiento durante el barrido (deskew) y puenteando huecos cortos de retornos perdidos;
   - asocia objetos de cámara con grupos **compactos** del LiDAR (una pared o su esquina no cuenta como letrero) (la distancia buena la da el LiDAR, el color la cámara); un objeto de cámara sin retorno LiDAR se trata con cautela y envejece (`stale_cm`);
   - **fuerza el lado de paso por color** con «cortinas» virtuales (celdas ocupadas detrás de cada bloque rojo/verde según `pass_side_*`);
   - elige el corredor con programación dinámica en el sistema de coordenadas del carril, puntuando rutas candidatas (7 primeras acciones × 3 arcos de 20 cm) con holgura, desvío lateral, rumbo y suavidad, y sigue la ruta con una mira adaptativa; la salida de volante es continua;
   - si no hay ruta hacia delante, ejecuta un escape con reversa (mínimo 15 cm, con reintento periódico) y vuelve a planificar en cada barrido.
5. `wro_controller` (misión) sin odometría:
   - **Esquinas (robustas a las esquivas)**: el muro frontal solo cuenta como esquina con el morro a menos de 30° del carril (`corner_max_heading_deg`), el respaldo del giroscopio exige haber recorrido 110 cm desde la última (`corner_gyro_min_travel_cm`) y una esquina que no termina en 160 cm se deshace (`corner_abort_cm`). Antes, una esquiva abierta hacia la pared interior se tomaba por esquina, el rumbo del recorrido saltaba 90° y el resto de la ronda salía mal.
   - **Giro en el sitio para salir de atascos** (`pivot_enabled`, solo robots diferenciales): sin ruta hacia delante durante 2 s y con el morro girado de la pista, gira en el sitio hacia ella si el chasis cabe girando.
   - **Esquinas** (detalle): se detectan por el plano de la pared frontal y un lado que se abre; el sentido de la ronda se aprende en la primera esquina. Si el robot llega torcido por una esquiva y el muro no se ve como plano, el giroscopio la delata (`corner_gyro_deg`, 60°: no se gira tanto hacia el lado de la ronda en una recta).
   - **Rumbo del recorrido**: se reancla contra las paredes solo con el morro a menos de `wall_align_heading_deg` (15°) del carril; torcido, las paredes se ven en diagonal y arrastraban el rumbo (era la causa de un atasco real: ver `simulator/tests/plano_atasco.json`).
   - **Letreros**: se siguen por dead-reckoning (giroscopio + velocidad ordenada × tiempo), anclados a los grupos compactos del LiDAR.
   - **Vuelta final**: si el último letrero de la segunda vuelta es rojo (`reverse_on_color`), da un giro en U al entrar en la recta de salida (con K-turn si no cabe).
   - **Final**: cuenta tres vueltas con la esquina, usa la pared interior como referencia y recorre `finish_extra_cm` más para parar dentro del tramo de salida.
   - **Seguridad**: un perro guardián para el robot si se pierde el LiDAR o el giroscopio.
6. `p4_runtime` es el bucle del P4: lee giroscopio, cámara y LiDAR por una tabla de funciones (`p4_hal_t`), alimenta al controlador, manda `drive(volante, velocidad)` y **para** si dejan de llegar barridos. Se prueba con hardware simulado.

## Parqueo en paralelo y salida del cajón

Reglamento 2026: el cajón mide 20 cm de ancho por **1,5 × el largo del robot** (25,5 cm con 17 cm), entre dos topes magenta de 20 × 2 × 10 cm junto a la pared exterior de la sección de arranque. Con este chasis la holgura máxima es de ~1,5 cm: es muy justo.

- **No «ve la salida»: conoce el cajón y se localiza en él.** `park_localize.c` casa el barrido del LiDAR con un modelo (la pared y los dos topes) y obtiene la pose (u a lo largo de la pared, w distancia a la pared, rumbo). Con eso sabe en cada barrido dónde está respecto al cajón, sin odometría.
- **Planificador de maniobras** (`park_planner.c`): un campo de costes sobre el espacio de poses (retícula de 1 cm × 32 rumbos). Cada movimiento es avanzar, retroceder o girar en el sitio; un Dijkstra inverso desde la meta da el coste de cada pose libre (holgura 1,5 cm; las poses con menos de 3,5 cm cuestan más, para pasar más lejos cuando se pueda). En cada barrido elige el movimiento que más lo baja, así que se recupera de cualquier desvío. Memoria: ~1 MB (en el P4, PSRAM).
- **Salir (`start_in_lot 1`)**: arranca centrado en el cajón, pegado al lado del carril (`start_lot_w_cm`, 12,5 cm de la pared). La pared que ve a un lado le dice el sentido de la ronda. El camino que calcula es: girar ~90° en el sitio, salir recto hacia el carril, volver a girar hasta quedar paralelo y seguir con el planificador normal.
- **Aparcar (`parking_enabled 1`)**: tras la última vuelta pega la ruta a la pared exterior (`park_lane_w_cm`), reduce la velocidad y busca el tope trasero con el LiDAR (exige ≥ 8 puntos sobre él para no confundirlo con otra cosa). Al localizar el cajón entra por el camino del campo de costes y para cuando está dentro y paralelo (±4°). Si no encuentra el cajón en `park_search_max_cm`, se detiene como sin parqueo.
- **Demostración** (`park_demo_dir -1|+1`, con `parking_enabled 1`): arranca directamente buscando el cajón, con el sentido dado y las vueltas por hechas. Sirve para ver el parqueo sin esperar tres vueltas.
- No se activa solo: el robot no puede saber si la pista tiene cajón, así que `parking_enabled` y `start_in_lot` son parámetros (el simulador los pone según la pista).

## Adaptabilidad

Todo se configura con texto `clave valor` (`wro_controller_config_set/parse_text`; las del planificador llevan el prefijo `planner.`):

```text
planner.pass_side_red 1       # +1 pasa por la derecha, -1 izquierda, 0 sin regla
planner.pass_side_green -1
planner.enforce_color_side 1
planner.speed_cm_s 20
lane_width_cm 100
laps 3
reverse_on_color 1            # color del último letrero que invierte la vuelta final (1 = rojo)
continue_on_color 2           # color que la mantiene (2 = verde)
turnaround_enabled 1
finish_extra_cm 35
motion_comp 1                 # compensa latencia y movimiento durante el barrido
latency_s 0.10
corner_gyro_deg 60            # 0 desactiva el respaldo por giroscopio
stuck_relax_s 0               # >0: tras tanto tiempo sin avanzar, suspende el lado de paso un trecho (puede costar una infracción)
```

Los valores por defecto están en `local_planner_config_default` y `wro_controller_config_default`. La lista completa de claves está en la tabla de `wro_controller.c`. Si las reglas cambian (colores ↔ lados, número de vueltas, ancho del carril, vuelta invertida), basta con cambiar estos valores.

## Archivos

| Archivo | Función |
| --- | --- |
| `main/local_obstacle_map.c/.h` | Rejilla, fusión cámara/LiDAR, cortinas, corredor, escape |
| `orchestrator/wro_controller.c/.h` | Misión: fases, esquinas, sentido, letreros, vuelta final, perro guardián |
| `main/imu_yaw.c/.h` | Giroscopio con sesgo, reanclaje, magnetómetro opcional |
| `main/camera_table_receiver.c/.h` | Decodificador de tablas v2 del S3 |
| `main/park_planner.c/.h` | Campo de costes de poses para entrar y salir del cajón |
| `main/park_localize.c/.h` | Localización respecto al cajón con el LiDAR |
| `orchestrator/p4_runtime.c/.h` | Bucle del P4 y tabla de funciones de hardware |
| `../shared/` | Protocolo con CRC16 y motor de configuración |
| `tests/` | `test_local_obstacle_map`, `test_planner_fusion`, `test_camera_table_receiver`, `test_imu_yaw`, `test_wro_controller`, `test_park_planner`, `test_p4_runtime` |
| `tools/route_planner_prototype.py` | Prototipo antiguo de escritorio (rejilla de 5 cm). **Obsoleto**: no corresponde al planificador actual |

## Pruebas

```powershell
cd "Vision System"; .\tools\simulate.ps1 -Quick     # todas las pruebas C + simulador + secuencias
```

GCC compila con `-Wall -Wextra -Werror -Wpedantic`.

## Lo que falta para el robot real

- **Driver del D6**: el formato del paquete no es público (el manual solo da 230400 baudios, ~10 Hz, ángulo/distancia/intensidad). Hay que implementar `poll_scan` y entregar los retornos dentro de la zona ciega como punto válido a distancia mínima.
- **Driver de giroscopio** (`poll_gyro`, grados/s horario positivo), **motor y servo** (`drive`), **botón de arranque** y UART del S3 (`read_camera`; TX del S3 en GPIO1 por defecto, 230400 8N1, comparte GND).
- **Medir**: adelanto del LiDAR y de la cámara respecto al eje trasero, ancho y largo del chasis, batalla, ángulo máximo del volante, velocidad real a una orden dada. Son los `planner.*` correspondientes.
- **Parqueo**: probado solo en simulación. Los topes tienen que verse con el plano del LiDAR a menos de 10 cm de altura: si el LiDAR queda por encima de las paredes no verá nada.

El simulador usa 3 cm (LiDAR) y 9 cm (cámara) por delante del eje motriz y 0,94 de ganancia de velocidad: son suposiciones, no medidas. Medidas del chasis (17 × 13 cm, voladizo trasero 7 cm, batalla 6 cm) las dio el equipo.

### Ajustes del planificador añadidos tras medir 24 pistas aleatorias

Todos son configuración (`planner.*` salvo el primero), no código:

- `planner.plan_w_path` (15, antes 6): peso con que la ruta sigue el corredor planeado. Con 6 el planificador veía que debía esquivar un pilar, pero seguir recto (a ~1 cm del pilar) costaba casi lo mismo y ganaba; con 15 terminan 22 de 24 pistas (antes 18), sin contactos.
- `planner.plan_margin_extra_cm` (3): el corredor se planea primero con este margen de más y, si no cabe, con el margen justo (`plan_margin_cm`).
- `planner.squeeze_cm` (3): si de golpe no hay ruta pero un instante antes sí la había, se reintenta con la huella tanto más estrecha antes de retroceder.
- `lap_memory_enabled` (apagado): memoria por sección de los letreros con color (los letreros no cambian entre vueltas). Al reencontrar un letrero conocido corrige la estima de avance de la sección. Medido: la estima de avance sin odometría se desvía 10–35 cm por recta, más que el radio de anclaje (20 cm), así que sin esa corrección casi nunca se ancla; con ella arregla alguna pista pero rompe otras (19 de 24 terminadas contra 22 sin ella), por eso sigue apagada.

## Odometría y mapa del campo (hay encoders)

El chasis tiene dos motores con encoder óptico (disco de 30 ranuras, 60 flancos por vuelta) en las ruedas motrices. Los datos de rueda y vía son valores de partida (rueda de 5,5 cm y 11,5 cm entre ruedas motrices) que se confirman con `odom_calibrate_*` sobre el chasis real. El firmware recibe el ángulo acumulado de cada rueda por `wro_controller_encoders(c, t_us, left_deg, right_deg)`; la lectura de los encoders en el P4 aún no está hecha (ver `ESTADO.md`).

| Módulo | Qué hace |
|---|---|
| `main/odometry.c/.h` | Convierte los dos ángulos de rueda en distancia y posición, con el rumbo del giroscopio. Detecta patinaje (una rueda gira más de lo que el robot se mueve) y lo descarta. Diámetro, vía, escala de cada lado y signo de cada motor son configuración (`odom.*`) y hay rutinas de autocalibración con el LiDAR y el giroscopio (`odom_calibrate_distance/track/balance`). |
| `main/arena_localizer.c/.h` | Casa el barrido del LiDAR con las paredes conocidas del campo (cuadrado exterior y bloque interior, `arena.*`) por mínimos cuadrados robustos y corrige la pose. Adquiere la pose inicial buscando en la sección de salida, mirando al este o al oeste. Letreros y topes quedan como valores atípicos. |

Medido en el simulador (ruedas con ±1,5 % de error de diámetro, patinaje esporádico, encoders con 1° de resolución, 50 Hz y 15 ms de retardo):

- **Encoders + giroscopio solos**: la posición deriva entre el 0,2 % y el 2 % del recorrido (4–38 cm tras tres vueltas); domina el rumbo del giroscopio.
- **Con el localizador**: 2,3 cm de error medio y 3,5 cm de máximo (rumbo: mediana 0,4°).

Lo que se hace con ello:

- El avance de la sección, el seguimiento de letreros, el progreso neto y el parqueo usan lo medido en vez de «velocidad ordenada × tiempo». Si dejan de llegar muestras de encoders, vuelve solo al método anterior (`odometry_enabled`).
- **Memoria de letreros en coordenadas del campo** (`arena_signs_enabled`, activa): cada letrero con color confirmado por cámara y LiDAR se guarda donde está; al volver a esa zona se reinyecta anclado a un pilar compacto del LiDAR (a menos de 8 cm), así que su cortina existe aunque la cámara no lo vea (fuera de su campo, de lado). La memoria anterior por sección no funcionaba porque la posición sin odómetro se desviaba 10–35 cm por recta.
- Frenado de un ciclo antes de invertir el sentido (el motor tarda ~0,1 s en obedecer).

Banco de 100 pistas aleatorias (50 semillas × 2 sentidos, 7 letreros, cajón): línea base 86 perfectas / 12 atascos; con memoria de campo, `footprint_shrink_cm` 1,0 y sin desplazamiento de lado: **95 perfectas / 4 con fallo / 0 contactos**. El desplazamiento de lado (`sidestep_enabled`) está apagado por defecto porque en pistas aleatorias estorba; sigue disponible para planos con cambios de lado extremos.


## Linea de carrera (vueltas 2 y 3)

Los letreros no se mueven y el robot conoce el campo, asi que despues de la primera vuelta puede **calcular** el recorrido en vez de improvisarlo. `main/race_line.c` lo hace sobre el mapa del campo (coordenadas del localizador, ver arriba):

1. Linea central redondeada del carril con una estacion cada 5 cm.
2. En cada estacion mide el tramo libre a lo largo de su normal: dentro de las paredes (con holgura), fuera del disco de cada letrero y del lado que su color permite (la "cortina"; empieza `approach_cm` antes del letrero). Si la normal cruza varios tramos, elige uno por estacion con programacion dinamica sobre el anillo para que la linea no salte de lado.
3. Desplaza cada estacion dentro de su tramo minimizando una mezcla de longitud y curvatura (minimos cuadrados con cotas, sobrerrelajacion proyectada). Prueba seis mezclas y se queda con la vuelta mas rapida segun el perfil.
4. Perfil de velocidad: `v <= sqrt(a_lat / curvatura)`, tope junto a letreros (`sign_before_cm` antes, `sign_after_cm` despues) y esquinas, y rampas de aceleracion y frenada dando la vuelta completa. Nunca baja de la velocidad base, que es la que ya se conduce con seguridad.

Es una **referencia**: el planificador reactivo (LiDAR + camara) sigue mandando. Un letrero que no se vio en la primera vuelta no esta en la linea, pero el robot lo esquiva igual; si el LiDAR ve un pilar compacto que no esta entre los conocidos, la velocidad vuelve a la base `race_unknown_hold_s` segundos.

Como se usa (`wro_controller.c`, claves `race_*` y `race.*`):

| Parametro | Defecto | Que hace |
|---|---|---|
| `race_enabled` | **apagado** (opcion) | calcula la linea al terminar `race_after_corners` (4) esquinas |
| `race_speed_enabled` | activo | la velocidad sigue el perfil (hasta `race.v_max_cm_s` 34 en recta libre; la 2.a vuelta usa `race_gain_lap2` 0,6 del exceso) |
| `race_lateral_enabled` | **apagado** | el corredor del planificador prefiere la posicion lateral de la linea (mezclada con el centro cerca de las esquinas) |
| `race_corner_extra_cm` | 20 | con la linea, la esquina empieza antes (solo si no hay un letrero conocido cerca de ella) |
| `race_stop_before_end` | 2 | las ultimas 2 esquinas se hacen sin linea: entrega limpia al parqueo |
| `race_slip_derate` | 0,25 | cada patinaje nuevo de los encoders quita esa fraccion del exceso de velocidad |
| `race_speed_dist_cm` | 12 | la velocidad extra solo se concede a menos de 12 cm de la linea |

Dos correcciones de posicion hacen que la linea sea fiable: el error de la posicion de los letreros recordados pasaba de 5,7 cm a 1,1 cm al llevar el centroide del LiDAR (que cae en la cara visible) al centro del letrero y usar la pose a la hora del mapa del planificador. Esas posiciones corregidas (`fx, fz`) solo las usa la linea: la reinyeccion de letreros sigue con las del LiDAR (`x, z`), porque corregirlas tambien ahi hacia que `tests/plano_errores.json` dejara de completarse.

**Resultado medido** (100 pistas aleatorias, 7 letreros, semillas 21-70, ambos sentidos):

| | perfectas | mediana de ronda |
|---|---|---|
| sin linea | 99 | 142 s |
| `race_enabled 1` (velocidad y esquina adelantada) | 97 | 141 s |
| con preferencia lateral (`race_lateral_enabled 1`) | 90 | 139 s |

Con 7 letreros casi toda la pista cae en zona de letrero o de esquina, asi que la ganancia es pequena; crece con menos letreros. La preferencia lateral recorta mas el camino, pero el reactivo no siempre llega a la linea a tiempo tras una esquina y baja la fiabilidad (pasos por el lado equivocado y parqueos que no terminan), por eso queda como opcion. Con la linea activa tampoco se gana tiempo en este banco, asi que **toda la linea esta apagada por defecto** (`race_enabled 0`) hasta medir el chasis real o probar pistas con menos letreros. Los limites reales de velocidad de este chasis (patinaje, retardo del motor) no se conocen hasta medirlos sobre el chasis real.

Herramientas: `node simulator/tools/race_check.js SEMILLA ccw|cw [--trace --lap 2] [--cfg "clave valor"]` (cuando se construye, velocidades por vuelta, tiempo activa) y `tests/test_race_line.c`. En el simulador la linea se dibuja coloreada por velocidad (casilla "linea de carrera") y hay una fila en el panel.

## Pegado a la pared interior (opcional, vueltas 2 y 3)

Alternativa simple a la linea de carrera, sin optimizador: la primera vuelta va centrada y recuerda los letreros (posicion y color); en las vueltas siguientes el robot va **pegado a la pared interior** en las rectas y solo se despega cuando un letrero conocido le exige estar en otro sitio (`hug_step` en `wro_controller.c`). Pide al planificador un desplazamiento del centro del carril (`lane_offset_cm`); esquivar, holguras y cortinas siguen siendo del reactivo.

- El atajo no esta en la recta sino en la esquina: el giro empieza a la misma distancia del muro, asi que llegar pegado a la pared interior hace que el robot empiece la recta siguiente ya avanzado (unos 20 cm por esquina). Por eso `hug_corner_clear_cm` es 0: llega pegado hasta el giro.
- Cada letrero conocido a menos de `hug_detach_cm` (90) por delante, y hasta `hug_after_sign_cm` (20) despues de pasarlo, fija el rango lateral permitido segun su color (con 13 cm de centro a centro). Si hay varios, se cruzan los rangos; si no hay hueco coherente, centrado.
- El cambio lateral tiene una pendiente maxima (`hug_rate` 0,6 cm por cm). Centrado tambien en la primera vuelta, tras una esquina (`hug_after_corner_cm` 10), en las ultimas esquinas (`hug_stop_before_end` 2, entrega limpia al parqueo), mientras gira o escapa, y `hug_unknown_hold_s` (1,5 s) tras ver un pilar que no esta en el mapa.
- `hug_enabled` esta **apagado por defecto**. Activarlo: `hug_enabled 1`. Herramienta: `node simulator/tools/hug_check.js SEMILLA ccw|cw [--signs N] [--trace]`. En el simulador, el panel *Controlador* muestra la fila "Pegado a la pared interior".
- **Sin medir en el banco de 100 pistas** (se hara cuando se pida). En 4 pistas sueltas: sin infracciones y 1-2 s menos por vuelta, con el robot activo entre el 27 % y el 63 % del tiempo.

### Configuración final validada (2026-10-13)

Elegida con dos bancos de 100 pistas aleatorias (semillas 21–70 de ajuste y 71–120 de validación, ambos sentidos, 7 letreros, cajón). Un valor solo se fijó si se mantuvo en los dos bancos (la escala de desviación 6 cm parecía mejor en el primero —98— y cayó a 90 en el segundo: sobreajuste).

| Parámetro | Valor | Efecto medido |
|---|---|---|
| `planner.speed_cm_s` | 23 | cierra la ronda ~16 s antes que a 20 (mediana 143 s, máximo 159 s) con la misma fiabilidad; a 26 ya pierde fiabilidad |
| `planner.plan_path_scale_cm` | 5 | 99 / 95 perfectas (con 10: 95 / 93) |
| `planner.plan_w_path` | 15 | sigue el corredor planeado |
| `planner.footprint_shrink_cm` | 1,0 | sin contactos |
| `arena_signs_enabled` | activo | memoria de letreros en el campo |
| `arena_gates_corners` | activo | una esquina solo empieza en una zona de esquina; evita falsas esquinas al esquivar con mucho giro |
| `sidestep_enabled` | apagado | desplazamiento de lado: estorba en pistas aleatorias |
| `peek_enabled` | apagado | girar hacia pilares sin color: ayuda a planos de S extremas, pero baja de 95 a 89 perfectas |

Resultado: **99 y 95 pistas perfectas de 100, sin contactos** (a 20 cm/s; a 23 cm/s, 99 y 95). El `plano_errores.json` del equipo (con dos cambios de lado que la geometría marca como casi imposibles) se completa con 62 puntos, parqueo incluido, en 177 s.
