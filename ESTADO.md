# ESTADO del proyecto (actualizado 2026-11-11)

> **Actualización del 10 nov 2026 (léela primero).** Todo lo de abajo que habla del parqueo de entrada es anterior. Lo nuevo:
>
> - **Parqueo (simulación, chasis `largo26`):** la entrada completa al cajón queda en 2-5 repasadas (media ~4, 16-17 s) y 0 fallos en las semillas 1-8; en las
>   semillas 9-32 hay 5 fallos de 56 (9 %, antes 17 %). La meta de entrada pide rumbo ≤ 4° (el juez da *completo* hasta 6,2°), el A\* tiene un coste suave de
>   holgura y el chequeo de arco nunca es más estricto que el plan. Causa de las repasadas de más: el plan va con ~1 cm de holgura real y el error de
>   ejecución más el de pose es del mismo orden. Detalle, experimentos descartados y límites abiertos en [`PENDIENTES_PARQUEO.md`](PENDIENTES_PARQUEO.md).
> - **MIC (Motor Intelligent Control):** módulo central de tracción en [`Motor Control/MIC/`](Motor%20Control/MIC/README.md): modelo del JGB37-520, observador,
>   planificador con anticipación, frenado progresivo y agresivo. Integrado en `hw_motors.c` tras `CONFIG_WRO_USE_MIC` (**apagado**) y en el simulador (`mic: true`).
>   Probado solo con un motor simulado. Procedimiento de calibración en [`CALIBRACION.md`](Motor%20Control/MIC/CALIBRACION.md).
> - **trace_map:** mapa persistente con espacio libre, objetos (color, posición) y plaza de parqueo detectada por geometría. Ver [`simulator/TRACE_MAP.md`](simulator/TRACE_MAP.md).
> - **Aviso de hardware:** el lazo abierto de `hw_motors.c` usa `WRO_MOTOR_MAX_CM_S = 40`; con el motor de 800 rpm el 100 % de ciclo son ~210 cm/s. No activar
>   los motores sin MIC o sin recalibrar ese valor. **Motor (16 nov):** el real es de 800 rpm (etiqueta; 844 medidas sin carga a 12,0 V, ver `MEDIDAS_HARDWARE_P4.md`) y es lo que asume el repo (MIC, Kconfig, simulador).
> - Nada de esto está probado en el coche real.
> - **16 nov (noche): ronda `ccw 22` con `relax_iterations` 300 = adquisicion falsa del cajon, no el plan de IT.** El plan salia igual (5,62 frente a 5,63 s) y el IT se comportaba igual; la diferencia fue 1,7 s en cuando empezo PARKING: `park_localize_acquire` acepto un ajuste con 38 puntos de pared, 6 del tope trasero y puntuacion 38,6 estando el coche a 54 cm del cajon, lo coloco 44 cm mas cerca (error de u de 43,8 cm; w y rumbo bien) y aparco donde no era (44 puntos). Los ajustes buenos de 79 rondas (`tools/park_acq_stats.js`) tienen pared >= 65 y puntuacion >= 65,5: el umbral pasa de (pared 10, puntuacion 20) a (50, 50) (`PARK_ACQ_MIN_WALL/SCORE`); no aceptar uno malo solo retrasa la adquisicion unos barridos. Con eso `relax_iterations` queda en **300** (mismo plan, la mitad de coste): 48 de 48 rondas con 62 puntos, 0 contactos. El P4 lleva aun el firmware anterior (relax 4000, adquisicion antigua): reflashear.
> - **16 nov (noche): optimizado en el P4, planes identicos.** IT: plan completo 18 s -> 4,2 s (evaluacion incremental exacta + matematica barata + tarea en el nucleo 1); A* de parqueo 11-27 s -> 5-11 s; pila de `app_main` a 12 KB; `-O3`. Detalle en `MEDIDAS_HARDWARE_P4.md` seccion 9. `relax_iterations` se queda en 4000 (con 300: 47 de 48 rondas frente a 48 de 48). Pendiente: el A* sigue lejos del tiempo real (decidir tabla de politica / planificar en la aproximacion / 1 variante) y el barrido del controlador (142 ms) supera los 100 ms.
> - **16 nov: medido en el P4 real** (ver `MEDIDAS_HARDWARE_P4.md`, seccion 8): el C del robot es ~40 veces mas lento que en el PC. Un barrido del controlador son 160 ms (presupuesto 100), el A* del parqueo tarda 12-27 s por plan, el plan del IT 18 s (una rebanada de 300 evaluaciones, 393 ms) y la pila de 6 KB de `app_main` se desborda. El simulador no modela el tiempo de calculo. Pendiente: pila a 12 KB, planes en una tarea aparte del segundo nucleo, A* con 1 variante.
> - **16 nov: IT frente al optimo, con el motor de 800 rpm (vmax 188,7 cm/s, aceleracion 300 -> 68 cm/s2 al 90 %).** Con el mapa real del robot el plan de IT sale en 9,23 s (replica Python: 9,235) y el mismo modelo permite ~8,0 s: la busqueda local del C se queda en un minimo (los pilares del mapa tienen hasta 2,8 cm de error y estrechan los pasillos). El control optimo (CasADi) da ~6,3 s con huella estatica o margen medido: esa brecha son las reglas (margen 14*(v/100)^2 y huella), no la busqueda. Anadido `it_plan_refine_begin` (refinado en segundo plano con recocido tras el primer plan, `it_refine_evals`, `it_refine_step_evals`): plan 9,23 -> 8,5 s y IT 3,3 % mas rapido por distancia, pero en 48 rondas completas el total no mejora (88,7 s frente a 86,9) y una ronda pierde puntos: queda APAGADO (`it_refine_evals 0`). Sin medir en el P4.
> - **11 nov (noche, 4): seguidor de IT.** Causa de la ronda `ccw 13` que siempre fallaba: un letrero verde no entro en el mapa de la vuelta 1 y IT lo atropellaba a 150 cm/s (la guardia de sorpresas solo valia en rectas, pedia 4 barridos = 60 cm y miraba 75 cm). Ahora `it_unknown_ahead` (wro_controller.c) busca puntos del LiDAR que no son pared, bloque ni obstaculo del plan dentro de la huella de las proximas estaciones (en curva tambien): baja a 45 cm/s al ver 3+ puntos y cede el mando en el segundo barrido; no se engancha mientras siga delante; la guardia de arco recto mira 20+0,75v cm y cede a los 2 barridos. 48 rondas, 5 letreros: 48 de 48 con 62 puntos (antes 31 de 32 en 32 rondas). Con la proteccion, `it.track_err_cm` 6-10 hace IT 10-17 % mas rapido por distancia (`tools/it_laps.js`), pero en 48 rondas completas pierde puntos en el parqueo (44-47 en 1-5 rondas) y una ronda no acaba (ccw 15 con 10: IT nunca cede): valor por defecto sin cambios (14). Abierto: por que IT no cede el mando en ese caso.
> - **11 nov (noche, 3): investigacion del IT con una replica en Python** (`simulator/tools/it_py/`, reproduce el C: 7,810 s frente a 7,809 s). Hallazgos: (1) la busqueda del C ya esta en el optimo de sus reglas (recocido, evolucion diferencial y control optimo con CasADi/IPOPT dan 7,6-7,8 s). (2) Lo que fija el tiempo son las reglas: margen de error de seguimiento 14*(v/100)^2 (el error real medido es ~4 veces menor) y la holgura de la huella con ese mismo margen (cuesta ~1,4 s por vuelta). Con margen medido y huella el optimo continuo es 6,4-6,6 s. (3) Pero NO se traduce en rondas: con `it.track_err_cm` 5-11, o `IT_GAP_ERR_SCALE_SQ` 2-4, el plan baja a 6,5 s y las rondas (32, 5 letreros) no ganan tiempo (74 -> 78-84 s) y aparecen contactos y rondas sin acabar: el limite esta en el seguidor (`it_track`), no en el plan. Valores por defecto sin cambios. (4) Arreglado: IT ya no se cae con mas letreros (reintento con mas volante, pilar sin color como obstaculo con cesion al reactivo): 24 de 24 con plan, antes 21.
> - **11 nov (noche, 2): aproximacion al parqueo e IT con mas letreros.** (1) `park_cruise_step`: la aproximacion ya no apunta a la pose de la salida (metia el morro en el cajon); ahora va paralela a 30-36 cm de la pared hasta u=20 cm y gira el morro de forma progresiva a 20 grados antes de la reversa (`PARK_APPROACH_*`, `PARK_APPROACH_MODE 0` = la antigua). 16 rondas: 18,8 s y 5,9 repasadas -> 16,6 s y 3,6; barrido de u, inclinacion y distancia a la pared en 24 rondas: 20 cm y 20 grados siguen siendo lo mejor (14 grados da un contacto). Aun no llega a 2 pasadas. (2) IT: un plan invalido por volante (why=1) se reintenta con mas volante (`IT_STEER_RETRIES`) y un pilar sin color ya no cancela el plan (color 3, el reactivo manda a <60 cm): con 5 letreros 24 de 24 rondas con plan (antes 21) y 79,8 -> 75,7 s. La trayectoria 'ovalada' de la linea corta pegada al muro interior NO se logro: el planificador rechaza esa curvatura. Herramientas nuevas: `simulator/tools/user_case.js`, `park_batch.js`, `it_diag.js`, `it_user_plan.js`.
> - **11 nov (revisión del simulador).** El simulador abría con IT, cajón y arranque en el cajón APAGADOS: ahora los tres vienen marcados. Cambios: (1) el sorteo `feasible` reponía mal los
>   letreros (3 de 6 era lo normal): ahora prueba otros asientos y coloca ~5,5 de 6 (el aviso explica por qué faltan); `refill: false` conserva el sorteo anterior. (2) La pose estimada se dibuja
>   adelantada por el retraso del barrido (antes iba detrás: la pose es la del instante del barrido, que llega ~procDelay después). (3) El parqueo para en cuanto la pose estimada cumple la meta
>   (antes terminaba el plan entero: hasta 12 s y 9 repasadas con el juez ya dando 'completo'): 105 repasadas en 24 rondas (antes 122), 24 completas, 0 contactos. (4) El juez espera 3 s
>   (antes 15) tras aparcar y el tiempo de la ronda es el del instante en que se detuvo. (5) IT lleva todo menos la última esquina (`it_stop_before_end` 1; con 0 el parqueo falla): ~3 s menos.
>   **Efecto secundario:** con ~5,5 letreros reales aparecen fallos que el sorteo escaso ocultaba: reactivo solo 21 de 24 (3 atascos contra un letrero en un paso de 27 cm: escape ↔ avance sin salida);
>   completo (IT+MIC) 24 de 24 terminan, 22 con 62 puntos y 2 con 44. Sin resolver.
> - **11 nov (asimetría cw/ccw):** causa = descentrado de 0,8° del servo del simulador (la salida acababa a 9° ccw / 4° cw); el simulador ahora lo calibra (`steerCalibDeg`). Diferencia cw−ccw de 2,3 s a 0,3 s. Ver `PENDIENTES_PARQUEO.md` 3f. Las pistas densas siguen sin resolverse.
> - **11 nov (noche): 24 de 24.** Cerrados los dos fallos que quedaban en pistas factibles (3 letreros, `largo26d`, 24 rondas: **62 puntos y 0 contactos en todas**, 118,9 s de media del cronómetro del simulador):
>   (1) atasco de 120 s en la esquina de la vuelta 3 (cw 5): el error de rumbo al disparar la esquina era 93° (el morro 3° atrasado), `local_obstacle_map.c` trata >90° como "detrás" e
>   invertía el giro (270° a la izquierda hacia la pared); ahora entre 90° y 135° se pide 85° hacia el lado corto. (2) contacto con el tope en el parqueo (cw 2): la comprobación de cada arco de
>   la entrada sube de 0,5 a 0,75 cm (`PARK_ENTRY_CHECK_MARGIN_CM`): 24 completas y 0 contactos en las semillas 21-32, a cambio de ~5 repasadas de media (antes 4,5).
> - **11 nov (tarde): causa del "1 de cada 3 acaba en wrong-side".** `measure_mission.js` y `bench/ab2.js` sorteaban pistas SIN `feasible: true`: con arranque en el
>   cajón salían letreros a ~45 cm de la salida que ningún chasis Ackermann puede pasar por el lado pedido (la interfaz sí usa `feasible`). `measure_mission.js` ya usa pistas
>   factibles por defecto (`--any` las permite todas). Resultado con `largo26d`, 3 letreros, 24 rondas: **22 terminan con 62 puntos**; fallan una por contacto con el tope en el parqueo (cw 2)
>   y una por atasco 120 s en la esquina del inicio de la vuelta 3 (cw 5: tras el primer giro a izquierda el reactivo oscila delante/atrás). Con la velocidad de mapeo en 32 o 33
>   fallan 3-4 de 24 en otras semillas: es caótico, no una velocidad mala. Corregido aparte: el parqueo se quedaba parado ('stalled') si la pose medida solapaba el cajón aunque sin holgura
>   (ahora se usa la pose libre más cercana, hasta 2 cm y 6°) y, sin plan desde una pose, se hace un movimiento corto libre para cambiar de pose (`park_noplan_moves`).
> - **11 nov:** la velocidad de mapeo subió a 35 cm/s (commit 628b88c; se dejó en 34 porque a 35 `plano_corregido.json` falla con `wrong-side`) y la entrada al cajón pasó de 2-5 a 8-12 repasadas en algunas semillas cw: la pose con la que se
>   llega a la búsqueda cambia unos cm y el plan reacciona de forma caótica. Se añadió `park_approach_speed_cm_s` (23): tope de velocidad desde la última esquina.
>   Con la velocidad de mapeo en 34 y 8 rondas (semillas 1-4, ccw/cw): 2-5 repasadas, 3,4 de media y 62 puntos en todas (con 35 y sin el tope: 5,4 de media y máximo 12; con todo el recorrido a 23: 3,3). No es una solución de fondo (ver
>   [`PENDIENTES_PARQUEO.md`](PENDIENTES_PARQUEO.md), límite 2): la causa es la sensibilidad del plan a la pose de llegada.

> **Cambio de chasis, 7 nov 2026:** la configuración diferencial descrita abajo
> es histórica. El nuevo robot usa un motor trasero con encoder integrado y
> dirección Ackermann por servo. La adaptación está incompleta; ver
> [`ACKERMANN.md`](ACKERMANN.md) antes de usar el firmware o el simulador.
> Las afirmaciones de pruebas en verde son históricas: la misión Ackermann no se
> ha validado de extremo a extremo y la prueba diferencial de salida falla.

> **Actualización de lectura:** este documento mezcla etapas y contiene pendientes
> históricos. En el árbol actual existen drivers del P4 para motores, encoders,
> LiDAR, IMU y enlace con el S3; su presencia no demuestra que estén integrados
> o validados en el robot. Consultar el código y confirmar el cableado antes de
> interpretar las tablas antiguas como el estado actual.

> El bucle del P4 comprueba la edad del giroscopio y del LiDAR antes de mandar
> movimiento. Ambos usan `sensor_timeout_us` (300 ms por defecto); si vuelven
> las muestras, el movimiento puede reanudarse. Este cambio requiere pruebas y
> no sustituye una parada física ni detecta por sí solo encoders desconectados.

Documento de relevo: dice qué está hecho, cómo comprobarlo y qué queda para la **próxima sesión** (drivers y hardware del P4). Empieza aquí si retomas el trabajo.

## 1. Resumen

El robot (chasis de dos ruedas con encoders ópticos, ESP32-P4 + ESP32-S3 CAM) ya navega, esquiva letreros, completa 3 vueltas, sale del cajón y aparca **en simulación**, usando encoders, giroscopio, LiDAR y cámara. El simulador ejecuta el mismo C del firmware (WebAssembly). Pase de pruebas del simulador en verde el 7 nov 2026 (`simulate.ps1 -Quick` y las rondas completas).

Lo que impide pasar al robot real es el hardware y los drivers del P4 (sección 5).

## 2. Resultados verificados

| Prueba | Resultado |
| --- | --- |
| `Vision System/tools/simulate.ps1` (C nativo, simulador, rondas, depurador, visión) | Verde el 7 nov 2026 con el WASM reconstruido; regenera `simulator/build/simulador.html` |
| `simulator/tests/test_missions.js` | Verde, incluida la misión de 62 puntos (salir del cajón, 3 vueltas, aparcar) en los dos sentidos con el chasis de 17 × 13 cm |
| `simulator/tests/plano_errores.json` (plano del equipo) | 62 puntos, parqueo completo, 24/24 secciones, 0 contactos, 177 s |
| Banco de 100 pistas, semillas 21–70 (ajuste) | 99 perfectas, 0 contactos |
| Banco de 100 pistas, semillas 71–120 (validación) | 95 perfectas, 0 contactos |
| Velocidad | 23 cm/s (mediana 143 s, máximo 159 s de 180 s) |
| Salida del cajón (6 combinaciones) | 20–55 s de los 180 de ronda; mediana ~22 s |
| Costo por paso de simulación (navegador, WASM actual) | 1,5 ms: techo de ~6,9 s simulados por segundo real (8× no se sostiene; 4× sí) |

Todo es **solo simulación**. Con el robot real solo se probó la cámara.

## 3. Qué se hizo (esta etapa)

- **Odometría** (`LIDAR System/main/odometry.c`): ángulos de rueda + rumbo del giroscopio → posición. Detecta patinaje, rechaza saltos imposibles, se reinicia tras un hueco de muestras (`max_gap_s` 0,5 s) y tiene autocalibración (`odom_calibrate_distance/track/balance`). Valores por defecto: rueda 5,5 cm, vía 11,5 cm, escala 1 por lado, signo +1.
- **Localizador del campo** (`arena_localizer.c`): casa el LiDAR con 8 segmentos de pared (cuadrado exterior y bloque interior) por Gauss-Newton con pesos Huber. Adquiere la pose en la sección de salida. Error medio 2,3 cm, máximo 3,5 cm.
- **Memoria de letreros en coordenadas del campo** (`arena_signs_enabled`): reinyecta letreros ya vistos anclados a un pilar compacto del LiDAR.
- **Compuerta de esquinas** (`arena_gates_corners`): una esquina solo empieza en una zona de esquina según la pose del campo. Arregló el sentido contrario del plano del equipo.
- **Planificador**: escala de desviación del corredor 5 cm, margen extra primero, squeeze de huella, frenado de un ciclo antes de invertir.
- **Simulador**: modelo de encoders (±1,5 % de diámetro, patinaje, 1°, 50 Hz, 15 ms, pérdidas) con generadores aleatorios propios; panel de odometría y pose con error contra la verdad; pose estimada dibujada (contorno amarillo = LiDAR corregido, cruz magenta = solo encoders).
- **Herramientas**: `depurar.py` (ahora también falla con salida/parqueo sin terminar), `odom_check.js`, `arena_check.js`, banco de 100 pistas en `simulator/tools/bench/`.
- **Pruebas nuevas**: `test_odometry.c`, `test_arena_localizer.c`, ampliación de `test_ui_smoke.js`.

## 3b. Linea de carrera (vueltas 2 y 3): hecha, apagada por defecto

Implementada en C (`LIDAR System/main/race_line.c` + integracion en `wro_controller.c` y el corredor del planificador) y en el simulador (linea dibujada coloreada por velocidad, fila de telemetria, `simulator/tools/race_check.js`). Calcula, tras la primera vuelta, la trayectoria y el perfil de velocidad con los letreros conocidos; el LiDAR sigue mandando. Detalle y parametros en `LIDAR System/README.md`, seccion "Linea de carrera".

**Resultado honesto (100 pistas, 7 letreros):** sin linea 99 perfectas y mediana 142 s; con la linea activa 97 y 141 s; con preferencia lateral 90 y 139 s. No gana tiempo apreciable con tantos letreros (casi toda la pista es zona de letrero o esquina) y baja la fiabilidad (mas parqueos que no terminan, algun paso por el lado equivocado), asi que `race_enabled` queda en **0**. Para activarla: `race_enabled 1` (y `race_lateral_enabled 1` para la preferencia lateral, mas ganancia y mas riesgo).

**Para retomarla:** probarla con menos letreros, y cuando se midan los limites reales del chasis; arreglar la entrega al parqueo (la causa de los parqueos que no terminan con la linea sigue sin resolverse del todo); que el reactivo llegue a la linea tras una esquina (hoy la referencia se mezcla con el centro cerca de ellas). Tambien: la posicion de los letreros recordados pasaba de 5,7 cm a 1,1 cm de error al corregir el centroide y la pose a la hora del mapa; esas posiciones solo las usa la linea (reinyectarlas asi rompia `plano_errores.json`).

## 4. Configuración final y decisiones

Detalle y tabla en `LIDAR System/README.md`, sección «Configuración final validada». Resumen:

- **Activo**: `arena_signs_enabled`, `arena_gates_corners`, `planner.plan_path_scale_cm` 5, `plan_w_path` 15, `footprint_shrink_cm` 1,0, velocidad 23.
- **Apagado a propósito** (dejaron peor las pistas aleatorias): `sidestep_enabled` (desplazamiento de lado), `peek_enabled` (girar hacia pilares sin color; bajaba de 95 a 89), `lap_memory_enabled` (memoria por sección, sustituida por la del campo), `tight_clear_cm` (frenada por holgura; costaba ~18 s y rompía el parqueo).
- **Rechazado por sobreajuste**: escala de desviación 6 cm (98 en un banco, 90 en el otro). Un valor solo se fija si aguanta en los dos bancos.
- **Velocidad 26** ya pierde fiabilidad.

## 5. LO QUE FALTA: hardware y drivers del P4

La lógica ya está hecha y probada en simulación. Falta conectar cada entrada y salida del P4 al hardware. Son las funciones que el equipo rellena en `p4_hal_t` (`LIDAR System/orchestrator/p4_runtime.h`):

| Función | Hardware | Estado |
| --- | --- | --- |
| `drive` | Puente H L298N: PWM de 1 kHz en IN1–IN4 (sin pin de enable) | Pendiente |
| `poll_scan` | LiDAR WitMotion D6 por UART a 230400 8N1. El decodificador está en `LIDAR System/sketch_oct14a` (Arduino); hay que portarlo a C | Pendiente; falta el GPIO RX |
| `poll_gyro` | IMU (INT1 47, SDA 53, SCL 48) | Pendiente; falta el modelo del chip |
| `read_camera` | ESP32-S3 CAM por USB (el P4 es el host; puente CH340 de la placa del S3, 921600 baudios) | Hecho en código (`hw_camera.c`, `Vision System`); sin probar con el S3 enchufado al P4 |
| encoders | Encoders ópticos en GPIO 27 y 45 (`Motor Control/Encoder/esp32p4`) | Pendiente de cablear: un canal por rueda, el sentido lo da el control |
| `start_pressed` | Botón de arranque | Pendiente; falta el GPIO |

Tareas, en orden:

1. **Confirmar los pines pendientes** (GPIO del RX del LiDAR, modelo de la IMU, placa del P4) y resolver los conflictos que ya se ven: IN4 del L298N va en GPIO33 (el GPIO34 ya no se usa), y en el programa de prueba de audio que tiene hoy la placa el GPIO53 es el enable del amplificador.
2. **Driver de motores** (`drive`): `v_izq = v + ω·vía/2`, `v_der = v − ω·vía/2` (ω horario-positivo, como en `p4_runtime.h`), convertido a duty PWM por rueda. Sentido: una entrada de cada puente a PWM y la otra a 0. Aplicar el signo de cada motor.
3. **Encoders**: `encoder_port_init` por rueda y `encoder_port_set_direction` con el sentido del último comando. Pasar el ángulo acumulado a `wro_controller_encoders`.
4. **LiDAR**: portar el decodificador a C y entregar `local_lidar_point_t` por vuelta.
5. **Enlace USB con el S3**: probarlo con el S3 enchufado al host del P4 (driver CH34x). El S3 envía la tabla por su UART0 a 921600 baudios.
6. **IMU**: driver y calibración del sesgo con el carrito quieto.
7. **Medir sobre el chasis real**: diámetro de rueda, vía, velocidad real frente a la ordenada, retardo del motor. Cargarlo en `odom.*` y `planner.*`. Los valores de partida de la odometría (5,5 cm, 11,5 cm) no están confirmados.
8. **Parada de seguridad**: el perro guardián ya para si faltan LiDAR o giroscopio; añadir la falta de encoders.

## 6. Fallos conocidos

- ~1 de cada 20 pistas aleatorias falla (sin contactos): letrero a ~10 cm con el morro girado 20–30°, o parqueo que no termina.
- Dos cambios de lado seguidos más rápidos de lo que permite el radio de giro no tienen trayectoria; son un problema del plano (`field.checkLayout` los avisa). El aviso de "entrada de la recta" ya mide desde el extremo por donde se entra en las cuatro rectas y en los dos sentidos.
- La salida del cajón cuesta ~9,5 s de los 180 de ronda con el volante a 40° (con 30° llegaba a 55 s en el peor caso; con el frenado por arco que había antes, 21–24 s). El parqueo de entrada, ~15–17 s.
- **La entrada al cajón necesita varias pasadas, no una (hoy 4–8).** Con 1,5 × L = 39 cm para un coche de 26 cm (holgura 13 cm) el hueco es "apretado", como aparcar entre dos coches. El plan de la maniobra **sí** es el correcto: `park_ackermann_entry_maneuver` busca tres tramos (colocarse → entrar girando → enderezar) y sale con **2 cambios de sentido** (`AAAA…RRRR…A`). Las pasadas que quedan de más son la **alineación final dentro del cajón**, donde no caben arcos largos y trabaja el A* de 3,5 cm. Historial: 10 con la penalización original, 8 con 12, **4–8** con la maniobra. Probado y descartado: S de dos arcos desde el carril (no cabe: la esquina trasera engancha el tope a los 6,5 cm), presupuesto de repasadas (empeoraba por alternar), y maniobra desde dentro del cajón (rompía la ronda: 180 s con parqueo parcial).
- **Entrada simétrica a la salida: implantada y medida.** El argumento de simetría es correcto —un camino libre al revés es libre— y el planificador aislado da **1 repasada** desde la pose donde acaba la salida, así que la entrada del controlador ahora arranca ahí: `PARK_EXIT` recuerda su pose final, un crucero lleva el coche por el carril a 2 cm más adelante y 1,5 más lejos de la pared, y desde ahí hace **un solo plan**. Para que el plan se ejecute entero (antes se abortaba a 10° de morro y la corrección costaba una repasada más) los arcos de la entrada se cortan por **rumbo del giroscopio**, se vigila el avance a ciegas y **se le da el volante al servo con el coche quieto** antes de mover (1 barrido si el cambio es pequeño, 2 si es una inversión completa: el servo tarda ~0,2 s en pasar de un tope al otro y, sin eso, el primer arco de cada cambio giraba menos que el modelo). La salida y la demostración desde la pista conservan el ejecutor de siempre: la salida sigue en **1 repasada y 8,1–8,6 s**. Medido en ronda completa (largo26, 4 semillas por sentido): repasadas **3, 3, 4, 4, 5, 4, 4, 4** (antes de todo esto, 8, 4, 4, 3 en las dos primeras), **62 puntos y parqueo completo en las 8**, sin contactos, entrada en 12–18 s. Se mide con `simulator/tests/parkentry.js`.
- **Lo que falta para que la entrada sea 3 SIEMPRE (probado, no supuesto).** Tres vías cerradas: (1) la cuenta del A* **no es un parámetro**: desde poses a 1 cm de distancia da 1 o 2 repasadas, y subir el castigo por cambio de sentido de 12 a 60 cm lo deja en 2, 3, 5, 7 o 10 según el caso (empeora); (2) la **S de dos arcos calculada** (cambio de carril, el aparcamiento en paralelo clásico) no cabe: con la rejilla exacta, la esquina del chasis pasa 0,9 cm por encima del tope delantero con la holgura de planeo de 1 cm —rozando—, y por detrás engancha el tope trasero; (3) la **maniobra de tres tramos** (`park_ackermann_entry_maneuver`) **no se acepta nunca**: el patrón de volante estaba invertido (con el sentido alternado, el volante tiene que ir *igual* para que los giros alternen: el giro es `dir * largo * tan(volante)/batalla`), y aun corrigiéndolo no encuentra solución desde el carril ni desde la pose de la salida. Siguiente vía realista: **modelar el retardo del servo (~0,2 s) en el A*** para que el plan de 1 repasada se ejecute sin corrección, o ejecutar la salida al revés arco a arco con un ajuste final explícito.
- El parqueo de la misión diferencial depende de que el cajón esté dibujado para ese chasis: los planos declaran su perfil (`robot` en el JSON) y la simulación corre ese chasis; si un plano no lo declara, el cajón y la pose de arranque se ajustan al robot simulado.
- Misión completa del chasis Ackermann (salir del cajón, 3 vueltas, aparcar): 62 puntos en 4 de 4 casos, terminando a los 141–155 s de 180, con y sin letreros.
- La velocidad de reproducción 8× no se sostiene (medido: 5,9× con la ventana en primer plano); 1× y 4× sí. Falta sacar la simulación del hilo de la interfaz para llegar a 8×.
- Todos los datos de hardware (rueda, vía, encoder, montaje de LiDAR/cámara, ganancia de velocidad) son supuestos hasta medirlos. El volante a 40° es lo que pediste; el servo real hay que medirlo igual que el resto.
- La interfaz del simulador se probó con un DOM simulado y, desde esta revisión, abriéndola en Chrome; no hay prueba automática de lo que se ve.

## 7. Cómo reproducir

```powershell
cd "Vision System"; .\tools\simulate.ps1            # pase completo y regenera simulador.html
python simulator\tools\depurar.py simulator\tests\plano_errores.json --sin-mapas
node simulator\tools\odom_check.js 100 3            # error de la odometría sola
node simulator\tools\arena_check.js 100 3           # error de la pose del localizador
```

- Recompilar el firmware a WebAssembly: `simulator\wasm\build.ps1` (necesita wasi-sdk) y luego `python simulator\build_simulator.py`.
- Banco de 100 pistas: `simulator/tools/bench/` (`big.sh`, `ab2.js`, `cmpbig.py`, `cmptime.py`, `rebuild_wasm.sh`). Los scripts ya usan rutas relativas (probado: `node simulator/tools/bench/ab2.js t 21:ccw` da 55 puntos); `big.sh TAG "cfg1;cfg2"` escribe los resultados `b_TAG_*.txt` junto al script. `rebuild_wasm.sh` usa `WASI_SDK_PATH` o `simulator/.toolchain`. El banco corre en paralelo (unos 22 procesos de Node) y tarda decenas de minutos.

## 8. Mapa de documentación

- [`README.md`](README.md): visión general, arquitectura, resultados.
- [`LIDAR System/README.md`](LIDAR%20System/README.md): planificador, controlador, odometría, localizador, parqueo, configuración final.
- [`simulator/README.md`](simulator/README.md): simulador, encoders, herramientas, cómo se genera `simulador.html`.
- [`Vision System/README.md`](Vision%20System/README.md): detector de visión y protocolo.
- [`PENDIENTES_PARQUEO.md`](PENDIENTES_PARQUEO.md): estado del parqueo, flags experimentales, límites abiertos, preguntas y cómo medir.
- [`Motor Control/MIC/README.md`](Motor%20Control/MIC/README.md) y [`CALIBRACION.md`](Motor%20Control/MIC/CALIBRACION.md): control de tracción y su calibración.
- [`simulator/TRACE_MAP.md`](simulator/TRACE_MAP.md): mapa persistente (rejilla, objetos, plaza, guardado).
- [`simulator/LINEA_BASE_PARQUEO_MAPA.md`](simulator/LINEA_BASE_PARQUEO_MAPA.md): mediciones de la línea base del parqueo y del mapa (9 nov).

## 9. Panel web, enlace P4-S3 y mapa (rama integracion-panel)

**Inicio:** el robot solo arranca con el boton fisico (GPIO5). La pagina web solo puede PARAR (`WRO_WEB_START_ENABLED=n`).
El signo del giroscopio no se cambia solo con el robot armado.

**Tamano del mapa del cuadrilatero** (campo: cuadrado exterior 300 cm, bloque interior 100 cm, carril 100 cm):
- El mapa es del robot: origen = donde arranca, norte = hacia donde miraba. El campo ocupa una caja de 300 x 300 cm.
- Con el robot en el centro del carril de salida (a 100 cm del centro del campo): la caja va de -50 a +250 cm en el eje que cruza el carril y de -150 a +150 cm en el otro. Si arranca desplazado hasta +-50 cm a lo largo del carril: peor caso 400 x 300 cm (-200..+200 por -50..+250).
- Alcance: la esquina mas lejana queda a sqrt(150^2 + 250^2) = 292 cm del arranque, menos que MAX_RANGE_CM (500) de `map_pose.c`.
- Pagina: 300 x 300 celdas de 2 cm = 6 x 6 m (antes 4 cm = 12 x 12 m), unas 22 500 celdas ocupadas del campo (150 x 150) de 90 000. 90 KB por nivel en el navegador.
- Enlace: ~406 puntos por vuelta (limite 512), trama de estado ~1,8 KB a 8 Hz = ~15 KB/s de los 92 KB/s de la UART a 921600.

**Pendiente (necesita el robot / la mano):** girar el robot a mano para comprobar rumbo y signo del giroscopio; flashear el S3 (`S3 Robot`, ya compila con IDF 5.5.4) con su red WiFi en `S3 Robot/main/wifi_secrets.h`; motores y encoders al final.
## 10. Nota de lectura sobre el chasis Ackermann

Las secciones 1-6 conservan resultados del chasis diferencial. Para el chasis actual (Ackermann, motor trasero de 800 rpm, servo MG90S) manda la
nota del 10-11 nov del principio, [`ACKERMANN.md`](ACKERMANN.md) y [`PENDIENTES_PARQUEO.md`](PENDIENTES_PARQUEO.md). Todo sigue siendo solo simulación:
las medidas del chasis real (dimensiones, servo, encoder, velocidad frente a PWM) están sin tomar.

## 16 nov (tarde) - IT atascado junto a un pilar sin color y parqueo en tiempo real
- **IT sin recuperar el mando** (ccw 40 con `it.track_err_cm 6`): IT cede a menos de 60 cm de un pilar sin color (r6) y el reactivo, con el morro girado y pegado al pilar, no ve su color y hace avanzar/retroceder 25 s. Ahora, si IT lleva `IT_UNK_STALL_S` (3 s) cediendo y el coche no sale de `IT_UNK_STALL_CM` (20 cm), deja de ceder y rodea el pilar con su plan. Esa ronda: 113,7 s -> 85,7 s. Por defecto sin cambios (87,4 s, 62 puntos, 0 contactos); con `track_err` 6 la ronda baja a 83,4 s pero 3 de 48 rondas pierden puntos, asi que sigue sin adoptarse.
- **Parqueo: 1 variante del A* (`PARK_PLAN_VARIANTS 1`)**. Medido: con 5 variantes un parqueo gasta 97-106k nodos (hasta 120.000 en un plan = 5-11 s en el P4); con 1, 5-7k (maximo 16.000, ~1 s en el P4). Calidad igual en 40 rondas de parqueo (18,1-20,2 s en PARKING y 5,7-6,3 repasadas, frente a 20,4-20,8 s y 6,3-6,5), todas completas y sin contactos. Rondas completas: 0 de 48 con 3 y 7 letreros distintas entre 1 y 5 variantes; con 5 letreros 1 ronda mas pierde puntos (ccw 14, limiter), dentro del ruido.
- **Emulacion del P4**: `park_emu_nodes_per_s 15000` hace esperar al simulador `expanded/15000` s (coche parado) tras cada plan. Con 5 variantes el parqueo pasaba de 20,0 a 28,8 s; con 1 variante, de ~20 a 21,7 s.
- **Plan en otra tarea** (`park_external_worker`, `wro_controller_park_worker_step`, en el P4 dentro de `it_worker_task`, nucleo 1): el barrido pide el plan, ordena parar y espera; ya no calcula dentro del barrido (antes el coche seguia con la ultima orden hasta que acababa el A*). El calculo es una funcion pura (`park_compute_plan`). En el simulador: 48 rondas completas con 62 puntos; el parqueo tarda ~0,7 s mas (un barrido de espera por plan). El firmware del P4 compila; **no esta flasheado ni medido en la placa**.
- Contadores nuevos para depurar: `core.parkPlan().plans/planNodes/plansS2`; `park_batch.js` imprime planes por parqueo y la distribucion de nodos por plan.
- Fallos que ya estaban y no son de esto: con 7 letreros 5 de 48 rondas pierden puntos (ccw 5 y 7 `wrong-side` con IT no valido, cw 1 `stalled`, ccw 8 y cw 10 con 124 y 20 contactos), iguales con 1 y 5 variantes.

## 16 nov (noche) - Barrido del P4 bajo 100 ms, memoria de los procesos y fallos con 7 letreros
- **Barrido del controlador 142,6 -> 84,5 ms (p95 92,5)** en el P4: ver `MEDIDAS_HARDWARE_P4.md` seccion 11. Resultado identico en 48 rondas (89,1 s). `local_obstacle_map.c`: composicion de giros en `route_cost`, `footprint_clearance_cs` con descarte exacto (`LOCAL_FP_SHORTCUT`, `LOCAL_ROT_TRIG`, `LOCAL_VERIFY_FP`). Perfil por secciones en el banco (`WRO_PROFILE`).
- **Bug de la emulacion de latencia del A***: tras consumir el plan guardado volvia a esperar; arreglado (`fresh_plan`). Con `park_emu_nodes_per_s` en modo `park_external_worker` tambien espera.
- **Memoria de los procesos de Node ('Fatal process out of memory: Zone')**: cada ronda usa ~96 MB (pico), asi que no es la simulacion; ocurria con la memoria libre justa (8 GB libres y 20 procesos) cuando V8 compila el wasm. `par.js` limita los procesos a 1 por cada 500 MB libres y reintenta hasta 2 veces un proceso caido sin resultado. 24 semillas por direccion: 3 pasadas seguidas sin caidas.
- **Tiempos de vuelta (simulador con los costes medidos del P4)**: IT con trabajador (`it_external_worker`, 250 evaluaciones por barrido), A* del parqueo en otra tarea (`park_external_worker`) y 11.000 nodos/s: vuelta 1 21,6 s, vuelta 2 10,8 s, ronda completa 90,3 s, 62 puntos, 0 contactos en 48 rondas. Sin costes (ideal): 22,9 / 11,8 / 89,1 s (una ronda `limiter`, ccw 14). No es una ronda real en el P4: el P4 no tiene pista.
- **Rondas con 7 letreros (5 de 48 pierden puntos, igual con cualquier variante del A*):**
  - ccw 5 y ccw 7 (`wrong-side`, 25 s): en la vuelta 1 el reactivo pasa por fuera un verde en (10,100) justo antes de otro verde en (-30,80); el generador lo da por factible. Es el reactivo, no el IT (IT aun no existe en la vuelta 1).
  - cw 1 (`stalled`, 36 puntos): IT entrega el coche al parqueo con w=23-24 cm y rumbo -18 grados (el morro hacia el cajon). La pose medida solapa el cajon, `park_nearest_free` no encuentra pose libre en 2 cm y 6 grados y `park_step` vuelve sin dar orden: parado hasta el final. Sin IT la misma ronda aparca bien (111 s). Falta una salida de emergencia para ese caso.
  - ccw 8 (124 contactos) y cw 10 (20 contactos): terminan con 62 puntos tras 142 y 154 s rozando un letrero; sin analizar.

## 16 nov (noche) - El simulador usa por defecto los costes del P4
- `sim.js` constante `P4`: retardo de la orden segun la fase (84,5 ms con el planificador reactivo, 37 ms en PARK_EXIT/PARKING), IT en tarea aparte con 287 evaluaciones por barrido, A* del parqueo en tarea aparte a 11.000 nodos/s con 0,55 s la maniobra de entrada. `WRO_SIM_IDEAL=1` o `p4: false` lo quita. Interruptores de depuracion: `WRO_P4_SCANMS`, `WRO_P4_IT=0`, `WRO_P4_PARK=0`.
- 48 rondas con 5 letreros: ideal 89,1 s y 61,69 puntos; P4 89,3 s (vueltas 22,1 y 10,4 s) y 61,38 puntos (2 `limiter`: ccw 8, cw 1). Aislado: el retardo de 85 ms en todo el parqueo daba 9 `limiter` (el parqueo es sensible a la latencia de la orden); A* y worker solos no pierden puntos (+2,3 s).
- Nueva herramienta `simulator/tools/capture_park_scan.js` (barrido de PARKING a cabecera C) y `perf_park_scan.h` para medir `park_localize_track` en el P4.
