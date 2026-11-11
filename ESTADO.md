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
> - **Aviso de hardware:** el lazo abierto de `hw_motors.c` usa `WRO_MOTOR_MAX_CM_S = 40`; con el motor de 600 rpm el 100 % de ciclo son ~157 cm/s (~270 con el de 1000). No activar
>   los motores sin MIC o sin recalibrar ese valor. **Motor (11 nov):** hoy se asume el de 600 rpm en todo el repo (MIC, Kconfig, simulador); el definitivo
>   será de 1000 rpm y se cambia con `mic_cfg_jgb37_520_1000rpm()` o `WRO_MIC_RPM_NOLOAD`.
> - Nada de esto está probado en el coche real.
> - **11 nov (revisión del simulador).** El simulador abría con IT, cajón y arranque en el cajón APAGADOS: ahora los tres vienen marcados. Cambios: (1) el sorteo `feasible` reponía mal los
>   letreros (3 de 6 era lo normal): ahora prueba otros asientos y coloca ~5,5 de 6 (el aviso explica por qué faltan); `refill: false` conserva el sorteo anterior. (2) La pose estimada se dibuja
>   adelantada por el retraso del barrido (antes iba detrás: la pose es la del instante del barrido, que llega ~procDelay después). (3) El parqueo para en cuanto la pose estimada cumple la meta
>   (antes terminaba el plan entero: hasta 12 s y 9 repasadas con el juez ya dando 'completo'): 105 repasadas en 24 rondas (antes 122), 24 completas, 0 contactos. (4) El juez espera 3 s
>   (antes 15) tras aparcar y el tiempo de la ronda es el del instante en que se detuvo. (5) IT lleva todo menos la última esquina (`it_stop_before_end` 1; con 0 el parqueo falla): ~3 s menos.
>   **Efecto secundario:** con ~5,5 letreros reales aparecen fallos que el sorteo escaso ocultaba: reactivo solo 21 de 24 (3 atascos contra un letrero en un paso de 27 cm: escape ↔ avance sin salida);
>   completo (IT+MIC) 24 de 24 terminan, 22 con 62 puntos y 2 con 44. Sin resolver.
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

Las secciones 1-6 conservan resultados del chasis diferencial. Para el chasis actual (Ackermann, motor trasero de 600 rpm, servo MG90S) manda la
nota del 10-11 nov del principio, [`ACKERMANN.md`](ACKERMANN.md) y [`PENDIENTES_PARQUEO.md`](PENDIENTES_PARQUEO.md). Todo sigue siendo solo simulación:
las medidas del chasis real (dimensiones, servo, encoder, velocidad frente a PWM) están sin tomar.
