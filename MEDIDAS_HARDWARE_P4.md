# Medidas y confirmaciones de hardware del P4 (16 nov 2026)

Primera sesión con el robot real conectado. Todo lo de esta página se midió con el ESP32-P4 conectado por USB (COM10, ESP-IDF 5.5.4), el motor alimentado con una **fuente de 12,0 V** (medida con multímetro: 12,0 V exactos) y las ruedas en el aire (sin carga). Sustituye a los supuestos de `ESTADO.md` solo en lo que aparece aquí; lo demás sigue siendo supuesto.

Leyenda: **[medido]** lo leyó el firmware o el multímetro; **[confirmado por el usuario]** lo comprobó a mano; **[supuesto]** sin medir.

## 1. Cableado confirmado

| Función | GPIO del P4 | Estado |
|---|---|---|
| LiDAR TX → P4 RX (UART1) | 22 | [medido] llegan datos |
| IMU SDA | 20 | [medido] responde |
| IMU SCL | 21 | [medido] responde |
| Puente H ENB | 26 | [medido] debe estar en alto para que gire |
| Puente H IN3 (motor adelante) | 53 | [medido] |
| Puente H IN4 (motor atrás) | **33** | [medido] |
| Encoder A | 32 | [medido] |
| Encoder B | 36 | [medido] |

- **IN4 en GPIO48 no funciona.** Con el cable de IN4 en GPIO48 el motor no giraba hacia atrás (0 cuentas); al intercambiar los pines por software el fallo seguía a GPIO48, y al mover el cable a GPIO33 funcionó en los dos sentidos. Queda sin determinar si falla el GPIO48 de la placa, ese cable o la entrada IN4 del módulo. No usar GPIO48.
- **GND común.** Al principio el motor no giraba porque faltaba unir el GND del P4 con el del puente H. Es obligatorio.
- **Encoder:** el signo es el correcto sin invertir (adelante = cuentas positivas, atrás = negativas).

En el firmware, el motor usa los canales "IN1/IN2" (adelante/atrás) y el pin ENB como habilitación:
`WRO_MOTOR_IN1_GPIO=53`, `WRO_MOTOR_IN2_GPIO=33`, `WRO_MOTOR_EN_GPIO=26`, `WRO_AXLE_ENCODER_A_GPIO=32`, `WRO_AXLE_ENCODER_B_GPIO=36`.

## 2. Sensores

| Sensor | Resultado | Fuente |
|---|---|---|
| LiDAR WitMotion D6 | Cabecera estable `D5 55 19 31`, **0 errores de checksum**. ≈ 13,8 KB/s, ≈ 171 paquetes/s, ≈ 10 vueltas/s, ≈ 4000 puntos/s. | [medido] 24 s de registro |
| IMU (MPU-6050/6500, 0x68) | ≈ 300 muestras/s, **0 errores**. Sesgo del giroscopio ≈ 1,5 °/s con el carro quieto. Cabeceo ≈ −3,4 °. | [medido] |
| Deriva del rumbo | Con el carro quieto el rumbo pasó de 9,5° a 13° en 25 s: hay que recalibrar el sesgo con el carro inmóvil. | [medido] |
| Encoder del motor | Funciona en cuadratura x4, cuenta en los dos sentidos. | [medido] |

`signo_imu=-1` es solo el valor por defecto; falta comprobarlo con un giro real.

## 3. Motor y encoder

| Dato | Valor | Fuente |
|---|---|---|
| Tensión de alimentación | 12,0 V (fuente, no batería) | [medido] multímetro |
| PWM | 5 kHz, 10 bits (antes 1 kHz) | firmware |
| Velocidad máxima sin carga (PWM 100 %) | **≈ 6190 cuentas/s** estables | [medido] |
| Cuentas por vuelta del eje de salida | **440** (11 pulsos × 4 flancos × reducción 10) | [confirmado por el usuario] girando una vuelta a mano |
| Velocidad máxima en el eje de salida | **≈ 844 rpm** (14,07 vueltas/s) | calculado con 440 |
| Etiqueta del motor | 800 rpm a 12 V | etiqueta |
| Diferencia con la etiqueta | +5,5 % (844 frente a 800) | calculado |
| Velocidad en el suelo sin carga | ≈ 292 cm/s | calculado con rueda provisional de 66 mm |
| Arranque al 100 % | 2850 cuentas/s a 100 ms; ≈ 6000 a 300 ms | [medido] |
| Frenada por inercia | 6230 → 3000 cuentas/s en 0,5 s | [medido] |
| Prueba al 40 % PWM, 1,2 s | adelante +2900 a +2975 cuentas; atrás −2897 | [medido] |

Notas:
- La etiqueta de 800 rpm es el valor nominal; sin carga a 12,0 V el motor da ≈ 844 rpm. **No** se debe a una batería cargada: la fuente es de 12,0 V. Es una tolerancia normal de este tipo de motores.
- `AXLE_ENCODER_COUNTS_PER_REV=440` queda validado. Con ese valor las rpm medidas son coherentes con la etiqueta.
- El repo se actualizó el 16 nov a **800 rpm** (etiqueta) en MIC (`WRO_MIC_RPM_NOLOAD=800`, `mic_cfg_jgb37_520_800rpm`), en el motor físico del simulador (`simulator/web/robot.js`) y en los documentos. Los perfiles de 600 y 1000 rpm se conservan. Las 844 rpm medidas quedan como dato; MIC sigue desactivado (`WRO_USE_MIC` apagado), por lo que no afecta al banco.

## 4. Cambios hechos en el firmware hoy

| Archivo | Cambio |
|---|---|
| `LIDAR System/p4_firmware/sdkconfig.local` | Pines del motor, encoder e IMU/LiDAR; `WRO_ACKERMANN_HW_CONFIGURED=y`; `WRO_BENCH_WEB=y` |
| `main/Kconfig.projbuild` | Nuevos `WRO_MOTOR_EN_GPIO`, `WRO_BENCH_WEB`, `WRO_MOTOR_BENCH_TEST` |
| `main/hw_motors.c`, `hw_motors.h` | ENB bajo al arrancar y alto al mover; PWM a 5 kHz; `hw_motors_raw()` (mando manual del banco); prueba de banco |
| `main/bench_web.c`, `bench_web.h` | Telemetría `$T {json}` cada 100 ms por USB y órdenes `M <-100..100>` / `S`; el motor se para solo tras 600 ms sin orden |
| `main/app_main.c`, `main/CMakeLists.txt` | Arranque del banco |
| `LIDAR System/p4_firmware/banco_p4.html` | Página (Web Serial, Chrome o Edge) con botones ADELANTE, ATRÁS y PARAR, cuentas, velocidad, IMU, LiDAR y gráfica |

- Para aplicar los pines hubo que regenerar `sdkconfig` (`sdkconfig.defaults` + `sdkconfig.local`); la copia anterior quedó en `sdkconfig.bak_antes_motor`.
- Compilar y flashear: `idf.py -D "SDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.local" -p COM10 build flash`.
- La página solo conecta si ningún otro programa tiene COM10 abierto (Arduino IDE, `idf.py monitor`).
- Mientras `WRO_BENCH_WEB` esté activo, el control normal no mueve el motor.

## 5. Pendiente de medir o hacer

- **Servo de dirección**: pin, centro, topes (µs) y ángulos reales. Sin servo el firmware no deja mover el motor en modo normal.
- **Cámara / ESP32-S3**: sin conectar; falta decidir USB o UART y los pines.
- Diámetro real de la rueda (hoy 66 mm provisional) para cm/s exactos.
- ENB: para conducir hay que subirlo al arrancar (hoy solo sube con el banco).
- Calibración de MIC con carga: R, masa, fricción.
- Motor bajo carga (todo lo anterior es sin carga).
- Sesgo del giroscopio con el carro inmóvil y signo del IMU con un giro real.
- Centro del servo (volante 0 = recto) y la asimetría cw/ccw (ver `PENDIENTES_PARQUEO.md`, 3f).
- Causa exacta del fallo de GPIO48.

## 6. Cámara del S3 (ESP32-S3 CAM, OV3660), 16 nov

Medido con el S3 conectado por el CH340 (COM14), firmware `Vision System/` y un ají rojo y otro verde delante. **Sin luz controlada y con la cámara apoyada a mano.**

| Dato | Valor | Fuente |
|---|---|---|
| Fotogramas, firmware de fábrica | 46,6 fps estables, periodo 21,43 ms, 744 tramas CRC correctas, 0 malas | [medido] 16 s |
| Temporización del sensor (QQVGA, binning) | total 2300 x 783 a SYSCLK 84 MHz = 21,44 ms por cuadro; la lectura de las filas ocupa casi todo el cuadro | [medido] registros y cálculo |
| Exposición automática | **en el máximo (784 líneas = 21,4 ms)** con la luz de la habitación, ganancia ~4x | [medido] |
| Brillo (verde, 0 a 63) a ganancia 1 | 16 con 780 líneas, 12 con 400, 8,5 con 200, 5,6 con 100, 3,5 con 50, 2,3 con 25 | [medido] |
| Subir el PLL (SYSCLK 96, 112, 124 MHz, PCLK 8 a 12 MHz) | **no llegan cuadros**: 84 MHz es el techo con este montaje | [medido] |
| Acortar la fila (`total_x`, regs 0x380C/D) | 2100: 51 fps, 1900: 56,5, 1700: cuadros perdidos, **1500: 71,5 fps limpios**, 1400: corrupto, 1200 o menos: sin cuadros | [medido] |
| Firmware real con `WRO_CAM_LINE_TOTAL=1500` | **71,4 fps durante 120 s**, 8570 tramas, 0 malas, 0 huecos de secuencia | [medido] |

Conclusiones:
- Con `total_x=1500` el cuadro dura 14 ms (en vez de 21,4): más fps, menos retraso y menos desfase del obturador rodante.
- Valores de `total_x` probados dentro del firmware: 1700 pierde cuadros de forma periódica y 1400 da cuadros con ruido alto (corruptos); 1500 es el punto limpio. (Una corrupción de imagen que vi en las primeras ráfagas era del enlace serie a 2 Mbaud del laboratorio, no de la cámara: a 921600 con checksum por cuadro llegaron 69 de 69 cuadros correctos.)
- Obturador rodante: lectura de 21,4 ms a 2300 (14 ms a 1500). Con `focal_px` 118,6 y un giro de 90 grados por segundo el desfase de arriba a abajo es de ~4 px a 2300 y ~2,7 px a 1500 (de 160 de ancho). Para detectar bloques de color eso es despreciable.
- El borrón de movimiento lo fija la exposición, no el obturador: con poca luz se queda en 21 ms. Con iluminación de pista normal (mucha más luz) la exposición baja sola.
- El ají (brillante) se parte en 2 o 3 componentes y el número de objetos por cuadro oscila; los bloques mate de la pista deberían ir mejor, pero **no se probó con bloques reales**.
- `Vision System/main/cam_lab.c` (opción `WRO_CAM_LAB`, apagada por defecto) es el laboratorio con el que se hicieron estas mediciones.

## 7. Simulador con el motor de 800 rpm (16 nov)

**Una sola fuente de rpm:** `WRO.robot.MOTOR_RPM = 800` (`simulator/web/robot.js`). La usan el motor físico del simulador y el modelo de MIC del wasm (`core_set_motor_rpm`). Antes el motor físico asumía 1000 rpm.

**Límites atados al motor:** `sim.js` toma del modelo de MIC la velocidad máxima (`core.micVMax()`: 142 cm/s a 600 rpm, 189 a 800) y la aceleración (`core.micMaxAccel()` al 50 % de la velocidad máxima: 300 cm/s2). Con ellos fija `it.v_max_cm_s` e `it.a_motor_cm_s2` del planificador IT y el `maxAccel`/`maxSpeed` de la planta cinemática de los chasis `largo26` y `largo26d` (marcados `motorModel`). Antes eran 150 cm/s y 350 cm/s2 sueltos en varios archivos. Quien pase esos valores a mano los sustituye. Herramienta de medida: `node simulator/tools/lap_times.js 5,6,7 ccw,cw 3`.

A 90 % de la velocidad máxima la aceleración del motor cae a 140 cm/s2 (600 rpm) y 68 cm/s2 (800 rpm).

Medias de las 4 rondas factibles (semillas 5 ccw/cw, 6 cw, 7 cw; 3 letreros y parqueo), en segundos. La vuelta 1 es de exploración (19,9 s) y es igual en todas:

| Variante | Vuelta 2 | Vuelta 3 | Ronda total |
|---|---|---|---|
| Sin IT (referencia) | 22,7 | 25,4 | 103,0 |
| ANTES: cinemática, IT 150 cm/s y 350 cm/s2 fijos | 9,9 | 11,4 | 80,9 |
| DESPUÉS: cinemática, límites del modelo de 800 rpm | 11,4 | 10,8 | 79,7 |
| MIC con motor de 600 rpm, límites del motor | 10,5 | 11,4 | 77,3 |
| MIC con motor de 800 rpm, límites del motor | 10,8 | 10,2 | 75,2 |

Conclusión: pasar de 600 a 800 rpm acorta la ronda unos 2 s (3 %) y la vuelta 3 en 1,2 s, pero con 4 rondas caóticas eso no es una mejora comprobable. La velocidad que alcanza el IT sube de 142 a 154 a 171 cm/s, sin acortar la vuelta: la limitan el agarre y el seguimiento de la trayectoria. Las semillas 6 ccw y 7 ccw terminan en "wrong-side" en todas las variantes (no tiene que ver con el motor) y se excluyen.

**Test de IT (`test_it_mission.js`):** con los límites atados, la ronda ccw 5 cedía el mando 2 veces en vez de 1. Medí el motivo (`it_drop_reason`): la primera cesión es el motivo 6, un pilar de color aún desconocido cerca de la trayectoria (el IT lo deja al controlador reactivo y sigue), no un fallo de seguimiento; la ronda acaba con 62 puntos y 0 contactos. En las otras 3 rondas solo hay la cesión prevista del final (motivo 0). El test ahora exige exactamente una cesión de fin previsto y solo admite antes el motivo 6; seguimiento perdido, arena inválida, arco o atasco siguen siendo fallo. No se cambió el código del IT. Los 9 tests del simulador pasan. Nota: el manifiesto del wasm incluye `Vision System/main`, así que hay que reconstruirlo (`simulator/wasm/build.ps1`) cada vez que cambie algo ahí.

Sin atar al motor: `race.v_max_cm_s` (52 cm/s, tope de la línea de carrera sin IT, ni se alcanza: 34 cm/s), `speedTauS` y `speedGain` de la planta (respuesta del lazo cerrado de MIC) y los valores por defecto en C del firmware del P4 (150 y 350).

Prueba con movimiento (ají agitado a mano a ~20-30 cm, ráfagas de 4 cuadros, checksum correcto en todos): hay **borrón de movimiento** marcado (la exposición está en el máximo: 21 ms a `total_x=2300` y 14 ms a 1500) y no se aprecia un desfase del obturador rodante que destaque sobre el borrón. Lo que hay que atacar es la exposición (más luz), no cambiar a global shutter.

## 8. Rendimiento del P4 con el C real del robot (16 nov)

Medido con `LIDAR System/p4_firmware/main/perf_bench.c` (opción `CONFIG_WRO_PERF_BENCH`, apagada por defecto; compilar con `sdkconfig.perf` en otro directorio de compilación):
ESP32-P4 a 360 MHz, ESP-IDF 5.5.4, PSRAM de 29 MB, optimización `-O2` (la de fábrica es `-Og` con aserciones: solo un 17 % más lenta). Datos sintéticos (la pista de `plano_it_ovalo.json`, 5 letreros; un LiDAR simulado de 400 puntos por vuelta). No mueve el motor. Memoria interna libre: 302 KB, PSRAM: 28,9 MB libres.

| Qué | P4 (360 MHz) | PC (para comparar) | Presupuesto |
|---|---|---|---|
| Controlador completo, un barrido (sin parqueo) | **160 ms** (p95 168; el primero, con `arena_acquire`, 2,58 s) | ~1,5 ms en wasm | 100 ms (10 barridos/s) |
| IT: una evaluación del coste | **1,5 ms** | 35 us (43 veces menos) | |
| IT: plan completo (11.961 evaluaciones) | **18 s** | 0,4 s | |
| IT: una rebanada de 300 evaluaciones (como el controlador) | **393 ms** (máx. 2,4 s: la preparación del corredor) | ~10 ms | unos pocos ms |
| IT: refinado de 20.000 evaluaciones en rebanadas de 800 | 1,06 s por rebanada, 28 s en total (el plan baja de 7,839 a 7,798 s) | | |
| IT: seguidor (`it_track`) | **8,7 us** | | despreciable |
| Parqueo: A* de entrada (5 variantes, hasta 24.000 nodos cada una) | **11,5 a 27,5 s por plan** (120.000 nodos expandidos sin plan en una pose, 27,5 s) | 0,30 a 0,74 s | un plan por arco |
| Parqueo: maniobra de tres tramos | 1,7 s (no encontró solución en esa pose) | 47 ms | |

**Pila.** La tarea principal (`app_main`) tiene 6144 bytes de pila (`CONFIG_ESP_MAIN_TASK_STACK_SIZE`) y el bucle del control corre en ella. Con ese tamaño el controlador cayó con *Stack protection fault* en su 7.o barrido. En una tarea de 32 KB usó unos 6,3 KB como mínimo (quedaron libres 26.476 bytes). Hace falta subir la pila (12288 sería prudente) antes de ejecutar el controlador en el robot.

Lectura sin adornos:
- **El P4 es unas 40 veces más lento que el PC en este código**, y el simulador no lo modela (en el wasm el plan es instantáneo). Con los tiempos de hoy el robot **no puede** hacer el parqueo en tiempo real: cada replan del A* son 12 a 27 s con el coche quieto, y un escaneo del controlador (160 ms) ya supera los 100 ms.
- El IT no puede construirse en el bucle de control: una rebanada de 300 evaluaciones son 393 ms. Haría falta una tarea aparte (el P4 tiene dos núcleos), menos estaciones, menos evaluaciones o una evaluación más barata.
- Lo más barato de arreglar: pasar el A* a una sola variante (`PARK_PLAN_VARIANTS 1`, 5 veces menos), bajar `MAX_NODES`, y repartir el cálculo en una tarea de baja prioridad en el segundo núcleo.
- Todo esto es con datos sintéticos y un solo recorrido de prueba: los tiempos reales del parqueo dependen de la pose. La carga de la CPU con el LiDAR, la IMU, la cámara y el enlace activos no se midió.

## 9. Tras optimizar (16 nov, misma noche)

Mismo banco (`perf_bench.c`), mismo P4 (360 MHz, `-O2` global y `-O3` sin errno en los archivos calientes). **Los planes salen iguales**: 7,838 s en la pista de prueba (IT) y los mismos pasos del A* (26, 18 y 12).

| Qué | Antes (seccion 8) | Ahora |
|---|---|---|
| IT: plan completo (11.961 evaluaciones) | 18,0 s | **4,2 s** |
| IT: una evaluacion | 1,5 ms | **0,35 ms** |
| IT: rebanada de 300 evaluaciones | 393 ms | **54 ms** (la primera, con 6 relajaciones, 2,0 s) |
| IT: refinado de 20.000 evaluaciones | 28 s (1,06 s por rebanada) | ~6 s (0,23 s por rebanada de 800) |
| Parqueo: A* de entrada (5 variantes) | 11,5 a 27,5 s | **4,8 a 11,1 s** |
| Parqueo: un arco (6 subpasos con colision) | | 9,9 us |
| Primer barrido del controlador | 2,58 s | 0,90 s |
| Barrido del controlador | 160 ms | 142 ms (presupuesto 100) |

Que se hizo (todo conserva el resultado): evaluacion **incremental** del coste del IT (solo se recalculan las ~30 estaciones que mueve cada candidato; comprobada contra la completa: error 0 en 31.952 casos con `IT_VERIFY_INCREMENTAL`), matematica barata (`sqrtf` en vez de `hypotf`, `atanf` de Cephes, barrido de velocidad en v^2, descarte exacto de obstaculos lejanos), A* sin llamadas a `sinf/cosf` por subpaso (composicion de giros), colision con los ejes del chasis conocidos, tabla de celdas en 16 bits, `-O3`/`-fno-math-errno` en los archivos calientes, pila de `app_main` a 12 KB y el plan del IT en una tarea del nucleo 1 (`it_external_worker`, `wro_controller_it_worker_step`).

Lo que queda: el A* sigue tardando **5 a 11 s por plan** (el limite de 24.000 nodos por variante se alcanza en las cinco), asi que el parqueo no es de tiempo real. Las salidas que cambian el algoritmo (tabla de politica precalculada, planificar durante la aproximacion para la pose esperada, o 1 variante) estan sin decidir. El barrido del controlador (142 ms) sigue por encima de 100 ms.

## 10. Parqueo con 1 variante del A* (16 nov, banco en el P4)

| Pose de partida | Nodos | Tiempo (antes, 5 variantes) | Tiempo (1 variante) |
|---|---|---|---|
| (25, 30, 5) sin plan, llega al limite | 24.000 | 4,8 a 11,1 s | **2,21 s** |
| (-30, 45, 0) | 17.775 | | **1,60 s** |
| (0, 28, 35) | 2.898 | | **0,26 s** |
| (20, 30, 20) | 427 | | **0,044 s** |

El A* expande ~11.000 nodos/s en el P4 (el simulador emula 15.000 con `park_emu_nodes_per_s`). En el simulador un parqueo medio gasta 5-7k nodos (maximo 16.000), unos 0,5 s de calculo en total; un plan sin solucion cuesta 2,2 s y se repite con paso de 2,5 cm (4,4 s). El plan va en la tarea del nucleo 1 (`park_external_worker`) con el coche parado. Sin cambios: barrido del controlador 142,6 ms (p95 150 ms), plan del IT 4,17 s, pila libre de la tarea del banco 26 KB.

## 11. Barrido del controlador: de 142 a 84 ms (16 nov, banco con perfil por secciones)

`CONFIG_WRO_PERF_BENCH` compila ahora `wro_controller.c` y `local_obstacle_map.c` con `WRO_PROFILE` (macros `PROF()`; acumula microsegundos por seccion del barrido).

| Seccion del barrido | Antes (ms por barrido) | Despues |
|---|---|---|
| Planificador reactivo (`local_obstacle_map_recommend_cfg`) | **131,2** | **73,1** |
| &nbsp;&nbsp;campo de distancias / corredor | 3,8 / 2,2 | 3,9 / 2,2 |
| trace_map | 4,2 | 4,2 |
| mapa local, analyze, compensacion | 1,7 / 1,8 / 1,0 | igual |
| **Barrido completo p50 / p95** | **142,6 / 150,6 ms** | **84,5 / 92,5 ms** |

El 90 % del barrido era la rejilla de 343 rutas (7^3) del planificador reactivo: cada paso de cada ruta recorria 84 muestras de la huella y llamaba 4 veces a `sinf/cosf`. Cambios, sin alterar el resultado: seno y coseno por composicion de giros (los obtenia de la libm en cada paso) y descarte exacto de la huella con el campo de distancias (si el centro esta a mas de lo que mide la huella en el metrico chamfer mas la preferida, no se recorren las muestras; se desactiva cuando las cortinas han retocado la tabla). Comprobado con `LOCAL_VERIFY_FP` sobre 1,6 millones de consultas (0 fallos) y con 48 rondas completas identicas a antes (89,1 s, 61,69 puntos). El primer barrido sigue tardando ~0,84 s (adquisicion del localizador). Queda margen en esa rejilla (compartir prefijos del arbol de rutas: ~-25 % de pasos).

## 12. Costes que el simulador aplica por defecto (16 nov)

`simulator/web/sim.js` (constante `P4`) usa lo medido en la placa; `WRO_SIM_IDEAL=1` (o la opcion `p4: false`) lo desactiva.

| Coste | Valor | Fuente |
|---|---|---|
| Barrido con el planificador reactivo (RUN, PARK_SEARCH, ...) | 84,5 ms (sigma 4,8: p95 92,5) | seccion 11 |
| Barrido en PARK_EXIT / PARKING (sin planificador reactivo) | 37 ms = 9 de mapa y analisis + **26,8** del seguimiento del cajon (`park_localize_track`, barrido real de 400 puntos, p50) | banco, `perf_park_scan.h` |
| Plan del IT (tarea del nucleo 1) | 0,3485 ms por evaluacion = 287 por barrido de 100 ms | seccion 8 |
| A* del parqueo (tarea del nucleo 1, coche parado) | 11.000 nodos/s | seccion 10 |
| Maniobra de entrada (`park_ackermann_entry_maneuver`) | 0,55 s (0,31 a 1,10 s en 6 poses; en el banco no encuentra plan en ninguna) | banco |

La orden llega `cmdDelayMs` (12) + el coste del barrido despues de recibir el barrido. El primer barrido (0,84 s) no se aplica: el P4 procesa barridos antes de pulsar el boton de salida.
Con esos costes, 48 rondas (5 letreros): 89,3 s (vuelta 1 22,1 s, vuelta 2 10,4 s), 61,38 puntos; ideal: 89,1 s, 61,69. Con 85 ms de retardo tambien en el parqueo (error, pues alli el barrido cuesta 37 ms) salian 9 rondas con `limiter`: el parqueo es sensible al retardo de la orden.
