# ESTADO del proyecto (2026-10-13)

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

El robot (chasis de dos ruedas con encoders ópticos, ESP32-P4 + ESP32-S3 CAM) ya navega, esquiva letreros, completa 3 vueltas, sale del cajón y aparca **en simulación**, usando encoders, giroscopio, LiDAR y cámara. El simulador ejecuta el mismo C del firmware (WebAssembly). Todo el pase de pruebas está en verde.

Lo que impide pasar al robot real es el hardware y los drivers del P4 (sección 5).

## 2. Resultados verificados

| Prueba | Resultado |
| --- | --- |
| `Vision System/tools/simulate.ps1` (pruebas en C, simulador, 20 rondas, depurador, visión) | Todo en verde; regenera `simulator/build/simulador.html` (marca `13/10 10:40`, fw `07c0fed`) |
| `simulator/tests/plano_errores.json` (plano del equipo) | 62 puntos, parqueo completo, 24/24 secciones, 0 contactos, 177 s |
| Banco de 100 pistas, semillas 21–70 (ajuste) | 99 perfectas, 0 contactos |
| Banco de 100 pistas, semillas 71–120 (validación) | 95 perfectas, 0 contactos |
| Velocidad | 23 cm/s (mediana 143 s, máximo 159 s de 180 s) |

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
- Dos cambios de lado seguidos más rápidos de lo que permite el radio de giro no tienen trayectoria; son un problema del plano (`field.checkLayout` los avisa).
- Todos los datos de hardware (rueda, vía, encoder, montaje de LiDAR/cámara, ganancia de velocidad) son supuestos hasta medirlos.
- La interfaz del simulador se probó con un DOM simulado, no visualmente en un navegador.

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

## 6. Panel web, enlace P4-S3 y mapa (rama integracion-panel)

**Inicio:** el robot solo arranca con el boton fisico (GPIO5). La pagina web solo puede PARAR (`WRO_WEB_START_ENABLED=n`).
El signo del giroscopio no se cambia solo con el robot armado.

**Tamano del mapa del cuadrilatero** (campo: cuadrado exterior 300 cm, bloque interior 100 cm, carril 100 cm):
- El mapa es del robot: origen = donde arranca, norte = hacia donde miraba. El campo ocupa una caja de 300 x 300 cm.
- Con el robot en el centro del carril de salida (a 100 cm del centro del campo): la caja va de -50 a +250 cm en el eje que cruza el carril y de -150 a +150 cm en el otro. Si arranca desplazado hasta +-50 cm a lo largo del carril: peor caso 400 x 300 cm (-200..+200 por -50..+250).
- Alcance: la esquina mas lejana queda a sqrt(150^2 + 250^2) = 292 cm del arranque, menos que MAX_RANGE_CM (500) de `map_pose.c`.
- Pagina: 300 x 300 celdas de 2 cm = 6 x 6 m (antes 4 cm = 12 x 12 m), unas 22 500 celdas ocupadas del campo (150 x 150) de 90 000. 90 KB por nivel en el navegador.
- Enlace: ~406 puntos por vuelta (limite 512), trama de estado ~1,8 KB a 8 Hz = ~15 KB/s de los 92 KB/s de la UART a 921600.

**Pendiente (necesita el robot / la mano):** girar el robot a mano para comprobar rumbo y signo del giroscopio; flashear el S3 (`S3 Robot`, ya compila con IDF 5.5.4) con su red WiFi en `S3 Robot/main/wifi_secrets.h`; motores y encoders al final.
# Estado actual de la adaptación Ackermann

El estado detallado que sigue conserva resultados históricos del chasis diferencial;
no acredita el funcionamiento del vehículo actual. El simulador usa por defecto
un chasis Ackermann provisional y encoder de eje. Se verificaron seis salidas de
parqueo aisladas, no la misión completa ni la entrada al cajón. Una salida tardó
55,21 s simulados. La prueba histórica de parqueo diferencial se detiene en la
salida; `test_missions.js` no está íntegramente en verde. No usar estas cifras
como validación del chasis real, cuyas medidas aún faltan.
