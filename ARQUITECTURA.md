# Arquitectura y flujo de AutoNex

De los sensores a los motores, en el orden en que ocurre. Las referencias son
`archivo:línea` del estado actual del repositorio (`9d6b59a`). Lo que no está
medido se dice: **medido** = hay un número en el repo o en `ESTADO.md`;
**supuesto** = es un valor de configuración sin comprobar en el banco.

## 1. Vista de pájaro

Dos placas y un simulador que ejecuta **el firmware real** (WASM):

```
        ┌─────────────────── ESP32-S3 ───────────────────┐
        │ OV2640/3660 RGB565 160x120 ─ vision.c ─ tabla  │
        │ (CRCs, 12 objetos, filas naranja/azul)         │
        └───────────────────────┬────────────────────────┘
                                │ enlace serie: trama A5 5A + CRC16
        ┌───────────────────────▼────────────────────────┐
        │ ESP32-P4  (un solo bucle cooperativo ~1 ms)     │
        │  LiDAR D6 230400 ─┐                             │
        │  IMU MPU-6050 ────┤ mapa local + localizadores  │
        │  encoder de eje ──┘ → wro_controller_scan (10Hz)│
        │                     → orden (v, volante, ω)     │
        │  servo (LEDC 50 Hz) ← PWM 1 kHz ← motor         │
        └─────────────────────────────────────────────────┘
```

El robot es **Ackermann con un solo motor** y un servo de dirección: `hw_motors_drive`
ignora la guiñada y mueve sólo el canal izquierdo (`hw_motors.c:113-121`); IN3/IN4
existen pero no se escriben y la mezcla diferencial `hw_motors_wheel_speeds` no se
llama nunca (`hw_motors.c:105-111`).

El simulador (`simulator/web/`) instancia el mismo `wro_core.wasm` y le entrega
barridos, tablas de visión e IMU con ruido y latencia; no es un gemelo digital
aparte: es el mismo C.

## 2. De los sensores al estado

Todo se refresca **por barrido del LiDAR (~10 Hz)**: no hay fusión asíncrona ni
filtro de estado global. En cada barrido se rehace el mapa, se re-analiza la
percepción y se emite **una** orden.

| Sensor | Cómo llega | Qué produce | Dónde | Tasa / unidades |
| --- | --- | --- | --- | --- |
| LiDAR D6 | UART1 230400, tramas `AA 55 …` | `local_lidar_point_t{angle_deg, distance_cm, valid, blind, intensity}` | `hw_lidar.c:52-103` | 1 vuelta ≈ 10 Hz (**supuesto**); grados, cm |
| Cámara S3 | RGB565 QQVGA → `vision_process` → tabla | hasta 12 objetos (color, `center_x`, profundidad, flags) + filas de línea | `vision.c:342-651`, `vision_table.c:26-47` | fps real desconocido (**sólo log**), 230400 bps |
| Enlace S3→P4 | trama `A5 5A | ver | seq | ts | … | CRC16` | `camera_table_t` | `sensor_table_protocol.c:15-49`, `camera_table_receiver.c:82-126` | 1 tabla por cuadro; latencia **no medida** |
| IMU | I²C, muestreo 2,5 ms | `yaw_deg` integrado, `rate_dps` | `hw_imu.c:113-132`, `imu_yaw.c:39-59` | 400 Hz de muestreo; grados (horario +) |
| Encoder de eje | PCNT, 1 lectura/ms | `scan_ds_cm`, `v_cm_s` (filtro τ=50 ms) | `hw_encoders.c:25-78`, `odometry.c:140-180` | cm, cm/s; **desactivado por defecto** hasta calibrar |
| Odometría | `odom_*` | posición/rumbo **no usados** por el control (sólo `scan_ds_cm`) | `odometry.c:71-76`, `wro_controller.c:1763` | cm |

Puntos que sorprenden y están en el código:

- `blind` **nunca** se pone en hardware: los retornos a 0 mm o > 12 m se descartan
  (`hw_lidar.c:92-101`), así que la rama de zona ciega del mapa sólo existe en el
  simulador. Un obstáculo pegado al morro simplemente **no aparece**.
- `intensity` se rellena y no lo lee nadie; no hay filtro por reflectividad.
- Las **filas naranja/azul** de la cámara se decodifican (`camera_table_receiver.c:50-51`)
  y ningún consumidor del P4 las usa: el canal de "línea de suelo" está implementado
  y desconectado (la "línea de carrera" es una trayectoria calculada).
- El P4 no valida `sequence` ni `frame_timestamp_ms`: una tabla perdida no se nota
  (`camera_table_receiver.c:44-47`).
- La focal del decodificador no se configura en el P4 (sólo el simulador la fija,
  `core_api.c:199-203`): el P4 asume 118,6 px (`camera_table_receiver.c:6`).

## 3. Percepción y fusión

| Bloque | Qué hace | Dónde | Salida |
| --- | --- | --- | --- |
| Mapa local | rejilla 160×160 de 2,5 cm (4×4 m) reconstruida entera en cada barrido; raycast Bresenham, `bridge` entre retornos, deskskew por guiñada, objetos de cámara anclados al retorno LiDAR compacto | `local_obstacle_map.c:463-694` | `cells[]`, 12 `local_colored_object_t` |
| Objetos/letreros | asociación angular 5° y de alcance ±50 % ±12 cm; confirmación por LiDAR; caducidad a 150 cm | `local_obstacle_map.c:586-650`, `wro_controller.c:632-685` | `c->signs[12]` |
| Percepción de carril | mediana de planos ±45° (frontal) y ±75..105° (laterales), prueba de planaridad | `wro_controller.c:466-511` | `perception_t{front_cm,left,right,wall_align}` |
| Pose del campo | `arena_acquire` (rejilla θ±8° sobre 8 paredes + Gauss-Newton) y `arena_update` (Huber, 3 iter) con rechazo >30 cm/8° | `arena_localizer.c:197-286` | `arena_pose_t{cm, grados}` |
| Pose del cajón | `park_localize_acquire` (pared + `u` + topes) y `park_localize_track` (búsqueda local 2 cm/2°) | `park_localize.c:20-156` | `park_fix_t{pose(u,w,θ), score}` |
| Línea de carrera | una vez por ronda: centro por estaciones de 5 cm, cortina del lado prohibido, DP + sobrerrelajación, perfil `v ≤ sqrt(a_lat/κ)` | `race_line.c:63-405` | 192 estaciones (`x,z,s,κ,heading,v`) |

## 4. Decisión

### 4.1 Máquina de fases (`wro_controller.c`)

Punto de entrada: **`wro_controller_scan`** (`wro_controller.c:1756`), no existe
`wro_controller_step`.

| Fase | Entra en | Sale a | Qué ordena |
| --- | --- | --- | --- |
| IDLE | `init` | `start` | STOP |
| RUN | `start`, fin de UTURN/PARK_EXIT | 3 vueltas → PARK_SEARCH/FINISH | retícula + esquinas + recuperaciones |
| UTURN | media vuelta armada (regla 2024) | giro terminado | `uturn_step` |
| FINISH | sin parqueo o búsqueda agotada | pared interior + `finish_extra` | retícula |
| PARK_SEARCH | demo o fin de vueltas | `park_localize_acquire` | carril pegado a la pared exterior a 12 cm/s |
| PARKING | cajón localizado | meta confirmada 3 barridos | `park_step` (ver 4.3) |
| PARK_EXIT | `start_in_lot` | meta confirmada 3 barridos | `park_step` (`exiting`) |
| DONE | cualquiera de las anteriores | terminal | STOP |
| FAULT | **nadie la asigna** | — | código muerto |

### 4.2 Planificador local (`local_obstacle_map.c`)

- Rejilla 2,5 cm y campo de distancias (chamfer) → **343 rutas** de 3 arcos de 20 cm
  (7 acciones × 7 × 7), puntuadas por holgura, desvío lateral, rumbo, curvatura,
  cambio de curvatura, avance y "paso" (`:1149-1246`).
- Histéresis en tres capas: banda de empate 2, preferencia por la acción anterior y,
  en empate, el giro más suave (`:1234-1246`).
- Después: recentrado a línea, corredor (DP lateral 36×97) y seguimiento fino
  (`:958-1088`, `:1350-1457`).
- Si no hay ruta: maniobra de escape y, si se agota, parada (`:1472-1691`).
- Velocidad de crucero 23 cm/s, maniobra 10 cm/s (`local_obstacle_map.c:43-46`).

### 4.3 Parqueo (`park_ackermann.c` + `park_step`)

A\* de poses: rejilla 121×71×36 (1 cm, 10°), primitivas `dir ∈ {−1,+1}` ×
`steer ∈ {−1,0,+1}` de 3,5 cm, coste `3,5 cm (1,1× en reversa) + 12 cm por cambio de
sentido`, heurística `2,5·distancia + 0,12·|θ|` (no admisible), presupuesto 24 000
nodos. Meta: entrar con las cuatro esquinas dentro del cajón (0,5 cm de holgura) y
|θ| ≤ 4° (`PARK_GOAL_TH_IN_DEG`; el juez da *completo* hasta 6,2°); salir con `w ≥ ancho_cajón + ancho/2 + 2` y |θ| ≤ 10°.

En `park_step` hay **un** camino para salir y **cuatro** para entrar, en este orden:

1. **Entrada simétrica**: crucero por el carril a `max(park_speed, park_search)`
   (12 cm/s) hasta 2 cm por delante y 1,5 cm más lejos de la pared que la pose donde
   terminó la salida, y allí **un solo plan** del A\*. Es el camino que hoy usa la
   ronda normal.
2. Maniobra de tres tramos (sólo desde el carril, `w > ancho−1`). **Hoy no se acepta
   nunca**: el patrón de volante estaba invertido (el giro es
   `dir·largo·tan(volante)/batalla`, así que al alternar el sentido el volante debe ir
   **igual**) y aun corrigiéndolo no encuentra solución. Código muerto en la práctica.
3. A\* de arcos cortos con la meta de entrar.
4. Campo de costes (Dijkstra inverso) sólo para el chasis diferencial.

Cambios del 10 nov: (a) **coste suave de holgura** en el A\* de entrada (`PARK_CLEAR_PENALTY` 1,0 por subpaso a menos de margen + 1,5 cm de un obstáculo):
las soluciones con ≥ 1,3 cm de holgura real se ejecutan enteras y las que rozan el 1 cm de planeo se abortan por el error de pose; (b) en la entrada simétrica el
chequeo de arco usa `min(margen del plan, 0,5 cm)` (si no, con el coche a < 1 cm de un obstáculo el plan se hacía con margen 0 y se rechazaba cada barrido para
siempre); (c) flags experimentales **apagados**: `PARK_RETRACE_ENABLED` (entrar siguiendo al revés la traza real de la salida), `PARK_TWIN_ENABLED` (gemelo como
filtro de pose), `PARK_PROJECT_ENABLED` y `park_exact_moves` (cada arco como avance exacto de MIC). Ver `PENDIENTES_PARQUEO.md` para los resultados.

Ejecución de los arcos (`wro_controller.c:1440-1510`): el arco se corta por
**distancia** (0,3 cm) o, al entrar, por **rumbo del giroscopio** (el giro exacto del
plan); antes de mover se da el volante **con el coche quieto** (1 barrido si el giro
es pequeño, 2 si es una inversión completa) porque el servo tarda ~0,2 s en pasar de
un extremo al otro; velocidad 8 cm/s con rampa sólo en el último centímetro.

### 4.4 Maniobras especiales

| Maniobra | Cuándo | Dónde |
| --- | --- | --- |
| Pivot (giro en el sitio) | 2 s sin ruta y rumbo desviado ≥15°, con hueco para girar | `wro_controller.c:1716-1744` |
| Sidestep (cambio de lado) | atascado o cambio de lado previsto | `wro_controller.c:1619-1711` (apagado por defecto) |
| Peek (asomarse) | pilar sin color entre 26° y 70° | `wro_controller.c:1996-2010` (apagado) |
| Media vuelta (K-turn) | regla 2024, color de fin de ronda | `wro_controller.c:588-626` (apagada) |

## 5. De la orden a los motores

`wro_command_t` (`wro_controller.h:161-183`): `steer_deg` (derecha +),
`speed_cm_s` (negativo = reversa), `yaw_rate_dps` (horario +),
`explicit_yaw_rate` (la guiñada **no** se deriva de v y volante), `action`, `phase` y
telemetría.

Desde el 10 nov lleva además `move_cm` y `move_id` (avance exacto pedido a MIC; 0 si no hay): solo se rellenan con `park_exact_moves 1`.

```
wro_controller_scan → wro_command_t        wro_controller.c:1756-2032
   → p4_runtime_step (1 ms)                p4_runtime.c:16-57
      ├─ giroscopio (drena muestras)       :23-28
      ├─ tabla de cámara (128 B/ms)        :29-37
      ├─ UN barrido nuevo → wro_controller_scan   :42-46
      ├─ guardián (sin barrido o sin giro → ceros) :51-53, :59-70
      └─ hal.drive(steer, speed, yaw)      :54-56   ← sólo con barrido nuevo (10 Hz)
           → hw_steering_set (LEDC 50 Hz, 16 bits)  hw_steering.c:38-49
           → hw_motors_drive (PWM 1 kHz, 10 bits)   hw_motors.c:113-121
                frac = |v| / MOTOR_MAX_CM_S (40)     :45
           con CONFIG_WRO_USE_MIC=y: hw_motors_drive solo cambia la orden y un temporizador de 1 kHz ejecuta mic_update() con las cuentas del encoder
```

Detalles: el servo se escribe **de golpe** en cada barrido (no hay rampa en
hardware; la rampa de ~0,2 s es física); el motor recibe `frac` instantánea, sin zona
muerta ni ganancia; si `WRO_MOTORS_ENABLED` o la calibración del servo están a `n`,
no se mueve nada (`hw_motors.c:116-119`, `hw_steering.c:17-24`).

## 6. Tiempos

| Etapa | Periodo | Nota |
| --- | --- | --- |
| Bucle del P4 | ~1 ms (`vTaskDelay(1)`) | un solo hilo cooperativo |
| `wro_controller_scan` | ~100 ms (1 por barrido) | **toda** la decisión vive aquí |
| Actuadores | 100 ms | sólo se reescriben con barrido nuevo |
| IMU | 2,5 ms de muestreo / 400 Hz | se integra a 100 Hz |
| Cámara S3 | desconocido (log `PERF`) | tabla por cuadro, 230400 bps |
| Watchdog | 300 ms (`sensor_timeout_us`) | sin barrido o sin giro → orden a cero |
| `latency_s` | 0,1 s | compensación de retardo usada al adelantar el mapa |

## 7. El simulador frente al hardware

El simulador ejecuta el mismo controlador, pero el mundo avanza con `dt = 0,01 s`,
`procDelayMs = 8` y `cmdDelayMs = 12`, y el chasis es un modelo de bicicleta con servo
de primer orden (`steerRateDegS 420`, `steerTauS 0,045`), motor con `speedTauS 0,16`,
`maxAccel 180`, `speedGain 0,94`, patinaje en ráfagas y colisión con deslizamiento
(`simulator/web/robot.js:17-141`).

Diferencias que pueden explicar que algo funcione en el simulador y no en el robot:

1. **Actuadores a saltos de 100 ms** en hardware contra servo continuo en el sim, con
   12 ms de retardo de orden (`sim.js:169-171`). El sim *suaviza* lo que el robot
   hace a escalones.
2. **Servo**: en hardware el tope sale de `CONFIG_WRO_SERVO_*_DEG` (default 0 ⇒
   tracción bloqueada) y no hay rampa; en el sim 40° fijos y rampa.
3. **Un motor**: el sim tiene mezcla diferencial en el perfil diferencial
   (`robot.js:58-59`); el hardware Ackermann no la tiene (y `pivot_enabled` se apaga
   solo para bicicleta, `sim.js:106-107`).
4. **Encoder de eje**: en hardware es uno real (440 cuentas/vuelta, sin patinaje
   observable); en el sim es el promedio de las dos ruedas con 1 % de pérdidas,
   0,15° de ruido y 1,5 % de error de diámetro (`sensors.js:248,265-269`).
5. **LiDAR**: el sim emite `blind` para obstáculos < 5 cm y ruido 0,5/2 cm; el
   hardware **no emite** esos puntos y siempre `blind=false`.
6. **Velocidad**: hardware tope 40 cm/s por `MOTOR_MAX_CM_S`; sim 60 cm/s sin recorte
   por tensión.
7. **Arranque**: el sim arranca a los 1,5 s sin mirar la calibración; el hardware
   espera 50 muestras de giroscopio quietas.

## 8. Riesgos e incoherencias (por orden de impacto)

1. **Dos fuentes de verdad en la geometría.** `lidar_forward_cm 10`, `camera_forward_cm 13`,
   `wheelbase 14`, `chassis_width 14` son **provisionales** y se usan para la huella,
   la asociación cámara↔LiDAR y el punto de disparo de esquina. El chasis medido es
   260×130 con batalla 184,5: si el firmware real no lleva esos números, todo el
   estacionamiento y las esquinas se calculan con otro coche.
2. **La entrada al cajón depende de una búsqueda sensible.** El A\* da 1 o 2 cambios de
   sentido según 1 cm de la pose de arranque; la entrada simétrica, el asentamiento
   del servo y el coste de holgura la dejan en 2–5 (media ~4) con ~9 % de fallos en semillas nuevas
   (ver `PENDIENTES_PARQUEO.md`); no hay garantía formal.
3. **Patinaje invisible.** Con un solo encoder de eje no se puede estimar; `slip_cm`
   queda a 0 y la derivación por patinaje no actúa (`odometry.c:108-124`).
4. **Nada vigila la frescura de la tabla de visión**: si el S3 baja de ~7 tablas/s el
   color desaparece del mapa sin aviso (`sync_max_age_us 150 ms` frente a
   `sensor_timeout_us 300 ms`).
5. **`FAULT` no se asigna nunca**: ante un sensor caído el robot se queda en la fase
   actual con velocidad 0, no en un estado diagnóstico.
6. **`time_limit_s` es un parámetro muerto** (se define, se parsea, nadie lo lee).
7. **Código muerto con mantenimiento aparente**: la maniobra de tres tramos (no se
   acepta), las filas naranja/azul, `hw_motors_wheel_speeds`, el `switch` inalcanzable
   de `command_from_action` (`wro_controller.c:523-533`) y el `median_of` que se tira
   (`:490`).
8. **Mediciones que faltan**: fps y latencia reales del S3→P4, frecuencia real del
   LiDAR (el deskew asume 10 Hz), calibración del servo y del encoder en el chasis.

## 9. Cómo se verifica

`Vision System/tools/simulate.ps1` compila y ejecuta: tests en C de visión, enlace,
mapa, planificador, controlador, P4 y motores; el contrato ABI del WASM; el simulador
(`test_sim_basics`, `test_contrato`, `test_parking_exit`, `test_parking_entry`,
`test_ui_smoke`, `test_missions`); el depurador sobre un plano limpio; las secuencias
de visión (129 comprobaciones) y el generador del HTML del simulador. `-Quick` se
salta las rondas completas. Los números de parqueo se miden aparte con
`simulator/tests/parkentry.js` (repasadas de entrada y salida, con traza por barrido).

## 10. MIC: control central del motor (`Motor Control/MIC`)

Un solo módulo gobierna la tracción; los niveles superiores piden velocidad, un avance exacto o una ruta con restricciones.

```
nivel 3   parqueo, curvas, rectas        mic_request_speed / _move / mic_route_*
nivel 1   MIC                            modelo (R, Ke, masa, fricción, L298N) + observador + planificador + ley de control
nivel 0   hw_motors (L298N, PWM)  <-  duty con signo + freno corto;   hw_encoders (PCNT x4, 440 cuentas/vuelta) -> counts
```

- **Ley de control**: dinámica inversa (aceleración deseada → fuerza → corriente → tensión) más PI de corrección; límite de 2 A del L298N.
- **Observador de tercer orden** (posición, velocidad y aceleración no modelada), 1 kHz, 0,047 cm por cuenta.
- **Planificador**: la ruta se convierte en un perfil `v(s)` con la aceleración y el frenado que el motor da a cada velocidad; frena lo más tarde posible.
- **Desaceleración**: *progresiva* (tensión ≥ 0, entre rodar y freno corto) y *agresiva* (tensión inversa, solo si hace falta; escala sola a esta en emergencia).
- **Estado**: probado contra un motor simulado (3 variantes de planta); en el simulador con `mic: true` el parqueo da 0 fallos en 40 rondas pero repasadas irregulares.
  Apagado en el firmware (`CONFIG_WRO_USE_MIC=n`). Supuestos a medir: resistencia, masa, fricción, diámetro efectivo de la rueda, cuentas por vuelta.

## 11. trace_map: mapa persistente (`LIDAR System/main/trace_map.c`)

Rejilla de 400 × 400 cm (celdas de 2,5 cm) en el marco del campo con espacio ocupado, **libre** y desconocido; los objetos se agrupan con identidad estable (pilar,
barra, pared), con color y posición; la **plaza de parqueo** se reconoce por el par de topes. Se actualiza en `wro_controller_scan` cada barrido (0,13 ms en PC)
y se refresca cada 10. No gobierna el vehículo: es la fuente de datos para el parqueo y el planificador (todavía no lo consultan). Detalle y mediciones en
`simulator/TRACE_MAP.md`.
