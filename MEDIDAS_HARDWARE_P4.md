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
