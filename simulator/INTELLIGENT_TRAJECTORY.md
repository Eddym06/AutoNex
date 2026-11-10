# Intelligent Trajectory (IT)

Estado: **planificador y seguidor hechos y medidos en el simulador con dinámica; NO integrados todavía en `wro_controller`** (ver el final).
Todo lo de abajo es de simulador. Nada se ha medido con el chasis real; los parámetros físicos son provisionales y están marcados.

## Qué hace

Después de la vuelta 1 (mapa construido) calcula, él solo, la trayectoria y el perfil de velocidad de una vuelta. No hay una línea de carrera dada:
el planificador recibe el espacio libre (paredes, letreros con su color) y la física del vehículo, y **busca por optimización** el desplazamiento lateral
de cada estación que minimiza el tiempo de vuelta.

`LIDAR System/main/it_plan.c/.h` (C99, sin memoria dinámica, mismo estilo que `trace_map`):

1. **Corredor libre por estación** (cada 5 cm sobre una línea central redondeada): dentro de las paredes, fuera de cada letrero y del lado que su color
   permite (rojo derecha, verde izquierda). Con varios tramos libres se elige uno por estación con programación dinámica sobre el anillo.
2. **Física** (todo calculado, nada fijado a mano): círculo de fricción con **peso trasero** (el eje trasero empuja y frena, el delantero va ligero),
   traslado de peso al acelerar y frenar, curvatura máxima del volante, velocidad de giro del servo, tope por holgura (a más velocidad hace falta más margen:
   error de pose + error de seguimiento, que crece con v²).
3. **Búsqueda**: arranque desde 6 mezclas longitud/curvatura (mínimos cuadrados con cotas, gradiente conjugado sin matriz) y después búsqueda local con
   "jorobas" suaves evaluando el **tiempo real de vuelta** (perfil adelante-atrás).
4. **Huella del chasis**: el coche es un rectángulo de 26 × 13 cm con el eje trasero como origen. Se penaliza en el coste la penetración de la huella,
   con el rumbo propio de cada estación, contra las paredes, el bloque interior (SAT) y los letreros.
5. **Agarre aprendido**: `grip` escala μ. Tras cada vuelta `it_adapt_grip` lo baja 0,15 si hubo roce o poca holgura y lo sube 0,05 si sobró. El límite lo encuentra el coche.
6. **Seguidor** (`it_track`): persecución pura con adelanto por el retardo del servo; la velocidad del perfil se recorta si el error lateral se come la holgura.

Si no hay plan válido (`why` ≠ 0) el llamador usa su respaldo: el controlador reactivo de siempre.

## Servo MG90S (lo que se encontró y cómo se usa)

| Dato | Valor encontrado | Fuente |
| --- | --- | --- |
| Par | 1,8 kg·cm a 4,8 V, 2,2 kg·cm a 6 V (otros vendedores dan 2,0-2,8) | [hoja de AZ-Delivery](https://cdn.shopify.com/s/files/1/1509/1638/files/MG90S_Micro_Servo_Datenblatt_AZ-Delivery_Vertriebs_GmbH.pdf), [tinytronics](https://www.tinytronics.nl/product_files/000263_Data_Sheet_of_MG90S_Analog_Servo_Motor.pdf) |
| Velocidad sin carga | 0,10-0,12 s/60° a 4,8 V; 0,08-0,10 s/60° a 6 V (≈ 500-750 °/s) | [components101](https://components101.com/motors/mg90s-metal-gear-servo-motor) y las anteriores |
| Alimentación | 4,8-6 V | idem |
| Zona muerta | ~5 µs (los vendedores dicen 1-10 µs) ≈ 0,5° | idem |
| Señal | PWM de 50 Hz, ~500-2400 µs para ~180° | [kpower](https://blog.kpower.com/servo/625.html) |
| Tipo | analógico según unos, digital según otros (el MG90D es el digital) | datos contradictorios |
| Velocidad bajo carga, retardo de la orden | **no hay medida publicada** | hay que medirlo |

Cómo entra en el simulador (`largo26d` en `robot.js`):

- velocidad máxima de giro **420 °/s** (≈ 75 % de la de vacío; las ruedas delanteras van ligeras, así que el servo trabaja poco cargado: es una cota prudente),
- **20 ms de retardo** de la orden (el PWM de 50 Hz da hasta 20 ms; es una suposición mía) y constante de 45 ms,
- zona muerta de 0,5°,
- el planificador solo usa el **72 % del volante** (40° máx.) y el **45 % de la velocidad del servo**; el resto es reserva para que el seguidor corrija.

**Medida pendiente en el chasis (10 minutos):** orden de −30° a +30° con el robot en el suelo y grabar a 240 fps (o un encoder en el volante) para sacar el retardo y la velocidad cargada, y poner
`steerRateDegS` / `steerDelayS` en el simulador y `steer_rate_dps` / `lag_s` en `it_config_t`.

## Neumáticos y reparto de peso

`robot.js`, bicicleta dinámica con neumáticos que saturan (tanh), 62 % del peso sobre el eje trasero, altura del centro de masa 4 cm, μ = 0,8 (provisional).
`tests/test_robot_dynamics.js` comprueba: coincide con el cinemático a baja velocidad (22,0 frente a 22,0 °/s), subvira cada vez más al subir la velocidad, la
aceleración lateral no pasa de μ·g, el tope de tracción (589 cm/s² al acelerar) y el de frenada (415 cm/s², solo frena el eje trasero) se cumplen, y el servo respeta el retardo.
A las velocidades actuales del controlador (≤ 22 cm/s) el perfil dinámico reproduce el cinemático: **92,2 s frente a 92,4 s** de misión media, mismos finales.

Consecuencia que importa: con el peso detrás, **frenar es más débil que acelerar** (415 contra 589 cm/s²) y el planificador lo sabe.

## Resultados (simulador, 3 letreros, 12 semillas × 2 sentidos)

Línea base del controlador actual: velocidad media 15,8 cm/s, máxima 22 cm/s, ~31 s por vuelta.

Planificación (`tools/it_plan_check.js`): 79 de 80 pistas con plan válido sin reserva de volante; con la reserva de volante 72 de 80. Las que no salen
son pasos en S demasiado cortos para el volante que se reserva (el otro caso, un rojo y un verde en la misma posición de la recta, no tiene solución para nadie).
Construcción: **~415 ms** en el wasm de un PC (en el P4 será más lento: **sin medir**; debe correr en una tarea de baja prioridad).

Banco con física (`tools/it_lab.js`, agarre de partida 0,6, `track_err` 6):

| Caso | Rondas completas | Vueltas con roce | Tiempo de vuelta |
| --- | --- | --- | --- |
| Ideal (el planificador conoce la física) | 20/24 (+ 2-3 sin plan) | 1 % | 5,2 s |
| μ real 0,55 (planifica con 0,8), agarre inicial 0,45 | 20/24 | 2 % | 5,4 s |
| μ real 0,40, agarre inicial 0,35 | 7/24 | 21 % | 5,8 s |
| Ruido de pose ×3 | 20/24 | 12 % | 5,2 s |
| Error de mapa 2,5 cm (en vez de 0,9) | 22/24 | 10 % | 5,5 s |
| Motor de 200 cm/s² (en vez de 350) | 20/24 | 1 % | 5,35 s |

Compromiso del margen de seguimiento (`track_err`): 3 → 10 % de vueltas con roce y 4,95 s; **6 → 1 % y 5,21 s**; 16 → 0 % y 5,89 s.

Lectura sin adornos:

- Frente a ~31 s la vuelta de IT en el banco es de ~5 s. **Es una cota superior optimista**: el banco usa los letreros reales con 0,9 cm de error, no el mapa del robot; la pose
  es la real más ruido, no la salida de `arena_localizer`; el motor es una constante de aceleración, no MIC; y no hay vuelta 1, cámara ni parqueo.
- El tiempo apenas depende del agarre (5,23 s con 0,6; 5,33 s con 0,45): el límite es el tope de 150 cm/s, el volante y la latencia, no la fricción. Empezar conservador sale casi gratis.
- **Falla si el agarre real es muy bajo** (μ 0,40) y es sensible al **ruido de pose** y al **error de mapa**: con 3 veces el ruido medido, 1 de cada 8 vueltas roza algo.
  `pose_err_cm` y `track_err_cm` tienen que salir de la medida del chasis (y la primera podría tomarse en línea del RMS de `arena_localizer`).
- El aprendizaje de agarre ayuda poco con tan pocas vueltas (solo hay 2 o 3 de IT por ronda): por eso el agarre inicial es conservador y la calibración de μ se hace en el taller.

## Lo que falta (no hecho)

1. **Integración en `wro_controller`**: arrancar IT al cerrar la vuelta 1 (`trace_map_is_built`), con los objetos de `trace_map` en vez de los letreros del banco; la transición suave desde la pose actual
   a la trayectoria; el respaldo al controlador reactivo si el LiDAR local ve algo que el mapa no tiene; apagarlo antes del parqueo con una puerta de traspaso (pose y velocidad de entrada fijas).
2. **MIC**: el banco usa un modelo simple de velocidad. Hay que pasar `v_obj` / `a_ff` a MIC y devolver el deslizamiento medido (encoder frente a IMU) para estimar el agarre en línea.
3. **Fusión encoder + IMU + `arena_localizer`** a alta tasa para la pose del seguidor (hoy es la verdad más ruido), y la cámara para confirmar el color y corregir la holgura en cada letrero.
4. **Calibración en el chasis**: retardo y velocidad del servo, μ (prueba de círculo y de frenada), reparto de peso real, aceleración del motor, `track_err` y `pose_err`.
5. Pruebas en el P4: tiempo de construcción del plan. Memoria: `it_plan_t` ocupa 21 KB (medido) más unos 40 KB de buffers de trabajo estáticos dentro de `it_plan.c` (estimado, sin medir); conviene PSRAM como `trace_map`.

## Reproducir

```
gcc -O2 -std=c11 -Wall -Wextra -Werror -I "LIDAR System/main" "LIDAR System/tests/test_it_plan.c" "LIDAR System/main/it_plan.c" -lm -o it_test && ./it_test
node simulator/tests/test_robot_dynamics.js
node simulator/tools/it_plan_check.js 40
node simulator/tools/it_lab.js 12 --laps 4 --grip0 0.45 [--mu 0.55] [--noise 3] [--map-err 2.5] [--track-err 6]
node simulator/tools/measure_mission.js 6 largo26d     # línea base del controlador actual con dinámica
```
