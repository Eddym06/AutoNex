# Intelligent Trajectory (IT)

Estado: **integrado en `wro_controller` y en el simulador (misión completa, también con MIC), desactivado por defecto (`it_enabled 0`)**. Falta calibrar el chasis real (ver el final).
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


## Integración en la misión (hecho)

`wro_controller.c` (`it_manage`, `it_override`, `wro_controller_it_fast`), `p4_runtime.c` y el simulador:

- **Cuándo**: con `it_enabled 1`, al cerrar la vuelta 1 (`trace_map_is_built`) el plan se calcula con los **pilares de `trace_map`** (color incluido) y los topes de la plaza. Un pilar sin color impide planear: el reactivo sigue.
- **Cálculo por rebanadas** (`it_plan_begin` + `it_plan_step`): `it_step_evals` (300) evaluaciones por escaneo (~10 ms en un PC) en vez de ~0,4 s de golpe; el plan en uso no se toca hasta que el nuevo está listo (doble búfer, 21 KB cada uno).
- **Quién manda**: el controlador reactivo corre siempre en la sombra (cuenta esquinas y vueltas); IT sustituye dirección y velocidad al final del ciclo. IT toma el mando a ≤ 25 cm de la trayectoria y con el morro a ≤ 35° de su rumbo, y **lo cede** si: se sale de la trayectoria (1), no hay pose del campo válida (2), el robot no avanza durante 1,5 s (5, con 3 s de espera) o, en una recta sin curva por delante, el arco que va a recorrer no está libre y no hay ningún objeto del plan cerca de la proa (4, sorpresa en el LiDAR). **No se usa el reactivo como emergencia**: ve cerrados los pasos estrechos que IT cruza a propósito y cedía justo ahí.
- **Entrega al parqueo**: IT cede el mando `it_stop_before_end` (2) esquinas antes del final. El reactivo hace la última media vuelta y el parqueo corre como siempre.
- **Lazo rápido** (`wro_controller_it_fast`, cada 20 ms desde el bucle del P4 y del simulador): entre dos escaneos propaga la última pose del LiDAR con la odometría de los encoders y el giroscopio y vuelve a calcular. **Es imprescindible**: con el seguidor ejecutándose solo en cada escaneo (10 Hz), a 150 cm/s el error lateral pasa de 4,7 a 10 cm y aparecen roces (banco, `it_lab.js --ctl 0.10`).
- **MIC**: IT entrega la velocidad objetivo por el camino de siempre (`micRequestSpeed`). Probado con MIC y su motor físico (JGB37-520): mismas 7 rondas completas, 62 puntos, 0 contactos, 151 cm/s.
- **Agarre aprendido**: al cambiar de vuelta `it_adapt_grip` usa el error lateral máximo, la holgura mínima y los patinajes de los encoders (`odom.slip_count`); si cambia, se rehace el plan.
- **Web**: casilla "trayectoria inteligente (IT)" (arranca el chasis `largo26d` con IT) y "dibujar IT" (trayectoria coloreada por velocidad) y una fila de estado.

### Dos fallos del resto del sistema que IT destapó

1. `odometry.c` descartaba como "error de lectura" toda muestra por encima de **150 cm/s** (`max_speed_cm_s`): justo la velocidad máxima del plan. A tope se perdía distancia real y el localizador de campo se quedaba atrás (100 cm de retraso en una recta, sin que `rms` lo delatara: en una recta las paredes no fijan la posición longitudinal). Ahora 300.
2. El tope de velocidad por holgura usaba la holgura lateral del punto, no la de la **huella** del chasis: junto a una pared el morro (22 cm por delante del eje) roza aunque el eje vaya sobrado. Ahora el tope usa `gap` (huella con su rumbo).

### Resultados de la misión completa (simulador, `largo26d`, 3 letreros, cajón de parqueo, 10 semillas × 2 sentidos)

Rondas en las que IT interviene (12 de 20; en las otras 8 la ronda acaba a los pocos segundos por `wrong-side`/`stalled`, igual con y sin IT):

| | sin IT | con IT |
| --- | --- | --- |
| Terminan con 62 puntos (parqueo incluido) | 11 de 12 (una por tiempo con 22) | **12 de 12** |
| Contactos | 0 | **0** |
| Vuelta 2 | ~34 s | **9-10 s** |
| Tiempo medio de ronda | 149 s | **107 s (−28 %)** |

La vuelta 1 (~29 s) y la última media vuelta más el parqueo siguen siendo del reactivo; ahí está ahora el tiempo. `test_it_mission.js` fija esto en dos rondas.

## Velocidad de la vuelta 1 (mapeo): 34 cm/s

Barrido de `planner.speed_cm_s` con IT activa (16 rondas por valor, `largo26d`, 3 letreros y cajón):

| Vuelta 1 a | Vuelta 1 media | Terminan | Plan de IT válido |
| --- | --- | --- | --- |
| 23 cm/s | 29,2 s | 9 de 16 | 9 |
| 30 cm/s | 22,6 s | 10 de 16 | 10 |
| 40 cm/s | 17,6 s | 10 de 16 | 11 (0,19 contactos de media) |
| 55 cm/s | 14,0 s | 8 de 16 | 8 (una ronda en sentido contrario) |

Se fijó 35 cm/s (punto medio de 30 y 40) y el 11 nov se bajó a **34 cm/s**: a 35 el plano del equipo (`plano_corregido.json`) acaba en `wrong-side` a los 19,7 s; a 31, 33 y 34 termina limpio (62 puntos, 0 contactos; 132,1 s con 34). El eslalon de `test_missions.js` (chasis diferencial histórico) roza letreros a 30, 33 y 36 cm/s y pasa a 32, 34 y 35: la fiabilidad no es monótona con la velocidad, y 34 es el valor que pasa los dos. Las cifras de abajo son de 35. Comprobación en 24 rondas con la configuración exacta: 15 perfectas con 35 frente a 14-15 con 23, 0 contactos, vuelta 1 de 29,6 a 19,4 s; con IT la ronda pasa de 98,6 s a 61,4 s. Un banco anterior del equipo (otro perfil de chasis) había dado 26 cm/s como límite de fiabilidad: **revalidar con el chasis real**. No se ha medido el error de posición de los letreros en el mapa a esta velocidad. Datos: `simulator/mediciones/vuelta1_v*.json` y `v35_*.json`.

## Lo que falta (no hecho)

1. **Calibración en el chasis real** (todo lo de arriba es de simulador): retardo y velocidad del servo, μ (círculo y frenada), reparto de peso, aceleración del motor, `track_err` (14, medido en la misión simulada) y `pose_err`. **Activar `it_enabled` solo después.**
2. **Tiempo de construcción en el P4**: ~0,4 s en un PC repartido en rebanadas de ~10 ms; en el P4 sin medir (`it_step_evals` se ajusta para que una rebanada quepa en el ciclo).
3. **Memoria**: `it_plan_t` 21 KB × 2 (en uso + en construcción) más ~40 KB de búferes estáticos de `it_plan.c`; conviene PSRAM como `trace_map`.
4. **Cámara en línea**: hoy la cámara solo entra por el color del mapa; falta que corrija la holgura en cada letrero y detecte un letrero que el mapa no tenía (hoy lo cubre la comprobación de arco en rectas y el atasco).
5. **Fusión más fina de la pose del seguidor** (encoder + IMU + localizador a alta tasa con un filtro, no solo propagación desde la última pose).
6. **Ceder el mando una esquina más tarde** (`it_stop_before_end 1`) para ganar tiempo con el parqueo: sin medir.

## Reproducir

```
gcc -O2 -std=c11 -Wall -Wextra -Werror -I "LIDAR System/main" "LIDAR System/tests/test_it_plan.c" "LIDAR System/main/it_plan.c" -lm -o it_test && ./it_test
node simulator/tests/test_robot_dynamics.js
node simulator/tests/test_it_mission.js
node simulator/tools/it_plan_check.js 40
node simulator/tools/it_lab.js 12 --laps 4 --grip0 0.45 [--ctl 0.10] [--mu 0.55] [--noise 3] [--map-err 2.5] [--track-err 6]
node simulator/tools/measure_mission.js 10 largo26d "it_enabled 1" salida.json --it --accel 350 [--mic]   # mision completa con IT (sin --it ni config: la linea base)
```
