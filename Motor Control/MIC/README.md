# MIC: Motor Intelligent Control

Módulo único y centralizado que gobierna el motor de tracción. Los niveles superiores (parqueo, curvas,
rectas, esquivar) **no mandan PWM**: piden velocidad, un avance exacto o una ruta con restricciones, y MIC
decide en cada instante la tensión que hay que aplicar.

> **Estado:** probado solo contra un motor y un coche *simulados* con la física del JGB37-520
> (`test_mic.c`), con parámetros distintos a los que MIC conoce (resistencia, masa, fricción, batería, rueda).
> **No está probado con el motor real ni integrado en `hw_motors.c`.** Varios parámetros son supuestos (sección 2).

```
nivel 3   parqueo, curvas, rectas           mic_request_move / mic_route_* / mic_request_speed
nivel 1   MIC  (este, un solo objeto)       modelo + observador + planificador + ley de control
nivel 0   hw_motors (L298N, PWM)            duty con signo + freno corto
          hw_encoders (PCNT, 440 cuentas/vuelta)  -> counts
```

## 1. Qué lo hace distinto de rampas + PID

1. **Modelo físico**: la tensión de cada instante sale de la *dinámica inversa* ("quiero esta aceleración → esta fuerza →
   esta corriente → esta tensión"), con fuerza contraelectromotriz, fricción, batería, caída del L298N y límite de corriente.
   El PI solo corrige lo que el modelo no sabe.
2. **Observador de tercer orden** (posición, velocidad y *aceleración no modelada*): predice con el modelo entre cuentas del encoder
   (0,047 cm por cuenta) y se corrige con él. Estima en línea la carga/fricción real y la compensa.
3. **Planificador con anticipación**: una ruta (recta → curva → parqueo) se convierte en un perfil `v(s)` con la aceleración y la
   desaceleración que el motor puede dar *a cada velocidad*. Mantiene la velocidad (impulso), frena lo más tarde posible con
   la fuerza justa y llega a cada punto a la velocidad pedida. No son rampas: la pendiente del perfil cambia con la velocidad.
4. **Dos desaceleraciones, una ley**: la tensión del motor `Vm` recorre un continuo.
   - `Vm = fcem` → rueda libre (corriente 0). `0 ≤ Vm < fcem` → **frenado progresivo** (sin tensión inversa): la tensión da la
     desaceleración exacta. `Vm = 0` → freno corto, el máximo sin invertir.
   - `Vm < 0` → **frenado agresivo** (tensión inversa), limitado por la corriente del L298N (2 A).
   - Si se pidió progresivo y ya no llega al punto (o supera por >6 cm/s un límite del perfil), escala a activo y lo registra
     (`out.emergency`).

## 2. El motor: qué se sabe y qué se supuso

| Dato | Valor en MIC | Origen |
| --- | --- | --- |
| Motor | JGB37-520, 12 V, reductora 10:1, **600 rpm en vacío** (el motor que hay hoy; el definitivo será de 1000 rpm) | `ACKERMANN.md` y tu indicación del 11 nov |
| Encoder | 11 pulsos/canal × 4 flancos × 10 = **440 cuentas/vuelta**, 0,047 cm/cuenta con rueda de 6,6 cm | `ACKERMANN.md` (propuesto por el equipo) |
| Ke | 0,0185 V·s/rad (de 600 rpm a 12 V y 0,15 A en vacío; con 1000 rpm sería 0,0111) | calculado |
| Resistencia | **2,6 Ω** | **SUPUESTO**: no encontré el dato fiable; medir con el rotor bloqueado a baja tensión |
| Masa, fricción | 1,2 kg, 0,6 N seca, 0,5 N·s/m | **SUPUESTOS** |
| Puente | 2 A, caída 2,0 V, ENA puenteado | `Kconfig.projbuild` |

Búsqueda en tiendas (no hay datasheet del fabricante): el encoder es Hall A/B en cuadratura, y los vendedores no coinciden en pulsos por
vuelta ni en corriente de bloqueo. **Decisión del 11 nov:** el perfil por defecto es el de **600 rpm** (`mic_cfg_jgb37_520_600rpm()`),
porque es el motor que se tiene; cuando llegue el de 1000 rpm basta con `mic_cfg_jgb37_520_1000rpm()` o `WRO_MIC_RPM_NOLOAD=1000`. Ojo:
la relación de reducción de la versión de 1000 rpm puede no ser 10:1 (cambiaría también las cuentas por vuelta de rueda): comprobarlo al
recibirlo. Confirmar con la placa del motor y el contador de cuentas por vuelta (girar la rueda una vuelta a mano y leer el PCNT).

Con el motor de 600 rpm la rueda llega a ~157 cm/s en vacío con 9,1 V efectivos (el techo `v_max_cm_s` de MIC se calcula como el 90 % de
eso, ~141 cm/s), y el coche hoy se usa a ~23 cm/s: **opera en el 15 % del rango**, por debajo de la zona muerta de la fricción. Por eso el
control en lazo cerrado con el encoder es imprescindible. Con Ke más alto el frenado progresivo ya llega al límite de adherencia
(`a_grip_ms2`, 3 m/s²), así que el frenado agresivo apenas acorta la parada; con el motor de 1000 rpm sí.

## 3. API

```c
mic_cfg_t cfg; mic_cfg_jgb37_520_600rpm(&cfg);  mic_t mic; mic_init(&mic, &cfg);

mic_request_speed(&mic, 60.0f, MIC_DECEL_PROGRESSIVE);             // mantener 60 cm/s
mic_request_move(&mic, 3.5f, 14.0f, 0.0f, MIC_DECEL_PROGRESSIVE);  // avanzar 3,5 cm y parar (arco de parqueo)
mic_request_stop(&mic, MIC_DECEL_AGGRESSIVE);

mic_route_begin(&mic);                       // ruta con anticipacion
mic_route_add(&mic, 200, 110, 28);           // 200 cm a 110 cm/s, entrar a la curva a <= 28
mic_route_add(&mic,  45,  28, 12);           // curva a 28, salir a <= 12
mic_route_add(&mic,  55,  12,  0);           // aproximacion al parqueo
mic_route_commit(&mic, 0.0f, MIC_DECEL_PROGRESSIVE);   // termina parado

// cada 1 ms:
mic_in_t in = {.counts = pcnt_total, .vbat = vbat};   mic_out_t out;
mic_update(&mic, &in, 0.001f, &out);        // out.duty (-1..1), out.brake_short, out.phase, out.done ...
```

`mic_max_accel_cm_s2`, `mic_max_decel_cm_s2` y `mic_stop_distance_cm` dejan a los niveles superiores consultar lo que el motor puede
hacer a cada velocidad (por ejemplo, para decidir si un arco de parqueo cabe).

## 4. Resultados (motor simulado, 3 variantes de planta)

| Prueba | Resultado |
| --- | --- |
| Recta a 110 cm/s → curva a 28 → aproximación a 12 → parqueo (300 cm) | llega a 300,8 cm (el 0,3 % es la rueda mal calibrada); 8,8 s frente a ~11,4 s de rampas conservadoras; tensión inversa 0 s; corriente máxima 2,0 A |
| El mismo recorrido con R +15 %, masa +30 %, fricción +40 %, batería baja | igual de preciso; 0 s de frenado activo |
| Avances de 3,5 / 7 / 12 / 25 cm y −3,5 / −12 cm | error de −0,05 a +0,10 cm |
| Parada desde 100 cm/s | progresiva 38,6 cm / 1,12 s; **activa 25,4 cm / 0,52 s** |
| Perturbación (fricción ×3 a mitad) | recupera la velocidad con error máximo de 0,28 cm/s |

## 5. Calibrar antes de usarlo en el coche

1. Cuentas por vuelta de rueda (440 supuestas) y **diámetro efectivo de la rueda**: el error de diámetro es error de distancia
   directo (0,3 % = 0,9 cm en 3 m).
2. Resistencia del motor, masa del coche, fricción seca y viscosa (el observador corrige parte, pero parte mejor si el modelo parte cerca).
3. Caída real del L298N y límite de corriente con carga.
4. Cambiar la frecuencia de PWM de 1 kHz a 15-20 kHz (LEDC) para que la tensión media baja sea limpia.
5. Bucle de MIC a 1 kHz: con la rueda a 23 cm/s llegan ~490 cuentas/s.

## 6. Cómo mejorarlo

- Identificación automática del motor al arrancar (pulsos de prueba) en vez de constantes.
- Perfil con *jerk* limitado y selección automática del tipo de frenado según energía y tiempo.
- Medir la corriente real (sensor) para cerrar el lazo de par y detectar calado.
- Sincronizar velocidad y ángulo del servo (retardo del volante).
- Integrar `encoder_core` (cotas de posición garantizadas) y pasar de cuentas a metros con su tabla de calibración.
- Coast real: controlar ENA para rodar libre cuando el perfil pida menos desaceleración que el freno corto.

## 7. Prueba

```
gcc -O2 -std=c11 -Wall -Wextra -I. mic.c test_mic.c -o test_mic -lm && ./test_mic
```

## 8. Integración en el firmware del P4 (hecha, apagada por defecto)

- `LIDAR System/p4_firmware/main/hw_motors.c`: con `CONFIG_WRO_USE_MIC=y`, un temporizador de 1 kHz ejecuta `mic_update()` con las cuentas del
  encoder (`hw_encoders_counts()`, PCNT x4) y aplica el ciclo que devuelve. `hw_motors_drive(v)` solo cambia la orden de velocidad;
  `hw_motors_stop()` corta y se mantiene cortado hasta la siguiente orden válida; sin encoder listo el motor no se mueve.
- Kconfig: `WRO_USE_MIC` (n), `WRO_MIC_GEAR_REDUCTION` (10), `WRO_MIC_RPM_NOLOAD` (600), `WRO_MIC_ENC_PPR` (11),
  `WRO_MIC_MOTOR_R_MOHM` (2600, supuesto) y `WRO_MIC_MASS_G` (1200, supuesto). El diámetro sale de `WRO_WHEEL_RADIUS_MM` y los engranajes
  de `WRO_MOTOR_GEAR_TEETH`/`WRO_DIFF_GEAR_TEETH`.
- Hoy el controlador manda solo **velocidad**; los avances exactos (`mic_request_move`) y las rutas con anticipación todavía no los usa
  ningún nivel superior. Es el siguiente paso para el parqueo.

**Aviso del lazo abierto actual:** `hw_motors.c` convierte la velocidad a ciclo con `WRO_MOTOR_MAX_CM_S` (40 cm/s por defecto = 100 % de ciclo).
Con el motor de 600 rpm y rueda de 6,6 cm el 100 % da ~157 cm/s (con el de 1000 rpm serían ~270), así que pedir 23 cm/s pondría el 57 %
de ciclo y el coche iría a ~90 cm/s. Los motores están desactivados por defecto (`WRO_MOTORS_ENABLED=n`); no activarlos en lazo abierto sin recalibrar ese valor.
