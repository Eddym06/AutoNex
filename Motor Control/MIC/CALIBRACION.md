# Calibración de MIC en el coche real

Objetivo: reemplazar los valores supuestos de `mic_cfg_jgb37_520_600rpm()` / `Kconfig` por medidas.
Cada paso dice qué medir, cómo, cuánto debe dar aproximadamente y qué parámetro cambia.
Hacerlo con las ruedas **en el aire** (coche levantado) salvo donde se indica, y con la batería 3S cargada.
El orden importa: cada paso usa el anterior.

> Seguridad: `WRO_MOTORS_ENABLED=y` solo para esta prueba, rueda en el aire, mano cerca del interruptor.
> Con 12 V el motor de 600 rpm da ~157 cm/s en vacío (el de 1000 rpm daría ~270): empezar con ciclos bajos (≤ 20 %).

## 0. Datos de placa y cableado (5 min)

| Qué | Cómo | Parámetro |
| --- | --- | --- |
| rpm del motor (hoy 600; el definitivo será de 1000) | leer la placa; o paso 3 | `WRO_MIC_RPM_NOLOAD` |
| Reducción | girar el eje del motor 10 vueltas a mano y contar las del eje de salida | `WRO_MIC_GEAR_REDUCTION` (10) |
| Pulsos por canal y vuelta de motor | girar la salida **una vuelta** a mano y leer el PCNT: cuentas = PPR × 4 × reducción | `WRO_MIC_ENC_PPR` (11); `WRO_AXLE_ENCODER_COUNTS_PER_REV` (440) |
| Signo del encoder | giro hacia delante debe dar cuentas positivas | `WRO_AXLE_ENCODER_INVERT` |
| Dientes motor/diferencial | contar | `WRO_MOTOR_GEAR_TEETH`, `WRO_DIFF_GEAR_TEETH` |

## 1. Diámetro efectivo de la rueda (10 min, **el que más pesa**)

Con el coche en el suelo, marcar el punto de partida y empujarlo a mano en línea recta 2 m medidos con cinta (repetir 3 veces, ida y vuelta).
`cuentas_por_cm = cuentas / distancia_cm`. Con 440 cuentas por vuelta: `D = 440 / (cuentas_por_cm · π)`.
→ `WRO_WHEEL_RADIUS_MM = D/2` (ahora 33). Un error de 1 % son 3 cm en 3 m.

## 2. Masa (2 min)

Pesar el coche montado, con batería. → `WRO_MIC_MASS_G` (1200 supuesto).

## 3. Velocidad en vacío y constante Ke (10 min)

Rueda en el aire, ciclo fijo `d` (0,2 / 0,4 / 0,6 / 0,8) durante 3 s; medir la velocidad en régimen con el encoder (cuentas/s ÷ cuentas por vuelta × 60 = rpm de salida)
y la tensión de batería `Vb` en ese momento.

`Vm = d · (Vb − Vdrop)`; la recta `rpm = k · Vm` debe pasar por ~0. Con 12 V: ~600 rpm con el motor actual (si da otra cosa, corregir `WRO_MIC_RPM_NOLOAD`; con el de 1000 rpm daría ~1000).
Si no pasa por 0 (hay zona muerta), anotar la `Vm` mínima a la que gira: es la **tensión de arranque** (≈ `R · f_coulomb / kf`).

## 4. Resistencia del inducido R (5 min)

Rotor **bloqueado** (sujetar la rueda, motor a baja tensión) con `Vm` = 1,0 V (ciclo = 1/(Vb − Vdrop)) durante < 2 s y leer la corriente (multímetro en serie, o fuente de laboratorio).
`R = Vm / I` (≈ 2,6 Ω supuesto). → `WRO_MIC_MOTOR_R_MOHM`. No pasar de 1,5 s bloqueado ni de 2 A: el L298N tiene ese límite.

## 5. Fricción en el suelo (10 min)

Con el coche en el suelo, ciclo creciente hasta que **arranca**: `Vm_arranque`. Fuerza seca ≈ `kf · Vm_arranque / R` (kf = 4,2 N/A con η = 0,75 y el motor de 600 rpm; 2,5 con el de 1000).
Luego velocidad en régimen a ciclo medio para estimar la viscosa. → editar `f_coulomb_n` y `b_viscous_ns_m` en `mic_cfg_jgb37_520()`
(el observador corrige parte del error en línea, pero parte de un modelo cercano).

## 6. Caída del puente y batería (5 min)

Con carga (suelo), medir la tensión entre los bornes del motor y la de la batería a ciclo 0,5 y 1,0: `Vdrop = Vb·d − Vmotor` → `WRO_MOTOR_DROP_MV` (2000).
Anotar la tensión de batería cargada y a media carga: si baja de 10 V, ajustar `vbat_min`.

## 7. Cambiar PWM de 1 kHz a 15-20 kHz

En `hw_motors.c` (`PWM_FREQ_HZ`). Con 10 bits y 20 kHz el LEDC de ESP32-P4 lo admite; comprobar que el L298N no se calienta más de lo normal.
A baja tensión media (ciclos del 8-15 %), 1 kHz da una corriente a saltos y un motor áspero.

## 8. Prueba de cierre

1. `WRO_USE_MIC=y`, coche en el suelo, pista libre de 1,5 m.
2. Pedir 20 cm/s durante 3 s y parar: la velocidad debe quedarse en 20 ± 1 cm/s y la parada, sin retroceder.
3. Pedir un avance de 3,5 cm y otro de 12 cm (cuando exista `mic_request_move` en el controlador): error < 0,3 cm.
4. Si el error de distancia en 3 m es > 1 cm, repetir el paso 1.

Registrar las medidas en este archivo (fecha, valor, método) y actualizar `PENDIENTES_PARQUEO.md`.
