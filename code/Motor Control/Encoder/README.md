# Encoder óptico de 30 ranuras: estimador de posición y velocidad (ESP32-P4)

Algoritmo en C para estimar **distancia y velocidad** de una rueda a partir de un
sensor ranurado (KLH-512) y el disco de 30 ranuras diseñado en Fusion 360
(`../design 3d/`). Parte de la idea de usar el **tiempo entre flancos** (Método T) con
una tabla de calibración por flanco, y la lleva más lejos: interpolación entre flancos,
cotas de error garantizadas, robustez ante fallas y alineación automática.

> **Todos los números de error de este documento salen de una simulación**
> (`sim/encoder_sim.c`) que ejecuta el mismo `encoder_core.c` que va al ESP32.
> El motor, el disco impreso y el sensor son *modelos* con parámetros que yo supuse
> (sección 8). **No está probado en hardware real.** Úsalos como orden de magnitud y
> verifica con una regla (sección 9).

---

## 1. Resumen: qué precisión se gana

Disco de 30 ranuras = 60 flancos/vuelta = 6° por flanco. Con rueda de R = 32.5 mm,
**cada flanco son 3.40 mm** de avance (si el disco va en el eje de la rueda).

Error de posición **en movimiento, a velocidad constante**, simulado con 20 discos
impresos distintos (error = estimado − real, en mm):

| Método | RMS | p99 | Máximo |
|---|---:|---:|---:|
| Solo contar flancos (mejor caso, centrado) | 1.14 | 2.56 | 3.49 |
| Tu algoritmo original (MRUA + Cᵢ) | 0.59 | 1.42 | 1.84 |
| **Este algoritmo (por defecto)** | **0.28** | **0.63** | **0.82** |
| Este algoritmo si conoces la geometría real del disco | 0.02 | 0.06 | 0.24 |

(0.30 m/s; a 0.05 y 0.65 m/s el resultado es casi idéntico: 0.26 y 0.24 mm RMS.)

- **≈4× mejor** que contar flancos y **≈2× mejor** que el algoritmo original.
- En arranques, paradas y reversas el error RMS es ~0.3-0.4 mm; **al quedar detenido**
  ~0.7 mm RMS tras un frenado suave. Si frenas en seco es ~1.2 mm RMS (máx. 2.8 mm),
  igual que contar flancos: es un límite físico (sección 7).
- En un recorrido de 100 m con 2 glitches/s y flancos perdidos, **el error final fue
  0.2 mm**; el conteo simple y el algoritmo original se desviaron 3.4 m.
- **Lo que domina el error total no es el encoder sino el radio de la rueda y el
  deslizamiento** (sección 5). Un radio con 1 % de error son 10 mm por metro.

---

## 2. Tu idea, y qué cambia

Tu idea: *"como diseño el disco en CAD, sé cuánto mide cada ranura y cuánto tiempo debe
tardar cada pulso; con eso corrijo la velocidad y la distancia, incluso al desacelerar."*

Es correcta en el fondo, y es lo que implementa este código, con **tres matices**
que descubrí al simularlo:

1. **El CAD da la geometría ideal, no la impresa.** En el CAD los flancos están
   exactamente cada 6° (el resultado que ya conocías: sale una tabla uniforme).
   La impresora mueve cada borde ±0.05-0.2 mm, y eso es un error de ±0.2-0.7 mm en la
   rueda. Hay que **medir** el disco real: por calibración (sección 4) o escaneándolo.
2. **La calibración a velocidad constante confunde dos cosas:** el error geométrico del
   disco y el *rizado de velocidad* del motorreductor (que depende del ángulo del eje).
   Ambos hacen que un flanco tarde distinto. El rizado se cuela en la tabla como si fuera
   geometría y empeora la posición (0.28 mm → 0.71 mm). Se corrige filtrando los
   armónicos bajos de la tabla (sección 4). **Este fue el hallazgo más importante.**
3. **Con tabla perfecta, el método es excelente:** el simulador da 0.02 mm RMS. O sea, la
   técnica de medir por tiempo y extrapolar funciona; el límite práctico es *qué tan bien
   conoces tu disco*.

### Comparación con el algoritmo original

| Aspecto | Original | Este código | Por qué |
|---|---|---|---|
| Posición en cada flanco | n × paso nominal (uniforme) | tabla calibrada de posiciones reales | el original corrige la velocidad pero no la distancia: hereda el error de impresión |
| Corrección | factor de tiempo Cᵢ | fracciones de vuelta fᵢ (posición angular real) | es lo mismo para la velocidad, pero da también la posición |
| Velocidad | Δθ / (Δt·Cᵢ) | longitud real del intervalo / Δt | equivalente, más directo |
| Aceleración | (v − v_prev) / Δt_corr | (v̄ₖ − v̄ₖ₋₁) / ((Δtₖ + Δtₖ₋₁)/2) | las velocidades medias corresponden a los *puntos medios* de los intervalos; dividir por un solo Δt sesga `a` |
| Extrapolación | v·t + ½·a·t² sin límite | igual, **acotada al intervalo** y con tratamiento de "flanco atrasado" | sin tope, el estimado puede pasar del siguiente flanco y luego retroceder |
| Robot que frena | no contemplado | saturación en v/\|a\| y punto de reposo | al desacelerar el flanco tarda más; el código lo detecta |
| Incertidumbre | no se reporta | `pos_lo`/`pos_hi` garantizados | sabes cuánto confiar |
| Flanco perdido / glitch | corrompe la distancia para siempre | paridad de nivel + cancelación de pares | ver sección 6 |
| Reversa | no | sí (dirección dada por el control) | |
| Índice del flanco al arrancar | se asume conocido | alineación automática por "huella" | sin marca de índice, la tabla se desalinea en cada reinicio |

---

## 3. Cómo funciona

### 3.1 Tabla de posiciones
La calibración produce `fᵢ` = fracción de vuelta del intervalo que **termina** en el
flanco `i` (i = 0..59; suma = 1). La posición del flanco `n`:

```
s(n) = 2π·R · ( vuelta(n) + Σ_{j=1..idx(n)} f_j )        (relativa al primer flanco)
```
Se calcula por índice, **sin acumular**: no hay deriva en recorridos largos (100 m de
simulación sin deriva).

### 3.2 En cada flanco (Método T)
```
Δtₖ   = tₖ − tₖ₋₁
Δsₖ   = s(nₖ) − s(nₖ₋₁)                       longitud REAL del intervalo
v̄ₖ    = Δsₖ / Δtₖ                              velocidad media del intervalo
aₖ    = (v̄ₖ − v̄ₖ₋₁) / ((Δtₖ + Δtₖ₋₁)/2)       filtrada con alpha = 0.75, tope ±12 m/s²
vₖ    = v̄ₖ + ½·aₖ·Δtₖ                          velocidad al instante del flanco
```

### 3.3 Entre flancos (cada 1 ms o cuando se pida)
Con τ = tiempo desde el último flanco:
```
δ(τ) = vₖ·τ + ½·aₖ·τ²                          avance desde el último flanco
pos  = s(nₖ) + dir · δ(τ)
```
- Si frena (a < 0), δ se satura cuando v llega a 0 (en τ = v/|a|).
- **Nunca pasa del siguiente flanco** (tope al 98 % del intervalo). Si el modelo dice que
  ya debió llegar y el flanco no llega, el robot está frenando más de lo previsto: la
  estimación se acerca suavemente al punto medio del intervalo.
- Sin velocidad conocida (arranque desde reposo, reversa, parada) se estaciona en el
  punto medio del intervalo: error máximo = D/2 = 1.7 mm.
- `pos_lo` y `pos_hi` son las posiciones de los dos flancos que encierran al robot: la
  posición real **siempre** está entre ambas (salvo error de calibración).

### 3.4 Robustez
- **Glitches:** dos eventos más cerca que `min_interval_us` (400 µs) no pueden ser
  flancos reales. Se descartan *y se cancela también el evento previo* (que era la otra
  mitad del par espurio).
- **Flancos perdidos:** el sensor alterna nivel. Si dos eventos seguidos traen el mismo
  nivel, se perdió un flanco: el contador avanza 2 y la posición se resincroniza sola.
- **Reversa:** el primer flanco tras invertir es el mismo que se acababa de cruzar
  (el contador no avanza).
- **Alineación por huella:** sin marca de índice, al reiniciar no sabes qué flanco físico
  es el "0" de la tabla. El código compara los últimos 24 intervalos con la tabla en las
  60 posiciones posibles (ajustando una tendencia lineal para tolerar aceleración) y fija
  la fase cuando un candidato gana claramente. En simulación: **100 % de aciertos,
  0 falsos, en 24-32 flancos** (≈ media vuelta). Hasta entonces usa la tabla uniforme.

---

## 4. Calibración

### 4.1 Procedimiento
1. **Rueda en el aire**, motor con PWM fijo (velocidad media, p. ej. 60-70 %).
2. `encoder_port_calibrate_start()`. Se registran 24 vueltas; las 2 primeras se ignoran.
3. Por cada vuelta, el intervalo de cada flanco se divide **entre el tiempo de esa
   vuelta** (esto elimina deriva lenta de velocidad). Se toma la **mediana** entre
   vueltas (rechaza valores atípicos). Se descartan vueltas cuya duración difiere >2 %.
4. Se filtran los armónicos bajos (4.2), se normaliza (suma = 1) y se guarda en NVS.

Es la generalización de tu `Cᵢ = Δt_promedio / Δt_bruto_i`, con tres ventajas:
varias vueltas, normalización por vuelta, y tabla en posiciones (no solo en tiempos).

### 4.2 Por qué se filtran los armónicos 1-3
Si la velocidad del eje varía con el ángulo (excentricidad de los engranajes, típica del
TT) con amplitud ε en el armónico `h`, la tabla queda contaminada en
`R·ε/h` de posición. Con los valores que supuse:

| Armónico | Ripple supuesto | Contaminación en la rueda |
|---|---:|---:|
| 1× por vuelta | 2 % | **0.65 mm** |
| 3× por vuelta | 1.5 % | 0.16 mm |
| 12× por vuelta | 1 % | 0.03 mm |

El error de impresión, en cambio, es "plano" (reparte su energía por igual entre
armónicos: ~0.05 mm por armónico). Así que los armónicos bajos son los que están
dominados por el rizado del motor, y **quitarlos de la tabla** reduce el error en
crucero de 0.71 a 0.28 mm. Se pierde la parte real de esos armónicos (excentricidad del
montaje del disco), pero es inseparable del rizado con este método.

Barrido en simulación (error RMS en crucero a 0.30 m/s, mm; 10 discos por fila, `explore`):

| Ripple del motor | tabla uniforme | calibrada sin filtro | filtro 1-3 | geometría real |
|---|---:|---:|---:|---:|
| ×0.5 (1 % / 0.8 % / 0.5 %) | 0.82 | 0.32 | 0.25 | 0.01 |
| ×1 (2 % / 1.5 % / 1 %) | 0.69 | 0.64 | 0.25 | 0.02 |
| ×2 (4 % / 3 % / 2 %) | 0.72 | 1.31 | 0.22 | 0.04 |
| ×4 (8 % / 6 % / 4 %) | 0.75 | 2.53 | 0.29 | 0.07 |

El filtro 1-3 (`harm_cut = 3`) es robusto en todo el rango. **Sin filtro, un motor con
mucho rizado es peor que no calibrar.**

### 4.3 Diagnóstico con tu motor real
`enc_cal_report_t.harm_deg[0..7]` da la amplitud (en grados de eje) de los armónicos 1-8
del error de la tabla **antes** de filtrar. Interprétalo así:
- Armónicos 1-3 mucho mayores que 4-8, y decayendo → rizado/excentricidad. El filtro ayuda.
- Todos parecidos (~0.1-0.2°) → error de impresión; el filtro quita poco.
- Referencia: 1° de eje en una rueda de R = 32.5 mm son 0.57 mm.

Si tus armónicos altos (5-8) también son grandes, sube `harm_cut` a 4-6.

---

## 5. Presupuesto de error

| Fuente | Magnitud | Comentario |
|---|---|---|
| Cuantización (solo contar) | ±1.70 mm | 3.40 mm/flanco. Es lo que elimina la interpolación |
| Tiempo: timestamp 1 µs + jitter ISR ~3 µs | < 0.003 mm | a 0.65 m/s; despreciable |
| Retardo distinto subida/bajada del sensor (~10 µs) | ~0.007 mm | despreciable; por eso **no** se calibra por velocidad |
| Geometría del disco (tabla) | ~0.25 mm RMS | **dominante en movimiento**; 0.02 si mides el disco aparte |
| Parada: posición dentro del último intervalo | ±1.7 mm | imposible de saber con un solo sensor si frenas en seco |
| Aritmética float | 0.007 mm a 100 m | |
| **Radio de la rueda** | **10 mm por metro por cada 1 %** | **el error más grande de todo el sistema** |
| **Deslizamiento de la rueda** | no medido | el encoder no lo ve |

**Conclusión práctica:** para una recta de 1 m, el encoder aporta ≲ 0.3-1.7 mm, pero un
radio mal calibrado (0.5 %) aporta 5 mm. Primero calibra el radio (sección 9).

---

## 6. Resultados de simulación

Ejecutar: `sim/encoder_sim.exe full` (20 discos por escenario; resultado completo en
`results/sim_results.txt`). Resumen (mm; RMS / máximo):

**Crucero 0.30 m/s**

| Estimador | RMS | p99 | máx |
|---|---:|---:|---:|
| 0 Solo conteo de flancos | 1.14 | 2.56 | 3.49 |
| 1 Original (MRUA + Cᵢ) | 0.59 | 1.42 | 1.84 |
| 2 Nuevo, sin calibrar | 0.86 | 2.29 | 3.05 |
| 3 Nuevo, calibración cruda | 0.72 | 1.49 | 1.67 |
| **4 Nuevo por defecto** | **0.28** | **0.63** | **0.82** |
| 7 Límite (geometría real) | 0.02 | 0.06 | 0.24 |

**Arranque-parada** (aceleraciones 1-5 m/s²; movimiento | en reposo)

| Estimador | RMS mov | máx mov | RMS reposo | máx reposo |
|---|---:|---:|---:|---:|
| 0 Solo conteo | 1.12 | 3.27 | 1.04 | 2.59 |
| 1 Original | 0.95 | 4.53 | 1.92 | 3.29 |
| **4 Nuevo por defecto** | **0.40** | 4.09 | **0.69** | 1.72 |
| 5 Nuevo sin término de aceleración | 0.57 | 4.35 | 1.02 | 2.50 |

Aquí se ve que **el término de aceleración (tu idea del MRUA) sí vale la pena:**
quitarlo sube el error en movimiento de 0.40 a 0.57 mm (y en reposo de 0.69 a 1.02).
El máximo de ~4 mm ocurre al *arrancar desde reposo*, cuando aún no hay velocidad.

**Frenado brusco** (15 m/s², parada en punto aleatorio): reposo 1.17 mm RMS, máx 2.8 mm,
igual que contar flancos (1.13 / 2.9). Es el límite físico: si el robot se detiene en
seco dentro de un intervalo, ningún algoritmo puede saber dónde.

**Reversa** (avance-reversa-avance): 0.32 mm RMS en movimiento, 0.23 en reposo.

**Recorrido de 100 m, sin fallas:** error final 0.49 mm (original: 2.0 mm).
**Con 2 glitches/s y 0.05 % de flancos perdidos:** error final **0.21 mm**; conteo simple
y original: **3.4 m**. (Transitorios de hasta ~5.6 mm justo después de perder un flanco,
hasta que llega el siguiente.)

**Sensibilidad a la calidad de impresión** (arranque-parada, RMS / máx):

| σ del borde de ranura | Conteo | Original | Nuevo |
|---|---:|---:|---:|
| 0.02 mm | 0.98 / 2.07 | 0.77 / 3.55 | 0.34 / 3.35 |
| 0.05 mm | 1.04 / 2.70 | 0.76 / 4.12 | 0.35 / 3.44 |
| 0.10 mm | 1.21 / 4.81 | 0.93 / 4.94 | 0.42 / 3.66 |
| 0.20 mm | 1.52 / 5.42 | 1.35 / 5.18 | 0.71 / 6.01 |

---

## 7. Qué tan bueno es (veredicto honesto)

**Bueno:** pasa de ±1.7 mm de resolución a ~0.3 mm RMS en movimiento, es robusto (no
deriva, se recupera de fallas, se alinea solo) y te dice su incertidumbre.

**Sus límites reales:**
1. **Con un solo sensor no hay dirección.** El código la toma del comando del motor;
   si el robot patina hacia atrás o se mueve por inercia, se equivoca.
2. **En reposo la incertidumbre es de ±1.7 mm** (frenado brusco). Frenado suave: ~0.7 mm.
3. **El techo lo pone el conocimiento del disco**, no el algoritmo (0.28 vs 0.02 mm).
4. **El radio de rueda y el deslizamiento** dominan la distancia total.
5. **Es un resultado de simulación.** La forma del rizado de tu motor, la calidad real de
   tu impresión y el ruido del sensor pueden cambiar los números.

---

## 8. Supuestos de la simulación (lo que *yo* inventé)

| Parámetro | Valor | Nota |
|---|---|---|
| Error aleatorio del borde de cada ranura | σ = 0.08 mm | impresión "típica"; barrido 0.02-0.20 mm |
| Sesgo de ancho de ranura | ±0.04 mm (subida/bajada) | ranuras más anchas/angostas |
| Excentricidad del disco | 0.06 mm | |
| Rizado de velocidad | 2 % (1×), 1.5 % (3×), 1 % (12×) por vuelta | **el supuesto más importante**; barrido ×0.5…×4 |
| Retardo del sensor | 20 µs subida / 10 µs bajada | |
| Jitter de la ISR | 0-3 µs, timestamps de 1 µs | |
| Rueda | R = 32.5 mm | |
| Radio medio de las ranuras | 9 mm | de tu diseño en Fusion |

La velocidad se integra en `double` cada 5 µs; el estimador corre en `float`.
El rizado se modela como función del *ángulo* (no del tiempo), que es el caso difícil.

**El "algoritmo original"** lo implementé tal como lo escribiste: posición en cada flanco =
n × paso nominal; velocidad con `Cᵢ` (calibrado con el mismo procedimiento, a partir de la
tabla cruda); `a = (v − v_prev)/Δt_corr`; `Δs = v·t + ½·a·t²`. Único añadido: no se
extrapola más de 1.5 intervalos, para que no diverja al detenerse. Si lo escribiste con
otra variante, los números del original pueden cambiar.

---

## 9. Cómo medir tu error real (hazlo antes de confiar en nada)

1. **Radio efectivo.** Marca una recta de 2 m en el piso de competencia. Avanza con el
   robot, lee `pos_m`, y corrige: `R_nuevo = R_actual · distancia_real / distancia_leída`.
   Repite 5 veces. Esto solo corrige hasta ~1 % de error, más que todo lo demás junto.
2. **Calibra el disco** (sección 4) y revisa `harm_deg` (4.3).
3. **Repetibilidad:** 10 recorridos de 500 mm a la misma velocidad; mide el punto final
   con regla. La dispersión es tu error de encoder + deslizamiento.
4. **Frenado:** compara frenado suave vs brusco; si el brusco es mucho peor, baja la
   rampa de desaceleración antes de detenerte.

---

## 10. Qué mejoraría (ordenado por impacto / esfuerzo)

1. **Medir la geometría del disco por separado** (impacto alto, esfuerzo medio). Con la
   geometría conocida el simulador baja de 0.28 a 0.02 mm. Idea: escanear el disco en un
   escáner de cama plana a 1200-2400 dpi con el disco sobre el cristal, extraer los
   ángulos de los 60 bordes y cargarlos como tabla. Hace falta precisión de borde ~0.03 mm
   en el disco (~0.1° de eje). **No lo he probado con un escaneo real.**
2. **Un segundo sensor en cuadratura** (impacto alto): colócalo a (k + ¼) × 12° del
   primero, p. ej. 87°. Da **dirección real** y **120 flancos/vuelta (1.7 mm)**, resolviendo
   los puntos débiles 1 y 2 de la sección 7.
3. **Disco en el eje del motor (antes del reductor 1:48)** (impacto muy alto): son
   48× más flancos: 2880 por vuelta de rueda = **0.07 mm por flanco**. Necesita otro disco
   (el eje del motor es más delgado) y manejar ~10 kHz de flancos con hardware de captura
   (PCNT / MCPWM) en vez de ISR por GPIO. El juego del reductor (típicamente 1-2° de
   salida, 0.6-1.1 mm; no medido en tu motor) aparece al invertir el sentido.
4. **Fusión con IMU** (giroscopio/acelerómetro): detecta deslizamiento y cubre el reposo
   y la dirección; un filtro complementario simple ya ayuda.
5. **Marca de índice** en el disco (una ranura ausente o más ancha): alineación inmediata
   y sin depender de la huella. Cambiaría el diseño de Fusion.
6. **Calibrar con el disco girado 180°** (el eje de doble D lo permite): el rizado del
   eje queda fijo en el ángulo del eje, pero la geometría del disco gira con él. Al
   alinear ambas tablas (la huella sirve para eso) y promediarlas, los armónicos
   *impares* del rizado (1× y 3×, los dominantes) se anulan y se conserva la geometría
   propia del disco. La excentricidad de montaje también se anula (cambia de signo).
   No implementado.
7. **Anticipar el rizado:** una vez conocida la geometría, el rizado de velocidad es
   repetible por ángulo y podría predecirse (alimentación adelantada). No implementado.
8. **Calibración a 2+ velocidades** para modelar el retardo del sensor: *no* vale la pena
   aquí (0.007 mm); sí sería necesario con un sensor lento.

---

## 11. Uso

### 11.1 Archivos
```
Encoder/
├─ README.md
├─ src/
│  ├─ encoder_core.h / .c      núcleo en C99 (sin dependencias; float)
├─ esp32p4/
│  ├─ encoder_port.h / .c      ISR, tarea, NVS, calibración (ESP-IDF)
│  ├─ example_main.c           ejemplo de app_main
│  └─ CMakeLists.txt           componente ESP-IDF
├─ sim/
│  ├─ encoder_sim.c            simulador + banco de pruebas
│  ├─ Makefile / build.bat
└─ results/sim_results.txt     salida completa de `encoder_sim full`
```

### 11.2 Simulador (PC)
Windows (MinGW-w64 / MSYS2): `cd sim` → `build.bat` → `encoder_sim.exe quick`
(~40 s) o `full` (varios minutos). Linux/macOS: `make run`.
Modos: `quick`, `full`, `explore` (barrido de armónicos y ripple), `explore2` (alpha, hold).

### 11.3 ESP32-P4 (ESP-IDF)
Copia `src/` y `esp32p4/` como componente (o `EXTRA_COMPONENT_DIRS`). Ver
`esp32p4/example_main.c`:

```c
encoder_port_config_t cfg = { .gpio = GPIO_NUM_4, .wheel_radius_m = 0.0325f,
                              .nvs_key = "enc_L", .harm_cut = 3, .pull_up = true };
encoder_port_init(&enc, &cfg);          // carga la tabla de NVS si existe
encoder_port_set_direction(enc, +1);    // desde tu control de motor
enc_est_t e;  encoder_port_get(enc, &e);// e.pos_m, e.vel_mps, e.pos_lo_m, e.pos_hi_m
```
Para dos ruedas, dos instancias con distinto GPIO y `nvs_key`.

> El código de `esp32p4/` **compila contra cabeceras de prueba (sin errores ni
> advertencias) pero no lo he ejecutado en un P4.** Revisa pines, prioridades y tamaños
> de pila al integrarlo.

### 11.4 Parámetros (`enc_config_t`)

| Campo | Defecto | Efecto |
|---|---:|---|
| `wheel_radius_m` | 0.0325 | **calíbralo** (sección 9) |
| `min_interval_us` | 400 | filtro de glitches; debe ser menor que el intervalo real mínimo |
| `stop_timeout_us` | 150000 | sin flancos tanto tiempo = detenido |
| `accel_max` | 12 | tope de \|a\| (m/s²) |
| `accel_alpha` | 0.75 | filtro de a (0 = sin término de aceleración; 0.75-1 equivalentes) |
| `hold_frac` | 0.5 | punto de reposo dentro del intervalo (0.5 = menor error en el peor caso) |
| `overdue_tol` | 0.25 | cuánto pasarse del siguiente flanco antes de dar por frenado |
| `align_cost_max` / `align_ratio_min` | 0.003 / 2.5 | exigencia de la alineación por huella |

Si cambias el disco (más ranuras): ajusta `ENC_EDGES` (= 2 × ranuras) y recalibra.
Si cambias de rueda o de disco, borra la tabla de NVS y recalibra.

---

## 12. Relación con el diseño 3D
- Disco Ø24 × 3 mm, 30 ranuras sectoriales de 6° entre r = 7.5 y 10.5 mm
  (`../design 3d/README.md`). Flancos nominales cada 6°: `θ_j = 3° + 6°·j`.
- Ranuras de ~0.8-1.1 mm: la resolución de impresión es lo que fija el error de la tabla.
  Una impresora de resina o láser acerca el disco al caso "σ = 0.02-0.05 mm".
