# arena_localizer: error contra la verdad y barrido de parámetros

Solo simulador (perfil `largo26`, 3 letreros, 17 rondas útiles de 20 semillas: 3 acaban pronto por `wrong-side`, un fallo del paso de letreros).
Reproducir: `node simulator/tools/diagnose_arena_error.js 10 salida.json "arena_gain 0.4"` (paralelo, con log por ronda, ~40 s).
El error se mide en cada tick en `RUN` contra la pose verdadera, y con el mismo adelanto de latencia que el controlador aplica antes de alimentar el mapa
(`v · latency_s`, 0,10 s).

## 1. Error del localizador

| Medida | Valor |
| --- | --- |
| Error de posición, RMS (todas las muestras, 139.325) | 0,78 cm |
| A lo largo del avance / transversal | 0,77 / 0,16 cm |
| En rectas (82.992 muestras) / en giros (56.333) | 0,70 / 0,89 cm |
| Rumbo, RMS (rectas / giros) | 0,79° / 2,0° (en giros incluye el error de la fórmula de adelanto de esta medida) |
| Sesgo medio a lo largo / transversal / rumbo | −0,11 cm / −0,04 cm / −0,09° |
| Parte repetible por lugar (casillas de 25 cm, vuelta 1 y 3) | 0,31 cm RMS, correlación 0,915; lo que cambia de una vuelta a otra: 0,13 cm |

**Sin el adelanto de latencia** la pose cruda va 2,2 cm por detrás de la real (sesgo −2,23 cm a lo largo del avance, RMS 2,38 cm). Eso no es un fallo
del localizador: es el retardo del LiDAR, y el controlador ya lo compensa.

La latencia asumida de 0,10 s es la que da menos error a lo largo del avance (RMS por latencia asumida: 0,06 s → 1,23 cm; 0,08 → 0,93; **0,10 → 0,77**;
0,12 → 0,82; 0,14 → 1,06 cm). Cada 10 ms de error de latencia cuestan ~0,4 cm a 40 cm/s: **en el chasis real hay que calibrar `latency_s`**.

## 2. Barrido de parámetros (mismas 17 rondas)

| Parámetro | Error de posición RMS (cm) | A lo largo | Transversal | Rumbo RMS (°) |
| --- | --- | --- | --- | --- |
| `arena_gain` 0,2 | 0,741 | 0,728 | 0,135 | 1,494 |
| `arena_gain` 0,3 | 0,757 | 0,745 | 0,130 | 1,463 |
| `arena_gain` 0,4 | 0,767 | 0,754 | 0,138 | 1,443 |
| **`arena_gain` 0,6 (actual)** | **0,782** | 0,766 | 0,155 | 1,430 |
| `arena_gain` 0,8 | 0,793 | 0,774 | 0,170 | 1,429 |
| `arena_gain` 1,0 | 0,804 | 0,783 | 0,182 | 1,434 |
| `arena_tol_cm` 4 | 0,801 | 0,789 | 0,141 | 1,426 |
| `arena_tol_cm` 6 (actual) | 0,782 | 0,766 | 0,155 | 1,430 |
| `arena_tol_cm` 8 | 0,778 | 0,760 | 0,169 | 1,435 |
| `arena_max_range_cm` 200 | 0,869 | 0,854 | 0,161 | 1,434 |
| `arena_max_range_cm` 300 (actual) | 0,782 | 0,766 | 0,155 | 1,430 |
| `arena_max_range_cm` 400 | 0,781 | 0,766 | 0,155 | 1,430 |

Amortiguación de la rotación (`gn_step`): con 0,001, 3, 10, 50 y 200 el resultado es idéntico a tres decimales; solo 10⁹ lo rompe (8,2 cm). El término
de 50 es despreciable frente a la diagonal del sistema (~10⁷): la rotación ya la fija el LiDAR sin freno.

## 3. Conclusión

El localizador **ya está en su óptimo en este simulador**: toda la rejilla de parámetros queda dentro de 0,06 cm (0,74-0,80 cm) salvo recortar el alcance a 200 cm
(+0,09 cm). La mejora máxima posible, `arena_gain` 0,2, es de 0,04 cm (5 %) a cambio de 0,06° más de error de rumbo y una corrección más lenta de la deriva de
la odometría: no compensa el riesgo de tocar un parámetro del que dependen el paso de letreros, las esquinas y el parqueo. **No se cambia ningún valor por defecto.**
El error que queda (0,7 cm a lo largo del avance) es sobre todo el jitter de la latencia por la velocidad, que ningún ajuste del ajuste a paredes puede quitar.

Lo que sí se hizo: `arena_gain`, `arena_tol_cm` y `arena_max_range_cm` pasan a ser configurables (texto de configuración del controlador) para ajustarlos
con datos del chasis real, donde el ruido del LiDAR y la latencia serán distintos.

Corrección de una explicación anterior: en `MEDICION_TRACE_VUELTAS.md` se había dicho que el error de pose era "sistemático, 1,6 cm" y por eso la vuelta 3 no mejoraba.
Ese 1,6 cm era la latencia sin compensar; con la compensación que ya aplica el controlador el error es 0,78 cm y casi nada es promediable.

## 4. Límites

- Solo simulador: el ruido, las reflexiones y la latencia real del LiDAR no están medidos.
- 17 de 20 rondas (las otras acaban antes de la vuelta 1).
- El rumbo en giros está inflado por la fórmula de adelanto angular de la propia medida.