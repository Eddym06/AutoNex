# Pendientes del parqueo (estado al 9 nov 2026)

Todo lo de este documento se midió **solo en simulación** (perfil `largo26`, cajón de 39 cm,
herramienta `simulator/tests/parkentry.js`). Nada se probó en el chasis real.

## 1. Qué quedó activo en el código

| Cambio | Archivo | Para qué |
| --- | --- | --- |
| Coste suave de holgura en el A* (`PARK_CLEAR_PENALTY 1.0`, `PARK_CLEAR_EXTRA_CM 1.5`) | `LIDAR System/main/park_ackermann.c` | El A* prefiere caminos a más de 2,5 cm de los obstáculos. Los planes con ≥1,3 cm de holgura real se ejecutan enteros. |
| Meta de entrada con rumbo ≤ 4° (`PARK_GOAL_TH_IN_DEG`) | `park_ackermann.c` | El juez da "completo" con \|sen θ\|·batalla ≤ 2 cm (θ ≤ 6,2°); la meta aceptaba 10° y 3 de 24 rondas acababan "parcial". |
| Chequeo de arco nunca más estricto que el plan | `LIDAR System/orchestrator/wro_controller.c` | Con el coche a <1 cm de un obstáculo el plan se hacía con margen 0 y el arco se rechazaba con 0,5: coche parado para siempre (ccw 12, 17). |
| `PARK_LOST_MAX_SCANS 30` (antes 8) | `wro_controller.c` | Sigue unos barridos con odometría+giroscopio al perder el LiDAR antes de parar. No cambió ningún resultado medido. |

## 2. Código experimental, APAGADO (no borrar, hay trabajo útil)

| Flag (valor) | Qué es | Resultado |
| --- | --- | --- |
| `PARK_RETRACE_ENABLED 0` | Entrada siguiendo al revés la traza real de la salida (pose estimada cada 0,8 cm, seguidor con realimentación). | 4 de 8 rondas bajan a 1-3 repasadas, pero otras suben a 7 y la suma empeora. El tramo final es sensible. |
| `PARK_TWIN_ENABLED 0` | Gemelo como filtro de pose (odometría y giroscopio predicen, LiDAR corrige). | El error de pose casi no baja (el error del localizador no es ruido blanco). |
| `PARK_PROJECT_ENABLED 0` | Proyección de la pose 0,12 s hacia delante en el seguimiento. | Empeora. |

## 3. Límites que siguen abiertos (por prioridad)

1. **Contactos con el tope en las correcciones** (~5 de 56 rondas completas: cw 16, cw 18, ccw 13 roce con la pared, y otros).
   El motor sigue ~0,15 s tras la orden y se pasa de la holgura de 1 cm. Frenar antes al final del arco baja los contactos pero sube las repasadas y no es estable.
   **Esto lo ataca MIC** (`Motor Control/MIC/`): parar exactamente a la distancia pedida (error de 0,05 a 0,1 cm en avances cortos, simulado).
2. **Repasadas**: 2-5 en la ronda completa (media ~4), 2-10 con arranque en la pista (media 5).
   El mínimo (2) se obtiene cuando el plan inicial se ejecuta entero. Las rondas con plan de holgura real < 1 cm abortan y replanifican.
3. **Fallos residuales**: 5 de 56 rondas (9 %) en semillas 1-32, antes 17 %.
4. **Arranque en la pista**: el plan sale desde donde el LiDAR ve el cajón (u≈−36, w≈39), de 24 a 40 arcos. Probar una pose previa estándar mejora ccw (73→43) pero empeora cw (47→96): el simulador coloca el arranque distinto en cada sentido. Hay que ver si es artefacto del simulador.
5. **ccw/cw asimétricos** en la ronda completa (cw peor: planes más largos porque la salida termina ~2-3 cm más lejos).

## 3b. Elección entre planes candidatos (11 nov) y diagnóstico de abortos

- `park_ackermann_plan` (solo la entrada) ahora planea 5 variantes (`PARK_PLAN_VARIANTS`: precio de la repasada y de la holgura distintos) y elige la de menor
  puntuación = longitud + cambios de sentido + holgura que falta hasta 3 cm. También rescata poses donde el A* base se queda sin camino.
  Cada variante es un A* completo: en el P4 el tiempo de plan sube casi lineal y **no está medido** (en PC/WASM es despreciable).
- Medido en semillas nuevas 21-32 (24 rondas): 108 repasadas (media 4,5), 23 completas, 0 contactos. Un solo A* daba 59 frente a 53 en las semillas 21-26 y 11 de 12 completas.
  Dos variantes con holgura *exigida* (no premiada) nunca ganaron y se quitaron.
- **Causa de las repasadas que quedan:** todas las repasadas de más son abortos por arco no libre (`park_abort_arc`; los abortos por desvío fueron 0). La
  comprobación de cada arco usa `PARK_ENTRY_CHECK_MARGIN_CM` (0,5). Bajarla baja las repasadas pero deja el chasis tan justo que el parqueo acaba parcial:
  0,5 → 104 repasadas / 23 completas; 0,25 → 92 / 22; 0 → 74 / 20 y 2 contactos. Se queda en 0,5.
- `test_park_ackermann.c` comprueba que 9 poses de llegada vecinas dan planes comparables. **Una de las 9 no tiene plan** (límite conocido del A* de rejilla, con el
  paso de 3,5 y también con 2,5 cm): el controlador se queda esperando en ese caso.
- La vía de fondo que sigue abierta es modelar el retardo del servo y del motor dentro del A* (o ejecutar con realimentación), no más candidatos.
- `rebuild_wasm.sh` admite `EXTRA_CFLAGS="-DPARK_ENTRY_CHECK_MARGIN_CM=0.25f"` para probar constantes sin editar el código.

## 3c. Simetría ccw/cw (11 nov)

- Prueba con la pista reflejada exacta (misma pista con x→−x y colores cambiados; 8 semillas, `largo26d`): 7 de 8 parejas dan el mismo resultado. Quitar el desvío del volante (0,8°) y el
  sesgo del giroscopio **no** vuelve el simulador más simétrico (5 de 8), así que no son la causa. cw tarda unos 3-9 s más (la salida acaba con otro rumbo, 4° frente a 10°, y el parqueo cw sale más largo).
- La pareja que no coincidía (semilla 4) fallaba en cw con `wrong-side` a los 14 s: tras la salida el reactivo cambia de carril a pendiente alta, el morro pasa de 30° respecto al carril y la guarda
  `corner_max_heading_deg` (30) impedía detectar la esquina; el coche entraba ancho y no llegaba a pasar el letrero por el lado pedido. Con 45 ese caso termina y el resto no cambia
  (24 rondas con 3 letreros: 22 de 24, igual que antes; 16 rondas con 6 letreros: 16 de 16 en ambos).
- Sigue siendo cierto que la mayoría de los fallos en pistas aleatorias distintas por sentido es azar de pistas distintas, no una asimetría del código.

## 4. Preguntas sin responder

- ¿Cuenta el reglamento el número de repasadas? El límite de 3 viene de los documentos del proyecto, no de una regla comprobada.
- ¿El escenario que puntúa es la ronda completa (sale del cajón y vuelve a entrar) o el arranque en la pista?
- Constantes reales del chasis: ganancia de velocidad, desvío del volante, retardo del servo y del motor (en el simulador: 0,94, 0,8°, ~0,2 s, 0,16 s).

## 5. Ideas sin probar

- Parar arcos por distancia exacta con MIC (límite 1 de arriba): integrarlo en `hw_motors.c` y en el parqueo.
- Estimar en línea ganancia de velocidad y desvío del volante para un gemelo identificado (el offline predice a 0,5 s con p95 0,12 cm / 0,22° si las constantes son exactas, frente a 1,9 cm / 3,8° del modelo ingenuo).
- Elegir entre varios planes candidatos el de mayor holgura mínima, en vez de un coste suave.
- Medir en el chasis real el error de pose dentro del cajón (en simulación p95 0,6-0,9 cm).

## 6. Cómo medir

```
node simulator/tests/parkentry.js largo26 ronda 180 4        # 8 rondas (4 semillas x 2 sentidos)
DIR=cw SEED=4 node simulator/tests/parkentry.js largo26 ronda 180 1
node simulator/tests/parkentry.js largo26 entrada 90 4       # arranque en la pista
bash simulator/tools/bench/rebuild_wasm.sh                   # recompilar el WASM tras tocar el C
```

Lección de método: las semillas 1-8 estaban sobreajustadas (1 fallo de 16); en 9-20 la base falla 6 de 24. Validar siempre en semillas nuevas (21 en adelante).

## 7. MIC (Motor Intelligent Control)

- Módulo en `Motor Control/MIC/` (probado en PC con un motor simulado). Integrado en `hw_motors.c` tras `CONFIG_WRO_USE_MIC` (apagado por defecto);
  el firmware del P4 compila con el flag encendido y apagado (ESP-IDF 5.5.4). **No probado con el motor real.**
- Falta: calibrar R, masa, fricción, diámetro de rueda y cuentas por vuelta (README de MIC, sección 5); que el controlador pida avances exactos
  (`mic_request_move`) y rutas en vez de solo velocidad. Desde el 11 nov todo asume el motor de 600 rpm (el de 1000 rpm llegará después).
- Aviso: el lazo abierto actual (`WRO_MOTOR_MAX_CM_S=40`) iría ~4 veces más rápido con el motor de 600 rpm (~6 con el de 1000). Motores desactivados por defecto.

## 8. Avances exactos con MIC y trace_map (10 nov)

- **Avances exactos (`park_exact_moves`, apagado):** el comando lleva `move_cm`/`move_id` y el simulador los pasa a `mic_request_move`.
  Probado dos veces con MIC en el lazo (arco a arco, y fusionando rectas): peor que MIC solo en velocidad (suma de repasadas 113-134 frente a 86; sin fallos).
  El ejecutor de arcos está afinado para una planta lenta y mide por odometría/giroscopio; hay que rediseñarlo a nivel de tramo (no arco).
  MIC solo en velocidad: 0 fallos en 40 rondas, pero con repasadas irregulares (a veces 15-17).
- **trace_map:** rejilla con espacio libre, objetos con color y posición, plaza detectada por geometría, guardado y consultas. Ver `simulator/TRACE_MAP.md`.
  Falta probarlo con el LiDAR real. El parqueo **no** lo usa para localizar: medido en 24 rondas, la plaza del mapa da el mismo error de pose que
  `park_localize` (0,64 frente a 0,66 cm de media), así que no aporta mejora. Ver `simulator/MEDICION_TRACE_VUELTAS.md`.

