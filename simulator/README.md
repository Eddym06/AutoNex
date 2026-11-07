# Simulador WRO Future Engineers

> El modelo predeterminado se cambió a bicicleta para el nuevo chasis Ackermann.
> La geometría, el encoder y el parqueo aún no están adaptados ni validados;
> las cifras y pruebas históricas descritas abajo corresponden al chasis anterior.

Simula una ronda completa de la pista WRO 2024 con sensores realistas y **ejecuta el C real del firmware** (visión del S3 no incluida: la cámara virtual produce imágenes RGB565 que pasan por el mismo `vision.c`; el planificador y el controlador del P4 corren compilados a WebAssembly). No hay reimplementación en JavaScript de la lógica, y el robot **no ve la verdad del mundo**: solo lo que miden sus sensores simulados.

## Uso

```powershell
python simulator\build_simulator.py      # genera simulator\build\simulador.html (un solo archivo)
```

Abre `simulador.html` en el navegador. Casillas **Cajón de parqueo**, **Arrancar dentro** (el robot sale solo del cajón, hace las vueltas y vuelve a aparcar) y **Empezar en pista (aparcar)**: una demostración que arranca en el carril con las tres vueltas dadas, busca el cajón y aparca (~35 s). Controles: **Simular / Reiniciar**, velocidad de reproducción (0,25×–8×), **Pista** (preajustes), **Sentido**, **Pista aleatoria** (semilla y número de letreros), **Plano ⇄ JSON** (importar/exportar el diseño), capas visibles (LiDAR, ruta y corredor, trayectoria, cortinas), lado de paso de rojo y verde (con la opción de que el juez use la misma regla), parámetros del controlador y de la visión con *Aplicar y reiniciar* / *Valores por defecto*, y los de realismo: velocidad, ruido de LiDAR y cámara, sesgo del giroscopio, ganancia de velocidad, desvío del servo, luz, latencia de cámara y porcentaje de tramas con error.

> La interfaz se probó con un DOM simulado (`test_ui_smoke.js`), **no visualmente en un navegador**. Si algo se ve mal, es lo primero que conviene revisar.

## Qué modela

- **Pista** (reglamento 2026): campo de 3 × 3 m, carril de 100 cm, paredes de 10 cm, bloques 5 × 5 × 10 cm en asientos de 5 × 5 cm (2 × 3 por recta, modelo `seatModel` parametrizable), **8 líneas naranja/azul de 2 cm en V** (una pareja por esquina, a 30° de los muros) y el cajón de parqueo (20 cm de ancho, 1,5 × el largo del robot, topes magenta de 20 × 2 × 10 cm) en la sección de arranque.
- **LiDAR D6**: barrido continuo a 10 Hz con 400 rayos, ruido, pérdida de retornos, zona ciega y máscara del propio chasis. El robot se mueve durante el barrido (el firmware lo corrige con deskew).
- **Cámara**: proyección por trazado de rayos con cámara inclinada, ruido, viñeteo, nivel de luz, desenfoque, latencia y errores de bits.
- **Giroscopio**: sesgo y deriva aleatoria.
- **Encoders** (uno por rueda): ver «Encoders, pose estimada y herramientas de medida».
- **Robot**: chasis de 17 × 13 cm, **tracción diferencial** (dos ruedas motrices con motor, retardo, zona muerta, ganancia de velocidad 0,94 y desajuste entre ruedas; rueda libre delante). Puede girar en el sitio. El modo con servo (`drive: 'bicycle'`) sigue disponible. Colisiones y bloques empujables.
- **Juez** (2026): 1 punto por sección y por vuelta, 10 por no mover bloques, 3 por parar en la sección de arranque, 7 por arrancar en el cajón y completar una vuelta, 15 por aparcar dentro y en paralelo (la diferencia entre distancias de dos ruedas del mismo lado a la pared no supera 2 cm) o 7 si es parcial; tocar un tope termina la ronda; lado equivocado o bloque fuera de su círculo, también. Máximo 62 puntos.

## Pruebas

```powershell
node simulator\tests\test_sim_basics.js     # geometría, sensores, juez
node simulator\tests\test_ui_smoke.js       # la interfaz arranca
node simulator\tests\test_missions.js       # rondas completas (minutos)
python simulator\sequence_simulation.py     # secuencias de visión (129 comprobaciones) -> build\sequence-simulation\report.html
```

Herramientas de depuración en `simulator/tests/` (`batch.js`, `scenario.js`, `inspect.js`, `planat.js`, `mapdump.js`…): ejecutan lotes de diseños aleatorios o muestran el mapa/plan en un instante dado.

## Recompilar el firmware a WebAssembly

El simulador carga `web/wro_core.wasm`, versionado en el repositorio. Si cambias el C de `LIDAR System/main`, `Vision System/main` o `shared/`, recompílalo:

```powershell
$env:WASI_SDK_PATH = "C:\ruta\wasi-sdk"   # o usa -Download para bajarlo a simulator\.toolchain
.\simulator\wasm\build.ps1                # o: Vision System\tools\simulate.ps1 -Wasm
```

`simulate.ps1` avisa si el `.wasm` es más antiguo que el C. La interfaz entre JS y C está en `wasm/core_api.c`.

## Resultados medidos

- Pista vacía, ambos sentidos: 3 vueltas, parada en el tramo de salida, 55 puntos, ~125–140 s.
- Con el chasis de 17 × 13 cm: salir del cajón, 3 vueltas y aparcar sin bloques, 32 de 32 con 62 puntos.
- Banco de 100 pistas aleatorias con 7 letreros y cajón, a 23 cm/s: 99 perfectas (semillas 21–70) y 95 (semillas 71–120), sin contactos. `tests/plano_errores.json`: 62 puntos, 177 s. Estado y pendientes en `ESTADO.md` (raíz).

## Depurar un plano: `tools/depurar.py`

```powershell
python simulator\tools\depurar.py plano.json                 # ronda estricta: se detiene en la primera infracción
python simulator\tools\depurar.py plano.json --indulgente    # anota todas las infracciones y sigue, para ver todos los fallos
python simulator\tools\depurar.py plano.json --json info.json --ctrl "planner.speed_cm_s 15"
```

Ejecuta el plano en **el mismo simulador** (firmware C en WebAssembly, sensores, robot, juez; con el cuadro de configuración por defecto como la interfaz) y escribe un informe: resultado, fases, y para cada hallazgo (infracciones, paradas, bucles de reversa, contactos, tramos lentos, salida/parqueo que no terminan, avisos del plano) el instante, la posición, la sección, lo que decide el firmware, los letreros reales cerca y el **mapa del planificador** en ese instante, con el comando exacto para mirarlo más (`tools/debug_trace.js snap plano.json T`). Pensado para que cualquier persona o modelo localice el problema sin leer trazas enormes. Devuelve código de salida 1 si encuentra infracciones, paradas, bucles, o una salida del cajón o un parqueo que no terminan.

Los **avisos del plano** (`field.checkLayout`, también visibles en la interfaz bajo el panel del juez) detectan letreros consecutivos —o la salida del cajón y un letrero— que exigen cambiar de lado más rápido de lo que cabe: es un problema del plano, no del firmware. Casos de ejemplo: `tests/plano_errores.json` (tiene dos) y `tests/plano_corregido.json` (sale limpio, 62 puntos).

## Reproducir un caso

Desde la interfaz, **Plano ⇄ JSON** exporta el diseño. Para ejecutarlo sin navegador y ver qué decide el robot:

```powershell
node simulator\tests\layout.js plano.json                 # ronda completa y veredicto del juez
node simulator\tests\layout.js plano.json 20 40 1         # traza del segundo 20 al 40
node simulator\tests\ctrltrace.js plano.json 10 30 1      # rumbo del giroscopio, rumbo del recorrido, paredes, esquinas
node simulator\tests\dumpat.js plano.json 26 45           # mapa local del planificador en ese instante
```

`simulator/tests/plano_atasco.json` es un caso real reportado (se quedaba yendo y viniendo): la causa era que, tras esquivar pegado a la pared, el rumbo del recorrido se reanclaba con las paredes vistas en diagonal y la esquina no se contaba. Está corregido; ver *Limitaciones físicas* abajo para lo que queda de ese plano.

## Linea de carrera

Con `race_enabled 1` (apagada por defecto) el robot calcula tras la primera vuelta una linea de carrera: el mapa la dibuja coloreada por la velocidad que pide (casilla *linea de carrera*) y el panel *Controlador* tiene una fila con su estado. `node simulator/tools/race_check.js SEMILLA ccw|cw [--cfg ..] [--trace --lap 2]` muestra cuando se construye, que letreros conoce frente a los reales, y velocidad media por vuelta. El banco de 100 pistas (`tools/bench/big.sh`) admite `WEB=carpeta` (copia congelada de `simulator/web`), `OUT=carpeta`, `FROM`/`TO` y `SIGNS`.

## Cómo es la pista y qué se ve

- **Pista**: es la del reglamento WRO Future Engineers 2024 (campo de 3 × 3 m, paredes negras de 10 cm, carril de 100 cm con el cuadrado interior de 100 × 100 cm). Las medidas salen del reglamento oficial; las posiciones de los letreros no (ver abajo).
- **Líneas naranja y azul**: son las líneas del suelo de las cuatro esquinas (2 naranjas y 2 azules, 20 mm de ancho) que el reglamento pinta cruzando el carril. Sirven al robot para contar esquinas y vueltas sin odometría y, según el orden en que las cruza, para saber el sentido. La cámara del S3 ya envía la fila donde las ve; el controlador aún no las usa (cuenta esquinas con el LiDAR y el giroscopio).
- **Circulitos grises**: son los **asientos de letrero**: en la pista real cada letrero se coloca dentro de un círculo impreso de 85 mm; si el robot lo saca de ese círculo, termina la ronda. Haz clic en uno para alternar rojo / verde / vacío. **Las posiciones de los círculos son una suposición mía** (a 0,4 y 0,8 de cada recta y a −20 %, 0 y +20 % del ancho del carril): hay que medirlas en tu pista (`seatModel`).
- **Punto celeste**: posición del LiDAR en el chasis.

## Limitaciones físicas de los diseños

(Texto histórico del modo con servo; el chasis actual es diferencial de 17 × 13 cm y puede girar en el sitio.) En el modo con servo, el giro mínimo es de ~21 cm de radio (batalla 12 cm, volante 30°). Dos letreros seguidos que exigen cambiar de lado más rápido de lo que ese radio permite no tienen trayectoria hacia delante: por ejemplo, verde en el borde exterior y rojo en el centro 40 cm después, recorridos en el sentido contrario (necesita 36 cm de desplazamiento en ~23 cm de recta; un giro en S de este radio pide ≥ 41 cm). Con el recorrido en el sentido original sí cabe. Si la tercera vuelta se conduce al revés, ese par se queda sin paso. El generador aleatorio descarta ahora esos pares en **ambos** sentidos; un plano dibujado a mano puede contenerlos, y el robot se queda alternando reversa y avance hasta agotar el tiempo (o termina por `stuck_relax_s`, si se activa, con una infracción de lado).

## Qué NO está validado

- **Posición de los asientos de bloques**: el reglamento da el rango; el simulador usa un modelo parametrizable. Mídelo en tu pista y ajústalo.
- Montaje real de cámara y LiDAR (altura, inclinación, adelanto), ganancia de velocidad, recorte del volante: son valores de partida.
- El ruido del LiDAR es acotado y reproducible, no la distribución real del D6; el modelo no incluye reflejos ni pérdidas de paquetes UART.
- No hay ultrasónicos ni ToF en el modelo.

## Encoders, pose estimada y herramientas de medida

El simulador modela los encoders de las ruedas: cada rueda tiene su diámetro real (±1,5 % respecto al nominal que cree el firmware), patina a ráfagas y gira poco contra un obstáculo; el sensor entrega grados enteros a 50 Hz con 15 ms de retardo y pierde alguna muestra (`encoder` en las opciones: `resolutionDeg`, `hz`, `latencyMs`, `dropRate`, `diameterErrorPct`). Usan generadores aleatorios propios, así que no alteran las secuencias del LiDAR, la cámara ni el giroscopio.

En la interfaz, el panel *Controlador* muestra la **odometría** (camino, velocidad, patinajes, error frente a la verdad) y la **pose en el campo** del localizador (con su error), y el mapa dibuja la pose estimada: contorno amarillo = odometría corregida con el LiDAR, cruz magenta = solo encoders y giroscopio (casilla *pose estimada*). El error se calcula con la verdad del simulador; en el robot real no existe.

Por línea de comandos:

```powershell
node simulator\tools\odom_check.js 100 3        # error de la odometria sola (segundos, semilla)
node simulator\tools\arena_check.js 100 3       # error de la pose del localizador del campo
```

## Cómo se genera `build/simulador.html`

`simulator/build_simulator.py` toma `web/index.template.html`, mete dentro (en línea) todos los `.js` de `web/` y el firmware `wro_core.wasm` en base64, y escribe **un solo archivo** `build/simulador.html` que funciona sin servidor ni red. Sobrescribe el anterior cada vez. Pone en el título una marca de compilación (`fecha · fw hash`); el hash es el del `.wasm`, así que cambia solo si cambia el firmware. `Vision System/tools/simulate.ps1` lo ejecuta como último paso (después de pasar todas las pruebas); también puede lanzarse a mano con `python simulator\build_simulator.py`.

El `.wasm` en sí **no** lo genera ese paso: se compila con `simulator\wasm\build.ps1` (o `simulate.ps1 -Wasm`) a partir del C de `LIDAR System/main` y `Vision System/main`. Cambiar el firmware exige recompilar el wasm y después regenerar el html.
