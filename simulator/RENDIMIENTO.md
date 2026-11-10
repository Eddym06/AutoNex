# Análisis de rendimiento: pruebas pequeñas de CPU

Mediciones locales del 7 de noviembre de 2026 en un Intel Core Ultra 9 185H
(16 núcleos, 22 hilos lógicos), Node.js y C nativo compilado con GCC `-O2`.
El WASM se reconstruyó con wasi-sdk 25 y la medición en el navegador está en la
sección siguiente; los tiempos de C nativo no son tiempos de WebAssembly y no se
deben sumar.

## Medición en el navegador (WASM versionado)

Chrome, `simulator/build/simulador.html`, pista aleatoria de la interfaz y robot por
defecto. Un paso de simulación (física del carrito, LiDAR y —uno de cada 30— un cuadro
de cámara con visión y trama del enlace) cuesta **1,5 ms de CPU**, con el controlador C
dominando el total: 10 barridos/s a ~10 ms cada uno suman la mayor parte del paso.

| Medida | Valor |
| --- | ---: |
| Costo por paso de simulación | 1,5 ms |
| Techo sostenible (sin contar dibujado) | ~6,9 s simulados por segundo real |
| Medido a 8× pedidos, ventana en primer plano | 5,9× reales |
| Velocidad pedida frente a alcanzada | 1× y 4× sobran; 8× no se sostiene |

La interfaz ya muestra la velocidad real (`actualSpeed`), así que la diferencia se ve en
pantalla. Nota de método: con la ventana de Chrome oculta o tapada, el navegador limita
las llamadas de dibujo a ~1 Hz y la cifra que se lee (0,1×) no mide el simulador; hay que
medir con la ventana visible. Acelerar de verdad exige sacar la simulación del hilo de la
interfaz (Web Worker) o bajar el costo del controlador, no tocar el dibujado: en las mismas
mediciones el dibujado entero costaba unos 3 ms por cuadro.

## Resultados orientativos (C nativo, componentes aislados)

| Operación aislada | Promedio o caso medido |
| --- | ---: |
| Cámara RGB565 de 160 × 120, configuración normal | 5,6–6,8 ms por cuadro |
| Cámara sin ruido | 4,0–4,2 ms por cuadro |
| Cámara sin líneas del suelo | 3,4–3,8 ms por cuadro |
| LiDAR: 400 rayos | 0,32–0,37 ms por barrido |
| Física del vehículo: un paso | 0,0018–0,0027 ms por paso |
| Controlador C nativo con 360 puntos sintéticos | 8,7 ms por barrido |
| Parqueo Ackermann: salida del cajón | 0,5 ms, 116 estados explorados |
| Parqueo Ackermann: tres aproximaciones | 11–54 ms según la pose y la carga del equipo |

La cámara se midió con 80–100 cuadros después de calentamiento del motor JS;
LiDAR con 100 barridos, física con 10 000 pasos y controlador con 150 barridos.
El controlador recibió un escenario sintético repetido, sin imágenes reales ni
cambios de posición. Las cifras de parqueo son muestras individuales, no una
distribución estadística. En un episodio real pueden variar mucho. El navegador,
WebAssembly, el dibujado, el detector de visión, las colas y la integración de
sensores no se han medido aquí.

En la cámara, quitar por separado el ruido o las líneas cambia el trabajo y el
resultado; estos experimentos indican dónde investigar, no son optimizaciones de
fidelidad equivalente y sus diferencias no se deben sumar.

## Por qué tarda

- `web/sensors.js` calcula suelo y líneas para cada píxel, recalcula los ejes
  de las líneas en el bucle interior y genera ruido gaussiano para cada canal.
  También prueba los objetos por columna y ordena las intersecciones.
- Cada barrido activa el mapa, el planificador y la localización del controlador
  C. La medición nativa anterior mide el conjunto, no identifica qué subfunción
  domina. Un intento de perfil con gprof en Windows no produjo muestras útiles.
- El nuevo parqueo explora estados discretos y verifica colisiones a lo largo
  de cada arco. Los casos difíciles expanden miles de estados. Su memoria y
  tiempo de ejecución en el ESP32-P4 siguen sin medirse.
- `web/app.js` calcula y dibuja en el hilo de la interfaz y limita cada bloque
  de cálculo a ~26 ms. Las velocidades elegidas son objetivos, no una garantía.

## Mejoras recomendadas, por orden

1. ~~Medir el simulador real en navegador con WASM reconstruido~~ (hecho: ver la
   tabla de arriba; el controlador C es el cuello de botella). Falta separar
   mapa/planificador/parqueo dentro del barrido y registrar p95 en escenarios cortos.
2. En cámara, precalcular ejes de líneas, coordenadas invariantes y tablas de
   píxeles; reducir objetos temporales. Mantener semilla, ruido y salida RGB565
   equivalentes con comparaciones por imagen.
3. Ejecutar la simulación en un Web Worker y dibujar con una frecuencia menor;
   esto mejora la respuesta de la interfaz, pero no garantiza acelerar un paso.
4. Reutilizar una ruta de parqueo y medir sus expansiones; mejorar la búsqueda
   y su memoria sin sacrificar las comprobaciones de colisión. Medir en P4.
5. Para muchas pistas independientes, usar varios procesos o workers de CPU con
   límites configurables. No cambia la velocidad intrínseca de una sola ronda.
6. Probar Rust/WASM o WebGPU para cámara/LiDAR solo tras identificar el cuello
   de botella. Comparar con el mismo escenario y contemplar las copias de datos.
   El controlador del robot debe seguir compartiendo el mismo C que el firmware.

## Repetir los microbenchmarks

`node simulator/tools/profile_components.js` mide componentes JavaScript sin
arrancar una ronda. `simulator/tools/profile_controller.c` es el programa de la
prueba nativa en Windows: se compila con GCC junto al controlador, planificador,
localizador, odometría, mapa y protocolos. Ninguna de estas pruebas usa GPU.

## Perfil por etapas y prueba de presupuesto (9 de noviembre de 2026)

En Windows, ejecutar `powershell -File simulator/tools/profile_controller.ps1` con
GCC disponible. El ejecutable se guarda en la carpeta temporal. El enlazador
instrumenta las llamadas del controlador sin cambiar el firmware. Se registran
p50, p95 y máximo por llamada; el barrido sintético repetido no representa una
ronda completa. El escenario usa 360 retornos y desactiva el parqueo: por eso no
produce muestras del localizador del cajón. Para A* se incluyen 40 llamadas
separadas a cuatro poses fijas (no son tiempos observados dentro de una ronda).
La instrumentación añade una pequeña sobrecarga. No mide WASM ni el ESP32-P4.

Una ejecución local con 150 barridos dio: barrido p50/p95 **7,95/9,20 ms**;
mapa **0,063/0,074 ms**; planificador local **7,75/8,90 ms**; localizador de
arena **0,068/0,094 ms**. El A* de las cuatro poses dio p50/p95
**29,7/53,8 ms**. Son cifras orientativas, no límites universales ni una suma
válida de percentiles. En este escenario el mapa no es el cuello de botella;
no hay base para recomendar una actualización incremental como primera mejora.

Para detectar regresiones en una máquina de referencia se pueden fijar límites
explícitos en milisegundos con `AUTONEX_SCAN_P95_MS` y
`AUTONEX_PLANNER_P95_MS`. El comando falla si se supera alguno; sin esas
variables informa sin imponer un umbral dependiente del equipo. Ejemplo en
PowerShell: `$env:AUTONEX_SCAN_P95_MS=15; ./simulator/tools/profile_controller.ps1`.
Para una comparación útil hay que usar el mismo equipo, compilador, carga y
escenario. Faltan escenarios reales de entrada/salida instrumentados y medir en P4.
