# Análisis de rendimiento: pruebas pequeñas de CPU

Mediciones locales del 7 de noviembre de 2026 en un Intel Core Ultra 9 185H
(16 núcleos, 22 hilos lógicos), Node.js y C nativo compilado con GCC `-O2`.
No se ejecutó una ronda completa, no se usó GPU ni se midió el navegador.
El WASM actual no fue reconstruido: los valores de C nativo no son tiempos de
WebAssembly y no deben sumarse para pronosticar la velocidad final.

## Resultados orientativos

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

1. Medir el simulador real en navegador con WASM reconstruido, separando cámara,
   detector, LiDAR, mapa, planificador, parqueo y dibujo; mostrar velocidad
   real y tiempos medios/p95. Hacerlo en escenarios cortos antes de las rondas.
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
