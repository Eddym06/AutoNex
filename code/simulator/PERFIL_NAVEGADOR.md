# Perfil de rendimiento en navegador

Medición exploratoria del 7 de noviembre de 2026, en Windows con Intel Core
Ultra 9 185H (16 núcleos / 22 hilos lógicos). Se usó Chrome Headless 156,
ventana 754 × 480, `--disable-gpu`, y el WASM reconstruido con wasi-sdk 25
desde las fuentes locales. La prueba usa JavaScript y WebAssembly en CPU, sin
GPU para los cálculos. No se ejecutaron rondas completas.

Los tiempos se midieron envolviendo funciones con `performance.now()`: incluyen
el coste de instrumentación y son tiempos de reloj, no un desglose oficial del
planificador del sistema operativo. La resolución del reloj redondea muchas
muestras cortas a 0,1 ms. Algunas funciones están anidadas en `tick`; sus
tiempos no deben sumarse de nuevo al total de `tick`.

## Velocidad alcanzada

Se avanzó el mismo escenario aleatorio durante unos 4 segundos reales por
ajuste, en tramos sucesivos; son condiciones cambiantes, no una comparación
controlada con reinicio idéntico. Las cifras son orientativas:

| Velocidad solicitada | Velocidad observada | FPS aproximados de la interfaz |
| ---: | ---: | ---: |
| 1× | 1,18× | 51 |
| 4× | 2,05× | 24 |
| 8× | 2,34× | 26 |

En otra medición breve a 4× se observaron 2,68×: la carga del equipo y el tramo
influyen. El bucle de `web/app.js` redondea los pasos hacia arriba y descarta el
resto al acabar el cuadro; eso puede explicar que 1× supere el tiempo real.
También limita el tiempo de cálculo por cuadro y puede quedarse por debajo del
objetivo en velocidades altas. No se midió navegador visible ni otra máquina.

## Desglose de un tramo a 4×

4,03 s reales, 8,28 s simulados, 828 pasos de 10 ms y 249 imágenes. El tiempo
total acumulado dentro de `tick` fue 3498 ms. Cada fila, salvo dibujo, está
incluida en ese total; los porcentajes reflejan solo este tramo instrumentado.

| Componente | Llamadas | Total | Media por llamada | Porcentaje de `tick` |
| --- | ---: | ---: | ---: | ---: |
| Renderizado de cámara virtual | 249 | 2682 ms | 10,77 ms | 76,7 % |
| Controlador WASM + exportación de resultados | 83 | 692 ms | 8,34 ms | 19,8 % |
| Avance del LiDAR | 828 | 50,9 ms | 0,061 ms por paso | 1,5 % |
| Visión WASM sobre RGB565 | 249 | 30,9 ms | 0,124 ms | 0,9 % |
| Juez | 828 | 10,3 ms | 0,012 ms | 0,3 % |
| Física del robot | 828 | 8,1 ms | 0,010 ms | 0,2 % |
| IMU, encoder, enlace y watchdog | varias | ~6,4 ms | — | ~0,2 % |

El callback de dibujo y simulación acumuló 3588 ms en 97 cuadros. Restando el
tiempo medido dentro de `tick`, quedaron unos 90 ms (~0,9 ms/cuadro) para dibujo
y demás trabajo del callback; esto es una estimación, no un perfil de cada
llamada de Canvas. En otra muestra, la cámara tuvo media de 8,43 ms y p95 de
11 ms; el controlador + exportación, media de 5,95 ms y p95 de 8,2 ms.

## Dónde pesa la cámara

Microprueba en el mismo Chrome, 80 imágenes por variante, con una pista de siete
letreros y sin movimiento angular:

| Variante | Media por imagen |
| --- | ---: |
| Normal | 7,55 ms |
| Sin ruido | 5,61 ms |
| Sin líneas del suelo | 3,30 ms |

Son variantes separadas y cambian la imagen; las diferencias no son aditivas ni
equivalen a una optimización gratuita. El código recalcula `G.boxAxes` para cada
píxel y cada línea (`web/sensors.js`), y genera ruido gaussiano por canal. El
renderizado es el candidato principal para precalcular datos, reducir trabajo
repetido y, después, comparar Rust/WASM o GPU manteniendo una referencia de
fidelidad. El detector C ya ocupa una fracción mucho menor en esta muestra.

## Prueba breve de salida del cajón

En dos ejecuciones cortas, con el modo de salida experimental, el robot terminó
detenido en `PARK_EXIT` con resultado `stalled` tras ~10,55 s simulados. En una
captura, el localizador reportaba `lost=0` y el controlador `STOP`; esto no
identifica por sí solo la causa. La media de controlador + exportación fue
1,4–1,9 ms/barrido en esas ejecuciones, con picos de 33–44 ms. No se completó
una ronda. **Las pruebas geométricas anteriores no validan esta integración.**

## Orden de optimización

1. Corregir la salida del cajón y mantener sus escenarios breves como regresión.
2. Optimizar cámara conservando las líneas, el ruido y RGB565; comparar imágenes
   y secuencias antes y después, además del tiempo por cuadro.
3. Separar controlador de la copia/exportación de mapa para medirlos por separado;
   reducir copias y frecuencia de exportación si son significativas.
4. Mover simulación a un Web Worker y limitar la frecuencia de dibujo. Esto ayuda
   a la respuesta de la interfaz, pero no hace más rápido por sí solo cada paso.
5. Corregir el acumulador temporal del bucle y mostrar la velocidad real.
6. Medir Rust/WASM o WebGPU para la cámara después de establecer la referencia;
   incluir el coste de transferir RGB565 de vuelta al detector C. No hay todavía
   una comparación medida ni una promesa de aceleración.

El script `tools/profile_browser.js` conecta a un Chrome con CDP en localhost
(puerto 9328 por defecto) y abre `build/simulador.html`. Es necesario reconstruir
primero el WASM y el HTML; ejecutar el navegador con GPU desactivada para repetir
estas condiciones. El perfil dura unos 12 segundos reales y no termina una ronda.

## Repetición tras optimizar la cámara

En Chrome sin GPU, el tramo solicitado a 4× alcanzó aproximadamente 4,00×
durante 4 segundos reales. La cámara consumió 3,59 ms por cuadro (482 cuadros),
frente a 10,77 ms por cuadro en la medición anterior. El controlador con
exportación consumió 5,29 ms por barrido (160 barridos). Son mediciones de
tramos diferentes, por lo que la comparación indica una mejora, pero no aísla
por completo las diferencias de escenario y carga.

La comparación directa de 12 escenas con la implementación anterior produjo
RGB565, RGBA y secuencia aleatoria idénticos. En la prueba de perfil, el tramo
posterior acabó con `wrong-side`; no constituye una validación de carrera.
La salida Ackermann desde el cajón también sigue pendiente: una prueba de 12 s
mostró una parada sin contacto. No se debe considerar validada para el robot.

Actualización: con una desaceleración progresiva al terminar cada arco, seis
pruebas aisladas de salida (tres semillas por sentido) completaron la maniobra
sin contactos, en 20,41–55,21 s simulados. Es una mejora experimental: el caso
de 55 s debe optimizarse y todavía faltan pruebas de entrada y calibración del
chasis real. La regresión está en `tests/test_parking_exit.js`; se detiene al
entrar en RUN y no ejecuta la carrera.
