# Revisión estática del simulador

> **Trabajo posterior:** se han preparado correcciones para la mayoría de los
> puntos descritos abajo. Este documento conserva los hallazgos originales; no
> acredita su cierre. El WASM ya se reconstruyó y la salida Ackermann pasó
> pruebas aisladas con LiDAR en CPU; faltan pruebas de entrada, rondas completas,
> mediciones reales y validación de pistas oficiales.

Revisión del código fuente sin ejecutar, compilar ni abrir el simulador. Los
hallazgos son deducciones del código, no resultados de pruebas. El WASM
versionado es un binario: no se ha podido verificar su equivalencia con el C
actual. Las pruebas automáticas, las herramientas y los planos existentes son
material de apoyo; la física y los resultados históricos no validan el nuevo
chasis.

## Prioridad alta

1. **El parqueo Ackermann no está implementado aunque la interfaz lo ofrece.**
   `web/sim.js:79-88` fuerza `parking_enabled 0` y `start_in_lot 0` en bicicleta,
   mientras `web/app.js:34-39` y `web/field.js:161` sí colocan el robot dentro
   del cajón cuando se marca esa opción. El juez aún considera esa salida para
   la puntuación (`web/judge.js:197`). Por tanto, la posición inicial y la misión
   no representan el mismo escenario. El controlador de parqueo antiguo requiere
   giros en el sitio. Falta desarrollar una maniobra con avance y reversa y
   deshabilitar o señalar claramente las opciones no disponibles.

2. **El simulador no ejercita el encoder del vehículo nuevo.**
   `web/sim.js:86-87` fuerza `odometry_enabled 0` en bicicleta; `web/sensors.js`
   genera dos encoders y `wasm/core_api.c` los entrega a
   `wro_controller_encoders`, no al nuevo `wro_controller_axle_encoder`. No
   modela encoder del motor, reducción externa ni diferencial. `odom_check.js`
   tampoco puede evaluar la odometría de eje en esta configuración. Hace falta
   una fuente única de cuentas del motor y llevarla por la misma interfaz de
   odometría usada por el P4.

3. **La vía de seguridad simulada no es la del P4.** El simulador llama
   directamente a `wro_controller_scan` y `wro_controller_watchdog`
   (`wasm/core_api.c:251-273`); no usa el ciclo `p4_runtime_step`, que también
   vigila la edad de la IMU y filtra muestras inválidas. `web/sim.js:199-201`
   comprueba únicamente el watchdog exportado para LiDAR. Tampoco reproduce el
   bloqueo de tracción, la calibración del servo ni el driver físico. No se
   pueden inferir de este simulador las paradas reales del P4.

4. **El WASM puede ser antiguo sin impedir la simulación.** `web/core.js` carga
   `wro_core.wasm`, que está versionado; `simulator/build_simulator.py` lo
   incorpora sin reconstruirlo ni comprobar su procedencia. El aviso de
   `Vision System/tools/simulate.ps1` compara fechas, no contenido, y no impide
   continuar. El WASM incluido no fue reconstruido durante esta revisión.
   Se necesita reconstrucción reproducible o un manifiesto con hashes de fuentes
   y compilador antes de atribuir un resultado al código actual.

5. **Pruebas históricas incompatibles con el modo predeterminado.**
   `web/robot.js:20` selecciona bicicleta, pero `tests/test_sim_basics.js:156`
   espera explícitamente diferencial y pivote. `tests/test_missions.js:75-105`
   exige salida y parqueo que el modo bicicleta desactiva. No se han ejecutado;
   las expectativas son estáticamente contradictorias. Separar escenarios
   diferenciales históricos de las pruebas específicas del vehículo nuevo.

## Prioridad media

6. **Las pistas aleatorias excluyen casos exigentes.** `web/field.js:222-285`
   filtra asientos cerca del inicio y descarta combinaciones con una heurística
   de pendiente y dimensiones antiguas. Estos supuestos pueden reducir los casos
   difíciles y no demuestran que todos los planos oficiales queden representados.
   Documentar la distribución y probar por separado un conjunto sin el filtro y
   escenarios determinados por las reglas de la competencia.

7. **Geometría provisional y no centralizada.** `web/robot.js:20-23` conserva
   dimensiones supuestas; `web/field.js:134,222` y `web/app.js:39` usan un largo
   de 17 cm para el cajón. Es posible cambiar la física sin cambiar las medidas
   de pista o del planificador. Reunir las dimensiones en un perfil común y
   marcarlo como provisional hasta medir el carrito.

8. **El juez depende del paso temporal.** `web/judge.js:69` suma 0,01 s por
   actualización para detectar detención, pero `web/sim.js:27` permite cambiar
   `dt`. Si cambia el paso, el tiempo de detención medido deja de ser el real.
   Acumular tiempo transcurrido a partir de las marcas de tiempo.

9. **Exportar e importar un plano puede cambiarlo.** `web/field.js:323-328`
   no guarda `seatModel`, `wallT` ni `robotBody`; `parse` tampoco los recupera.
   Un plano con estos parámetros puede producir una pista o posición inicial
   distinta al volver a cargarlo. Incluirlos y validar el formato versionado.

10. **La visualización LiDAR y un ajuste no coinciden con la física.**
    `web/app.js:138` dibuja los puntos con un adelanto fijo de 12 cm, mientras
    `web/sensors.js:17` usa 3 cm por defecto y admite configuración lateral.
    Además, `web/app.js:56` pasa `wheelMismatch` desde el campo «Desajuste de
    motores», pero `web/robot.js` solo lo usa en modo diferencial. En bicicleta
    ese control no cambia la simulación.

11. **El juez puede perder intersecciones parciales del cajón.**
    `web/judge.js:165-178` determina si hay parqueo parcial contando esquinas
    del robot dentro del rectángulo. Dos rectángulos pueden cruzarse sin que
    ninguna esquina del robot quede dentro; en ese caso se devuelve `null` pese
    a haber intersección. Conviene usar intersección entre polígonos y comparar
    contra ejemplos oficiales.

12. **Una línea de carrera puede dibujarse con datos anteriores.**
    `wasm/core_api.c:236-241` solo copia los puntos cuando cambia el número de
    estaciones. Si se calcula otra línea válida con el mismo número, conserva la
    geometría anterior. Actualizar al cambiar la trayectoria o su versión.

## Herramientas y pruebas

- `tests/test_ui_smoke.js` usa un DOM simulado; no valida el navegador ni la
  apariencia. Sus elementos ficticios se crean al solicitarlos, por lo que la
  comprobación de IDs no verifica realmente que todos existan en el HTML.
- `tools/bench/big.sh` elimina archivos que coinciden con el tag y usa rutas sin
  comillas; revisar la validación del tag, las rutas con espacios y la detección
  de fallos de cada proceso antes de usar los resultados como evidencia.
- `tools/hug_check.js` y `tools/race_check.js` formatean `elapsed` sin manejar
  una ronda aún en curso, lo que puede interrumpir el informe. Varias herramientas
  de `tests/` son depuradores históricos, no pruebas automatizadas.
- `sequence_simulation.py` usa su propio modelo geométrico simplificado de
  cámara y vehículo; sus resultados de visión no validan la dinámica completa.

## Orden recomendado

1. Definir perfil de geometría, transmisión y servo con datos reales.
2. Conectar encoder y seguridad al mismo flujo que el firmware; asegurar que el
   WASM corresponde a las fuentes.
3. Diseñar parqueo y salida Ackermann y adecuar interfaz y pruebas.
4. Corregir el juez, la exportación de planos y la visualización; ampliar las
   pruebas de pista y contrastarlas con datos reales.
# Actualización de la revisión

Este documento conserva una revisión histórica. Desde entonces se reconstruyó el
WASM, se verifica un manifiesto de fuentes y binario, se cambió al encoder del
eje Ackermann, se corrigió el tiempo del juez y se dibuja la posición del LiDAR.
Las limitaciones que continúan incluyen la integración parcial del runtime P4,
la física simplificada de contactos y la falta de validación de la misión completa.
