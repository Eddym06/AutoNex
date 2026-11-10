# Línea base de medición: parqueo y mapa

Mediciones del 9 de noviembre de 2026. Son resultados del simulador y de C nativo
en esta computadora; no son mediciones del ESP32-P4 ni del chasis físico. La finalidad
es decidir qué necesita mejorar antes de añadir un modelo ML o controladores nuevos.

## Localización de campo, perfil Ackermann largo26

Comandos ejecutados (pista sin letreros ni parqueo, 100 s simulados, sentidos opuestos):

```
node simulator/tools/arena_check.js 100 1 --profile largo26
node simulator/tools/arena_check.js 100 2 --profile largo26
```

Se compara la pose de campo que expone el controlador con la pose verdadera del
simulador, cada ~0,5 s; puede haber desfase entre el último barrido y el instante
de comparación. Se cuentan únicamente muestras con pose considerada válida. No es
una medición directa del error de los puntos del mapa ni del localizador del cajón.

| Medida | Semilla 1, antihorario | Semilla 2, horario |
| --- | ---: | ---: |
| Pose disponible | 196/199 muestras | 196/199 muestras |
| Posición: p50 / p95 | 2,77 / 3,63 cm | 2,83 / 4,02 cm |
| Posición: media / máximo | 2,67 / 3,84 cm | 2,70 / 4,73 cm |
| Rumbo: mediana / p95 | 0,38 / 8,62 grados | 0,30 / 7,58 grados |
| Rumbo: máximo | 8,86 grados | 8,31 grados |

Las muestras no son independientes y dos escenarios no establecen un límite general.
Los errores de rumbo y la comparación temporal deben investigarse, especialmente en
giros, antes de atribuirlos únicamente al sensor o a la localización. La disponibilidad
se refiere a las muestras del experimento, no a todas las situaciones posibles.

## Parqueo: referencia de las ocho rondas

Con `node simulator/tests/parkentry.js largo26 ronda 180 4`, sin letreros, cuatro
semillas y dos sentidos, las ocho rondas terminaron con 62 puntos, parqueo completo
y cero contactos. Los cambios de sentido de la entrada fueron **2, 2, 3, 4, 4, 4,
4, 5**. El juez mide la verdad del simulador. La comprobación automática opcional
`MAX_REVERSALS=3` detecta el incumplimiento; no se pretende que pase actualmente.

## CPU nativa: escenario sintético

`powershell -File simulator/tools/profile_controller.ps1`: 150 barridos sintéticos
sin parqueo; p50/p95 del barrido 8,66/10,30 ms, mapa local 0,069/0,096 ms,
planificador local 8,43/9,70 ms. Cuatro poses fijas de A* repetidas diez veces:
p50/p95 agregados 31,35/60,98 ms. No son p95 de una ronda de parqueo ni tiempos del
P4; las mediciones pueden variar según la carga. Los percentiles por etapa no deben
sumarse. El localizador del cajón no se ejecutó en este escenario.

## Qué falta medir antes de elegir mejoras

- Error del mapa respecto a las superficies y objetos verdaderos, incluido sesgo,
  p50/p95, máximos, puntos falsos/ocultos y cambios entre vueltas. La pose del coche
  no sustituye esta medición. Medir por separado el mapa local reconstruido y la
  memoria persistente de objetos; la cifra histórica de 1,1 cm de letreros no es
  precisión garantizada de todo el mapa.
- Pose relativa al cajón durante entrada y salida; comparar verdad y mediciones en
  instantes sincronizados. Registrar desviación de cada arco y holgura mínima de
  todas las esquinas, no solo si hubo contacto.
- Cámara: detección de cada tope magenta, falsos positivos, asociación con LiDAR y
  distancia/ángulo/iluminación; confirmar si realmente se conserva el cajón entre
  vueltas. Probar posiciones del cajón variadas y arranque desde la pista.
- Costes en escenarios completos, incluyendo localizador del cajón y A* por barrido,
  y tiempo de cálculo y memoria en el P4 bajo carga. Repetir con relojes y equipos
  documentados.
- Chasis real: calibración de encoder, IMU, montajes y servo; comparación con una
  referencia externa de posición/orientación, retardos, repetibilidad y frenado.
  El repositorio contiene drivers, pero no prueba que el cableado ni la integración
  real estén validados; `ESTADO.md` mezcla generaciones de chasis.
- Evaluar comparación de predicción y control actuales antes de entrenar ML. No hay
  mediciones que demuestren actualmente error máximo de 1 cm, necesidad de ML ni
  que un controlador concreto cumpla el presupuesto del P4.

## Ampliación: localización por barrido, parqueo y mapa local

### Pose de campo al entregar cada barrido

`node simulator/tools/measure_arena_sync.js 1 100` y `... 2 100`, perfil largo26,
pista vacía. Se compara la pose estimada con la verdad **en el instante de entrega**
del barrido, antes del siguiente paso físico. La captura del barrido terminó 10 ms
antes de la entrega; esto no elimina todos los desfases. Se clasificó como giro una
velocidad angular verdadera superior a 15°/s. Hubo pose válida en 985 de 999 barridos
por recorrido. Las muestras están correlacionadas.

| Semilla y tramo | Posición p50 / p95 (cm) | Rumbo p50 / p95 (°) | Muestras |
| --- | ---: | ---: | ---: |
| 1, recta | 1,24 / 1,53 | 0,14 / 0,48 | 783 |
| 1, giro | 0,64 / 1,11 | 3,17 / 3,38 | 202 |
| 2, recta | 0,91 / 1,26 | 0,14 / 0,63 | 783 |
| 2, giro | 1,69 / 2,61 | 2,22 / 2,47 | 202 |

Estas cifras no son directamente equivalentes a las del muestreo cada 0,5 s anterior:
aquella herramienta puede comparar un comando antiguo con una pose verdadera más
reciente. Los resultados por barrido muestran que parte de las diferencias anteriores
puede proceder del momento de comparación; no demuestran que todo el error de giro sea
un problema de sincronización.

### Pose relativa durante las ocho entradas de ronda completa

`node simulator/tools/measure_parking.js 4 180`, perfil largo26, cuatro semillas por
sentido. Pose verdadera y estimada se comparan a la entrega del barrido; el barrido
terminó antes y sus rayos corresponden a diferentes instantes. La maniobra se define
para esta medida desde la primera orden de reversa. La holgura es la distancia mínima
entre los rectángulos verdaderos del chasis y paredes/topes, muestreada cada 10 ms de
simulación durante PARKING; no es una medición del chasis físico.

| Sentido / semilla | Cambios | Error posición p50 (cm) | p95 (cm) | Máx. (cm) | Rumbo p95 (°) | Holgura mínima (cm) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| ccw 1 | 2 | 0,73 | 1,14 | 1,33 | 2,79 | 1,33 |
| ccw 2 | 2 | 0,55 | 1,12 | 2,36 | 2,97 | 0,82 |
| ccw 3 | 3 | 0,59 | 1,19 | 1,30 | 2,66 | 0,93 |
| ccw 4 | 4 | 0,50 | 1,09 | 1,46 | 1,90 | 0,88 |
| cw 1 | 4 | 0,74 | 1,13 | 1,27 | 2,15 | 0,97 |
| cw 2 | 4 | 0,76 | 1,38 | 2,15 | 2,78 | 1,76 |
| cw 3 | 4 | 0,73 | 1,43 | 2,46 | 2,59 | 0,53 |
| cw 4 | 5 | 0,79 | 1,22 | 2,60 | 2,23 | 0,52 |

Todas terminaron con 62 puntos y cero contactos; en los barridos durante PARKING no
se reportó pérdida de localización. El error de pose y la holgura son cantidades
diferentes: no deben restarse directamente ni interpretarse como probabilidad de
colisión. El archivo `mediciones/parqueo_8_rondas_2026-11-09.jsonl` conserva valores
por ronda y tiempos de `core.scan` en Node/WASM, incluida copia de datos; no son tiempos
puros del A* ni del P4. Las pruebas se ejecutaron con otras mediciones compartiendo CPU.

### Celdas ocupadas del mapa local: coche inmóvil

`node simulator/tools/measure_local_map.js` ejecuta 50 barridos estáticos por sentido,
con ruido y pérdidas simulados y sin objetos de colores extra. Cada centro de celda
ocupada se compara con el sólido verdadero más cercano; un centro dentro del sólido
cuenta distancia cero. La rejilla mide **2,5 cm por celda** y se reconstruye en cada
barrido, no es un mapa persistente de toda la pista.

| Sentido | Muestras de celdas ocupadas | Distancia p95 al sólido | Más de 2,5 cm | Más de 5 cm |
| --- | ---: | ---: | ---: | ---: |
| ccw | 13 694 | 1,50 cm | 447 | 8 |
| cw | 13 943 | 1,50 cm | 457 | 10 |

El máximo fue 6,5 cm en ambos casos. Las celdas y barridos se repiten y no son muestras
independientes. Esta métrica **no mide cobertura, falsos libres, obstáculos ocultos,
mapa global, movimiento ni exactitud del sensor real**; una celda puede estar cerca de
un sólido sin corresponder al retorno correcto. No permite afirmar un error de mapa
inferior a un centímetro.

### Cámara magenta y asociación con LiDAR

En dos demostraciones desde pista (semilla 1, ambas direcciones), la cámara produjo
alguna detección magenta en 778/964 cuadros antihorarios y 728/925 horarios. No se
midió visibilidad verdadera ni falsos positivos, por lo que no son sensibilidad ni
precisión del detector. Entre 321/308 barridos totales, el mapa local registró alguna
entrada magenta en 259/242, pero **ninguna se marcó confirmada por LiDAR** en estos dos
escenarios. El asociador aplica una comprobación de compacidad pensada para pilares;
es una posible causa que debe comprobarse, no una causa demostrada. El detector de
visión conserva una sola caja de parqueo magenta por cuadro y la memoria global de
letreros filtra rojo y verde: no demuestra memoria persistente de ambos topes.

Para reproducir: `$env:MODE='entrada'; $env:DIR='ccw'; $env:SEED='1'; node
simulator/tools/measure_parking.js 1 45` (cambiar `ccw` por `cw`). No mover el entorno
ni cambiar el controlador durante una medición; reiniciar o borrar las variables de
entorno para volver a correr las ocho rondas.

## Investigación adicional: asociación magenta y mapa con movimiento

### Por qué se rechazan candidatos magenta

`node simulator/tools/diagnose_parking_association.js` evaluó 40 poses inmóviles,
ambos sentidos, sin letreros. En 18 poses hubo entradas magenta, con 198 muestras
repetidas de barrido. En las 198, la ventana aproximada de ángulo y distancia
incluyó al menos un retorno a menos de 2 cm de un tope verdadero; el retorno más
cercano al sensor en esa ventana también estaba cerca de un tope. El filtro de
compacidad rechazó todos los candidatos y el mapa reportó cero confirmaciones.

El diagnóstico reconstruye los filtros en JavaScript a partir de retornos y la
estimación no confirmada; con el coche quieto coincidió con el resultado de C en
estas muestras. La cercanía a un tope no prueba que la caja de la cámara corresponda
correctamente a ese mismo tope, y las muestras sucesivas no son independientes.
No se deduce la sensibilidad del detector de las 22 poses sin entrada magenta.

Se contrastó además el código C con superficies sintéticas e inyectando una detección
magenta en el mismo ángulo y distancia (sin generar una imagen de cámara):

```
gcc -O2 -std=c11 -I "LIDAR System/main" "LIDAR System/main/local_obstacle_map.c" simulator/tools/diagnose_association_synthetic.c -o "$env:TEMP/autonex-assoc.exe" -lm
& "$env:TEMP/autonex-assoc.exe"
```

| Superficie de retornos | Radio de agrupación 6 cm (actual) | Radio 10 cm |
| --- | --- | --- |
| Pilar de 5 cm | Confirmado | Confirmado |
| Superficie de 20 cm (ejemplo de tope) | Rechazada | Confirmada |
| Superficie de 40 cm (ejemplo de pared) | Rechazada | Confirmada |

La compacidad es efectivamente el filtro que impide esas asociaciones en las poses
medidas; cambiar únicamente el radio eliminaría una protección contra asociar paredes.
Este experimento no incluye errores de color, oclusión, asincronía, objetos en primer
plano ni visión real. Antes de implementar una asociación para topes se necesitan
pruebas de negativos y correspondencia visual real, y conviene distinguir la geometría
del tope de la de un pilar o una pared. Resultados en
`mediciones/asociacion_magenta_2026-11-09.txt`.

### Mapa con movimiento: referencia temporal

`node simulator/tools/measure_map_motion.js 30` recorrió 30 s simulados por sentido
y número de letreros (0, 3 y 7), con perfil largo26 y una semilla por sentido. Cada
segundo se muestreó el mapa local. El controlador compensa el barrido hacia un horizonte
de 0,10 s después del fin de captura; la entrega ocurre unos 0,01 s después del fin.
Se comparó el mismo mapa con la geometría verdadera al entregarlo y con la geometría
en el horizonte futuro. La segunda comparación tampoco es perfecta: usa la pose
verdadera futura y un mapa de celdas, no demuestra que el modelo de compensación sea
exacto. Incluye zonas a ±80 cm del eje del coche.

Se cuentan centros de celdas ocupadas a más de 5 cm de cualquier sólido (`ocupadas
lejanas`) y centros libres situados más de 0,75 cm dentro de un sólido (`libres dentro`).
Con celdas de 2,5 cm y paredes de 2 cm, son indicadores aproximados, no un conteo
confirmado de obstáculos peligrosos ni de colisiones. Las muestras están correlacionadas.

| Sentido | Letreros | Barridos | Ocupadas lejanas entrega / futuro | Libres dentro entrega / futuro | De ellas en letrero, futuro |
| --- | ---: | ---: | ---: | ---: | ---: |
| ccw | 0 | 28 | 142 / 6 | 56 / 14 | 0 |
| ccw | 3 | 28 | 118 / 0 | 62 / 11 | 4 |
| ccw | 7 | 28 | 82 / 6 | 46 / 20 | 3 |
| cw | 0 | 28 | 127 / 8 | 37 / 8 | 0 |
| cw | 3 | 28 | 122 / 0 | 46 / 13 | 5 |
| cw | 7 | 28 | 72 / 2 | 35 / 8 | 1 |

La referencia temporal explica una parte grande de la discrepancia observada. Quedan
celdas que requieren análisis individual, particularmente las que caen en letreros;
no puede decidirse todavía si se deben a oclusión, discretización, compensación,
rayos entre barridos o errores del sensor. Datos completos, incluidas filas por
barrido, en `mediciones/mapa_movimiento_2026-11-09.jsonl`.

### Cobertura de rayos ideales con coche inmóvil

La prueba `measure_local_map.js` se amplió con rayos ideales visibles desde la pose
estática. Para 17 100 rayos elegibles por sentido (50 repeticiones), el mapa marcó
ocupada la celda exacta ideal en 14 188 / 14 383 casos y alguna celda del vecindario
3×3 en 16 951 / 16 947, ccw / cw. Un vecino puede haber sido marcado por otro rayo o
por unión de retornos, incluso si el rayo individual se perdió. Se excluyen ángulos
ciegos y objetivos fuera del mapa o rango. No mide visibilidad detrás de obstáculos,
recuperación de cada objeto ni cobertura en movimiento; el vecindario equivale a una
búsqueda espacial en una rejilla de 2,5 cm, no a un error máximo de 2,5 cm.

## Prototipo aislado de asociación magenta: investigación posterior

`node simulator/tools/prototype_parking_association.js` prueba una idea en JavaScript
sin integrarla en el controlador. Para cada dirección usa siete posiciones longitudinales
y cuatro distancias laterales: 56 poses por escenario antes de excluir poses inválidas.
El coche permanece inmóvil, con sensores y cámara simulados y la semilla 1. Se excluyen
poses con obstáculos solapados con el coche y, en pruebas con bloqueador, casos donde el
tope objetivo está detrás de la cámara. Cada pose produce varios barridos correlacionados;
las cifras siguientes **no son tasas independientes ni garantías sobre el robot real**.

El prototipo agrupa retornos LiDAR consecutivos separados por hasta 3,5 cm, conserva
segmentos de 8 a 26 cm que coincidan aproximadamente con ángulo y alcance de una
caja magenta, y selecciona el más cercano. Es una hipótesis de investigación, no un
algoritmo validado de identificación. Para evaluar se compara el punto elegido con la
geometría verdadera: `cerca` significa hasta 2,5 cm de algún tope real. Estar cerca no
prueba que la caja visual y el retorno pertenezcan al mismo objeto, particularmente
junto a paredes. La detección de cámara se consulta desde el último cuadro simulado;
las poses inmóviles evitan movimiento entre captura y barrido, no todos los errores.

| Escenario | Poses válidas | Barridos con magenta | Candidatos del prototipo | Cerca de un tope / en otra parte |
| --- | ---: | ---: | ---: | ---: |
| Sin bloqueador | 56 | 272 | 138 | 138 / 0 |
| Bloqueador parcial de 5 cm | 22 | 160 | 55 | 55 / 0 |
| Bloqueador de 20 cm | 16 | 0 | 0 | 0 / 0 |
| Objeto magenta con forma similar, sin topes reales | 56 | 96 | 27 | 0 / 27 |

El bloqueador grande ocultó la detección en las poses válidas de este ensayo; eso no
prueba que se detecte toda oclusión. En el último escenario se sustituyeron los topes
por un objeto magenta de geometría similar: muestra que color y forma de un objeto por
sí solos no demuestran que sea el parqueo; hacen falta contexto y evidencia adicional.
No se midieron sensibilidad ni especificidad respecto a la visibilidad real de la cámara.

El mapa actual se observó en los mismos barridos con magenta: en el escenario sin
bloqueador confirmó 24, de los que 6 anclajes quedaron cerca de un tope verdadero y 18
fuera; en el escenario de bloqueador parcial confirmó 99, de los que 1 quedó cerca y
98 fuera. Son repeticiones de unas pocas poses. Un ejemplo en el escenario limpio
ancló sobre la pared a más de 53 cm del tope más cercano; otro con bloqueador parcial
ancló sobre el bloqueador a unos 24 cm del tope. La posición y la etiqueta `confirmado`
se extraen del mapa real del controlador, no del prototipo.

La configuración actual del decodificador conserva el centro, distancia y banderas,
pero no pasa al mapa la anchura de la caja, aunque la anchura existe en la trama. El
modelo de visión exporta una sola caja magenta seleccionada por cuadro. Con esas
limitaciones, no es seguro convertir la nueva hipótesis de segmento en un tope
confirmado. Harían falta pruebas con contexto de pared y pareja de topes, distancia
visual e incertidumbre, banderas de oclusión, objetos delanteros, sincronización, y
contraste con las imágenes y la geometría en movimiento. Una pareja de distractores
idénticos puede ser indistinguible sin otras referencias; debe mantenerse incertidumbre.

**Decisión:** no modificar el firmware ni activar el prototipo. Los resultados se
conservan en `mediciones/prototipo_asociacion_magenta_2026-11-09.jsonl`. En esta sesión
pasaron las pruebas existentes de fusión LiDAR–cámara, contrato WASM y entrada desde
pista; no demuestran validación en hardware ni resuelven las tres repasadas.

## Prototipo de pared y pareja de topes: criterios inicial y estricto

Se amplió `simulator/tools/prototype_parking_association.js` con un prototipo aislado
que busca líneas candidatas de pared, retornos que sobresalen hacia un lado y dos
agrupaciones separadas aproximadamente por el largo esperado del cajón. La imagen
magenta debe ser compatible con al menos una agrupación. El sistema prueba hipótesis
geométricas; no modifica el WASM ni manda órdenes al robot. Se evaluó un solo barrido
por pose (`PAIR_ONLY=1`) para reducir la repetición temporal de muestras. Poses
estáticas y ruido de sensores simulados; no se mide desempeño en movimiento ni en P4.

La referencia geométrica de prueba exige que las dos posiciones elegidas estén a
menos de 2,5 cm de **dos topes verdaderos distintos**. Estar cerca no prueba la
correspondencia visual ni la exactitud del centro de cada tope. Las poses con choque
inicial o bloqueador colocado detrás de cámara se excluyen y el archivo registra sus
cantidades. No se mide si todos los topes eran físicamente visibles en cada pose.

| Escenario, semilla 1 | Observaciones con magenta | Parejas propuestas | Ambas cerca de topes distintos | Otras propuestas |
| --- | ---: | ---: | ---: | ---: |
| Criterio inicial, limpio | 34 | 23 | 10 | 13 |
| Criterio inicial, bloqueador parcial | 20 | 4 | 1 | 3 |
| Criterio estricto, limpio | 34 | 12 | 12 | 0 |
| Criterio estricto, bloqueador parcial | 20 | 0 | 0 | 0 |
| Criterio estricto, bloqueador grande | 0 | 0 | 0 | 0 |
| Criterio estricto, un distractor magenta | 12 | 0 | 0 | 0 |

El criterio estricto exige mayores indicios de extensión perpendicular, anchura
limitada y solapamiento angular con la caja visual. Los umbrales se eligieron tras
observar las mismas escenas: los resultados de semilla 1 no son validación independiente.
Repeticiones exploratorias con semilla 2 y giro +6°: limpio 8 propuestas / 8 cercanas
a topes distintos de 29 observaciones magenta; bloqueador parcial 0/20; distractor 0/12.
Con semilla 3 y giro -6°: limpio 8/8 de 29; bloqueador parcial 1/1 de 20; distractor
0/12. Cada semilla usa solo el giro indicado; hay pocas poses y las condiciones no
cubren todas las combinaciones. No se miden falsos negativos respecto a visibilidad real.

Prueba de indistinguibilidad: se reemplazaron ambos topes reales por dos objetos de
igual forma, color y posición en el simulador, etiquetados como distractores. La versión
estricta produjo 12 parejas en los mismos 34 casos magenta, igual que en la escena
limpia. Esta escena es deliberadamente idéntica para los sensores: no puede resolverse
solo con LiDAR e imagen si ninguna información externa distingue los dos casos. Se
necesitan supuestos de sede, contexto de pista u otra evidencia según el uso.

Para reproducir en PowerShell, por ejemplo:

```
$env:PAIR_ONLY='1'; $env:PAIR_STRICT='1'; $env:BENCH_SEED='2'; $env:BENCH_YAW='6'
node simulator/tools/prototype_parking_association.js
```

Con `BENCH_MODE='fake_pair'` se ejecuta solo la escena indistinguible. Para obtener la
versión inicial quitar `PAIR_STRICT`. Borrar estas variables de entorno antes de repetir
otros escenarios. Archivos crudos: `mediciones/pareja_geometrica_*.jsonl`.

**Decisión:** el criterio estricto es una prueba de concepto conservadora, no está
listo para firmware: no da cobertura suficiente con oclusiones, se ajustó a un conjunto
pequeño, ignora movimiento y latencias y no estima adecuadamente la incertidumbre. La
próxima mejora medible es conservar observaciones inciertas entre vistas y examinar si
la segunda vuelta aumenta confirmaciones sin asociaciones incorrectas, comparando con
una referencia de mapa y manteniendo separado el localizador existente.

## Prototipo de memoria de observaciones entre vueltas

`node simulator/tools/prototype_parking_memory.js [semilla] [segundos] [letreros]`
prueba por fuera del firmware la idea de guardar observaciones individuales y parejas.
Cuatro recorridos de 120 s simulados, perfil largo26, semillas 1/2 y 0/3 letreros,
empezando en la pista sin haber salido del cajón. Se inspeccionó cada quinto barrido
solo en RUN y con pose global considerada válida. El prototipo usa el último cuadro
magenta efectivamente **entregado por la cola simulada**, no el último capturado: en
estas observaciones su edad fue 70 ms. El ensayo no compensa el movimiento entre esa
imagen, los rayos del barrido y la pose usada; una localización comparada en esas
condiciones no equivale al rendimiento de una fusión bien sincronizada. No modifica
el mapa persistente ni las órdenes del firmware.

Las referencias de un solo tope se obtienen mediante la hipótesis experimental de
segmentos; se transforman al marco de la pose global estimada y se agrupan a 6 cm.
Para el análisis, una observación `cercana` se define como situada a 4 cm o menos de
algún tope verdadero. No se demuestra que corresponda a la caja visual, y puntos de
una misma superficie y observaciones sucesivas no son independientes. Se intentó
formar pareja con agrupaciones observadas en al menos dos secciones temporales
(`corners` distintos), separadas entre 27 y 40 cm; no se fuerza el resultado.

| Sentido / letreros | Observaciones magenta muestreadas | Candidatos individuales | Cercanos a tope | Parejas simultáneas / cercanas a ambos | Parejas inferidas de memoria |
| --- | ---: | ---: | ---: | ---: | ---: |
| ccw / 0 | 16 | 11 | 10 | 0 / 0 | 0 |
| ccw / 3 | 18 | 6 | 5 | 2 / 2 | 0 |
| cw / 0 | 21 | 11 | 7 | 1 / 1 | 0 |
| cw / 3 | 27 | 17 | 13 | 0 / 0 | 0 |

En ccw/3 las dos parejas simultáneas se agruparon en una hipótesis observada en t=1,5 y
103,5 s, cercana geométricamente a ambos topes; esto no es prueba de identificación
semántica correcta. En cw/0, tres observaciones de secciones 3 y 11 se agruparon en un
punto a 7,62 cm del tope más próximo, y ninguna de las tres quedó dentro de 4 cm. En
cw/3 hubo un grupo de cuatro observaciones de secciones 3, 7 y 11 cuyo promedio quedó a
5,32 cm del tope; solo una de esas cuatro observaciones estaba dentro de 4 cm. Por ello
repetir una detección no basta para confirmar un tope: el sesgo puede repetirse y las
referencias de distintas caras de un mismo tope pueden quedar en grupos separados.

En una repetición local de ccw/0, la función JS de búsqueda de pareja consumió p50
1,29 ms y p95/máximo 4,03 ms para solo 16 llamadas. Incluye calentamiento y depende de
Node y de esta computadora; no mide firmware, P4, casos difíciles ni presupuesto de
control. Archivos de resultados: `mediciones/memoria_*_2026-11-09.jsonl`.

**Conclusión de diseño:** no activar aún esta memoria. Debe distinguir segmentos y
superficies de un mismo obstáculo, incorporar geometría de pared y pareja, conservar
incertidumbre, sincronizar la pose con el momento del cuadro y el barrido, y contrastar
con distractores y oclusiones. La ausencia de parejas inferidas no demuestra que la
información no exista: puede deberse a los umbrales, a la extracción de candidatos o a
no ver ambos topes. Conservar las observaciones como hipótesis y volver a verificar en
proximidad antes de usarlas para una maniobra.

### Alineación temporal experimental de cámara y LiDAR

Se añadió la opción `ALIGN_CAMERA=1` a `prototype_parking_memory.js`. En el instante
simulado de captura se guarda la última pose global publicada por el controlador. Al
asociar el cuadro entregado con un barrido posterior, se transforma el barrido entero
aproximadamente al marco de esa pose. Es una prueba fuera del firmware, no usa la pose
verdadera para decidir; no reconstruye el movimiento rayo por rayo, y la pose guardada
puede estar desactualizada respecto al instante de captura. La cámara no presenta
corrupción ni pérdidas en esta prueba. Cuando falta pose válida en la captura, se
omite la evaluación alineada.

En dos recorridos idénticos de 120 s con y sin esta opción:

| Caso | Referencias individuales / cerca de un tope, sin alinear | Con alineación aproximada | Parejas inferidas de memoria, sin / con alineación |
| --- | ---: | ---: | ---: |
| ccw semilla 1, sin letreros | 11 / 10 | 7 / 7 | 0 / 0 |
| cw semilla 2, tres letreros | 17 / 13 | 17 / 10 | 0 / 2 |

Las dos parejas inferidas con alineación aproximada en cw/3 **no** correspondían a dos
topes verdaderos distintos con la referencia geométrica de 4 cm. Sus separaciones eran
34,17 y 35,50 cm: cumplir la separación y repetirse entre vueltas no bastó para evitar
las asociaciones falsas. El valor de referencia se utiliza solo para evaluar, no para
construir las hipótesis. Estas pruebas no descartan la compensación temporal: muestran
que esta implementación aproximada, con estas entradas y umbrales, no es suficiente y
puede empeorar los resultados. No se cambió el firmware.

Para reproducir la variante en PowerShell: `$env:ALIGN_CAMERA='1'; node
simulator/tools/prototype_parking_memory.js 2 120 3`. Borrar la variable para la
versión sin alinear. Datos en `mediciones/memoria_alineada_*_2026-11-09.jsonl`.

## Pasada lateral: detección magenta y reconstrucción LiDAR progresiva

Experimento añadido `simulator/tools/measure_parking_sweep.js`. Se mueve el chasis
Ackermann con un mando de prueba recto de 12 cm/s; **no** se evalúa la navegación ni la
maniobra de parqueo. Se espera una detección magenta entregada por la cola de cámara;
a partir de ella se acumulan todos los retornos LiDAR válidos en una nube global usando
la pose de campo estimada por el controlador. La geometría verdadera **no decide qué
puntos se guardan**; solo se utiliza después para evaluar. No se ha construido todavía
una memoria de objetos del firmware ni asociado automáticamente cada punto a un tope.

En paralelo, el evaluador del simulador reconstruye la intersección ideal del rayo con
el primer sólido visible, usando la pose verdadera del LiDAR en ese instante. Solo para
estadísticas por superficie se etiquetan esos rayos ideales y se excluyen retornos cuyo
alcance medido difiere más de 3 cm del ideal. Ángulo nominal, ruido angular, muestreo y
ambigüedades en bordes limitan esa etiqueta; no se trata de etiquetas disponibles en el
robot. No se utiliza la pose verdadera para construir la nube con pose estimada.

Los segmentos ideales se agrupan en tramos de 1 cm: el perímetro rectangular de cada
tope tiene 44 tramos y el tramo de pared evaluado 79. `visible ideal` significa que al
menos un rayo geométrico nominal lo alcanzó sin máscara ni oclusión en la pasada después
de la detección. `registrado` exige al menos un retorno de ese mismo rayo cuyo punto
reconstruido quede a 2,5 cm o menos de la intersección ideal. Una zona sin rayos se
informa como **no observada**, no se completa con puntos de otras superficies. Los
puntos y rayos se repiten entre barridos y no son muestras independientes.

### Doce pasadas sin obstáculos adicionales

`node simulator/tools/measure_parking_sweep.js 2`: dos sentidos, semillas 1/2 y
distancias laterales del eje a la pared 35/45/55 cm. La cámara activó las doce pasadas;
en cada una se acumularon 130 barridos, sin contactos y sin barridos posteriores al
aviso descartados por falta de pose. En cada caso, la referencia ideal vio 42 de 44
tramos de cada tope y 75 de 79 de la pared seleccionada. **Todos esos tramos visibles**
tuvieron al menos un rayo registrado con el criterio de 2,5 cm. Los tramos restantes
no se observaron en esta pasada; esto no significa que estén libres ni demuestra que un
algoritmo pueda clasificarlos correctamente a partir de la nube.

El error de registro de los rayos etiquetados de los topes, usando la pose estimada al
fin del barrido frente a la pose verdadera de cada rayo, dio p95 por tope y caso entre
0,54 y 0,93 cm. Este dato aísla la diferencia de pose sobre el retorno medido: no añade
el error propio de distancia/angular del LiDAR, usa retornos aceptados y solo vale para
estas pasadas rectas. La distancia p95 de esos puntos a las superficies verdaderas,
que sí incluye ruido simulado, varió entre 0,89 y 1,24 cm; la filtración de retornos
ambiguos impide considerarlo un límite general. Interpolar entre poses estimadas de dos
barridos tuvo p95 de registro entre 0,68 y 1,16 cm: en este ensayo fue peor, pero las
poses publicadas pueden representar instantes distintos a sus marcas nominales; no se
concluye que la interpolación temporal sea en general incorrecta.

### Control de oclusión y activación

A 45 cm de la pared, un obstáculo oscuro simulado de 60 × 3 cm y 10 cm de alto entre el
coche y el cajón no produjo contactos y redujo la visibilidad ideal a 25/44 y 22/44
tramos de los topes, y 36/79 de la pared. Con el criterio de mismo rayo se registraron
24/25, 22/22 y 36/36 de esos tramos visibles. El primer tope tuvo un tramo ideal sin
registro aceptado. La oclusión demuestra que no se puede considerar automáticamente
observado lo que queda detrás del obstáculo.

Al desactivar la clase magenta, no hubo aviso ni se guardaron barridos. Es un control del
flujo experimental, no una medición de sensibilidad o falsos positivos de la cámara.
Los resultados de oclusión con un bloqueador anterior a 35 cm que produjo contactos se
descartaron y no se usan como evidencia de una pasada válida.

### Archivos y reproducción

- `mediciones/barrido_lidar_12_pases_2026-11-09.jsonl`: métricas y máscaras por tramo.
- `mediciones/barrido_lidar_ocluido_2026-11-09.jsonl`: pasada ocluida válida.
- `mediciones/barrido_lidar_sin_magenta_2026-11-09.jsonl`: control sin activación.
- `mediciones/barridos/barridos_ccw_semilla1_distancia45.json` y su variante
  `_occlusion`: barridos originales, marcas de tiempo y poses estimadas. Las columnas
  de rayos son tiempo, ángulo, distancia, válido y zona ciega; no incluyen geometría
  verdadera. Los archivos fueron generados con `EXPORT_DIR` y permiten repetir el
  procesamiento posterior.
- Cada `idealVisibilityMask` usa `#` para tramos alcanzados por rayos ideales y `.`
  para tramos no observados. En los topes, los tramos recorren el perímetro del
  rectángulo: borde inferior, derecho, superior e izquierdo en coordenadas del
  rectángulo; en la pared recorren el segmento de izquierda a derecha.
- `node simulator/tests/test_parking_sweep.js`: comprueba activación, ausencia de nube
  sin magenta, datos de ambos topes en escenario libre y reducción de visibilidad con
  oclusión, sin contactos en el escenario ocluido.

Límites: no se evalúan giros, otros letreros, distintas alturas o reflectividades reales,
precisión de la cámara ni asociación semántica automática; solo dos semillas y un
modelo de sensor simulado. El dispositivo físico y el coste de mantener el mapa en P4
siguen sin medirse. Para el siguiente paso hay que segmentar la nube y verificar la
pose/marcas temporales antes de crear una memoria de parqueo confirmada.

## Segmentación offline de pared y topes desde una detección magenta

Se desarrolló `simulator/tools/segment_parking_cloud.py`, un prototipo separado del
firmware que lee los barridos y las poses estimadas almacenadas por
`measure_parking_sweep.js`. Una sola caja magenta y la pose estimada de la cámara al
capturarla delimitan una región aproximada. El prototipo acumula retornos en celdas de
1 cm, busca una pared larga y dos franjas de puntos aproximadamente perpendiculares,
y guarda una hipótesis con la pared, los posibles topes y el intervalo respaldado por
puntos. No recibe la geometría verdadera. Usa NumPy y dimensiones de cajón/topes
configurables: por defecto 39 × 20 cm y 2 cm de grosor, correspondientes al perfil
simulado `largo26`. El final `lane_end_nominal` es una extrapolación de esa dimensión,
no superficie observada; `evidence_start/end` resume percentiles 5–95 de los puntos
en la franja candidata, no garantiza que todos pertenezcan físicamente al tope.

Estados: `sin_deteccion_magenta`, `sin_pose_de_camara`, `evidencia_insuficiente`,
`sin_pared_suficiente`, `sin_topes_suficientes`, `sin_pareja_suficiente`,
`hipotesis_no_confirmada` y `hipotesis_ambigua`. Ninguno autoriza conducir hacia el
cajón. La ambigüedad se activa si varias orientaciones de pared tienen apoyo parecido
o si aparecen varias parejas compatibles; es una heurística, no una garantía de detectar
todas las falsas hipótesis. La cámara puede ver solo un tope: no se exige una caja
visual de cada uno.

`evaluate_parking_cloud.js` reconstruye aparte la geometría verdadera del simulador
solo para comparar los resultados, después de que el prototipo haya terminado. Se
ensayaron las 12 pasadas despejadas iniciales (dos semillas, dos sentidos, tres
distancias a pared) y otras 12 con semillas 3/4 y rumbo inicial +2°/-2°, con movimiento
recto de prueba. En los 24 casos apareció una pareja `hipotesis_no_confirmada`; el mayor
error de los dos puntos estimados sobre la pared, comparados con los centros verdaderos
de los topes, fue 0,458 cm. Son escenarios estrechamente relacionados, con paredes
alineadas a los ejes, dimensiones y superficies idealizadas, y el método se ajustó en
la primera pasada: no constituyen precisión garantizada ni validación independiente de
campo. El error sobre la pared no prueba que las caras o extremos visibles estén bien.

Con un bloqueador opaco artificial de 60 × 3 cm y 10 cm de alto se probaron ocho pasadas
(dos sentidos, dos semillas y 45/55 cm). A 45 cm, en las cuatro se produjo
`hipotesis_ambigua`: si se hubiera usado la pareja seleccionada, el mayor error de un
tope habría sido 4,089–6,298 cm. A 55 cm la cámara no generó aviso y el prototipo
respondió `sin_deteccion_magenta`. No hubo contactos en esos ocho recorridos. En una
pasada adicional se colocó un distractor magenta que produjo dos parejas compatibles;
el estado fue `hipotesis_ambigua`. Estos negativos no cubren todos los distractores,
paredes, oclusiones, reflejos ni condiciones de luz. No basta con no detectar ambigüedad
para asegurar que una hipótesis sea verdadera.

Ejemplo para guardar una hipótesis sin etiquetas verdaderas:

```
python simulator/tools/segment_parking_cloud.py simulator/mediciones/barridos/barridos_ccw_semilla1_distancia45.json --output simulator/mediciones/hipotesis_parqueo_despejado_2026-11-09.json
```

`--lot-length`, `--limiter-width` y `--limiter-thickness` permiten variar supuestos.
Las salidas de ejemplo para despejado, bloqueado y distractor están en `mediciones/`;
`evaluacion_segmentacion_2026-11-09.jsonl` contiene las evaluaciones de los 32 casos.
`node simulator/tests/test_segment_parking_cloud.js` comprueba guardado, evidencia,
falta de detección, ambigüedad en dos escenas y una transformación del marco de
coordenadas; `node simulator/tests/test_parking_sweep.js` comprueba activación y
visibilidad. Las pruebas son simuladas y el segmentador corre offline con NumPy; no se
midió memoria/tiempo en P4, ni variaciones de pista reales, cámaras reales o giros.
Antes de integrarlo hay que comparar cámara y LiDAR con marcas temporales fiables,
analizar obstáculos y espacio libre, evaluar escenarios adicionales y comprobar las
hipótesis de dimensiones. No se ha activado en firmware ni puesto una ruta al coche.

## Mapa progresivo y comparación experimental de rutas

`simulator/tools/progressive_parking_cloud.py` acumula celdas de 1 cm a medida que
llegan los barridos y ejecuta el segmentador cada diez barridos. Los retornos se
transforman con la pose estimada al final del barrido, igual que el ensayo anterior;
no se compensa cada rayo ni se separan libres, ocupados y desconocidos. La memoria
acumulada es una nube con recuentos, sin caducidad ni límites de tamaño; no es todavía
un mapa de ocupación completo. El segmentador se ejecuta de nuevo sobre las celdas
acumuladas en cada evaluación. Permanece en Python/NumPy, aislado del firmware.

En la pasada despejada ccw/semilla 1/distancia 45, la primera pareja candidata apareció
tras 40 barridos, 3,95 s simulados después de la señal magenta. Comparada a posteriori
con la geometría verdadera, su mayor error de posición de un tope fue 0,88 cm; al
final de 130 barridos fue 0,24 cm. En la pasada cw con semilla 4 y rumbo -2°, el primer
resultado (30 barridos) se marcó ambiguo; el de 40 barridos fue una hipótesis no
confirmada con error máximo de 1,53 cm. Estos son dos casos, no una cota general.

Con el bloqueador, la primera pareja apareció a los 110 barridos y quedó ambigua; con
el distractor magenta, la primera pareja apareció a los 40 y quedó ambigua. Las reglas
son heurísticas y no garantizan rechazar todo falso positivo. Las máscaras de visibilidad
y las etiquetas de verdad no intervienen en la construcción de la hipótesis.
`test_progressive_parking_cloud.py` comprueba que la última nube incremental produce
la misma hipótesis que el procesamiento por lote y que dos escenas de referencia se
mantienen ambiguas.

### Puente experimental al planificador y pruebas de trayectoria

`compare_parking_routes.js` compila un adaptador nativo que llama al planificador C
Ackermann real. Para evitar usar información futura, selecciona un momento usando el
avance estimado desde la captura magenta y genera la hipótesis **solo con los barridos
anteriores a ese instante**. Transforma los dos topes y la pared de la hipótesis al
marco del planificador, y compara con un plan calculado desde la misma pose usando la
geometría verdadera. La geometría verdadera se emplea solo en la segunda referencia y
para evaluar el resultado, no para crear la hipótesis. Se omiten las hipótesis ambiguas
o insuficientes. Se usa la dimensión supuesta de cada tope.

El planificador recibe por ahora **la pared y los dos topes parametrizados**, no toda
la nube de ocupación. La prueba de colisiones con obstáculos reales es externa y se
realiza después: una ruta ideal sin contacto en este ensayo no demuestra que el A*
pueda evitar obstáculos arbitrarios detectados en la nube. Las poses iniciales de
ambos planes son la misma pose estimada considerada como verdadera en el banco, con
el vehículo inicialmente detenido; no se mide el error real de ese estado inicial.

Se evaluaron 32 registros: 24 despejados y 8 con oclusión. Los 8 ocluidos se omitieron
por no existir pared suficiente al instante seleccionado o no haberse detectado
magenta. En 23 de los 24 despejados, tanto la geometría estimada como la verdadera dieron
un plan; en el otro ninguno encontró plan. Los 23 planes geométricos de cada grupo
terminaron dentro del cajón sin solape al muestrear cada arco en seis pasos. No se trata
de una comprobación continua ni de validación en hardware. Para los 23 planes de mapa,
el cómputo incluye el cambio desde avance en el carril: 8 tuvieron tres cambios y 15
cuatro. Los de geometría verdadera dieron 2 con dos, 11 con tres y 10 con cuatro.

Como prueba complementaria, se ejecutaron las secuencias en `W.robot.Robot` con servo,
motor y colisiones simuladas, partiendo de reposo y agrupando arcos de mismo mando. Es
un seguidor experimental con frenado aproximado, no el ejecutor actual ni una ronda
completa; las comprobaciones de colisión y la condición final usan la geometría real
del simulador. Para los 23 planes basados en mapa: 13 terminaron completos sin
contacto, 5 tocaron un obstáculo y 5 quedaron parciales sin contacto. Con geometría
verdadera: 14 completos, 6 contactos y 3 parciales. Las mismas rutas ejecutadas con una
corrección de dirección que usa la **pose verdadera** —un oráculo no disponible en el
robot— dieron 13 completos / 3 contactos / 7 parciales con mapa y 15 / 3 / 5 con
geometría verdadera. No se ha optimizado ese seguidor ni se infiere que el controlador
actual tenga esos resultados; sirven para separar geometría ideal y ejecución física.

`test_parking_route_experiment.js` verifica una ruta ideal de referencia, que no se
usen barridos futuros y que dos escenas ambiguas no produzcan un plan experimental.
Los resultados por caso están en `mediciones/comparacion_rutas_2026-11-09.jsonl`.

### Coste y preparación P4

En esta computadora, con Python `tracemalloc` activo, la inserción incremental en los
cuatro escenarios medidos tuvo p95 aproximado de 3,84–5,35 ms por barrido y la última
segmentación 31–67 ms de tiempo transcurrido. El pico registrado por `tracemalloc` fue
0,87–1,02 MB, sin incluir necesariamente toda la memoria nativa ni el archivo de
entrada cargado antes de iniciar la medición. Son mediciones de Python/NumPy bajo
instrumentación y carga variable, **no** tiempos, RAM ni predicciones para ESP32-P4.
El A* nativo de esta computadora se registró con `clock()` por caso en el archivo de
rutas; su resolución y carga no permiten extrapolarlo al P4.

Se compiló el firmware existente para ESP32-P4 con ESP-IDF 5.5.4 usando destino
`esp32p4` explícito y carpeta de compilación temporal. La compilación terminó; el binario
existente mide 379 488 bytes y la partición de aplicación queda con 669 088 bytes libres.
`idf.py size` informa 3 477 104 bytes de `.bss` en RAM externa y 302 688 bytes de DIRAM;
el espacio de direcciones reportado no demuestra que la placa tenga esa cantidad física
de PSRAM. Estas cifras **no incluyen** el prototipo Python ni prueban su viabilidad en
P4. No se flasheó ningún dispositivo. Los puertos serie detectados eran Bluetooth, no
se identificó un P4 conectado y no se pudo medir tiempo real en el dispositivo.

El siguiente trabajo para una integración real es sincronizar cámara/LiDAR/pose,
limitar y envejecer el mapa, incorporar obstáculos de la nube en las comprobaciones del
planificador, diseñar un seguidor cerrado probado con sensores reales y medir en el P4.
No activar el banco de investigación como controlador del coche.

### Comprobación adicional de rutas contra retornos LiDAR

El banco de rutas reconstruye una rejilla experimental de 1 cm a partir de los
retornos disponibles hasta el instante de planear y examina la huella del coche,
ampliada 0,5 cm, en los mismos seis pasos por arco. Una celda se considera con
evidencia si recibió al menos dos retornos; esos retornos pueden proceder del mismo
barrido y no constituyen una confirmación independiente. Las celdas sin retornos se
tratan como desconocidas, no como espacio demostrado libre. Esta comprobación es
posterior al A*: **el planificador aún no genera ni modifica su ruta usando esos
obstáculos**, y el prototipo no está conectado al control del robot.

En las 23 trayectorias ideales encontradas desde la geometría estimada, ninguna tuvo
solape con los sólidos verdaderos muestreados; la separación mínima verdadera de cada
trayectoria estuvo entre 0,971 y 2,338 cm. La comprobación contra puntos LiDAR señaló
retornos dentro de la huella ampliada en 16 de las 23 trayectorias, con un máximo de
cuatro muestras señaladas en una trayectoria. El mapa incluye ruido, error de registro
y puntos acumulados; la comparación es discreta y usa tolerancias distintas a la
comprobación de sólidos. No se puede interpretar esa diferencia como una tasa de
seguridad ni afirmar que los avisos sean contactos reales. Muestra que hace falta
calibrar la incertidumbre y los criterios de ocupación antes de usar la comprobación
para bloquear o aceptar maniobras.

`node simulator/tests/test_parking_cloud_collision.js` verifica la detección geométrica
de un retorno dentro de la huella y que una celda desconocida no inventa un retorno;
no prueba que una zona desconocida sea segura. `test_parking_route_experiment.js`
comprueba el flujo con un prefijo sin barridos futuros y omite hipótesis insuficientes
o ambiguas. No hay validación de detección de todos los obstáculos ni del P4.
