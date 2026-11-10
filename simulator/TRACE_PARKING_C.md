# Localización experimental del parqueo a partir de trace_map

`trace_parking.c` prueba paredes alineadas con ambos ejes cardinales y dos
franjas que salen de ellas, con separación compatible con las
dimensiones configuradas. Una sola detección magenta aporta el sector angular y
el alcance aproximado; no se necesita ver los dos topes en cámara. Utiliza celdas
observadas en al menos tres barridos; `trace_map` cuenta como máximo una vez por
celda y barrido y reserva 3.200 bytes de trabajo adicionales. La función
devuelve sin evidencia, candidato o ambiguo. Una segunda pared suficientemente
respaldada, varias parejas o escasa evidencia de pared causan ambigüedad. Un
candidato no es una confirmación de seguridad ni una estimación de incertidumbre.

El controlador ejecuta el detector cada diez barridos con pose válida, guarda
el diagnóstico en memoria y el simulador lo dibuja. Conserva además el último
candidato como recuerdo no confirmado cuando la evidencia deja de respaldarlo.
Ese recuerdo puede estar desactualizado y no se emplea para conducir. Solo se acepta una detección
magenta reciente durante RUN. Si no existe candidato, se puede volver a intentar
con otra observación tras cinco segundos. El resultado **no alimenta el
planificador ni modifica los comandos**. Se reinicia con el controlador.

Limitaciones de la integración: la cámara aporta dirección y distancia sin la
anchura de la caja en `local_camera_object_t`; el detector usa un margen de ±10°
más otros 8° internos. La pose se toma al recibir la observación, no en el
instante exacto de captura; la latencia no se compensa. `trace_map` contiene
retornos previos y posteriores a la detección, sin peso temporal ni eliminación
de fantasmas. Se elige una sola observación activa a la vez. La detección depende
de la pose global y de paredes alineadas con los ejes del marco; no sirve para
la demostración de parqueo que no adquiere pose de campo. Una pared o patrón
parecido puede producir falsos candidatos. No se ha probado en el P4 físico.
Las parejas posteriores que se alejan más de 8 cm de la primera pareja recordada
se marcan ambiguas y no reemplazan el recuerdo; la comparación contempla que los
dos extremos se intercambien. El ancla inicial puede ser incorrecta: la coherencia
evita algunos saltos, pero no confirma la identidad del parqueo.

## Pasada lateral aislada

`measure_parking_sweep.js` condujo a velocidad fija, guardó pose estimada,
rayos y detección de cámara; `evaluate_trace_parking.py` reprodujo esos datos con
`trace_map` y `trace_parking` en C nativo. La geometría verdadera se consultó solo
después de ejecutar C. La pasada no es una misión autónoma y usa el centro de la
caja para aproximar lo que recibe el P4, con pose del instante de captura, más
favorable que la integración actual.

En 12 pasadas despejadas, 2 sentidos, 2 semillas y distancias laterales de
35/45/55 cm, surgieron 12 candidatos. Sus intersecciones estimadas con la pared
quedaron a entre 1,25 y 1,60 cm de las verdaderas. En 12 pasadas adicionales con
rumbo inicial ±8° y otras dos semillas, surgieron 11 candidatos y un caso sin
evidencia; el mayor error de esas 11 parejas fue 1,77 cm. Un caso ocluido y otro
con un distractor se devolvieron ambiguos al final. No son pruebas exhaustivas:
los criterios se desarrollaron con algunos de estos mismos casos y no prueban
ausencia de falsos positivos. En la pasada despejada de referencia, se obtuvo el
primer candidato en torno a los 40 barridos posteriores a la detección.

## Misión autónoma completa

`node simulator/tools/measure_trace_parking_online.js simulator/mediciones/trace_parking_online.json`
evalúa la integración real del controlador. Se ejecutaron seis rondas: semillas
1 y 2 sin obstáculos y semilla 3 con tres obstáculos, cada una en ambos sentidos.
Las seis terminaron y produjeron un candidato; el peor error por extremo entre
las muestras con candidato fue 4,08 cm. Se mide aproximadamente cada 0,5 s y
las muestras están correlacionadas. En algunas rondas la evidencia se volvió
insuficiente o ambigua, conservándose solo un recuerdo pendiente de verificación.
En una ejecución previa a añadir la comprobación del ancla, el caso CCW con
semilla 3 y obstáculos llegó a aceptar una pareja con error de 48,55 cm; con el
ancla fija se rechazó esa pareja y el peor error de candidatos medidos en ese
caso fue 2,20 cm. Esto no prueba que la primera pareja o nuevos escenarios sean
correctos. El diagnóstico no garantiza una localización confirmada cuando el
coche vaya a aparcar.

Para pasar de diagnóstico a control hace falta medir y corregir la pose de
captura de cámara, manejar observaciones históricas y falsos retornos, verificar
la identidad con más casos y velocidades, medir tiempo y pila en hardware y
validar contra sensores en vivo antes de conectar una pose al planificador.
Compilación cruzada estática con `-O2 -fstack-usage`: 112 bytes para
`trace_parking_estimate` y 816 bytes para `estimate_axis` cuando se invoca,
excluyendo sus otros llamadores y subfunciones; se amplió la pila principal
del firmware a 6144 bytes. Eso no sustituye medir el margen de pila en hardware.
En una sola nube reproducida 100 veces en el PC, el coste medio nativo del
detector fue 0,36 ms por evaluación; no es p95, no incluye la actualización del
mapa y no representa el ESP32-P4.
