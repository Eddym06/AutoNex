# Visibilidad de trace_map por rayo

Reproducir: `node simulator/tools/measure_trace_visibility.js simulator/mediciones/trace_map_visibility.json`.
El evaluador intercepta las mediciones del simulador y reconstruye, con la pose
verdadera de cada rayo y el jitter angular ya usado por el propio sensor, el
primer sólido que intersecta. No consume números aleatorios adicionales ni
entrega la geometría verdadera al controlador. Como comprobación, los dos casos
terminaron en los mismos instantes y con los mismos motivos que en la medición
anterior. Hay una comprobación sintética de oclusión en el propio script.

Se consideran rayos desde que arrancó el vehículo y la interfaz informa pose
global válida. Se excluyen la máscara del chasis y los impactos fuera de rango.
Esta condición aproxima el intervalo del mapa: no comprueba exactamente, rayo por
rayo, el instante en que C aceptó un barrido y su umbral de ajuste. Las caras se
dividen en segmentos de aproximadamente 1 cm. Un segmento es «visible» si al
menos un rayo impacta primero en él; «retorno válido» si alguno de esos impactos
produjo una medida. No se distinguió si un segmento sin rayo se debía a oclusión,
máscara, rango, resolución angular o recorrido; por eso se denomina no observado.

## Ronda completa: horario, semilla 1

De 644.480 rayos emitidos, 573.023 tenían impacto elegible después de adquirir
pose; 563.452 dieron un retorno válido. Para cada tope, de 44 segmentos de
perímetro, 42 recibieron al menos un retorno válido: 20 de cada cara larga y 2 de
una cara corta. La otra cara corta (2 segmentos) no recibió ninguno. Esa ausencia
no demuestra por sí sola la razón física. Los resultados describen el recorrido
completo, incluida salida y entrada, no solamente una pasada lateral.

Los 42 segmentos visibles y con retorno de cada tope tenían alguna celda
acumulada a menos de 3,75 cm. La distancia al centro de celda más cercano tuvo
p95 de 1,42 cm y 1,50 cm, respectivamente. Las distancias desde los extremos
verdaderos del eje largo al centro de celda más cercano fueron 1,50/1,50 cm y
1,42/1,42 cm. **Estas no son estimaciones de los extremos:** se usa su posición
verdadera y se busca cualquier celda cercana; podría provenir de la pared, del
otro lado del bloque o de un fantasma. No hay asociación por procedencia de rayo
en trace_map ni un detector que haya devuelto dos extremos.

En las paredes, los segmentos visibles con retorno estuvieron a 1,77 cm o menos
de alguna celda acumulada. A la vez, la medición inversa anterior mostró que
720 de 3.784 celdas estaban a más de 5 cm de cualquier sólido verdadero. Ambas
cosas pueden ser ciertas: la nube densa puede cubrir la pared y, a la vez,
contener muchos puntos desplazados. La proximidad de verdad a mapa no mide la
pureza del mapa.

Con pose verdadera por rayo, el error absoluto de distancia del sensor simulado
tuvo p95 de 3,01 cm para paredes y 1,22/2,04 cm para los topes. Esto evalúa solo
la distancia de retornos válidos, no incluye error de pose del vehículo ni
caracteriza el LiDAR físico. Hay valores aislados mayores.

## Segundo caso: antihorario, semilla 3

Terminó a los 44,5 s por `wrong-side`, por lo que no equivale a una ronda
completa. En ambos topes se observaron 42 de 44 segmentos con al menos un retorno
válido; una cara corta de cada uno quedó sin rayos. La distancia p95 de segmentos
con retorno a cualquier celda cercana fue 1,64/1,56 cm. El p95 del ruido de
distancia fue 3,02 cm en paredes y 2,96/2,24 cm en los topes.

## Qué falta

Registrar la procedencia de los puntos dentro del análisis, medir únicamente la
pasada lateral previa al parqueo y estimar geométricamente las paredes y los dos
topes sin usar la verdad. Después comparar los extremos y su incertidumbre contra
la verdad y descartar las hipótesis con poca evidencia. Los resultados actuales
no autorizan a usar el mapa como único sensor de navegación.
