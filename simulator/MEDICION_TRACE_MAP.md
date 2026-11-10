# Medición de trace_map contra la pista simulada

Reproducir: `node simulator/tools/measure_trace_map.js simulator/mediciones/trace_map_ground_truth.json`.
Las geometrías verdaderas se usan solo después de la simulación para evaluar, no
se entregan al controlador. Se probaron semillas 1 y 3 en los dos sentidos, con
tres obstáculos y el perfil largo26. Son cuatro casos, no una prueba estadística
exhaustiva. Tres acabaron anticipadamente por `wrong-side`; solo el caso horario,
semilla 1, completó la ronda. El detalle se guarda en el JSON generado.

Cada celda LiDAR acumulada se compara por su centro con el sólido verdadero más
cercano. P50 y p95 son percentiles por **celda única**, no por rayo ni por objeto;
la distancia es cero dentro de un sólido. Los valores incluyen cuantización de
2,5 cm, ruido, pose, movimiento y todos los demás efectos de la simulación. No
identifican cuál fue la causa ni miden error por eje o el error de las esquinas.

| Sentido, semilla | Fin (s) | Celdas | P50 (cm) | P95 (cm) | Máx. (cm) | A más de 5 cm |
|---|---:|---:|---:|---:|---:|---:|
| CCW, 1 | 3,07 | 684 | 1,25 | 6,25 | 8,75 | 39 |
| CCW, 3 | 44,50 | 4.139 | 3,75 | 9,25 | 16,25 | 989 |
| CW, 1 | 161,12 | 3.784 | 1,75 | 8,75 | 13,75 | 720 |
| CW, 3 | 9,73 | 1.350 | 1,25 | 4,25 | 8,75 | 45 |

El error de posición del localizador, muestreado aproximadamente una vez por
segundo, tuvo p95 de 3,41 cm y máximo 4,40 cm en la ronda completa. No es una
serie sincronizada con cada rayo; no permite atribuirle todo el error del mapa.
Las celdas alejadas son posibles puntos fantasma, no prueba de su causa.

Se muestrearon los perímetros verdaderos cada aproximadamente 1 cm y se contó si
existía una celda LiDAR a 3,75 cm o menos. Para los topes se excluyeron puntos
a 7,5 cm o menos de una pared para reducir confusión. En la ronda completa, las
26 muestras restantes de cada tope tenían alguna celda cercana. **Esto no prueba
que se haya visto cada superficie**, ni que se distingan ambos topes: la rejilla
mezcla todos los barridos, los puntos cercanos pueden ser de otras superficies o
fantasmas, y no se calculó visibilidad por rayo ni oclusión. Las caras ocultas no
deben contarse como observadas. Los porcentajes de perímetro tampoco son error de
longitud o de posición de los extremos.

En esa ronda había 48 celdas con la etiqueta magenta candidata: 35 estaban a 5 cm
o menos de un tope verdadero y 13 no. Son celdas, no detecciones independientes;
el dato no mide exhaustividad ni demuestra identificación de ambos topes. Las
etiquetas rojas y verdes fueron pocas (3 y 4), todas cerca de un objeto del color
respectivo bajo ese mismo umbral. No prueba que todos los obstáculos hayan sido
identificados.

Conclusión: el mapa acumula información útil pero aún no justifica navegar ni
parquear exclusivamente con él. Conviene registrar por rayo qué sólido fue visible
en cada instante, medir cobertura de caras realmente observables, comparar paredes
y extremos por separado y analizar la dispersión sincronizada con pose y tiempo.
