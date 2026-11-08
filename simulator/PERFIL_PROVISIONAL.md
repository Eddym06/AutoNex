# Perfil provisional de simulación — 12-móvil

Estas medidas son **suposiciones para desarrollar el simulador**, no medidas
del equipo ni dimensiones estándar exigidas por WRO. Cambiar al medir el
vehículo montado; no usar para habilitar la tracción física.

| Dato | Valor provisional |
| --- | ---: |
| Largo × ancho de carrocería | 22 × 14 cm |
| Distancia entre ejes | 14 cm |
| Saliente detrás del eje trasero | 4 cm |
| Vía trasera | 12 cm |
| Diámetro de ruedas | 6,6 cm |
| Dirección efectiva máxima | 30° a cada lado |
| Radio mínimo ideal del eje trasero | 14 / tan(30°) ≈ 24,2 cm |
| LiDAR delante del eje trasero | 10 cm |
| Cámara delante del eje trasero | 13 cm |
| Altura del LiDAR / cámara | 6 / 5 cm |
| Cuentas x4 por vuelta de salida del motorreductor | 440, provisionales |
| Engranajes externos motor / diferencial | 1 / 1, provisionales |
| Cajón de parqueo calculado | 20 × 33 cm |

El radio ideal no incluye el barrido de la carrocería, la holgura, el servo,
patinaje ni el diferencial. Los 440 conteos suponen 11 pulsos por vuelta del
motor, cuadratura ×4, reductor 10:1 y engranajes externos 1:1; verificar los
pulsos y dientes reales y actualizar `motorOutputCountsPerRev`, `motorGearTeeth`,
`diffGearTeeth` y los parámetros correspondientes del P4.

El perfil físico base está en `web/robot.js` y los montajes de sensores en
`web/sensors.js`. El simulador comunica esos valores al planificador al iniciar.
Los valores de partida correspondientes del P4 están en
`LIDAR System/main/local_obstacle_map.h` y en
`LIDAR System/p4_firmware/main/Kconfig.projbuild`. Estos archivos son fuentes
distintas y deben mantenerse sincronizados; la calibración y los GPIO físicos
siguen bloqueados hasta verificarse.

## Reglamento de referencia

El [reglamento internacional WRO Future Engineers 2026](https://wro-association.org/wp-content/uploads/WRO-2026-Future-Engineers-Self-Driving-Cars-General-Rules.pdf)
limita el vehículo a 300 × 200 mm y 300 mm de altura; el cajón mide 20 cm de
ancho y 1,5 veces el largo del vehículo. También especifica que en el reto de
obstáculos se pasa rojo por la derecha y verde por la izquierda, se hacen tres
vueltas y se aparca en paralelo. Consultar las
[preguntas y respuestas oficiales](https://wro-association.org/competition/questions-answers/)
y las adaptaciones de la competencia local.

El parqueo Ackermann es experimental. El binario WASM debe reconstruirse y el
perfil, la integración de sensores y las maniobras deben validarse antes de
sacar conclusiones.
