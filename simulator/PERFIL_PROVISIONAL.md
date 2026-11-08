# Perfil provisional de simulación — 12-móvil

Estas medidas son **suposiciones para desarrollar el simulador**, no medidas
del equipo ni dimensiones estándar exigidas por WRO. Cambiar al medir el
vehículo montado; no usar para habilitar la tracción física.

## Chasis medido (perfil `largo26`)

Medidas dadas por el equipo: **260 × 130 mm**, batalla **184,5 mm**, rueda Ø**66 mm**.
Con eso el cajón del reglamento mide **20 × 39 cm** (1,5 × el largo) y los voladizos
quedan simétricos en **37,75 mm**. Está en `web/robot.js` como `PROFILES.largo26`:

```js
new WRO.sim.Simulation(core, { field, robot: WRO.robot.PROFILES.largo26 })
```

Medido en el simulador con ese perfil (40° de volante):

| Prueba | Resultado |
| --- | --- |
| Salida del cajón, 6 combinaciones sentido × semilla | **8,1–8,9 s**, sin contactos |
| Ronda completa de 62 puntos (salir, 3 vueltas, aparcar) | **4 de 4**, terminando a los 138–145 s |
| Aparcar desde la pista (0 y 2 letreros) | **55 puntos, parqueo completo**, 138–141 s |

Frente al chasis provisional de 22 × 14 (salida 9,2–9,7 s, ronda 141–155 s): sale un
segundo antes y deja más margen, porque el cajón crece con el largo (la holgura a lo
largo es siempre 0,5 × el largo: 13 cm en vez de 11) y el coche es 1 cm más estrecho.

> El volante de 40° y la vía trasera de 12 cm siguen siendo suposiciones: la vía no se
> midió y afecta a la odometría del eje (las dos ruedas giran sobre el mismo eje).

## Perfil provisional (por defecto)

| Dato | Valor provisional |
| --- | ---: |
| Largo × ancho de carrocería | 22 × 14 cm |
| Distancia entre ejes | 14 cm |
| Saliente detrás del eje trasero | 4 cm |
| Vía trasera | 12 cm |
| Diámetro de ruedas | 6,6 cm |
| Dirección efectiva máxima | 40° a cada lado |
| Radio mínimo ideal del eje trasero | 14 / tan(40°) ≈ 16,7 cm |
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
