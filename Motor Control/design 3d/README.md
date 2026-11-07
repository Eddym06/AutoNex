# Disco encoder óptico de 30 ranuras (motor TT 1:48)

Diseño 3D del disco ranurado para encoder óptico (sensor de ranura tipo KLH-512), pensado para montarse en el eje del motorreductor DC "TT" amarillo (1:48, 5-6 V).
Modelado en Autodesk Fusion 360 (diseño paramétrico, unidades en mm).

## Archivos de esta carpeta

| Archivo | Para qué sirve |
|---|---|
| `DiscoEncoder_30ranuras.f3d` | Archivo nativo de Fusion 360 con el historial paramétrico (boceto -> extrusión -> corte -> patrón). Editable. |
| `DiscoEncoder_30ranuras.step` | Formato CAD neutro, para abrir en otros programas CAD. |
| `DiscoEncoder_30ranuras.stl` | Malla (refinamiento alto) para laminar e imprimir en 3D. |

## Especificaciones

| Parámetro | Valor |
|---|---|
| Diámetro exterior | 24 mm |
| Espesor | 3 mm |
| Agujero del eje | Ø5.5 mm con dos planos paralelos separados 3.7 mm (perfil doble D del eje TT) |
| Número de ranuras | 30 |
| Paso angular | 12° (una ranura + un brazo) |
| Ancho angular de ranura | 6° (ciclo de trabajo 50 %: ranura 6°, brazo 6°) |
| Forma de las ranuras | Sectores radiales, de r = 7.5 mm a r = 10.5 mm (3 mm de largo radial) |
| Aro exterior | 1.5 mm (de r = 10.5 a r = 12 mm) |
| Ancho de ranura | ~0.79 mm en r = 7.5 mm, ~1.10 mm en r = 10.5 mm (0.94 mm en el radio medio) |

La primera ranura está centrada sobre el eje +X del boceto. Cada ranura se genera a partir de un solo corte y un patrón circular de 30 copias (360°) alrededor del eje Z.

## Geometría teórica de los flancos

- Flancos por vuelta: 60 (30 de subida + 30 de bajada), uno cada 6°.
- Posición angular nominal del flanco `j` (j = 0..59): `theta_j = 3° + 6° * j`, medida desde el eje +X.
- Resolución angular: 6° por flanco.
- Con rueda de radio 32.5 mm (circunferencia ~204.2 mm) y el disco en el eje de la rueda, cada flanco equivale a ~3.40 mm de avance.

Los valores son los **nominales del CAD**. La pieza impresa tendrá desviaciones de ancho de ranura, así que la tabla de calibración por flanco (factor `C_i`) sigue siendo necesaria para corregirlas.

## Notas de impresión

- Las ranuras miden menos de 1.1 mm de ancho; con boquilla de 0.4 mm en FDM pueden salir cerradas o irregulares.
- Se recomienda resina o impresión con capas finas, o ensanchar el radio interior de las ranuras.
- La pieza debe imprimirse plana (cara de 24 mm sobre la cama) y ser opaca al infrarrojo del sensor.
- Verificar el ajuste del agujero con una pieza de prueba: ajustar el diámetro/planos según la tolerancia de la impresora.

## Cómo modificarlo

Abrir el `.f3d` en Fusion 360. En el timeline: `DiscoBase` (Ø24 x 3) -> `AgujeroEje` (perfil doble D) -> `Ranura` (primer sector) -> `Patron30Ranuras` (cantidad y ángulo editables).
