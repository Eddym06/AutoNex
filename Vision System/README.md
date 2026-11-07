# Sistema de visión — ESP32-S3 CAM

Proyecto ESP-IDF 5.3.1 para la **Aideepen ESP32-S3-WROOM CAM N16R8 con OV3660** (16 MB de flash, 8 MB de PSRAM). Captura RGB565 **QQVGA 160 × 120**, detecta bloques rojos y verdes y las líneas naranja y azul de las esquinas, y envía al P4 una **tabla compacta** (nunca imágenes). No decide giros: el P4 es el único que mueve el carrito.

## Qué hace el detector (`main/vision.c`, C independiente del hardware)

1. Convierte RGB565 a HSV y clasifica con **ventanas configurables** por color (rojo, verde, magenta, naranja, azul): `hue_lo/hi`, `sat_min`, `val_min`, `enabled`.
2. Agrupa muestras vecinas en componentes conexos dentro de la región de interés (`roi_start`–`roi_end`) y descarta los menores de `min_samples` o `min_span_px`. Hasta `max_objects` (12) por cuadro, cada uno con su caja; bloques pegados que parecen uno solo se marcan `MERGED`.
3. **Distancia**: con la cámara inclinada usa la fila donde el bloque toca el suelo (`horizon_row`, `camera_height_cm`, `focal_px`); si el borde inferior está cortado, tapado o fuera de la imagen, recurre al ancho (`pillar_width_cm`). Las banderas del objeto dicen cuál se usó: `CLIPPED`, `GROUND`, `OCCLUDED`, `MERGED`. El P4 sabe así cuánto fiarse.
4. **Lado de paso por color**: `pass_side_red` / `pass_side_green` (+1 derecha, −1 izquierda, 0 libre) y `enforce_color_side`. La recomendación `action` respeta ese lado, y el P4 recibe además el color de cada objeto para aplicar su propia regla.
5. Filas de las líneas **naranja y azul** (`line_row_start/end`, `line_min_samples`), enviadas en la cabecera para que el P4 detecte esquinas.
6. Parqueo magenta experimental (`WRO_PARKING_MODE`, desactivado por defecto). **No hay maniobra de parqueo en el P4.**

## Todo es configurable, siempre con valores por defecto

`vision_config_default()` define cada valor; `vision_config_sanitize()` rechaza lo imposible. Puedes cambiar parámetros por la consola (UART0 / USB) o por el enlace con el P4, sin recompilar:

```text
set pass_side_red -1        cambia un campo (clave valor)
set camera.exposure 300
save                        guarda en NVS (sobrevive al reinicio)
defaults                    borra la copia guardada y vuelve a los valores de fábrica
dump                        imprime la configuración activa
help
```

La misma línea acepta una **trama binaria** de configuración (`A6 6A` + CRC16, `vision_config_encode`) por si el P4 o una herramienta de PC la envía. La lista completa de claves está en la tabla de `vision.c` (`hsv`, `pass_side_*`, `focal_px`, `camera_height_cm`, `horizon_row`, `roi_*`, `act_distance_cm`, `corridor_*_px`, `camera.*`…).

### Cámara (`camera.*`)

`camera.apply` es **0 por defecto**: el firmware deja el sensor exactamente como estaba (el perfil ya probado en hardware). Con `camera.apply 1` aplica `auto_exposure`, `exposure`, `ae_level`, `auto_gain`, `gain`, `auto_wb`, `wb_mode`, `brightness`, `contrast`, `saturation`, `hmirror`, `vflip`. `camera.lock_after_frames` (por defecto 0 = nunca; también `WRO_LOCK_AUTO_AFTER_FRAMES` en menuconfig) congela exposición, ganancia y balance de blancos tras N cuadros, para que la iluminación cambiante de una pista no mueva los colores a mitad de ronda.

## Protocolo S3 → P4 (v2)

Trama binaria con cabecera de 14 bytes (secuencia, estado, acción, filas de las líneas naranja/azul…) y 6 bytes por objeto (color, centro, ancho, distancia en mm, banderas), CRC16-CCITT. Definido en [`../shared/sensor_table_protocol.h`](../shared/sensor_table_protocol.h) y compartido con el receptor del P4. Por defecto el enlace sale por la **UART0 de la placa (el puente USB-UART CH340) a 921600 baudios 8N1** (`WRO_LINK_ON_CONSOLE_UART`, `WRO_LINK_BAUD`): un solo cable USB al host del P4 lleva la tabla (S3→P4) y los comandos de configuración (P4→S3). Los logs quedan limitados a errores para no corromper tramas, y el monitor serie del PC debe usar los mismos 921600 baudios. Si se desactiva esa opción, vuelve el enlace UART1 de antes: 230400 baudios 8N1, TX en GPIO1 (`WRO_LINK_TX_GPIO`), RX en `WRO_LINK_RX_GPIO` (−1 = un solo sentido) y masa común con el P4.

## Módulos

| Archivo | Responsabilidad |
| --- | --- |
| `main/app_main.c` | Ciclo de captura, configuración, tabla y consola |
| `main/vision.c/.h` | Detector, configuración, calibración (`vision_calibrate_focal/horizon`) |
| `main/vision_settings.c/.h` | Persistencia NVS de la configuración |
| `main/vision_config_cmd.c/.h` | Intérprete de comandos y trama binaria de configuración |
| `main/vision_config_rx.c/.h` | Recepción de configuración por el enlace |
| `main/camera_source.c/.h` | Inicialización de la cámara y aplicación de `camera.*` |
| `main/vision_table.c/.h`, `vision_uart_link.c/.h` | Tabla v2 y envío por UART |
| `../shared/config_fields.c/.h` | Motor genérico de configuración `clave valor` |
| `tests/` | `test_vision.c` (586 comprobaciones), `test_vision_config_cmd.c`, `test_vision_table.c`, `sequence_runner.c` |
| `tools/idf.ps1`, `patch_ov3660_pll.py`, `activate.ps1` | Entorno ESP-IDF y perfil PLL del OV3660 |
| `tools/simulate.ps1` | Ejecuta **todas** las pruebas del repositorio |

## Placa y pines

Mapa GOOUU/S3-EYE del [ejemplo oficial de esp32-camera](https://github.com/espressif/esp32-camera/blob/master/examples/camera_example/main/camera_pinout.h). No hay esquema de Aideepen que confirme la revisión: comprueba los GPIO de tu unidad (la cámara ya se probó en hardware con este mapa).

| Señal | GPIO |
| --- | --- |
| PWDN / RESET | -1 / -1 |
| XCLK | 15 |
| SCCB SDA / SCL | 4 / 5 |
| D0–D3 | 11 / 9 / 8 / 10 |
| D4–D7 | 12 / 18 / 17 / 16 |
| VSYNC / HREF / PCLK | 6 / 7 / 13 |

## Compilar y probar

```powershell
.\tools\idf.ps1 build                # no invoques idf.py directamente: el parche del PLL se aplica aquí
.\tools\idf.ps1 menuconfig
.\tools\idf.ps1 -p COM5 flash monitor
.\tools\simulate.ps1                 # todas las pruebas (-Quick sin rondas completas, -Wasm recompila el firmware del simulador)
```

Requiere GCC, Node.js y Python 3 para las pruebas. GCC compila con advertencias como errores.

## Rendimiento medido en hardware (9 oct 2026)

QQVGA estándar: ~46,6 FPS durante más de 70 s, detector ~2,6 ms por cuadro. QVGA (320 × 240) baja a ~8,9 FPS y el PLL rápido falla con él, por eso QQVGA es el estándar. Un bloque de 5 cm mide ~7 px a 80 cm, ~13 px a 45 cm y ~20 px a 30 cm: más allá de 60–80 cm la detección es inestable.

## Qué no se ha comprobado

- Las ventanas HSV, la altura/inclinación de la cámara (`camera_height_cm`, `horizon_row`) y `focal_px` por defecto son valores de partida; calibra con la lente y la iluminación de tu pista (`vision_calibrate_*` y el comando `dump`).
- La configuración en caliente por el enlace con el P4 (RX) compila y se prueba en C, pero **no** se ha probado con el cableado real.
- El firmware con las nuevas opciones compila sin avisos; no se ha vuelto a cargar en la placa tras estos cambios.
