# WitMotion D6 (COIN-D6) — LiDAR, protocolo y driver

Documentación de referencia para programar y conectar el LiDAR. Cubre el
hardware, el protocolo serie verificado, el visor de PC, el sketch del ESP32
y el problema de pantallas azules con el driver CH340.

---

## 1. Hardware

| Elemento | Valor |
|---|---|
| Sensor | WitMotion D6 = COIN-D6 (Guoke Optics), dToF 360° |
| Alcance usado | 12 m (`MAX_RANGE_MM` = 12000) |
| Interfaz | UART 3.3 V, 4 pines 1.5 mm: VCC, GND, RXD, TXD |
| Velocidad | **230400 baudios, 8N1**, sin control de flujo |
| Salida | Ángulo, distancia e intensidad |
| Muestreo | ~4000 Hz nominal (según distribuidor) |

Conexión al PC (usada en pruebas):

```
LiDAR TXD  ──►  RXD  CP2102 / CH340  ──► USB  ──► PC (COM9)
LiDAR GND  ───  GND
LiDAR VCC  ───  5 V (alimentación del LiDAR, no del adaptador)
```

Conexión al ESP32 (sketch del proyecto):

```
LiDAR TXD  ──►  GPIO16 (RX2 del ESP32)
LiDAR GND  ───  GND del ESP32 (GND común obligatorio)
```

El LiDAR consume corriente en pico al arrancar el motor. Si se alimenta desde
el puerto USB de una laptop, un puerto débil o un hub puede provocar reinicios
o pérdida de datos. Usa un puerto USB trasero directo o una fuente de 5 V.

---

## 2. Protocolo serie (verificado)

Fuente del formato: decodificador de referencia en
[1-rafael-1/explorer-robot](https://github.com/1-rafael-1/explorer-robot)
(`coin-d6/src/decoder.rs`). Verificado contra una captura real de COM9:
checksum correcto en un paquete de 85 bytes con LSN = 25.

### 2.1 Cabecera (10 bytes)

| Offset | Campo | Descripción |
|---|---|---|
| 0 | `0xAA` | Cabecera 0 |
| 1 | `0x55` | Cabecera 1 |
| 2 | `CT` | Bit 0 = **inicio de vuelta** (ring start). Los demás bits no se usan aquí |
| 3 | `LSN` | Número de muestras en el paquete |
| 4–5 | `FSA` | Ángulo de la primera muestra, little-endian |
| 6–7 | `LSA` | Ángulo de la última muestra, little-endian |
| 8–9 | `CS` | Checksum de 16 bits, little-endian |

### 2.2 Muestras

Después de la cabecera hay `LSN` muestras de **3 bytes** cada una:

| Byte | Campo |
|---|---|
| `Si_L` | Intensidad (bits altos) |
| `Si_2nd` | Bits 0–1: intensidad (bits bajos). Bits 2–7: distancia (bits bajos) |
| `Si_H` | Distancia (bits altos) |

Longitud total del paquete: `10 + 3 × LSN` bytes. Con LSN = 25, son 85 bytes.

### 2.3 Fórmulas

```
distancia_mm   = Si_H × 64 + (Si_2nd >> 2)
intensidad     = (Si_2nd & 0x03) × 64 + (Si_L >> 2)     // 0–255
angulo_grados  = (valor >> 1) / 64                       // valor = FSA o LSA
```

Ángulo de cada muestra, interpolado entre FSA y LSA:

```
si LSA < FSA:  LSA += 360
paso = (LSA − FSA) / (LSN − 1)
angulo_i = (FSA + i × paso) mod 360
```

Una distancia de 0 significa **sin retorno válido** y debe descartarse.

### 2.4 Checksum (XOR de 16 bits)

```
cs = 0x55AA
cs ^= CT | (LSN << 8)
cs ^= FSA
cs ^= LSA
para cada muestra i:
    cs ^= Si_L
    cs ^= (Si_H << 8) | Si_2nd
válido si cs == (byte[8] | byte[9] << 8)
```

Los bytes `CS` (offsets 8 y 9) no entran en el cálculo.

**Error corregido:** antes el proyecto usaba cabecera `0x54`, CRC8 de 47 bytes
y 12 puntos. Eso no corresponde a este sensor. El formato correcto es el de
esta sección.

### 2.5 Ejemplo real capturado

```
AA 55 00 19 F9 2C C9 37 DB 48 D8 C4 01 3C 8D 01 CC 00 00 70 ...
│  │  │  │  └FSA┘ └LSA┘ └CS┘ └ muestra 0 = D8 C4 01 (Si_L Si_2nd Si_H) → 113 mm
│  │  │  └ LSN = 25 (0x19)
│  │  └ CT = 00 (no es inicio de vuelta)
└──┴ cabecera
```

- FSA = 89.94°, LSA = 111.56°, checksum `0x48DB` OK.
- Tamaño del paquete: 85 bytes.
- Muestras válidas en esa captura: 12 de 25 (el resto son sin retorno).

---

## 3. Herramientas del proyecto

| Archivo | Uso |
|---|---|
| `LIDAR System/tools/lidar_viewer.html` | Visor en el PC con Web Serial (Chrome o Edge). Elige COM9 y 230400 baudios |
| `LIDAR System/sketch_oct14a/sketch_oct14a.ino` | Firmware del ESP32: red WiFi "LIDAR" y radar web en http://192.168.4.1 |
| `LIDAR System/tests/test_witmotion_d6.c` | Prueba del parser en PC con un paquete real |

Probar el parser en PC (MSYS2 / MinGW):

```bash
gcc -Wall -o test_witmotion_d6 test_witmotion_d6.c && ./test_witmotion_d6
```

Resultado esperado: `RESULTADO: OK`.

### 3.1 Visor en el PC

1. Conecta el adaptador con el LiDAR.
2. Abre `lidar_viewer.html` en Chrome o Edge.
3. Pulsa **Conectar puerto**, elige COM9 y deja **230400** baudios.
4. Si todo es correcto, verás paquetes OK y el radar en vivo.

Antes de abrirlo, cierra cualquier otro programa que use COM9 (el Monitor
serie de Arduino IDE, por ejemplo). Si no, la página no podrá abrir el puerto.

---

## 4. Driver del adaptador CH340 — pantallas azules (resuelto)

### 4.1 Síntoma

- Pantallas azules (BSOD) **0x000000D1** `DRIVER_IRQL_NOT_LESS_OR_EQUAL`,
  con parámetros `P1 = 0x38`, `P2 = 2` (IRQL DISPATCH), `P3 = 0` (lectura).
- Ocurrieron el **14/10/2026 a las 10:51** y **11:11**.
- Volcados: `C:\Windows\Minidump\101426-14796-01.dmp` y
  `C:\Windows\Minidump\101426-15937-01.dmp`.

### 4.2 Causa

Análisis con `cdb` (WinDbg), `!analyze -v`:

- `MODULE_NAME: CH341S64`, `IMAGE_NAME: CH341S64.SYS`.
- Falla en `CH341S64+0x48a1`, en un **DPC** (rutina diferida) que se ejecuta
  con la CPU en nivel DISPATCH.
- El driver lee la dirección `0x38` desde un **puntero nulo**. Es un error del
  driver, no de RAM ni del LiDAR.
- Ambos crashes terminan en el mismo desplazamiento `+0x48a1`, así que es el
  mismo bug.
- Versión afectada: `ch341ser.inf` **3.9.2024.9** (16/09/2024), `CH341S64.SYS`
  versión 3.90, proveedor wch.cn. Se instaló como `oem265.inf`.

Reportes de terceros con el mismo patrón (sin parche oficial confirmado en la
versión 3.9):

- [SparkFun, RedBoard con CH341S64 (0xD1)](https://community.sparkfun.com/t/redboard-qwiic-dev-15123-keeps-crashing-windows-ch341s64-driver-irql-not-less-or-equal-d1-need-help/60269)
- [Aqua Computer, bluescreen en usb serial driver](https://forum.aquacomputer.de/weitere-foren/english-forum/114291-bluescreen-in-usb-serial-driver-solved-unrelated-to-aquasuite/)
- [Arduino IDE #2592, BSOD con impresión continua a 115200](https://github.com/arduino/arduino-ide/issues/2592)

Factores que aumentan el riesgo: tráfico serie continuo y a alta velocidad,
desconexiones en caliente y clones del CH340.

### 4.3 Solución aplicada

1. **Respaldo** del paquete 3.9.2024.9 en
   `scratchpad\backup_ch341_3.9.2024.9` (exportado con `pnputil /export-driver`).
2. **Desinstalación** del paquete problemático con
   `pnputil /delete-driver oem265.inf /uninstall`. Se eliminó `CH341S64.SYS`
   de `System32\drivers`.
3. **Instalación del driver oficial WCH 4.0.2026.2**, descargado de
   <https://www.wch-ic.com/downloads/CH341SER_ZIP.html> (696 KB, subido el
   26/06/2026). Verificación:
   - Firma: Microsoft WHQL (`CH341SER.CAT` y `CH341S64.sys` válidos).
   - Versión del `.sys`: 4.00. ID: `USB\VID_1A86&PID_7523`.
   - Instalación: `pnputil /add-driver CH341SER.INF /install` → `oem265.inf`.
4. **Bloqueo de Windows Update:** `HKLM\SOFTWARE\Policies\Microsoft\Windows\WindowsUpdate`
   → `ExcludeWUDriversInQualityUpdate = 1`. Windows Update no instalará drivers
   automáticamente, así que no vuelve a poner el 3.9.

Estado tras la solución: COM9 OK, `CH341SER_A64` en ejecución con 4.00, sin
bugchecks desde el arranque de las 11:11.

### 4.4 Cómo revertir

- Quitar la política: poner `ExcludeWUDriversInQualityUpdate` en 0 o borrarla.
- Restaurar el 3.9: `pnputil /add-driver` sobre la carpeta de respaldo. **No
  recomendado:** ese driver causó los crashes.

### 4.5 Comprobaciones periódicas

```powershell
# Versión cargada y estado
Get-CimInstance Win32_SystemDriver | Where-Object Name -match 'CH34'
(Get-Item C:\Windows\System32\drivers\CH341S64.SYS).VersionInfo.FileVersion

# Bugchecks recientes
Get-WinEvent -FilterHashtable @{LogName='System'; Id=1001; StartTime=(Get-Date).AddDays(-7)}

# Política de Windows Update
(Get-ItemProperty 'HKLM:\SOFTWARE\Policies\Microsoft\Windows\WindowsUpdate').ExcludeWUDriversInQualityUpdate
```

El valor esperado de `FileVersion` es `4.00` y el de la política es `1`.

---

## 5. Firmware ESP32 (sketch_oct14a)

- Placa: ESP32 DevKit clásico (`esp32:esp32:esp32`). Core instalado: esp32:esp32 3.3.11.
- Compilar con arduino-cli: `arduino-cli compile --fqbn esp32:esp32:esp32 "LIDAR System/sketch_oct14a"`.
- Subir: `arduino-cli upload --fqbn esp32:esp32:esp32 --port COMx "LIDAR System/sketch_oct14a"`.
- Salida de datos al puerto serie del ESP32 (115200): contadores de paquetes
  OK, checksum, vueltas, puntos y descartados cada 2 s.
- Red WiFi "LIDAR", contraseña `12345678`, página en http://192.168.4.1.

### 5.1 Problemas al subir (ESP32 en COM11)

- Síntoma: `Serial data stream stopped: Possible serial noise or corruption`.
- Diagnóstico: `esptool` conecta y detecta el chip (ESP32-D0WD-V3, rev 3.1),
  pero la transferencia se corta siempre a ~16 KB (2.6 %), a cualquier
  velocidad (921600, 115200, 57600).
- Causa probable: alimentación o cable USB. No es el software.
- Qué probar: puerto USB trasero directo, cable de datos, sin hub.

---

## 6. Referencias

- Protocolo: [1-rafael-1/explorer-robot, coin-d6/src/decoder.rs](https://github.com/1-rafael-1/explorer-robot)
- Visor de referencia en C++ (no usado en este proyecto): [TabulateTech/D6-LIDAR](https://github.com/TabulateTech/D6-LIDAR)
- Driver oficial: [WCH, CH341SER.ZIP](https://www.wch-ic.com/downloads/CH341SER_ZIP.html)
