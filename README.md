<div align="center">

# AutoNex

### WRO 2026 · Future Engineers · Self-Driving Cars

**12-móvil** — vehículo autónomo de dirección tipo Ackermann

![WRO](https://img.shields.io/badge/WRO-2026-0A7B3E?style=flat-square)
![Categoría](https://img.shields.io/badge/Categor%C3%ADa-Future%20Engineers-1F6FEB?style=flat-square)
![Plataforma](https://img.shields.io/badge/Plataforma-ESP32--P4-E7352C?style=flat-square)
![Estado](https://img.shields.io/badge/Estado-En%20desarrollo-orange?style=flat-square)

</div>

---

## Tabla de contenidos

- [1. El equipo y el reto](#1-el-equipo-y-el-reto)
- [2. El vehículo: 12-móvil](#2-el-vehículo-12-móvil)
- [3. Arquitectura del sistema](#3-arquitectura-del-sistema)
- [4. Hardware](#4-hardware)
- [5. Software](#5-software)
- [6. Puesta en marcha](#6-puesta-en-marcha)
- [7. Estructura del repositorio](#7-estructura-del-repositorio)
- [8. Estado del proyecto y hoja de ruta](#8-estado-del-proyecto-y-hoja-de-ruta)
- [9. Documentación y reglamento](#9-documentación-y-reglamento)
- [10. Riesgos y mitigaciones](#10-riesgos-y-mitigaciones)
- [11. Equipo](#11-equipo)

---

## 1. El equipo y el reto

**AutoNex** es un equipo que compite en **WRO 2026**, en la categoría **Future Engineers**, dentro del desafío *Self-Driving Cars* de la World Robot Olympiad.

En esta categoría los equipos no reciben un robot: reciben un **problema de ingeniería completo**. Hay que diseñar el chasis, elegir la electrónica, desarrollar el software de percepción y control, documentar cada decisión y demostrar que el vehículo es capaz de recorrer un circuito **de forma totalmente autónoma**. Es una competencia de proceso, no solo de resultado.

### El formato de competencia

Cada temporada se corre en dos modalidades sobre el mismo circuito:

| Modalidad | Descripción | Puntaje máximo |
|---|---|---|
| **Open Challenge** | Tres vueltas cronometradas sin obstáculos. Mide consistencia y velocidad. | 30 pts |
| **Obstacle Challenge** | Tres vueltas con señalización vial aleatoria (pilares rojos y verdes) y estacionamiento final. | 62 pts |
| **Documentación** | Bitácora de ingeniería y repositorio público en GitHub. | 30 pts |

El puntaje total de la competencia es de **122 puntos**, de los cuales cerca del **25 % corresponde a la documentación**. Un vehículo rápido con mala documentación no gana.

### Qué se randomiza en cada ronda

- La **dirección de conducción** (horario o antihorario).
- La **sección de arranque** y la **zona de estacionamiento**.
- La **posición y el color** de los pilares de señalización.
- Reglas sorpresa que se anuncian el día de la competencia.

Por eso el vehículo **no puede estar programado para una pista fija**: tiene que leer el entorno y decidir en tiempo de ejecución.

### Reglas técnicas que condicionan el diseño

| Restricción | Valor |
|---|---|
| Dimensiones máximas | 300 × 200 × 300 mm |
| Peso máximo | 1.5 kg |
| Tipo de tracción | Dirección obligatoria (Ackermann). **Tracción diferencial: descalificación** |
| Ruedas | Prohibidas las omnidireccionales, esféricas y las ruedas locas tipo *ball caster* |
| Comunicación | Totalmente autónomo. Sin radio, sin control remoto, sin cables durante la ronda |
| Electrónica | Libre elección de controladores, sensores y actuadores |

### Señalización vial

- **Pilar rojo** → el vehículo debe pasar por la **derecha**.
- **Pilar verde** → el vehículo debe pasar por la **izquierda**.

Los pilares pueden tocarse o moverse, pero hacerlo cuesta puntos. Pasar por el lado incorrecto invalida la sección.

---

## 2. El vehículo: 12-móvil

**12-móvil** es nuestro vehículo de competencia. El nombre resume la arquitectura: un chasis compacto de dirección Ackermann, con la inteligencia repartida entre dos nodos de cómputo ESP32-P4 y la percepción dividida entre un LiDAR 360° y una cámara.

### Filosofía de diseño

1. **Percepción redundante.** El LiDAR entrega geometría precisa pero es ciego al color; la cámara entrega color y contexto pero es sensible a la iluminación. Juntos cubren lo que ninguno resuelve solo.
2. **Cómputo distribuido.** El nodo de visión procesa imagen; el nodo principal fusiona sensores, planifica y controla. Cada uno hace lo que mejor sabe hacer.
3. **Trazabilidad.** Todo el proceso —decisiones, pruebas, fallos y correcciones— queda registrado en este repositorio, porque así se evalúa.

### Especificaciones objetivo

| Parámetro | Objetivo |
|---|---|
| Configuración | Ackermann, tracción trasera, dirección delantera |
| Masa | ≤ 1.5 kg (objetivo interno: 1.2 kg) |
| Dimensiones | ≤ 300 × 200 × 300 mm |
| Autonomía por ronda | ≥ 3 minutos de operación continua |
| Velocidad de crucero | 0.6 – 1.0 m/s (ajustable por software) |
| Actualización del lazo de control | ≥ 50 Hz |

---

## 3. Arquitectura del sistema

### Arquitectura física

```mermaid
flowchart TB
    subgraph PER["Percepción"]
        LIDAR["LiDAR D500<br/>360° · 12 m · 10 Hz"]
        CAM["ESP32-P4-CAM<br/>Cámara MIPI 2 MP"]
        IMU["MPU-6500<br/>IMU 6-DOF"]
        ENC["Encoders Hall<br/>JGB37-520"]
    end

    subgraph CORE["Cómputo"]
        MAIN["ESP32-P4 DevKit<br/>Fusión · Planificación · Control"]
    end

    subgraph ACT["Actuación"]
        SERVO["Servo MG90S<br/>Dirección"]
        DRV["Driver de motor<br/>Puente H"]
        MOTOR["Motor JGB37-520<br/>Tracción trasera"]
    end

    subgraph PWR["Potencia"]
        BAT["LiPo 3S · 11.1 V"]
        BEC["Regulador / BEC 5 V"]
    end

    LIDAR -->|UART 230400| MAIN
    CAM -->|UART / SPI| MAIN
    IMU -->|I2C / SPI| MAIN
    ENC -->|GPIO · cuadratura| MAIN

    MAIN -->|PWM 50 Hz| SERVO
    MAIN -->|PWM + dirección| DRV
    DRV --> MOTOR

    BAT --> BEC --> MAIN
    BAT --> DRV
```

> **Nota sobre conectividad:** el ESP32-P4 no tiene radio integrada. Esto **no es una limitación para la competencia** —el reglamento prohíbe la comunicación inalámbrica durante la ronda— y simplifica el diseño. La telemetría se obtiene por USB o UART antes y después de cada prueba.

### Pipeline de software

```mermaid
flowchart LR
    A["Adquisición<br/>LiDAR · Cámara · IMU · Encoders"] --> B["Preprocesado<br/>Filtrado · ROI · calibración"]
    B --> C["Percepción<br/>Muros · pilares<br/>color rojo/verde"]
    B --> D["Odometría<br/>encoders + IMU"]
    C --> E["Localización<br/>pose en la pista"]
    D --> E
    E --> F["Planificación<br/>trayectoria · lado del pilar<br/>conteo de vueltas"]
    F --> G["Control<br/>Stanley / Pure Pursuit + PID"]
    G --> H["Actuación<br/>ángulo de servo · PWM motor"]
    H --> A
```

### Subsistemas

**Percepción geométrica — LiDAR D500.** Escaneo de 360° con alcance de 12 m a 10 Hz sobre UART a 230400 bps. Se usa para detectar los muros del circuito, estimar el ancho del carril y localizar los pilares como obstáculos cilíndricos. Los pilares son geometría idéntica en ambos colores, así que el LiDAR los detecta pero no los clasifica.

**Percepción visual — ESP32-P4-CAM.** Placa compacta basada en ESP32-P4 con cámara MIPI de 2 MP. Su tarea es la **clasificación cromática** de los pilares (segmentación en espacio HSV, robusta a cambios de iluminación) y la detección de la línea de meta y de la zona de estacionamiento. El ESP32-P4 incluye ISP y encoder H.264 por hardware, lo que permite trabajar a resolución reducida con buen framerate.

**Localización — MPU-6500 + encoders.** El MPU-6500 aporta velocidad angular y aceleración lineal a alta frecuencia; los encoders Hall del JGB37-520 aportan desplazamiento longitudinal. La fusión corrige la deriva del giroscopio con la información de las ruedas y permite estimar el giro acumulado para el conteo de vueltas.

**Control de dirección — MG90S.** Servo de engranajes metálicos que posiciona la dirección delantera. El ángulo de consigna se calcula con geometría de Ackermann a partir de la curvatura deseada. La tracción es trasera y proporcional, mediante un puente H controlado por PWM.

**Gestión de energía.** Batería LiPo 3S (11.1 V nominal, 12.6 V a plena carga). Un regulador entrega 5 V estables para lógica, LiDAR y servo; los motores se alimentan directamente desde la batería a través del driver. La separación de rieles evita que el pico de arranque del motor reinicie el cómputo.

---

## 4. Hardware

### Componentes principales

| # | Componente | Modelo | Función | Interfaz |
|---|---|---|---|---|
| 1 | Controlador principal | **ESP32-P4 DevKit** | Fusión de sensores, planificación, control, telemetría | — |
| 2 | Nodo de visión | **ESP32-P4-CAM** | Captura y procesamiento de imagen | MIPI-CSI · UART/SPI |
| 3 | Sensor de distancia | **LDROBOT D500** | Escaneo 360°, 0.03–12 m, 5000 Hz, 10 Hz | UART 230400 |
| 4 | Unidad inercial | **MPU-6500** | Giroscopio + acelerómetro de 3 ejes | I2C / SPI |
| 5 | Motor de tracción | **JGB37-520 + encoder Hall** | Tracción trasera, caja reductora metálica | PWM + cuadratura |
| 6 | Servo de dirección | **MG90S (engranajes metálicos)** | Ángulo de dirección Ackermann | PWM 50 Hz |
| 7 | Batería | **LiPo 3S · 11.1 V** | Alimentación del sistema | — |

### Detalle técnico

| Componente | Especificaciones relevantes |
|---|---|
| **ESP32-P4** | Dual-core RISC-V @ 400 MHz · 768 KB SRAM · hasta 32 MB PSRAM · MIPI-CSI/DSI · ISP y encoder H.264 1080p · USB 2.0 · **sin radio integrada** |
| **D500** | DTOF · 360° · 0.03–12 m · 5 000 mediciones/s · 10 Hz · resolución angular ~1° · UART 230400 · clase láser 1 |
| **MPU-6500** | 6 ejes (3 giroscopio + 3 acelerómetro) · 16 bits · hasta 4 kHz · I2C hasta 400 kHz / SPI hasta 1 MHz |
| **JGB37-520** | 12 V DC · reductor metálico · encoder Hall de cuadratura · eje D de 6 mm · variante de 333 rpm |
| **MG90S** | Servo micro de engranajes metálicos · ~2.2 kg·cm a 4.8 V · control por PWM |
| **LiPo 3S** | 11.1 V nominal · 12.6 V máximo · requiere BEC para el riel de 5 V |

### Elementos pendientes de definir

El diseño eléctrico aún no está cerrado. Falta especificar y documentar:

- **Driver de motor (puente H).** No incluido en la lista de componentes. Se requiere para controlar el JGB37-520 en ambos sentidos con PWM. Candidatos: TB6612FNG, DRV8871 o BTS7960 según corriente de arranque.
- **Regulador de 5 V (BEC).** Debe soportar el pico del servo más el LiDAR y el cómputo.
- **Protección de batería.** Monitoreo de voltaje por celda y corte por baja tensión.
- **Chasis y tren delantero.** Geometría de Ackermann, relación de transmisión y montaje del servo.

Estos puntos se irán resolviendo en `docs/` y en el BOM conforme avance el proyecto.

---

## 5. Software

### Stack

| Capa | Tecnología |
|---|---|
| Firmware | **ESP-IDF** (C/C++) sobre ESP32-P4 |
| Visión | Procesamiento de imagen en el nodo ESP32-P4-CAM |
| Control | Lazo en tiempo real sobre FreeRTOS |
| Herramientas | Git, CMake, `idf.py` |
| Documentación | Markdown + Mermaid en este repositorio |

### Lazo de control

```
┌──────────────────────────────────────────────────────────┐
│  Cada 20 ms (50 Hz)                                      │
│                                                          │
│  1. Leer LiDAR  → nube de puntos en coordenadas locales  │
│  2. Leer cámara → color de pilares + offset de carril    │
│  3. Leer IMU    → yaw y velocidad angular                │
│  4. Leer encoder→ distancia recorrida                    │
│  5. Fusionar    → pose estimada (x, y, θ)                │
│  6. Decidir     → lado del pilar, curvatura, velocidad   │
│  7. Calcular    → ángulo de servo, PWM de motor          │
│  8. Actuar      → servo + driver                         │
│                                                          │
└──────────────────────────────────────────────────────────┘
```

### Máquina de estados de la ronda

```mermaid
stateDiagram-v2
    [*] --> INIT
    INIT --> DRIVE: sistema OK
    INIT --> SAFE_STOP: fallo de sensor
    DRIVE --> OBSTACLE: pilar detectado
    OBSTACLE --> DRIVE: pilar superado
    DRIVE --> LAP_CHECK: sección de meta
    LAP_CHECK --> DRIVE: vueltas < 3
    LAP_CHECK --> PARK_SEARCH: vueltas = 3
    PARK_SEARCH --> PARK_ALIGN: zona de estacionamiento
    PARK_ALIGN --> PARKED: dentro y paralelo
    PARKED --> [*]
    SAFE_STOP --> [*]
```

### Decisiones de ingeniería registradas

Las decisiones técnicas, sus alternativas y la justificación de cada elección se documentan en `docs/`. El formato es deliberadamente simple: contexto, opciones consideradas, decisión y consecuencias.

---

## 6. Puesta en marcha

> **Estado:** el repositorio está en fase de arranque. Las instrucciones siguientes describen el entorno previsto; se actualizarán cuando el firmware esté publicado.

### Requisitos

- **ESP-IDF v5.3** o superior
- Python 3.10+
- Toolchain RISC-V para ESP32-P4
- Drivers USB-C (el ESP32-P4 usa USB 2.0 nativo)

### Instalación del entorno

```bash
# Clonar el repositorio
git clone https://github.com/Eddym06/AutoNex.git
cd AutoNex

# Instalar ESP-IDF (Linux/macOS)
git clone -b v5.3 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf && ./install.sh esp32p4 && . export.sh
```

```powershell
# Instalar ESP-IDF (Windows)
git clone -b v5.3 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf ; .\install.bat esp32p4 ; .\export.ps1
```

### Compilar y flashear

```bash
cd firmware/main
idf.py set-target esp32p4
idf.py menuconfig        # configuración de pines y sensores
idf.py build
idf.py -p <PUERTO> flash monitor
```

### Verificación rápida

1. Encender sin motores conectados y verificar el arranque por monitor serie.
2. Confirmar lectura del LiDAR (escaneo estable a 10 Hz).
3. Confirmar lectura de la IMU (yaw sin deriva apreciable en reposo).
4. Verificar el sentido de giro de los encoders y la dirección del servo.
5. Solo entonces conectar la tracción y probar en piso elevado.

---

## 7. Estructura del repositorio

```text
AutoNex/
├── README.md              # este documento
├── docs/                  # bitácora de ingeniería y decisiones de diseño
│   ├── 01-requisitos.md
│   ├── 02-arquitectura.md
│   ├── 03-hardware.md
│   └── 04-pruebas.md
├── firmware/
│   ├── main/              # nodo principal: fusión, planificación, control
│   └── vision/            # nodo de visión: captura y clasificación
├── hardware/
│   ├── cad/               # modelos 3D del chasis
│   ├── pcb/               # esquemáticos y placa
│   └── bom/               # lista de materiales y costos
├── tools/                 # scripts de calibración y análisis
└── .github/               # plantillas y automatizaciones
```

Las carpetas marcadas corresponden a la estructura objetivo. Se irán creando conforme avance el desarrollo; este README se mantiene sincronizado con el estado real del repositorio.

---

## 8. Estado del proyecto y hoja de ruta

| Fase | Entregable | Estado |
|---|---|---|
| 0 | Definición del reto y reglamento | ✅ Completado |
| 1 | Selección de componentes y arquitectura | ✅ Completado |
| 2 | Chasis y tren delantero Ackermann | 🔄 En curso |
| 3 | Bring-up de electrónica y drivers | ⬜ Pendiente |
| 4 | Percepción: LiDAR y cámara | ⬜ Pendiente |
| 5 | Odometría y localización | ⬜ Pendiente |
| 6 | Control de trayectoria | ⬜ Pendiente |
| 7 | Open Challenge completo | ⬜ Pendiente |
| 8 | Obstacle Challenge y estacionamiento | ⬜ Pendiente |
| 9 | Bitácora de ingeniería final | ⬜ Pendiente |

**Hitos de validación:** el vehículo debe completar tres vueltas sin intervención antes de optimizar velocidad. Primero se gana consistencia, después se gana tiempo.

---

## 9. Documentación y reglamento

El reglamento de Future Engineers exige un **repositorio público en GitHub** con la documentación del proceso de ingeniería. Los puntos que condicionan nuestro flujo de trabajo:

- La documentación debe cubrir **movilidad, energía, percepción y programación** del vehículo, no solo el resultado final.
- El **primer commit** debe realizarse **al menos dos meses antes** de la competencia y contener **no menos de un quinto del código final**.
- El código y el diseño deben ser **trabajo propio del equipo**. Los jueces pueden hacer preguntas técnicas sobre cualquier decisión.
- Se evalúa la **bitácora de ingeniería**: pruebas, errores, iteraciones y aprendizajes, no solo los aciertos.

Por eso este repositorio se construye **en paralelo al vehículo**, no al final.

---

## 10. Riesgos y mitigaciones

| Riesgo | Impacto | Mitigación |
|---|---|---|
| Torque del MG90S insuficiente para la dirección | Pérdida de precisión en curva | Reducir masa del eje delantero, geometría de baja fricción, evaluar servo de mayor torque |
| Peso por encima de 1.5 kg | Descalificación en revisión técnica | Presupuesto de masa por subsistema y pesaje en cada iteración |
| Deriva del giroscopio del MPU-6500 | Error acumulado en el conteo de vueltas | Fusión con encoders + corrección por detección de la línea de meta |
| LiDAR a 10 Hz con el vehículo en movimiento | Detección tardía de pilares | Limitar velocidad, predicción de trayectoria y ventana de anticipación |
| El LiDAR no distingue color | Imposible decidir el lado del pilar | Clasificación cromática obligatoria por cámara |
| Caída de tensión al arrancar motores | Reinicio del cómputo en plena ronda | Riel de lógica separado con BEC y capacitores de desacople |
| Falta de driver de motor definido | Bloqueo del bring-up | Selección y compra en fase 3, antes de las pruebas dinámicas |
| Iluminación variable en la pista | Falsa clasificación rojo/verde | Segmentación HSV calibrada + validación con LiDAR |

---

## 11. Equipo

**AutoNex** — WRO 2026, categoría Future Engineers.

> Completar con los integrantes, roles y coach antes de la revisión técnica.

| Rol | Responsabilidad |
|---|---|
| Líder de proyecto | Coordinación, cronograma y documentación |
| Ingeniería mecánica | Chasis, tren Ackermann, montaje de sensores |
| Ingeniería electrónica | Potencia, cableado, integración de sensores |
| Software | Percepción, localización, control y pruebas |

---

<div align="center">

**AutoNex · 12-móvil**

*Construido por el equipo, documentado para el jurado, probado en la pista.*

</div>
