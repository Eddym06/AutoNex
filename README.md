<div align="center">

<img src="t-photos/TeamLogo.webp" alt="Team AutoNex (RDX), Instituto Tecnológico de Las Américas" width="470">

<sub>Team AutoNex (RDX) · Instituto Tecnológico de Las Américas · 14 November 2026</sub>

# AutoNex

### An Ackermann self-driving car that maps the track once, then drives the fastest line it finds

**WRO 2026 · Future Engineers · Self-Driving Cars** &nbsp;·&nbsp; 🇪🇸 [Leer en español](README.es.md)

![WRO](https://img.shields.io/badge/WRO-2026-0A7B3E?style=flat-square)
![Category](https://img.shields.io/badge/Category-Future%20Engineers-1F6FEB?style=flat-square)
![MCU](https://img.shields.io/badge/Main%20MCU-ESP32--P4-E7352C?style=flat-square)
![Vision](https://img.shields.io/badge/Vision-ESP32--S3-E7352C?style=flat-square)
![Language](https://img.shields.io/badge/Firmware-C99-555?style=flat-square)
![Status](https://img.shields.io/badge/Validated-in%20simulation-orange?style=flat-square)

<img src="docs/assets/round.svg" alt="Animation of a simulated round: lap 1 maps the field at 34 cm/s, laps 2 and 3 follow the optimised line up to 150 cm/s, then the car parks" width="900">

<sub>One full round simulated with the same C firmware that runs on the robot. The red line is the playhead on the speed graph.</sub>

</div>

The software is finished and tested in simulation. The full mission, the parking manoeuvre and the motor control give repeatable results: 24 of 24 rounds with 62 points and no contacts. The physical car is another story. The v2 chassis exists in CAD and has not been printed yet, so the car has not driven. Throughout this document, every figure states where it comes from: measured, CAD or assumed.

---

## Contents

1. [In one minute](#1-in-one-minute)
2. [The challenge](#2-the-challenge)
3. [The team](#3-the-team)
4. [The vehicle](#4-the-vehicle)
5. [Mechanical design](#5-mechanical-design)
6. [Electronics](#6-electronics)
7. [Software](#7-software)
8. [The simulator](#8-the-simulator)
9. [Results](#9-results)
10. [Problems and solutions](#10-problems-and-solutions)
11. [Repository guide](#11-repository-guide)
12. [Build and run](#12-build-and-run)
13. [Rule compliance](#13-rule-compliance)
14. [What comes next](#14-what-comes-next)
15. [Credits](#15-credits)

---

## 1. In one minute

The track is drawn anew every round, so a fixed route is useless. The car works in three phases:

| Phase | What it does | Why |
|---|---|---|
| **1 · Map** (lap 1, 34 cm/s) | Drives the field with LiDAR and camera and builds a map: walls, every pillar with its colour and the parking bay. | The pillars do not move during the round. One lap is enough to learn the whole track. |
| **2 · Optimise** (laps 2 and 3, up to 150 cm/s in simulation) | The Intelligent Trajectory computes the fastest line between the pillars and a speed for every centimetre. A 50 Hz loop with encoders and IMU follows it. | It is the racing-car idea: brake only where a pillar or a wall leaves no margin, keep speed everywhere else. |
| **3 · Park** (last lap) | The Intelligent Trajectory hands over to the parking planner, which puts the car in the bay with a two-cusp parallel manoeuvre. | Parking needs slow, precise Ackermann manoeuvres, not racing. |

If something goes wrong (it loses its pose, the path is blocked, an unexpected obstacle appears), the reactive controller takes over. The Intelligent Trajectory never has the last word.

| | |
|---|---|
| Team | AutoNex, within the multidisciplinary team RDX |
| Institution | Instituto Tecnológico de Las Américas (ITLA), Boca Chica, Santo Domingo, Dominican Republic |
| Category | WRO 2026 · Future Engineers · Self-Driving Cars |
| Vehicle | Ackermann steering with a servo, rear-wheel drive, mechanical differential, two ESP32 |
| Status | Software validated in simulation, hardware under construction |
| Repository | [github.com/Eddym06/AutoNex](https://github.com/Eddym06/AutoNex) |

---

## 2. The challenge

In the Obstacle Challenge the car drives three laps of a track drawn anew every round. It has to pass red signs on the right and green signs on the left, and finish parked inside the bay, complete and aligned. No radio during the round, Ackermann steering (no turning by braking one wheel) and within 300 × 200 × 300 mm and 1.5 kg.

Because the track changes, recording a trajectory is useless. The car has to read the field, store it and then drive it fast. That splits the work into two different problems: understanding with imperfect sensors, while moving and against the clock, and optimising on top of that understanding, knowing the car model has error.

We decided to simulate before soldering. The track changed every round and the chassis was not ready, so we built a simulator that runs the same C firmware compiled to WebAssembly, with LiDAR, synthetic vision, IMU, noise and latency. There we closed the mission logic and found errors that would have cost us an afternoon each in the workshop.

That has a limit worth stating: until the car drives, everything is simulation. Grip, weight distribution, servo delay and motor speed are values we set by hand, not measurements.

### 2.1 The surprise rule

For 2026 the WRO added an optional extra task to the Obstacle Challenge. Before each official attempt the team says whether it wants to try it, and the answer cannot be changed once the attempt starts.

If the team takes part, the judges pick one sign, never the first one, and replace it with the same sign in the opposite colour: red becomes green and green becomes red. They also move it to a random height of 50 mm or 150 mm instead of the standard 100 mm. The 50 × 50 mm base and the position on the field stay where they were. The judges write down the original colour and the modified height, and the same sign and height are used for every team in that round.

The car then has to ignore the colour it sees. It must pass on the side the original colour asked for: if the sign used to be red, it goes to the right even though the sign now looks green.

Each successful pass is worth 5 points, up to three passes, so 15 points. The Obstacle Challenge maximum rises from 62 to 77 points and the total for the competition to 137. Teams that do not take part keep the standard field, rules and scoring.

This is why we added a VL53L0X time-of-flight sensor. The LiDAR was set up for the standard 100 mm obstacles and scans in one plane, so a sign at 50 mm passes underneath and never shows up. The ToF watches that low band and tells the car the sign is there. The height of 150 mm is not a problem for the LiDAR, but the 50 mm one is, and both are part of the same rule.

---

## 3. The team

We are three ITLA students and each one owns a subsystem:

| Role | Name | Responsibility |
|---|---|---|
| Software Engineer | [Eddy Manuel Piantini Martinez](https://github.com/Eddym06) | Perception, localisation, planning, control, simulator and testing |
| Mechanical and Design Engineer | Sebastian Josue Diaz Encarnacion | Chassis, Ackermann steering, mounts, CAD and mass |
| Electronics Engineer | Yeremy Manuel Bautista Portes | Power, wiring, drivers, sensors and bring-up |

Each area has an owner, but some decisions do not fit in one area. The maximum steering angle is set by mechanics and limits the line the planner may request. Servo current is measured by electronics, which decides whether the 5.1 V bus holds or a separate branch is needed. When one of those conversations changed something, it was noted in the logbook.

The institution is the Instituto Tecnológico de Las Américas, in Boca Chica, Santo Domingo.

The team came out of an internal ITLA competition. The team Sebastián and Yeremy were on finished first and Eddy's finished second. From the two, the three best were chosen: Sebastián and Yeremy from the first and Eddy from the second. AutoNex is the name we compete under and RDX is the multidisciplinary team we came from.

Rafael Vargas joins us as coach. His support is more logistical than technical.

This is our first WRO.

---

## 4. The vehicle

### 4.1 Specifications

| Figure | Value | Source |
|---|---|---|
| Configuration | Ackermann steering with a servo, rear-wheel drive, mechanical differential | CAD |
| Overall length | 260 mm | Measured |
| Overall width | 130 mm | Measured with the car assembled; the chassis alone is 120 mm |
| Overall height | ~80 mm | Measured |
| Wheelbase | 163 mm | Measured; the firmware still uses 184.5 mm |
| Track width | Pending | Not measured yet |
| Ground clearance | 34 mm | Measured |
| Wheel | Ø65 × 25 mm | Bought; CAD models it at 69 × 30 mm and the firmware uses 66 mm |
| Mass | Pending; target ≤ 1.2 kg, limit 1.5 kg | Not weighed yet |
| Centre of gravity | Pending; the software assumes 40 mm height and 62 % of the weight at the rear | Assumed |
| Chassis | 260 × 120 × 45.65 mm, PETG printed in 3D, 4 mm base | CAD |
| Brains | ESP32-P4 for planning and control, ESP32-S3-CAM for vision | Datasheet |
| Sensors | 360° LiDAR D500/D6, MPU-6050 IMU, axle encoder, OV3660 or OV2640 camera, VL53L0X ToF | Datasheet |
| Actuators | SCS0009 bus servo, JGB37-520 motor at 800 rpm with encoder, DRV8871 H-bridge | Datasheet |
| Power | 3S LiPo 11.1 V 2200 mAh 50C, XL4015 buck converter to 5.1 V | Datasheet |

Three figures are missing that can only be taken with the car assembled: mass, centre of gravity and track width. We will fill them in when the chassis is printed. Until they exist, any track number would be a guess.

### 4.2 The six views

<div align="center">
<table>
<tr>
<td align="center"><img src="v-photos/front.jpg" width="300"><br><sub><b>Front</b></sub></td>
<td align="center"><img src="v-photos/back.jpg" width="300"><br><sub><b>Back</b></sub></td>
</tr>
<tr>
<td align="center"><img src="v-photos/left.jpg" width="300"><br><sub><b>Left side</b></sub></td>
<td align="center"><img src="v-photos/right.jpg" width="300"><br><sub><b>Right side</b></sub></td>
</tr>
<tr>
<td align="center"><img src="v-photos/top.jpg" width="300"><br><sub><b>Top view</b></sub></td>
<td align="center"><img src="v-photos/bottom.jpg" width="300"><br><sub><b>Bottom view</b></sub></td>
</tr>
</table>
</div>

These images are renders of the complete CAD assembly, not photographs of the assembled car. They are here to show the geometry and the packaging. As soon as the chassis is assembled we will replace them with real photographs, keeping the same file names.

<div align="center">
<img src="docs/assets/vehicle/dimetric.jpg" alt="Isometric view of the v2 CAD assembly" width="700">
<br><sub>Isometric view of the v2 assembly. It is not one of the six required views, which is why it lives in <code>docs/assets/vehicle/</code>.</sub>
</div>

### 4.3 What we bought and what we made

| Bought | Made in-house |
|---|---|
| LiDAR, ESP32-P4, ESP32-S3-CAM, IMU, camera, FPC adapter and ribbon, and VL53L0X ToF | Complete chassis and its covers |
| Motor with encoder, servo, servo adapter, H-bridge | Gears, ring gear and differential housing |
| Battery, buck converter, fuse holders and fuse | Mounts for servo, motor, LiDAR, IMU and battery |
| Ø65 × 25 mm wheels | Steering arms, tie rod and wheel holders |
| Threaded inserts, M3 fasteners, PETG filament | LiDAR plate and wheel retainers |

### 4.4 Bill of materials

The main components we bought, with the price we paid. All figures in US dollars, purchase price, without shipping or taxes. Each row is one purchase, not one piece: the packs are noted in the specification.

| # | Component | Qty | What it does | Model and specification | Price |
|---|---|---:|---|---|---:|
| 1 | <img src="docs/assets/bom/lidar.png" width="90"><br>360° LiDAR | 1 | Maps walls, pillars and the parking bay | D500/D6, time of flight, 12 m, 10 Hz, UART at 230400 baud | $75.00 |
| 2 | <img src="docs/assets/bom/camera.png" width="90"><br>Vision board | 1 | Detects the pillars and their colour | ESP32-S3-CAM by FORIOT, OV3660 or OV2640, PSRAM | $18.00 |
| 3 | <img src="docs/assets/bom/fpc-adapter.png" width="90"><br>Camera extension adapter | 1 | Joins the camera flex to the long ribbon | uxcell 24-pin to 24-pin FPC adapter board, one ZIF socket on each side | $7.79 |
| 4 | <img src="docs/assets/bom/fpc-cable.png" width="90"><br>Camera ribbon | 1 | Carries the camera signal to the nose | FFC 24-pin, 0.5 mm, 200 mm, pack of 20 | $6.69 |
| 5 | <img src="docs/assets/bom/imu.png" width="90"><br>IMU | 1 | Heading and turn rate | MPU-6050 on a GY-521 board, I²C at 0x68, pack of 3 | $10.00 |
| 6 | <img src="docs/assets/bom/tof.png" width="90"><br>Time-of-flight sensor | 1 | Sees the 50 mm signs the LiDAR misses | VL53L0X, I²C, up to 2 m, pack of 2 | $6.99 |
| 7 | <img src="docs/assets/bom/p4.png" width="90"><br>Main board | 1 | Mission, localisation, planning and control | ESP32-P4 DevKit by Waveshare, 32 MB PSRAM, two cores | $27.00 |
| 8 | <img src="docs/assets/bom/servo-adapter.png" width="90"><br>Servo adapter | 1 | Talks half-duplex to the servo | Waveshare Bus Servo Adapter (A), SKU 25514 | $13.00 |
| 9 | <img src="docs/assets/bom/motor.png" width="90"><br>Traction motor | 1 | Rear drive | JGB37-520, 12 V, 800 rpm, Hall encoder, 440 counts per revolution | $13.00 |
| 10 | <img src="docs/assets/bom/servo.png" width="90"><br>Steering servo | 1 | Turns the front wheels | SCS0009 bus servo, metal gears, real position feedback | $7.00 |
| 11 | <img src="docs/assets/bom/drv8871.png" width="90"><br>H-bridge | 1 | Forward and reverse | DRV8871, 3.6 A, PWM and direction on two pins | $9.00 |
| 12 | <img src="docs/assets/bom/xl4015.png" width="90"><br>Buck converter | 1 | 11.1 V down to 5.1 V | XL4015, 5 A, non-isolated | $11.00 |
| 13 | <img src="docs/assets/bom/battery.png" width="90"><br>Battery | 1 | Single power source | HOOVO 3S LiPo, 11.1 V, 2200 mAh, 50C, EC3, pack of 2 | $37.00 |

The full budget by category:

| Category | Total |
|---|---:|
| Perception (LiDAR, camera, extension, IMU, ToF) | $124.47 |
| Control (P4, servo adapter) | $40.00 |
| Actuation (motor, servo, H-bridge) | $29.00 |
| Power (battery, converter, fuse) | $53.00 |
| Structure and manufacturing (filament, wheels, inserts, fasteners) | $43.00 |
| **Total** | **$289.47** |

The wheels are bought, but we do not have their exact price yet, so they are not in the total. The full project budget, with shipping, registration and spares, is still open.

The working file with the links and the per-category formulas is `BOOM.xlsx` in `Documentos_Referencia/03_Presupuesto/`.

The 13 November logbook mentions the L298N because that was the planned bridge at the time. The design later moved to the DRV8871, which is what the car and the budget use.

---

## 5. Mechanical design

### 5.1 From v1 to v2

The first version did not fail on paper. It failed on the track: it slipped and skidded at the rear. The axle had no bearings and the steering travel was short.

| Aspect | v1 (16 Oct) | v2 (12 Nov) |
|---|---|---|
| Motor | Yellow dual-shaft DC motor, no encoder | JGB37-520 12 V, 800 rpm, with encoder |
| Drivetrain | No differential; slipped | Differential and rear bearings |
| Servo angle | 70° programmed | 62° programmed |
| Chassis (CAD) | 300 × 38 × 150 mm | 260 × 120 × 45.65 mm |
| Mass | Not weighed | Not weighed |
| Tested? | Yes, rear-slip failure | No |

Between v1 and v2 there was an intermediate step: the JGB37-520 with a 1:1 two-gear transmission. The axle took the same force and the slip stayed the same. That is what pushed us to the differential and the bearings.

There is no numerical comparison between v1 and v2 because we did not measure weight, stiffness, torsion, slip or wheel angles. The improvement is in the design, not in measured results.

### 5.2 Chassis and manufacturing

The v2 chassis is a printed structure of 260 × 120 × 45.65 mm, with a 4 mm base and 3 mm walls in the motor cradle. There is no folded sheet, laser cutting or drilling: the only processes are printing and assembly.

- Base: battery, electronics and LiDAR plate.
- Rear: motor cradle, differential housing, axle and bearings.
- Front: steering servo and wheel holders.
- Top deck: LiDAR on its 60 × 62 × 2 mm plate and the IMU flat at 60 mm.

The material is PETG printed in FDM. We chose it for two practical reasons: it handles the heat from the motor inside the cradle better and it is less brittle when the car takes a hit. The downside is that it absorbs moisture, so the filament has to be dried before printing, and it leaves more strings and burrs on the parts.

The print parameters (nozzle, layer height, perimeters, infill, orientation) are not documented. We have to write them down with the final set so the design can be reproduced.

### 5.3 Drivetrain

```
JGB37-520 motor → 6 mm shaft → gear3 pinion (Ø26.8) → gear_4 ring (Ø45.4)
→ differential (Base_of_gear, gear1/gear2) → Axis_of_wheel → wheels
```

The ratio calculated from the outer diameters is 1.7:1. The SolidWorks files do not store tooth count, module or pressure angle, so the real ratio is still to be confirmed.

The rear axle is Ø9.8 mm and fits 10 × 15 × 4 mm bearings. The differential is mechanical, with bevel gears, not two independent motors.

Watch out for this: the firmware assumes a solid rear axle and a single encoder. With a differential the two wheels can turn at different speeds. The odometry and the Intelligent Trajectory loop have to be validated on the real car.

### 5.4 Ackermann steering

The chain is: servo, servo arm, steering bar, tie rod and knuckles. The bar and the tie rod share a 50 mm hole spacing.

The servo arm has five holes at 2.5 mm. The closest ones give less travel and more force; the farthest ones, the opposite. The programmed angle is 62° in v2 and was 70° in v1. Be careful with the difference: 62° is the servo shaft rotation, not the wheel angle.

| Figure | Value | Status |
|---|---|---|
| Programmed servo angle | 62° in v2 | Design |
| Maximum wheel angle | Not measured; the firmware uses 40° | Pending |
| Turning radius | Not measured | Pending |
| Toe | Not measured | Pending |
| Bar and tie rod spacing | 50 mm | CAD |

True Ackermann geometry requires the inner wheel to turn more than the outer one. The CAD does it with a common bar and tie rod, but without measuring the left and right angles at full travel we cannot claim it complies. It is one of the first measurements of the setup.

### 5.5 Mass and centre of gravity

We have not weighed the car or measured its centre of gravity. The v1 CAD reports 377.7 g of parts and 705.6 g of assembly, but with no material assigned and with impossible densities: the wheel comes out at 7.8 g/cm³ and the chassis at 1.0 g/cm³. It is not a usable estimate.

The software works with a target of 1.2 kg or less, a centre of gravity 40 mm high and 62 % of the weight at the rear. All three are assumptions. The rule limit is 1.5 kg.

To measure it we will use a scale for the total mass and two crossed weighings to split the weight between axles. With the height of each axle known, the centre of gravity follows. That figure goes straight into the planner and the motor control.

### 5.6 Sensor and electronics mounts

| Element | Mounting |
|---|---|
| LiDAR | On a 60 × 62 × 2 mm plate, 60 mm high, flat |
| IMU | Flat at 60 mm, next to the LiDAR |
| Camera | At the nose, separated from the S3 board by a 24-pin FPC adapter and a 200 mm ribbon; height and tilt pending |
| Battery | Modelled in CAD as 103 × 34 × 24 mm |
| Electronics | At the rear, next to the motor |
| Servo | Two M2 screws in the ears; the arm sits on the spline with a centre screw |
| Motor | Bolted to a flat base; Ø37 mm cylindrical cradle with 3 mm walls |

### 5.7 CAD files

`CAD_File/` holds 30 SolidWorks files and weighs 47.7 MB. It is split into four folders:

| Folder | Contents |
|---|---|
| `Main part` | `Car assembly.SLDASM`, chassis, wheels, servo and steering |
| `Diferencial` | Gears, axles, base and differential assembly |
| `Dc motor` | Motor model and shaft |
| `Electronic part` | Battery, ESP32-P4, LiDAR and a stray MG90S |

The main assembly is `Car assembly.SLDASM`, inside `Main part`. It holds the chassis, the `Wheel D65x25` wheels, the `Feetech SCS0009 v9` servo and the whole front end.

A `Servo-MG90S.SLDPRT` was left in `Electronic part` and no longer matches the car. The servo we use is the SCS0009, and its assembly is in `Main part`.

### 5.8 Open mechanical issues

1. Front or rear bearings. The v2 manual puts them in the front knuckles and Sebastián corrects that they go in the rear drivetrain. The CAD still shows front seats, so this has to be settled before manufacturing.
2. Wheelbase. The firmware uses 184.5 mm and the measured value is 163 mm. That is 21.5 mm, between 12 and 13 % of the wheelbase, and it affects every turn, the parking and the simulated line. It is the first value to correct.
3. Wheel diameter. The bought wheel is 65 mm, the CAD models it at 69 and the firmware uses 66. A 6.15 % difference in circumference goes straight into the odometry.
4. Steering angle. The servo programs 62° and the firmware limits the wheel to 40°, with no measured mapping between the two.
5. Computer model. The file is called `Esp32p4`, but the model inside is a Raspberry Pi 4B of 85 × 56 mm. It has to be renamed or replaced.

---

## 6. Electronics

### 6.1 Blocks

```mermaid
flowchart LR
    BAT["3S LiPo<br/>11.1 V 2200 mAh"] --> FUSE["5 A fuse<br/>+ switch"]
    FUSE --> H["DRV8871<br/>H-bridge"]
    FUSE --> BUCK["XL4015<br/>5.1 V / 5 A"]
    BUCK --> P4["ESP32-P4"]
    BUCK --> S3["ESP32-S3-CAM"]
    BUCK --> LID["360° LiDAR"]
    BUCK --> IMU["MPU-6050"]
    BUCK --> TOF["VL53L0X ToF"]
    BUCK --> ADP["Servo adapter<br/>Waveshare (A)"]
    ADP --> SRV["SCS0009 servo"]
    H --> MOT["JGB37-520 motor"]
    MOT --> ENC["Encoder 440 counts/rev"]
    ENC --> P4
    LID --> P4
    IMU --> P4
    TOF --> P4
    S3 <-->|"serial link 921600"| P4
```

### 6.2 Power

The battery enters through the XT60H (EC3 with an adapter lead), goes through a 5 A fuse and the main switch. From there two branches leave: one straight to the DRV8871 at 12.6 V and one to the XL4015 at 5.1 V.

The 5.1 V bus feeds both boards, the LiDAR, the IMU, the bridge logic and the servo adapter. The adapter does not regulate: it delivers whatever enters DC+, and 5.1 V is within the 4 to 7.4 V range of the SCS0009. It carries a 470 µF capacitor between DC+ and DC− to absorb the servo peak.

| 5.1 V bus load | Typical | Peak |
|---|---:|---:|
| ESP32-P4 | 350 mA | 550 mA |
| ESP32-S3-CAM | 180 mA | 300 mA |
| LiDAR | 290 mA | 320 mA |
| Servo | 150 mA | 1000 mA |
| Adapter, bridge logic and IMU | 49 mA | 64 mA |
| VL53L0X ToF | 20 mA | 40 mA |
| **Total** | **≈ 1.0 A** | **≈ 2.2 A** |

The XL4015 is rated at 5 A, but without a heatsink it should not go above 2.5 A. Including the motor, the battery delivers at most about 3.5 A peak, well below what a 2200 mAh 50C pack can supply.

All grounds go to a single point. A floating GND between motor and signal turns into garbage data.

### 6.3 Motor and encoder

The motor uses a single DRV8871 channel. Commands go through IN1 and IN2 with PWM on the inputs, and the bridge delivers up to 3.6 A.

| IN1 | IN2 | Action |
|---|---|---|
| PWM | 0 | Forward |
| 0 | PWM | Reverse |
| 0 | 0 | Free |
| 1 | 1 | Hard brake; not used to stop |

The six-pin encoder connector is silkscreened M1, GND, C1, C2, VCC, M2. The vendor schematic does not match, so we checked it with a multimeter: M1 and M2 are the only pair with low resistance. The channels use 4.7 kΩ pull-ups to 3.3 V.

At 440 counts per revolution, each count is 0.047 cm of travel. That is the resolution the odometry uses to estimate the pose between LiDAR scans.

### 6.4 Steering servo

The SCS0009 speaks over a single half-duplex wire. The Waveshare adapter separates TX and RX, keeps the response at 3.3 V and distributes power. Both jumpers go in row A (UART). TX goes to GPIO26 and RX to GPIO21, without crossing them: RX to RX and TX to TX.

The adapter does not regulate the servo voltage. If the PWR LED does not light, the fix is to power only the adapter from a 6 V branch, not to raise the whole bus.

### 6.5 P4 pin map

| Function | GPIO | Note |
|---|---|---|
| Motor IN1 / IN2 | 32 / 33 | 1 kHz PWM |
| Servo TX / RX | 26 / 21 | To the adapter, 3.3 V |
| Encoder A / B | 27 / 23 | 4.7 kΩ pull-up |
| LiDAR RX | 22 | 230400 baud |
| S3 link RX / TX | 38 / 37 | From GPIO1 and to S3 GPIO14 |
| IMU SDA / SCL | 7 / 8 | I²C, address 0x68 |
| ToF SDA / SCL | Pending | I²C, address 0x29, same bus as the IMU |
| START button | 5 | To GND, internal pull-up |
| Battery (future) | 20 | ADC, divider pending |

Pins 2, 3, 4, 6, 46, 47 and 48 are free. Do not use 24 and 25 (USB OTG/JTAG), 0, 34, 35 and 36 (boot), or the audio, microSD, Ethernet and LED pins of the board.

### 6.6 Camera and FPC extension

The S3-CAM does not sit where the camera sits. The v2 CAD reserves the nose for the module and leaves the board at the rear, with the rest of the electronics. We separate them with two pieces: a 24-pin to 24-pin FPC adapter board, with a ZIF socket on each side, and a 24-pin, 0.5 mm, 200 mm FFC ribbon. The camera's own flex goes into one socket and the ribbon goes from the other socket to the S3-CAM, so the module stays at the front without moving the connector on the board.

The adapter joins the contacts one to one and does not change their order or their side. The ribbon is the same type as the original flex: 24 contacts at 0.5 mm pitch. A Raspberry Pi Zero Camera flex does not work, because it has 22 pins and uses MIPI CSI-2, a different signal and connector.

The length is short on purpose. The camera runs over DVP, eight parallel bits, and at 20 MHz a long flex behaves like an antenna: shifted pixels and broken frames appear. 200 mm should be enough. If errors appear, the first adjustment is to lower the camera clock, not to lengthen the cable.

### 6.7 Bring-up

1. Without the battery, check that 12 V and 5 V have no continuity to GND. Power the XL4015 from a 12 V lab supply and set it to 5.1 V with no load. Repeat with a 10 Ω resistor and measure again at the board pin.
2. Power the board alone and confirm it boots. Connect the LiDAR and check that it spins and delivers data. Connect the IMU and check address 0x68.
3. Power the S3-CAM and check the 921600 link without broken frames. Turn the motor shaft by hand and confirm the encoder counts in both directions.
4. Before the servo: jumpers in A, 5.1 V between DC+ and DC−, 5.1 V between V and G and the PWR LED on. Only then connect the servo and move it with the wheels off the ground.
5. Finally the motor, off the ground and at 10 % PWM.

If something fails, we disconnect the last thing added and step back. That way the error shows up where it is, not somewhere among the twenty cables of the car.

### 6.8 Electronics pending

- Measure the real motor current with the complete car. The 2.5 A stall figure is from the datasheet.
- Test the servo at 1 Mbit/s with the adapter and the real cable, with the servo mounted.
- Measure the adapter RX with an oscilloscope (0 to 3.3 V) before connecting GPIO21.
- Measure servo torque at 5.1 V with the steering assembled. If it is not enough, use a 6 V branch.
- Confirm that the adapter's internal regulator starts at 5.1 V (PWR LED).
- Install the voltage divider on GPIO20 to monitor the battery.
- Test the camera extension, adapter board and 200 mm ribbon, with the car assembled and set the camera height and tilt.
- Confirm whether the S3 camera is OV3660 or OV2640 and run a continuous three-minute test.

---

## 7. Software

### 7.1 Two brains

The ESP32-P4 carries the mission: local map, localisation, planning, parking and control. The ESP32-S3-CAM only looks. It processes the image, detects the pillars and their colour, and sends the objects over a serial link at 921600 baud. That split leaves the P4 free to decide at 10 Hz without a slow image stealing its time.

### 7.2 The control loop

The P4 loop is cooperative and takes about 1 ms. Every decision happens once per LiDAR scan, about 10 times per second: the local map is rebuilt, the pose is updated, the command is chosen and the actuators are written. Between scans, the Intelligent Trajectory fast loop propagates the pose at 50 Hz with encoders and gyroscope. The IMU is sampled at 400 Hz and integrated at 100 Hz. A 300 ms watchdog sets the command to zero if scans or gyroscope data stop arriving.

The full flow:

```mermaid
flowchart LR
    subgraph S["Sensors"]
        L["LiDAR<br/>10 Hz"]
        C["Camera (ESP32-S3)<br/>red / green pillars"]
        I["IMU gyroscope"]
        E["Encoder"]
    end
    L --> LOC["Field localisation<br/>(wall fitting)"]
    I --> LOC
    E --> LOC
    L --> MAP["trace_map<br/>walls · pillars + colour · bay"]
    C --> MAP
    LOC --> MAP
    MAP --> IT["Intelligent Trajectory<br/>line + speed profile"]
    LOC --> IT
    IT --> FAST["Fast loop 50 Hz<br/>encoders + IMU"]
    MAP --> PARK["Parking planner"]
    LOC --> REACT["Reactive controller<br/>(safety net)"]
    FAST --> MIC["MIC<br/>motor control"]
    REACT --> MIC
    PARK --> MIC
    MIC --> M["Rear motor"]
    FAST --> SV["SCS0009 steering servo"]
```

### 7.3 Phase machine

| Phase | What it does |
|---|---|
| `IDLE` | Waits for the physical button |
| `RUN` | Three laps: reactive on the first, Intelligent Trajectory on the next ones |
| `PARK_SEARCH` | Looks for the bay against the outer wall |
| `PARKING` | Enters the bay with two direction changes |
| `PARK_EXIT` | Leaves the bay if the car starts inside |
| `DONE` | Stops the car once the goal is complete |

Safety is the same in every phase. If the LiDAR or the gyroscope fails, the speed command goes to zero. Starting is physical, with the button on GPIO5. The web page can stop the car but cannot start it, which is how we meet the no-radio rule during the round.

### 7.4 Perception, local map and localisation

The local map is a 160 × 160 grid of 2.5 cm cells, or 4 × 4 m, rebuilt on every scan with Bresenham raycasting. The camera contributes up to 12 objects with colour, centre and depth. If the LiDAR does not confirm the return, they expire.

`arena_localizer` fits the pose against 8 wall segments with Gauss-Newton and Huber weights, and rejects jumps larger than 30 cm or 8°. `park_localize` uses the bay wall, its opening and the stops.

The odometry combines the wheel angle with the gyroscope heading, detects slip and resets after a sample gap longer than 0.5 s. The controller advances the pose by the LiDAR latency, which is 0.10 s.

In simulation the mean error is 2.3 cm and the maximum is 3.5 cm.

There is a limitation that comes from the hardware. Returns at 0 mm or beyond 12 m are discarded and blind mode is not enabled. An obstacle against the nose simply does not appear. That branch exists only in the simulator.

### 7.5 The persistent map

<details>
<summary>How it is built and what it stores</summary>

It is a 4 × 4 m grid with 2.5 cm cells in field coordinates. Each lap adds LiDAR returns with a weight that makes repeated passes average instead of thickening. The objects, pillars and parking stops, have a stable id, position and colour. The bay is detected by geometry.

It is written to a flash partition at the end of the round, not during it, because writing to flash stops the CPU. The save code is written but not tested on the P4, and map accuracy at the 34 cm/s mapping speed is not measured. In simulation the RMS error is 0.78 cm.

The technical detail is in [`code/simulator/TRACE_MAP.md`](code/simulator/TRACE_MAP.md).

<div align="center"><img src="docs/assets/map.png" alt="Map built after lap 1 next to the simulator ground truth" width="780"></div>

</details>

### 7.6 Intelligent Trajectory

<details>
<summary>What makes it intelligent</summary>

- It discovers the line instead of being given one. It minimises real lap time over the free space the pillars leave, respecting red to the right and green to the left.
- It knows the car: rear weight, friction circle, 40° steering limit, servo speed and the 26 × 13 cm chassis footprint. The nose sticks out 22 cm ahead of the rear axle.
- Margins grow with speed. Pose and tracking error rise as the car accelerates, so narrow gaps are crossed slowly and wide ones quickly.
- It learns grip lap after lap. It starts cautious and only raises the grip it assumes while the car runs clean.
- Encoders and IMU close the loop. Between LiDAR scans, which at 150 cm/s are 15 cm, the pose is propagated at 50 Hz. Without that the lateral error doubles.

A plan costs about 0.4 s on a PC. We have not measured the time on the P4, so the plan is sliced into chunks of about 10 ms to avoid blocking the loop.

One warning: the simulator models an MG90S servo, but the car's servo is an SCS0009. Before trusting the line the Intelligent Trajectory computes, we have to measure the real servo's speed, delay and torque.

</details>

### 7.7 MIC, motor control

MIC (Motor Intelligent Control) is the central traction control. It uses a physical model of the JGB37-520 with back-EMF, friction, battery, bridge drop and current limit. A third-order observer predicts between encoder counts and corrects with the model, and a look-ahead planner decides when to brake.

It has two braking modes: progressive, without reverse voltage, and aggressive, with reverse voltage. The aggressive mode was limited by the 2 A of the L298N documented at the time. The DRV8871 allows 3.6 A, so that limit has to be recalibrated.

| Test with a simulated motor | Result |
|---|---|
| 300 cm route with a curve and parking | 300.8 cm in 8.8 s, against about 11.4 s with conservative ramps; 0 s of reverse voltage; 2.0 A peak |
| Same route with R +15 %, mass +30 %, friction +40 % and low battery | Equally accurate, 0 s of active braking |
| Stop from 100 cm/s | Progressive: 38.6 cm in 1.12 s. Aggressive: 25.4 cm in 0.52 s |

MIC has only been tested against a simulated motor. It is integrated in `hw_motors.c` behind `CONFIG_WRO_USE_MIC` and disabled. The parameters it uses are assumed: resistance 2.6 Ω, mass 1.2 kg, dry friction 0.6 N and viscous friction 0.5 N·s/m.

### 7.8 Parking

The bay is 1.5 times the length of the car. The car enters with a two-cusp parallel manoeuvre: forward, S in reverse and a final forward to engage, always within the steering limit.

<div align="center">
<img src="docs/assets/parking.png" alt="Two-cusp parallel parking trajectory with steering angle, speed and clearance telemetry" width="900">
</div>

- Planner: A* over a pose grid of 121 × 71 × 36 (1 cm, 10°), 3.5 cm primitives and steering in {−1, 0, +1}.
- Cost: 3.5 cm per step, 1.1× in reverse, plus 12 cm for each direction change.
- Entry goal: all four corners inside the bay with 0.5 cm clearance and |θ| ≤ 4°. The exit is accepted with |θ| ≤ 10°.
- Normal entry: symmetric to the exit, crossing the lane up to 2 cm ahead and 1.5 cm farther from the wall than the pose where the exit ended.
- Candidate plans: five, if the arrival pose has no plan. If the measurement overlaps the bay, the nearest free pose is used, up to 2 cm and 6°.
- Arcs: cut by gyroscope heading, and steering is applied with the car stopped.

The simulation result is 2 to 5 passes, about 4 on average, in 16 to 17 s. On seeds 1 to 8 there were no failures. On seeds 9 to 32, 5 of 56 failed, 9 %, where it used to fail 17 %.

### 7.9 Reactive control and safety

<details>
<summary>The net that never stops working</summary>

The reactive planner evaluates 343 paths of three 20 cm arcs and scores them by clearance, lateral deviation, heading, curvature, advance and step. It uses three hysteresis layers: a 2-point tie band, a preference for the previous action and, on a tie, the smoothest turn.

If there is no path, it performs an escape manoeuvre and, if that is exhausted, stops. Pivot in place is used after 2 s without a path and a heading deviation of 15° or more, if there is room.

The 300 ms watchdog sets the command to zero if LiDAR or gyroscope data is missing. It does not detect a disconnected encoder by itself, and that is still pending. Starting is physical; the web page can only stop the car.

</details>

### 7.10 Discarded decisions

Documenting what did not work is also part of the method. These ideas were tested and left out:

| Idea | Why it was discarded |
|---|---|
| Historical racing line (`race_line`) | With 7 signs it saved no time and reduced reliability |
| Looking at uncoloured pillars (`peek`) | Dropped perfect tracks from 95 to 89 on the bench |
| Clearance braking (`tight_clear`) | Cost about 18 s and broke parking |
| Three-segment manoeuvre | The steering pattern was inverted and, once fixed, found no solution |
| Two-arc S from the lane | The rear corner caught the stop at 6.5 cm |

---

## 8. The simulator

### 8.1 What it is

The simulator is not a separate model. It loads `wro_core.wasm`, which is the same C code of the P4 and the S3 compiled to WebAssembly. What is validated there is the real firmware, not a copy in JavaScript.

The world advances with dt of 0.01 s, 8 ms of process delay and 12 ms of command delay. The car is a bicycle model with a first-order servo at 420°/s and τ of 0.045 s, a motor with τ of 0.16 s, maximum acceleration of 180 cm/s², gain 0.94, saturation slip and sliding collision.

### 8.2 What it models and what it does not

| It models | It does not model, or does it differently |
|---|---|
| LiDAR with noise and latency | The real LiDAR does not emit blind mode or points closer than 5 cm |
| Synthetic vision with colours | The real camera can fail because of light or the ribbon |
| IMU and encoder with noise | The real car has a single encoder; the simulator averages two wheels |
| First-order MG90S servo | The real servo is an SCS0009 |
| JGB37-520 motor with a physical model | Real actuators receive commands in 100 ms steps |
| Stops, walls and signs | The real servo has stops, not a ramp |
| Judge with the scoring rules | The hardware speed limit is 40 cm/s |

Some things can work in the simulator and fail in the car: the configurable servo against the servo with stops, one encoder against two, the LiDAR without close points, and the wait for 50 clean gyroscope samples before starting.

### 8.3 How to run it

```bash
python code/simulator/build_simulator.py          # builds code/simulator/build/simulador.html
# open it, tick "trayectoria inteligente (IT)" and press "Simular"
node code/simulator/tests/test_it_mission.js      # full round with IT: points, no contacts, handover
```

<div align="center">
<img src="docs/assets/simulator_it.png" alt="Simulator screenshot: Intelligent Trajectory in green, camera, local map and trace_map" width="900">
<br><sub>The simulator driving IT: planned line in green, real path in blue, camera view, local map and persistent map.</sub>
</div>

There are more tests: geometry and sensors, firmware and simulation contract, axle encoder, bay entry and exit, interface and full missions. They are all listed in [`code/simulator/README.md`](code/simulator/README.md).

One simulation step takes 1.5 ms in the browser. It holds at 4× and not at 8×.

We recorded a full round on 10 November: [youtu.be/XHigaxjfDUE](https://youtu.be/XHigaxjfDUE). It is simulation, not a track test.

---

## 9. Results

### 9.1 Simulation

<div align="center">
<img src="docs/assets/lap_times.png" alt="Lap and round times: original reactive 145 s, reactive at 35 cm/s 110 s, with Intelligent Trajectory 88 s" width="820">
</div>

| Test | Result |
|---|---|
| Full mission on feasible tracks, 3 signs, 24 rounds | 24 of 24 with 62 points and 0 contacts, 118.9 s average |
| Parking entry, seeds 1 to 8 | 2 to 5 passes, about 4 on average, 16 to 17 s, 0 failures |
| Parking entry, seeds 9 to 32 | 5 failures out of 56, 9 %, where it used to be 17 % |
| Lap 1 mapping at 34 cm/s | 19.4 s against 29.6 s at the previous speed |
| Intelligent Trajectory on the full mission | 12 of 12 with 62 points, 11 of 12 before the last fix |
| Intelligent Trajectory with MIC | 7 full rounds, 62 points, 0 contacts, 151 cm/s top speed |
| 100-track bench, earlier stage | 99 of 100 in tuning and 95 of 100 in validation, 0 contacts |
| Localiser and map | 2.3 cm mean and 3.5 cm maximum, 0.78 cm RMS |
| Controller scan on the P4 | 84.5 ms, p95 92.5, down from 142.6 ms |
| Intelligent Trajectory plan on the P4 | 18 s down to 4.2 s with the incremental evaluation |
| Parking A* on the P4 | 11 to 27 s down to 5 to 11 s |
| Camera on the S3 | 71.4 fps stable for 120 s, 0 bad tables and no sequence gaps |
| 48 rounds with 5 signs, simulator using the measured P4 costs | 62 points and 0 contacts, 90.3 s per round, laps of 21.6 s and 10.8 s |

<div align="center">
<img src="docs/assets/trajectory.png" alt="The line discovered by the Intelligent Trajectory coloured by speed and the speed over a full round" width="900">
</div>

On feasible random tracks, the ones the interface draws, 24 of 24 rounds end with 62 points and no contacts. The cw direction takes a few seconds longer than ccw, because the exit ends with another heading and the cw parking plan comes out longer. The old "1 in 3 ends in wrong-side" came from the measurement tool drawing tracks no chassis can pass, such as a sign 45 cm from the bay exit. Map accuracy at the 34 cm/s mapping speed is still not measured. The planner takes about 0.4 s on a PC; on the P4 it took 18 s and is now 4.2 s, which is why the work is split into chunks and moved to the second core.

The 16 November measurements changed two things. First, the C on the P4 runs about 40 times slower than on a PC, so the 142 ms controller scan did not fit the 100 ms budget; it is now 84.5 ms. Second, the simulator now applies those real costs by default, so its round times stopped being optimistic: 90.3 s instead of 89.1 s, and the difference between an ideal run and a real one is small. Details in `MEDIDAS_HARDWARE_P4.md`.

### 9.2 Hardware

Three things have been measured on real hardware. The camera on the S3 runs at 71.4 fps for 120 s with no bad tables. The motor turns at 844 rpm with no load at 12.0 V, which is why the documentation uses 800 rpm. The P4 was timed with the real firmware: the controller scan, the Intelligent Trajectory plan and the parking A* all have real numbers now, and the memory and pin map were checked against the board.

The motors, encoders, LiDAR, IMU, servo and the flash map save are integrated in the firmware but have not been tested on the car. The power electronics is documented and being assembled, with final measurements pending. The VL53L0X is bought and its use is defined, but it is not wired or in the firmware yet.

The v1 chassis was tested and failed because of rear slip. The v2 is still untested, unweighed and without a measured centre of gravity or real angle.

### 9.3 Risks when moving to the real car

| Risk | Why |
|---|---|
| Servo | The simulator models an MG90S; the car carries an SCS0009 and its real values have to be measured |
| Grip | µ, weight distribution and centre-of-gravity height are assumptions. If real grip is 0.40 or less, the Intelligent Trajectory fails |
| Single encoder | The simulator averages two wheels; slip and effective diameter change the odometry |
| LiDAR | It does not emit blind mode or points closer than 5 cm; an obstacle against the nose does not appear |
| Actuators | The car receives discrete commands every 100 ms and the real servo has stops |
| Plan and flash | The plan time on the P4 and the map save are not measured |
| ToF | It is bought but not wired or in the firmware; the mounting height and the detection of the 50 mm sign are untested |

---

## 10. Problems and solutions

### 10.1 The 120-second stall

The hardest problem we had was not in mechanics or electronics. It was a sign in a comparison.

In full-mission tests the car would stay still for 120 seconds at the lap-3 corner. It did not crash, it did not lose the map and it did not reboot. It just stopped, with the planner convinced there was no way out. Reproducing it was easy. Understanding it was not.

The diagnosis was a mix of two things. The heading error at that corner reached 93°, and the code treated any error above 90° as if the target were behind. With that reading it requested a turn in the wrong direction. The car turned, measured 93° again and repeated. A perfect loop.

The fix was to narrow the case: between 90° and 135° of error, request 85° towards the short side instead of treating the target as if it were behind. With that, the manoeuvre leaves the stall in one turn and the full mission went to 24 of 24 without stalls.

What this case teaches is that the fault was not in perception or control, but in a sign convention that only broke in a narrow range of angles. In simulation we could reproduce it a hundred times. On the track it would have been a car stopped for no apparent reason and twenty minutes lost.

### 10.2 The other problems

| Problem | Diagnosis | Change | Result |
|---|---|---|---|
| Contact with the stop while parking | The arc check was stricter than the plan | Margin from 0.5 to 0.75 cm | 0 contacts on seeds 21 to 32 |
| "1 in 3 ends in wrong-side" | The tool drew impossible tracks | `measure_mission.js` uses feasible tracks by default | 22 of 24 with 62 points |
| Parking stayed stalled | The pose overlapped the bay without clearance | Nearest free pose, 2 cm and 6° | Unblocked |
| Intelligent Trajectory lost distance at 150 cm/s | `odometry.c` discarded samples above 150 cm/s | Limit raised to 300 cm/s | The localiser stops lagging |
| 10 cm lateral error at 150 cm/s | The follower ran at 10 Hz | Fast loop at 50 Hz | 4.7 cm lateral error |
| At 35 cm/s the team's track failed | `plano_corregido.json` ends in wrong-side | Mapping speed set to 34 cm/s | Clean at 31, 33 and 34 |
| The judge waited 15 s after parking | Inherited fixed delay | 3 s and stop when the car halts | Less dead time |

### 10.3 What we learned

1. A result counts when it repeats on new seeds and on two benches. Seeds 1 to 8 were used for tuning and became overfitted. Validation starts at 21.
2. Measurement tools fail too. The "1 in 3" was a draw error, not the car. Before blaming the firmware, doubt the instrument.
3. Every figure needs its configuration next to it. A time without the speed, the grip and the plan says nothing.
4. In electronics, test by blocks: connect the next thing only when the previous one is verified, and disconnect the last addition when something fails.
5. Discarded ideas are documented. They stop someone from trying them again and explain why the design is what it is.

---

## 11. Repository guide

### 11.1 Tree

```text
AutoNex/
├── code/
│   ├── LIDAR System/    # mission controller, planners, Intelligent Trajectory (main/it_plan.c), P4 firmware
│   ├── Vision System/   # camera processing on the ESP32-S3
│   ├── S3 Robot/        # S3 application: web panel and camera link
│   ├── Motor Control/   # MIC (motor control) and encoder design files
│   ├── shared/          # protocols and configuration shared by both chips
│   └── simulator/       # WebAssembly simulator, tests, measurements and reports
├── docs/                # design, electronics, logbook and README assets
├── CAD_File/            # SolidWorks parts and assemblies
├── t-photos/            # team photos
├── v-photos/            # vehicle photos, six views
├── video/               # competition video links
└── ARQUITECTURA.md · ESTADO.md · ACKERMANN.md · PENDIENTES_PARQUEO.md · MEDIDAS_HARDWARE_P4.md
```

### 11.2 Documentation

Each subsystem is documented by the engineer who built it.

| Document | What it covers |
|---|---|
| [`docs/DESIGN.md`](docs/DESIGN.md) | Chassis, Ackermann geometry, mass budget, CAD and manufacturing |
| [`docs/ELECTRONICS.md`](docs/ELECTRONICS.md) | Power, wiring, sensors, motor driver, bring-up |
| [`ARQUITECTURA.md`](ARQUITECTURA.md) | Software architecture: sensors, state, decision and motors |
| [`ESTADO.md`](ESTADO.md) | Verified results, final configuration and what is missing |
| [`ACKERMANN.md`](ACKERMANN.md) | What the Ackermann chassis changes in the firmware |
| [`PENDIENTES_PARQUEO.md`](PENDIENTES_PARQUEO.md) | Parking: open limits, experiments and measurements |
| [`code/simulator/README.md`](code/simulator/README.md) | Simulator: how it works and how to measure with it |
| [`MEDIDAS_HARDWARE_P4.md`](MEDIDAS_HARDWARE_P4.md) | Measurements on the real P4: scan time, plans, memory, camera and motor |
| [`docs/LOGBOOK.pdf`](docs/LOGBOOK.pdf) | Engineering logbook, 6 to 16 November 2026 |

### 11.3 Where to start

| If you are | Read, in this order |
|---|---|
| A judge | This README, [`ESTADO.md`](ESTADO.md), [`docs/LOGBOOK.pdf`](docs/LOGBOOK.pdf) and [`ARQUITECTURA.md`](ARQUITECTURA.md) |
| A software engineer | [`ARQUITECTURA.md`](ARQUITECTURA.md), `code/LIDAR System/` and [`code/simulator/README.md`](code/simulator/README.md) |
| A mechanical engineer | [`docs/DESIGN.md`](docs/DESIGN.md) and `CAD_File/` |
| An electronics engineer | [`docs/ELECTRONICS.md`](docs/ELECTRONICS.md) and [`ACKERMANN.md`](ACKERMANN.md) |
| Someone who just wants to see the car | [`v-photos/`](v-photos/) and [`t-photos/`](t-photos/) |

---

## 12. Build and run

You need Python 3.10 or newer, Node 18 or newer and ESP-IDF 5.5 for the P4 firmware. Rebuilding the WebAssembly needs a WASI SDK, but the `.wasm` is already versioned in the repository and is not needed to use the simulator.

Simulator:

```bash
python code/simulator/build_simulator.py          # builds code/simulator/build/simulador.html
node code/simulator/tests/test_it_mission.js      # full round with IT
```

P4 firmware:

```bash
cd "code/LIDAR System/p4_firmware"
idf.py set-target esp32p4
idf.py build
```

Do not enable the traction motor before reading [`ACKERMANN.md`](ACKERMANN.md). The original firmware mixed both wheels as if there were an electric differential. With the Ackermann chassis, `hw_motors_drive` uses a single channel and the steering gains are not the old ones.

If you change the C in `code/LIDAR System/main`, `code/Vision System/main` or `code/shared/`, rebuild the WebAssembly:

```powershell
$env:WASI_SDK_PATH = "C:\path\to\wasi-sdk"   # or use -Download to fetch it into code\simulator\.toolchain
.\code\simulator\wasm\build.ps1
```

---

## 13. Rule compliance

| Requirement | How it is met | Evidence |
|---|---|---|
| Ackermann steering, no turning by speed difference | Servo and knuckles; the differential is mechanical | [`ACKERMANN.md`](ACKERMANN.md) and the firmware |
| Maximum size 300 × 200 × 300 mm | 260 × 130 mm chassis and about 80 mm high | v2 CAD and Sebastián's report |
| Maximum mass 1.5 kg | Target 1.2 kg or less | Pending measurement |
| Conventional wheels, not omnidirectional | Four wheels | Mechanical design |
| Red to the right, green to the left | Rule in the planner and the reactive controller | This README and the tests |
| Three laps and parking | Phase machine | `wro_controller.c` |
| No radio during the round | Wi-Fi and Bluetooth off; physical start | `hw_start.c` and [`docs/ELECTRONICS.md`](docs/ELECTRONICS.md) |
| Own work and open repository | Commit history, code, documents and own CAD | GitHub |
| Surprise rule, optional | A ToF covers the 50 mm signs; the team decides before each attempt | Section 2.1 of this README |

| Required material | Where |
|---|---|
| Team photos | [`t-photos/`](t-photos/) |
| Vehicle photos, six views | [`v-photos/`](v-photos/) |
| Competition video | [`video/video.md`](video/video.md) |

The rules ask for video of both challenges. This documentation describes the autonomous driving of the Obstacle Challenge, which is the one that requires mapping a random track. The video will include both challenges once they are recorded.

---

## 14. What comes next

### 14.1 Roadmap

1. Close mechanics: weights, real wheelbase, maximum angle and sensor mounting.
2. Measure servo and motor: speed, delay and torque of the SCS0009; speed against PWM and current of the JGB37-520.
3. Bench-test by blocks: LiDAR, IMU, ToF, encoder, P4-S3 link, camera extension and flash map.
4. Calibrate odometry and localiser on the real car.
5. Enable MIC and then the Intelligent Trajectory, only with the previous measurements loaded.
6. Wire the VL53L0X, set its mounting height and test detection of the 50 mm sign.
7. Run a continuous three-minute test and adjust the final times.

### 14.2 Measurements still pending

| Figure | Why it matters |
|---|---|
| Total mass and split between axles | 1.5 kg limit and the MIC and Intelligent Trajectory models |
| Centre-of-gravity height | Stability in corners and under braking |
| Real wheelbase in the firmware | Today it uses 184.5 mm against 163 mm measured, between 12 and 13 % error |
| Real wheel diameter | 65, 66 or 69 mm change the odometry by up to 6 % |
| Real wheel angle and turning radius | The planner asks for 40° and the servo programs 62° |
| Speed, delay and torque of the SCS0009 | Without these the Intelligent Trajectory cannot trust its line |
| Real motor current | The 2.5 A stall figure is from the datasheet |
| Map accuracy at 34 cm/s | The map is the base of the Intelligent Trajectory and parking |
| Plan time on the P4 | Today we only know the PC time, about 0.4 s |
| Wheel price and supplier | The only budget line still without a cost |
| Track times | They do not exist yet. Any number would be invented |
| Height and detection range of the VL53L0X | The 50 mm sign only appears if the ToF is aimed at the right band |

---

## 15. Credits

Team AutoNex (RDX), Instituto Tecnológico de Las Américas, Boca Chica, Santo Domingo, Dominican Republic.

- Eddy Manuel Piantini Martinez: software, simulator and testing.
- Sebastian Josue Diaz Encarnacion: mechanics, CAD and Ackermann steering.
- Yeremy Manuel Bautista Portes: electronics, power and bring-up.
- Rafael Vargas: coach and logistics.

We thank ITLA for the space, the tools and the support to take the project to WRO, and WRO República Dominicana for organising the national competition.

<div align="center">

**AutoNex** · built by the team, documented for the judges, proven in simulation.

</div>
