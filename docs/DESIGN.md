# Mechanical & Design

> **Owner:** Sebastian, Mechanical & Design Engineer · back to the [README](../README.md)
>
> [!NOTE]
> This page is pre-filled with what the software already assumes about the chassis (so nothing is lost) and has clearly marked blanks for the design explanation. Replace every `✍️` block with your own words, photos and renders. Keep it short: judges read this page in two minutes.

## 1. The design in one picture

✍️ *Add one render or photo of the finished car (top and side) and one sentence on the idea behind the design.*

<!-- ![Chassis render](assets/chassis_render.png) -->

## 2. Key numbers

The firmware reads these values, so they must match the real car. The first column is what the repository uses today; **measure each one on the finished chassis and correct it.**

| Parameter | In the firmware today | Measured on the car | Config key |
|---|---|---|---|
| Body length × width | 260 × 130 mm | ✍️ | `planner.chassis_length_cm`, `chassis_width_cm` |
| Wheelbase | 184.5 mm | ✍️ | `planner.wheelbase_cm` |
| Rear overhang (axle to rear bumper) | 37.75 mm | ✍️ | `planner.chassis_rear_cm` |
| Wheel diameter | 66 mm | ✍️ | `odom.wheel_diameter_cm` |
| Maximum steering angle (road wheel) | 40° | ✍️ | `planner.steer_hard_deg` |
| Mass | goal ≤ 1.2 kg (limit 1.5 kg) | ✍️ | — |
| Weight on the rear axle | 62 % *(assumed)* | ✍️ | `it.rear_weight` |
| Centre-of-gravity height | 40 mm *(assumed)* | ✍️ | `it.cg_height_cm` |
| LiDAR forward of the rear axle | see config | ✍️ | `planner.lidar_forward_cm` |
| Camera forward of the rear axle | see config | ✍️ | `planner.camera_forward_cm` |

**Rules to respect:** maximum 300 × 200 × 300 mm and 1.5 kg; Ackermann steering (differential drive is disqualified); no omnidirectional, spherical or ball-caster wheels.

## 3. Chassis and drivetrain

✍️ *Explain: material and manufacturing (3D print / laser / CNC), how the parts are laid out, why rear drive, how the motor reaches the axle (gear ratio, differential or solid axle), and what you did to keep the mass low.*

## 4. Steering geometry (Ackermann)

✍️ *Explain: how the servo (MG90S) is linked to the stub axles, the Ackermann angle you chose, how you measured the left/right angles and the steering trim. Include a photo of the linkage.*

**Why it matters to the software:** the servo is slow and has some play (the simulator models ~420°/s, 20 ms of command delay and a 0.5° dead band). The faster the car goes, the more this limits the line it can follow, so any play removed here is lap time gained there.

## 5. Weight distribution

The motor and battery sit at the back, so the front wheels carry less load. The trajectory planner knows this: **the front axle grips less (the car understeers earlier) and only the rear axle brakes**. A wrong guess here makes the planner too brave or too timid, so please measure:

1. Put one scale under the front axle and one under the rear axle (car fully assembled, battery in). Rear share = rear / (front + rear).
2. Tilt the car on its side to find the CG height (or hang it from two points).

✍️ *Write the measured values and the method.*

## 6. Sensor mounting

✍️ *Explain where the LiDAR, camera and IMU are mounted and why (field of view, vibration, blind zone). The LiDAR needs a clear 360° view of the walls and pillars.*

## 7. Files

| File | What it is |
|---|---|
| [`Motor Control/design 3d/DiscoEncoder_30ranuras.f3d`](../Motor%20Control/design%203d/) | Fusion 360 source of the 30-slot encoder disc (`.step` and `.stl` next to it) |
| ✍️ | *Chassis, steering, mounts: add links to the `.f3d` / `.step` / `.stl` files* |

## 8. What we tested and what we changed

✍️ *A short log: iteration, problem found, change made, result. The judges value the process more than the final answer (for example: "v1 steering linkage had 3° of play → redesigned arm → play < 0.5°").*

| Iteration | Problem | Change | Result |
|---|---|---|---|
| ✍️ | ✍️ | ✍️ | ✍️ |

## 9. Bench tests the software team needs from you

* [ ] **Servo speed and delay:** command a step from −30° to +30° and film at 240 fps (or use an angle sensor). We need the delay (ms) and the speed (°/s). They go to `steerDelayS` / `steerRateDegS` in the simulator and `it.lag_s` / `it.steer_rate_dps` in the firmware.
* [ ] **Tyre grip (μ):** pull the car on the competition mat with a spring scale until it slides: μ = pull force / weight. Go to `it.mu`.
* [ ] **Maximum steering angle** at each wheel for the full servo travel.
* [ ] **Mass and weight split** (section 5).
