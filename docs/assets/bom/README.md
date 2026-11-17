# BOM images

The bill of materials in `README.md` and `README.es.md` loads a photo for the main
components from this folder. Each file is referenced by name, so the name has to match
exactly.

| File | Component | What the photo shows |
|---|---|---|
| `lidar.png` | 360° LiDAR | The D500/D6 alone, connector visible |
| `camera.png` | Vision board | The ESP32-S3-CAM board with the camera |
| `fpc-adapter.png` | Camera extension adapter | The 24-pin adapter board with both ZIF sockets |
| `imu.png` | IMU | The GY-521 module |
| `tof.png` | VL53L0X ToF | The module with the two lenses |
| `p4.png` | Main board | The ESP32-P4 DevKit, top side |
| `servo-adapter.png` | Servo adapter | The Waveshare board (A) |
| `motor.png` | Traction motor | The JGB37-520 with its encoder connector |
| `servo.png` | Steering servo | The SCS0009 with its cable |
| `drv8871.png` | H-bridge | The DRV8871 module |
| `xl4015.png` | Buck converter | The XL4015 board |
| `battery.png` | Battery | The 3S pack with the EC3 connector |

The fuse, the filament, the wheels, the threaded inserts and the fasteners do not carry a
photo in the table. Neither does the camera ribbon.

Keep the background plain and the part centred. A photo around 800 px wide is enough;
the tables scale it down to 90 px. Transparent PNG or white background both work.

The wheels are still missing their price and supplier. Until that arrives, the row stays
as pending in the total.
