cmake_minimum_required(VERSION 3.16)

# Firmware todo-en-uno del ESP32-S3 CAM: vision + controlador + LiDAR/IMU/motores/encoders + WiFi con panel web.
# Reutiliza el C de "Vision System", "LIDAR System" y "shared" sin copiarlo.
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(wro_s3)
