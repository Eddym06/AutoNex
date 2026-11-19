cmake_minimum_required(VERSION 3.16)

# Firmware del ESP32-P4 del carrito. Compila el C de "LIDAR System/main" y "shared" tal cual,
# y los drivers de hardware de "main/" (pines y parametros en menuconfig: "WRO - pines y hardware del P4").
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(wro_p4)
