cmake_minimum_required(VERSION 3.16)
# Prueba de diagnostico: fuerza IN1..IN4 del L298N a 0000 y no usa PWM. No es el firmware del carrito.
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(diag_motores)
