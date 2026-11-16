set(WRO_CORE_DIR "${CMAKE_CURRENT_LIST_DIR}/../../main")
set(WRO_ORCHESTRATOR_DIR "${CMAKE_CURRENT_LIST_DIR}/../../orchestrator")
set(WRO_SHARED_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../shared")
set(WRO_MIC_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../Motor Control/MIC")

# Logica del robot (sin hardware): el mismo C que prueba el simulador.
file(GLOB WRO_CORE_SRC "${WRO_CORE_DIR}/*.c")
list(REMOVE_ITEM WRO_CORE_SRC "${WRO_CORE_DIR}/trace_map.c")
list(REMOVE_ITEM WRO_CORE_SRC "${WRO_CORE_DIR}/trace_parking.c")

idf_component_register(
    SRCS
        "app_main.c"
        "hw_motors.c"
        "hw_steering.c"
        "hw_lidar.c"
        "hw_camera.c"
        "hw_encoders.c"
        "hw_start.c"
        "hw_imu.c"
        "hw_pinscan.c"
        "hw_link.c"
        "hw_mapstore.c"
        "bench_web.c"
        "perf_bench.c"
        ${WRO_CORE_SRC}
        "${WRO_CORE_DIR}/trace_map.c"
        "${WRO_CORE_DIR}/trace_parking.c"
        "${WRO_ORCHESTRATOR_DIR}/p4_runtime.c"
        "${WRO_ORCHESTRATOR_DIR}/wro_controller.c"
        "${WRO_SHARED_DIR}/config_fields.c"
        "${WRO_SHARED_DIR}/sensor_table_protocol.c"
        "${WRO_SHARED_DIR}/link_protocol.c"
        "${WRO_MIC_DIR}/mic.c"
    INCLUDE_DIRS
        "."
        "${WRO_CORE_DIR}"
        "${WRO_ORCHESTRATOR_DIR}"
        "${WRO_SHARED_DIR}"
        "${WRO_MIC_DIR}"
    REQUIRES driver esp_timer nvs_flash esp_driver_gpio esp_driver_ledc esp_driver_pcnt esp_partition esp_driver_usb_serial_jtag
)

# Camino caliente del robot (IT, parqueo, mapa local, localizadores): -O3 y sin errno/trampas de punto flotante. Con errno cada sqrtf/fminf lleva una
# comprobacion y una llamada a la libm de newlib. (NO -ffinite-math-only: el codigo valida entradas con isfinite.)
foreach(hot it_plan park_ackermann park_planner park_localize local_obstacle_map arena_localizer odometry)
    set_source_files_properties("${WRO_CORE_DIR}/${hot}.c" PROPERTIES COMPILE_OPTIONS "-O3;-fno-math-errno;-fno-trapping-math")
endforeach()
