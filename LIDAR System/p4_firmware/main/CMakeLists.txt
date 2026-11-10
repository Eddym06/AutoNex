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
    REQUIRES driver esp_timer nvs_flash esp_driver_gpio esp_driver_ledc esp_driver_pcnt esp_partition
)
