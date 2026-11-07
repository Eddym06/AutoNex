idf_component_register(SRCS "app_main.c" "camera_source.c" "vision.c"
    "vision_table.c" "vision_uart_link.c" "vision_settings.c" "vision_config_cmd.c"
    "vision_config_rx.c" "${CMAKE_CURRENT_LIST_DIR}/../../shared/sensor_table_protocol.c"
    "${CMAKE_CURRENT_LIST_DIR}/../../shared/config_fields.c"
    INCLUDE_DIRS "." "../../shared" REQUIRES driver esp32-camera esp_timer nvs_flash)
