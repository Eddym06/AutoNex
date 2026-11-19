set(ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")
set(VISION_DIR "${ROOT}/Vision System/main")
set(SHARED_DIR "${ROOT}/shared")

idf_component_register(
    SRCS
        "app_main.c" "web.c"
        "${VISION_DIR}/camera_source.c" "${VISION_DIR}/vision.c" "${VISION_DIR}/vision_table.c"
        "${VISION_DIR}/vision_settings.c" "${VISION_DIR}/vision_config_cmd.c"
        "${SHARED_DIR}/config_fields.c" "${SHARED_DIR}/sensor_table_protocol.c" "${SHARED_DIR}/link_protocol.c"
    INCLUDE_DIRS "." "${VISION_DIR}" "${SHARED_DIR}"
    EMBED_TXTFILES "page.html"
    REQUIRES driver esp_driver_uart esp_timer nvs_flash esp_wifi esp_netif esp_event esp_http_server
        esp32-camera app_update
)
