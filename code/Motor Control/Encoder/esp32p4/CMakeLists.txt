# Componente ESP-IDF "encoder". Copia la carpeta Encoder (o solo src/ y esp32p4/)
# a components/ de tu proyecto, o referencia esta ruta con EXTRA_COMPONENT_DIRS.
idf_component_register(
    SRCS "encoder_port.c" "../src/encoder_core.c"
    INCLUDE_DIRS "." "../src"
    REQUIRES driver esp_timer nvs_flash
)
