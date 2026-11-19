#ifndef ESP_ERR_STUB_H
#define ESP_ERR_STUB_H
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_ERROR_CHECK(x) ((void)(x))
#endif
