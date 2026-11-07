#include "hw_imu.h"

#include "driver/i2c_master.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "wro_config.h"

static const char *TAG = "imu";

/* MPU-6050 (GY-521 o similar). Registros segun la hoja de datos de InvenSense. */
#define MPU_REG_GYRO_CONFIG  0x1B
#define MPU_REG_CONFIG       0x1A
#define MPU_REG_SMPLRT_DIV   0x19
#define MPU_REG_ACCEL_XOUT_H 0x3B        /* accel xyz, temperatura, gyro xyz: 14 bytes seguidos */
#define MPU_REG_GYRO_ZOUT_H  0x47        /* Z: alto, bajo (big-endian) */
#define MPU_REG_PWR_MGMT_1   0x6B
#define MPU_REG_WHO_AM_I     0x75
#define MPU_WHO_AM_I         0x68        /* el chip responde con su direccion */
#define GYR_LSB_PER_DPS      131.0f      /* rango +-250 dps */
#define SAMPLE_PERIOD_US     2500        /* ~400 Hz con DLPF de 44 Hz */

static i2c_master_dev_handle_t s_dev;
static uint64_t s_next_us;
static uint32_t s_samples, s_errors;
static esp_err_t s_last_err;
static float s_last_dps;
static float s_bias_lsb;
static int s_sign = CONFIG_WRO_IMU_SIGN;   /* se invierte solo si el LiDAR demuestra que el giroscopio gira al reves */
static float s_acc_g[3];             /* ultimo acelerometro (g), +-2 g: 16384 LSB/g */
static bool s_acc_valid;

static esp_err_t write_reg(uint8_t reg, uint8_t val)
{
    const uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_dev, buf, 2, 20);
}

static esp_err_t read_regs(uint8_t reg, uint8_t *out, size_t n)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, out, n, 20);
}

int hw_imu_sign(void) { return s_sign; }

void hw_imu_set_sign(int sign)
{
    s_sign = sign < 0 ? -1 : 1;
    nvs_handle_t h;
    if (nvs_open("imu_cal", NVS_READWRITE, &h) == ESP_OK) { nvs_set_i8(h, "sign", (int8_t)s_sign); nvs_commit(h); nvs_close(h); }
    ESP_LOGW(TAG, "signo del giroscopio guardado: %+d", s_sign);
}

void hw_imu_init(void)
{
    nvs_handle_t nh;
    int8_t sv;
    if (nvs_open("imu_cal", NVS_READONLY, &nh) == ESP_OK) {
        if (nvs_get_i8(nh, "sign", &sv) == ESP_OK) s_sign = sv < 0 ? -1 : 1;
        nvs_close(nh);
    }
    const i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = CONFIG_WRO_IMU_SDA_GPIO,
        .scl_io_num = CONFIG_WRO_IMU_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus;
    if (i2c_new_master_bus(&bus_cfg, &bus) != ESP_OK) { ESP_LOGE(TAG, "bus I2C no disponible"); return; }
    for (uint8_t a = 0x08; a < 0x78; ++a)   /* diagnostico: que hay en el bus */
        if (i2c_master_probe(bus, a, 20) == ESP_OK) ESP_LOGI(TAG, "I2C: responde 0x%02X", a);
    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = CONFIG_WRO_IMU_ADDR,
        .scl_speed_hz = 400000,
    };
    if (i2c_master_bus_add_device(bus, &dev_cfg, &s_dev) != ESP_OK) { ESP_LOGE(TAG, "no se pudo anadir el MPU-6050"); s_dev = NULL; return; }

    uint8_t id = 0;
    if (read_regs(MPU_REG_WHO_AM_I, &id, 1) != ESP_OK || id == 0x00 || id == 0xFF) {
        ESP_LOGE(TAG, "sin respuesta del IMU (WHO_AM_I=0x%02X): revisa SDA/SCL, AD0 y alimentacion", id);
        s_dev = NULL;
        return;
    }
    if (id != MPU_WHO_AM_I)
        ESP_LOGW(TAG, "WHO_AM_I=0x%02X, el MPU-6050 deberia dar 0x68: se continua para ver si el giroscopio da datos", id);
    write_reg(MPU_REG_PWR_MGMT_1, 0x80);            /* reinicio */
    vTaskDelay(pdMS_TO_TICKS(100));
    write_reg(MPU_REG_PWR_MGMT_1, 0x01);            /* despierta, reloj PLL con el giroscopio X */
    write_reg(MPU_REG_CONFIG, 0x03);                /* DLPF 44 Hz */
    write_reg(MPU_REG_SMPLRT_DIV, 1);               /* 1 kHz / (1+1) = 500 Hz internos */
    write_reg(MPU_REG_GYRO_CONFIG, 0x00);           /* +-250 dps */
    vTaskDelay(pdMS_TO_TICKS(50));
    /* Sesgo del giroscopio: promedio con el robot quieto durante el arranque (~0.8 s). */
    int32_t acc = 0;
    int n = 0;
    for (int i = 0; i < 300; ++i) {
        uint8_t raw[2];
        if (read_regs(MPU_REG_GYRO_ZOUT_H, raw, 2) == ESP_OK) { acc += (int16_t)((raw[0] << 8) | raw[1]); ++n; }
        vTaskDelay(pdMS_TO_TICKS(3));
    }
    if (n > 100) s_bias_lsb = (float)acc / (float)n;
    ESP_LOGI(TAG, "sesgo del giroscopio: %.2f dps (%d muestras)", s_bias_lsb / GYR_LSB_PER_DPS, n);
    s_next_us = (uint64_t)esp_timer_get_time();
    ESP_LOGI(TAG, "MPU-6050 listo (SDA %d, SCL %d, 0x%02X, INT %d sin usar)",
             CONFIG_WRO_IMU_SDA_GPIO, CONFIG_WRO_IMU_SCL_GPIO, CONFIG_WRO_IMU_ADDR, CONFIG_WRO_IMU_INT_GPIO);
}

bool hw_imu_poll_gyro(void *ctx, float *dps, uint64_t *t_us)
{
    (void)ctx;
    if (!s_dev) return false;
    const uint64_t now = (uint64_t)esp_timer_get_time();
    if (now < s_next_us) return false;
    s_next_us = now + SAMPLE_PERIOD_US;
    uint8_t raw[14];
    const esp_err_t err = read_regs(MPU_REG_ACCEL_XOUT_H, raw, 14);
    if (err != ESP_OK) {
        if (!s_errors) ESP_LOGE(TAG, "lectura del IMU fallo: %s", esp_err_to_name(err));
        s_last_err = err;
        ++s_errors;
        return false;
    }
    for (int i = 0; i < 3; ++i) s_acc_g[i] = (float)(int16_t)((raw[2 * i] << 8) | raw[2 * i + 1]) / 16384.0f;
    s_acc_valid = true;
    const int16_t z = (int16_t)((raw[12] << 8) | raw[13]);
    *dps = (float)s_sign * ((float)z - s_bias_lsb) / GYR_LSB_PER_DPS;
    s_last_dps = *dps;
    ++s_samples;
    *t_us = now;
    return true;
}

void hw_imu_stats(uint32_t *samples, float *last_dps, uint32_t *errors)
{
    *samples = s_samples;
    *last_dps = s_last_dps;
    *errors = s_errors;
}

bool hw_imu_accel(float a_g[3])
{
    if (!s_acc_valid) return false;
    for (int i = 0; i < 3; ++i) a_g[i] = s_acc_g[i];
    return true;
}
