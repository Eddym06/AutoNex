#ifndef P4_RUNTIME_H
#define P4_RUNTIME_H

#include "camera_table_receiver.h"
#include "wro_controller.h"

/* Bucle del ESP32-P4: une los drivers con el controlador de la mision.
 *
 * Los drivers (UART del D6, bus del MPU, UART del S3, PWM del puente H y del
 * servo, boton de arranque) dependen de la placa y NO estan aqui: el equipo
 * rellena esta tabla de funciones con su hardware. Todo lo que decide ya esta en
 * wro_controller y se prueba en el simulador; este archivo solo reparte datos y
 * aplica la regla de seguridad "ante la duda, parar". */
typedef struct {
    void *ctx;
    uint64_t (*now_us)(void *ctx);                    /* reloj monotono del P4 */
    bool (*start_pressed)(void *ctx);                 /* flanco del boton unico de arranque */
    /* Una vuelta completa del LiDAR, en orden de barrido, angulos derecha-positivo. Devuelve
     * false si todavia no hay vuelta nueva. t_us: instante del fin de la vuelta. */
    bool (*poll_scan)(void *ctx, local_lidar_point_t *points, size_t capacity, size_t *count, uint64_t *t_us);
    /* Una muestra del giroscopio (grados/s, horario positivo). false si no hay. */
    bool (*poll_gyro)(void *ctx, float *dps, uint64_t *t_us);
    /* Bytes recibidos del S3 desde la ultima llamada. */
    size_t (*read_camera)(void *ctx, uint8_t *buffer, size_t capacity);
    /* Velocidad del eje motriz en cm/s (negativa = reversa) y dos formas de pedir el giro:
     * volante en grados (derecha positiva) para un robot con servo, o yaw_rate_dps
     * (grados/s, horario positivo) para uno diferencial, que reparte velocidad y giro entre
     * sus dos ruedas (v_izq = v + w*vía/2, v_der = v - w*vía/2, con w en rad/s). */
    void (*drive)(void *ctx, float steer_deg, float speed_cm_s, float yaw_rate_dps);
} p4_hal_t;

typedef struct {
    p4_hal_t hal;
    wro_controller_t controller;
    camera_table_decoder_t decoder;
    wro_command_t last;
    uint32_t tables_ok, scans, gyro_samples;
} p4_runtime_t;

void p4_runtime_init(p4_runtime_t *rt, const p4_hal_t *hal, const wro_controller_config_t *cfg);
/* Una pasada: llamala en un bucle sin esperas largas (por ejemplo cada 1 ms). */
void p4_runtime_step(p4_runtime_t *rt);

/* Misma comprobación de frescura para el P4 y el simulador. */
bool p4_runtime_guard(wro_controller_t *controller, uint64_t now_us, wro_command_t *command);

#endif
