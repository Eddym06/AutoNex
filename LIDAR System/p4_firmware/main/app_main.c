/* Firmware del ESP32-P4: une los drivers de hardware con el controlador de la mision
 * (p4_runtime + wro_controller). Pines y parametros: menuconfig, "WRO - pines y hardware del P4".
 *
 * Inicio/parada: el boton fisico (GPIO de WRO_START_GPIO) o las ordenes del S3 (pagina web, shared/link_protocol.h).
 * Los motores solo se mueven con el robot "armado" (tras iniciar) y se paran con PARAR, con la mision terminada
 * o, si se inicio desde la pagina, cuando esta deja de dar latidos (WRO_LINK_DEADMAN_S). */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "hw_camera.h"
#include "hw_encoders.h"
#include "hw_imu.h"
#include "hw_lidar.h"
#include "hw_link.h"
#include "hw_motors.h"
#include "hw_steering.h"
#include "hw_pinscan.h"
#include "hw_start.h"
#include "nvs_flash.h"
#include "height_est.h"
#include "map_pose.h"
#include "motor_test.h"
#include "p4_runtime.h"
#include "wro_config.h"

static const char *TAG = "wro_p4";

static p4_runtime_t rt;                      /* grande: fuera de la pila */
static volatile bool s_armed;                /* iniciado: los motores pueden moverse */
static motor_test_t s_mtest;                 /* prueba de motores desde la pagina (solo con el robot parado) */
static bool s_link_run;                      /* el inicio vino del S3: aplica la parada por latido */
static bool s_start_pending;                 /* orden de iniciar esperando muestras de giroscopio */
static SemaphoreHandle_t s_scan_lock;
static height_est_t s_hgt;                  /* altura relativa con la inclinacion del IMU */
static volatile bool s_pitch_cal_req;
static map_pose_t s_mp;                      /* rumbo (giroscopio) y posicion (LiDAR) para el mapa de la pagina */
static volatile uint8_t s_map_epoch;         /* cambia al fijar el norte */
static volatile uint8_t s_cal_msg;            /* LINK_CAL_*: resultado de la ultima calibracion del LiDAR */
static struct {
    local_lidar_point_t pts[LOCAL_MAP_MAX_POINTS];
    size_t n;
    uint64_t t_us;
    uint32_t revs;
    float heading_deg, e_cm, n_cm;               /* pose del robot cuando se tomo este barrido */
} s_scan;

static uint64_t now_us(void *ctx)
{
    (void)ctx;
    return (uint64_t)esp_timer_get_time();
}

static bool hal_start_pressed(void *ctx)
{
    if (hw_start_pressed(ctx)) {             /* boton fisico */
        s_armed = true;
        s_link_run = false;
        height_start(&s_hgt);
        return true;
    }
    /* Orden del S3: espera a tener muestras de giroscopio quieto para el sesgo. */
    if (s_start_pending && rt.gyro_samples >= rt.controller.cfg.gyro_min_samples) {
        s_start_pending = false;
        s_armed = true;
        s_link_run = true;
        height_start(&s_hgt);
        return true;
    }
    return false;
}

static bool hal_poll_gyro(void *ctx, float *dps, uint64_t *t_us)
{
    if (!hw_imu_poll_gyro(ctx, dps, t_us)) return false;
    map_pose_gyro(&s_mp, *dps, *t_us);                   /* el rumbo del mapa sale del giroscopio */
    static uint64_t t_prev;
    float acc[3];
    if (hw_imu_accel(acc)) {
        height_accel(&s_hgt, acc, t_prev ? (float)(*t_us - t_prev) * 1e-6f : 0.0f);
        static bool ref_taken;
        if (!ref_taken && s_hgt.have_u) { height_start(&s_hgt); ref_taken = true; }   /* referencia inicial: el robot al arrancar */
    }
    t_prev = *t_us;
    return true;
}

static bool hal_poll_scan(void *ctx, local_lidar_point_t *pts, size_t cap, size_t *count, uint64_t *t_us)
{
    if (!hw_lidar_poll_scan(ctx, pts, cap, count, t_us)) return false;
    /* Avance de los encoders desde el barrido anterior: solo es fiable con los motores armados (el sentido lo da el
     * control); con el robot movido a mano la posicion sale solo del alineado de barridos. */
    static float enc_l_prev, enc_r_prev;
    float enc_l, enc_r;
    hw_encoders_position_m(&enc_l, &enc_r);
    const float fwd_cm = s_armed ? 50.0f * ((enc_l - enc_l_prev) + (enc_r - enc_r_prev)) : 0.0f;
    enc_l_prev = enc_l;
    enc_r_prev = enc_r;
    const float e0 = s_mp.e_cm, n0 = s_mp.n_cm;
    map_pose_scan(&s_mp, pts, *count, fwd_cm);
    if (s_mp.flip_request && s_armed) s_mp.flip_request = 0;   /* nunca se cambia el signo con el robot en marcha */
    if (s_mp.flip_request) {                             /* el LiDAR vio girar al reves que el giroscopio */
        s_mp.flip_request = 0;
        hw_imu_set_sign(-hw_imu_sign());
        s_mp.heading_deg = -s_mp.heading_deg;            /* lo integrado con el signo malo queda corregido */
        ++s_map_epoch;                                   /* el mapa dibujado con el rumbo malo se descarta */
        ESP_LOGW(TAG, "el giroscopio giraba al reves: signo corregido");
    }
    const float th = s_mp.heading_deg * 3.14159265f / 180.0f;
    if (height_move(&s_hgt, (s_mp.e_cm - e0) * sinf(th) + (s_mp.n_cm - n0) * cosf(th)))
        ESP_LOGI(TAG, "cambio de altura: nivel %d (altura %.0f cm)", s_hgt.level, s_hgt.height_cm);
    xSemaphoreTake(s_scan_lock, portMAX_DELAY);
    s_scan.heading_deg = s_mp.heading_deg;
    s_scan.e_cm = s_mp.e_cm;
    s_scan.n_cm = s_mp.n_cm;
    s_scan.n = *count < LOCAL_MAP_MAX_POINTS ? *count : LOCAL_MAP_MAX_POINTS;
    memcpy(s_scan.pts, pts, s_scan.n * sizeof(pts[0]));
    s_scan.t_us = *t_us;
    ++s_scan.revs;
    xSemaphoreGive(s_scan_lock);
    return true;
}

static void drive(void *ctx, float steer_deg, float speed_cm_s, float yaw_rate_dps)
{
    (void)ctx;
    if (s_mtest.mask) return;                    /* prueba de motores en curso: manda la prueba */
    if (!s_armed) speed_cm_s = yaw_rate_dps = 0.0f;
    hw_steering_set(s_armed ? steer_deg : 0.0f);
    hw_motors_drive(speed_cm_s, yaw_rate_dps);
}

static void reset_runtime(void)
{
    wro_controller_config_t cfg;
    wro_controller_config_default(&cfg);
    cfg.pivot_enabled = false;       /* Ackermann no gira sobre su eje. */
    cfg.sidestep_enabled = false;
    cfg.peek_enabled = false;
    cfg.parking_enabled = false;     /* el parqueo actual incluye giros sobre el eje */
    cfg.start_in_lot = false;
    cfg.odometry_enabled = CONFIG_WRO_AXLE_ENCODER_CALIBRATED && hw_encoders_ready();
    cfg.odom.wheel_diameter_cm = 0.2f * (float)CONFIG_WRO_WHEEL_RADIUS_MM;   /* rueda de 68 mm */
    cfg.odom.track_width_cm = 0.1f * (float)CONFIG_WRO_TRACK_MM;
    const p4_hal_t hal = {
        .ctx = NULL,
        .now_us = now_us,
        .start_pressed = hal_start_pressed,
        .poll_scan = hal_poll_scan,
        .poll_gyro = hal_poll_gyro,
        .read_camera = hw_link_read,
        .drive = drive,
    };
    p4_runtime_init(&rt, &hal, &cfg);
}

static void stop_all(const char *why)
{
    motor_test_request(&s_mtest, MOTOR_TEST_REQ_STOP, now_us(NULL));
    s_armed = false;
    s_link_run = false;
    s_start_pending = false;
    hw_motors_stop();
    reset_runtime();                          /* la mision vuelve a espera */
    ESP_LOGW(TAG, "PARADO: %s", why);
}

/* Calibracion guiada del LiDAR con el ultimo barrido: busca el objeto mas cercano (entre 8 y 120 cm) y promedia
 * su angulo. FRONT: lo que esta al frente pasa a ser 0. RIGHT: lo que esta a la derecha debe dar +90; si da -90
 * el sentido del giro esta invertido y se corrige (el frente se conserva). */
static void lidar_calibrate(uint8_t kind)
{
    float best_d = 1e9f, best_a = 0.0f;
    xSemaphoreTake(s_scan_lock, portMAX_DELAY);
    for (size_t i = 0; i < s_scan.n; ++i) {
        const float d = s_scan.pts[i].distance_cm;
        if (s_scan.pts[i].valid && d >= 8.0f && d < best_d) { best_d = d; best_a = s_scan.pts[i].angle_deg; }
    }
    float sn = 0.0f, cs = 0.0f;
    for (size_t i = 0; i < s_scan.n && best_d <= 120.0f; ++i) {          /* promedio circular del objeto */
        const float d = s_scan.pts[i].distance_cm;
        float da = s_scan.pts[i].angle_deg - best_a;
        while (da > 180.0f) da -= 360.0f;
        while (da < -180.0f) da += 360.0f;
        if (s_scan.pts[i].valid && d <= best_d + 6.0f && fabsf(da) <= 20.0f) {
            sn += sinf(s_scan.pts[i].angle_deg * (float)M_PI / 180.0f);
            cs += cosf(s_scan.pts[i].angle_deg * (float)M_PI / 180.0f);
        }
    }
    xSemaphoreGive(s_scan_lock);
    if (best_d > 120.0f) { s_cal_msg = LINK_CAL_TOO_FAR; return; }
    const float a_cur = atan2f(sn, cs) * 180.0f / (float)M_PI;
    float sign, off;
    hw_lidar_calibration_get(&sign, &off);
    if (kind == LINK_CMD_CAL_FRONT) {
        hw_lidar_calibration_set(sign, off - a_cur);
        s_cal_msg = LINK_CAL_FRONT_OK;
    } else if (fabsf(a_cur - 90.0f) <= 45.0f) {
        s_cal_msg = LINK_CAL_RIGHT_OK;
    } else if (fabsf(a_cur + 90.0f) <= 45.0f) {
        hw_lidar_calibration_set(-sign, -off);                           /* el frente (0) queda donde estaba */
        s_cal_msg = LINK_CAL_SIGN_FLIPPED;
    } else {
        s_cal_msg = LINK_CAL_RIGHT_BAD;
    }
}

static void init_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

/* Que esta haciendo el robot, para la pagina. */
static link_msg_t current_msg(void)
{
    const wro_controller_t *c = &rt.controller;
    const wro_command_t *m = &rt.last;
    if (s_start_pending) return LINK_MSG_CALIBRATING;
    if (c->phase == WRO_PHASE_IDLE) return LINK_MSG_WAITING;
    if (c->phase == WRO_PHASE_DONE) return LINK_MSG_DONE;
    if (c->phase == WRO_PHASE_FAULT) return LINK_MSG_FAULT;
    if (!s_armed || fabsf(m->speed_cm_s) < 0.5f)
        return c->pivoting || m->explicit_yaw_rate ? LINK_MSG_PIVOT : LINK_MSG_STOPPED;
    if (m->speed_cm_s < 0.0f) return LINK_MSG_REVERSE;
    const bool right = m->yaw_rate_dps > 0.0f;
    if (c->corner_active) return right ? LINK_MSG_CORNER_RIGHT : LINK_MSG_CORNER_LEFT;
    if (fabsf(m->yaw_rate_dps) < 6.0f) return LINK_MSG_STRAIGHT;
    if (c->last_sign_color == 1) return right ? LINK_MSG_RED_RIGHT : LINK_MSG_RED_LEFT;
    if (c->last_sign_color == 2) return right ? LINK_MSG_GREEN_RIGHT : LINK_MSG_GREEN_LEFT;
    return right ? LINK_MSG_TURN_RIGHT : LINK_MSG_TURN_LEFT;
}

static uint16_t cm_u16(float v) { return v < 0.0f ? 0 : v > 65535.0f ? 65535 : (uint16_t)(v + 0.5f); }

/* Tarea de baja prioridad: arma el estado y lo manda al S3 unas 8 veces por segundo. */
static void link_tx_task(void *arg)
{
    (void)arg;
    static link_state_t st;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(125));
        const wro_controller_t *c = &rt.controller;
        const wro_command_t *m = &rt.last;
        uint32_t ok, chk, revs, pts, drop, imu_n, imu_err;
        float imu_dps;
        hw_lidar_stats(&ok, &chk, &revs, &pts, &drop);
        hw_imu_stats(&imu_n, &imu_dps, &imu_err);
        memset(&st, 0, sizeof st);
        st.phase = (uint8_t)c->phase;
        st.msg = (uint8_t)current_msg();
        st.armed = s_armed;
        st.direction = (int8_t)m->direction;
        st.laps = (uint8_t)m->laps;
        st.corners = (uint8_t)m->corners;
        st.speed_cm_s = m->speed_cm_s;
        st.yaw_rate_dps = m->yaw_rate_dps;
        st.yaw_deg = m->yaw_deg;
        st.front_cm = cm_u16(m->front_cm);
        st.left_cm = cm_u16(m->left_cm);
        st.right_cm = cm_u16(m->right_cm);
        st.elapsed_s = m->elapsed_s;
        hw_encoders_position_m(&st.enc_left_m, &st.enc_right_m);
        float cal_sign, cal_off;
        hw_lidar_calibration_get(&cal_sign, &cal_off);
        st.cal_msg = s_cal_msg;
        st.map_epoch = s_map_epoch;
        st.gyro_bias_dps = s_mp.bias_dps;
        st.height_cm = s_hgt.height_cm;
        st.pitch_deg = s_hgt.pitch_deg;
        st.level = (uint8_t)s_hgt.level;
        st.pitch_cal = s_hgt.fwd_cal;
        st.motor_test = s_mtest.mask;
        st.cal_sign = (int8_t)cal_sign;
        st.cal_offset_deg = cal_off;
        st.imu_dps = imu_dps;
        st.lidar_revs = s_scan.revs;
        st.tables_ok = rt.tables_ok;
        for (size_t i = 0; i < c->camera.object_count && st.cam_count < LINK_MAX_CAM_OBJECTS; ++i) {
            const local_camera_object_t *o = &c->camera.objects[i];
            if (!o->valid) continue;
            st.cam[st.cam_count].bearing_deg = o->bearing_deg;
            st.cam[st.cam_count].distance_cm = cm_u16(o->distance_cm);
            st.cam[st.cam_count].color = o->color;
            ++st.cam_count;
        }
        xSemaphoreTake(s_scan_lock, portMAX_DELAY);
        st.lidar_age_ms = s_scan.revs ? (uint16_t)fminf((float)((esp_timer_get_time() - (int64_t)s_scan.t_us) / 1000), 65534.0f) : 0xFFFF;
        st.map_heading_deg = s_scan.heading_deg;
        st.map_e_cm = s_scan.e_cm;
        st.map_n_cm = s_scan.n_cm;
        for (size_t i = 0; i < s_scan.n && st.scan_count < LINK_MAX_SCAN_POINTS; ++i) {
            if (!s_scan.pts[i].valid || s_scan.pts[i].distance_cm <= 0.0f) continue;
            st.scan[st.scan_count].angle_deg = s_scan.pts[i].angle_deg;
            st.scan[st.scan_count].distance_cm = cm_u16(s_scan.pts[i].distance_cm);
            ++st.scan_count;
        }
        xSemaphoreGive(s_scan_lock);
        hw_link_send_state(&st);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "WRO P4: motores %s (tope 12 V = %d %% de PWM, bateria %d mV), S3 por USB %s, LiDAR RX GPIO %d",
             CONFIG_WRO_MOTORS_ENABLED ? "habilitados" : "deshabilitados",
             100 * CONFIG_WRO_MOTOR_VMAX_MV / CONFIG_WRO_BATT_MV, CONFIG_WRO_BATT_MV,
             CONFIG_WRO_CAM_USB ? "si" : "no", CONFIG_WRO_LIDAR_RX_GPIO);
    hw_motors_safe_boot();                       /* primero: el puente no se mueve durante el arranque */
    if (CONFIG_WRO_PIN_SCAN) hw_pinscan_run();   /* antes de configurar ningun pin */
    init_nvs();   /* las tablas de calibracion de los encoders se guardan en NVS */
    s_scan_lock = xSemaphoreCreateMutex();

    hw_motors_init();
    hw_encoders_init();
    hw_steering_init();
    hw_lidar_init();
    hw_camera_init();
    hw_start_init();
    hw_imu_init();
    reset_runtime();
    xTaskCreate(link_tx_task, "link_tx", 6144, NULL, 2, NULL);

    uint64_t last_report = now_us(NULL);
    for (;;) {
        const uint8_t cmd = hw_link_take_cmd();
        if (cmd == LINK_CMD_MOTOR_M1 || cmd == LINK_CMD_MOTOR_M2 || cmd == LINK_CMD_MOTOR_SEQ) {
            ESP_LOGW(TAG, "prueba de dos motores deshabilitada para el chasis Ackermann");
        } else if (cmd == LINK_CMD_MOTOR_STOP) motor_test_request(&s_mtest, MOTOR_TEST_REQ_STOP, now_us(NULL));
        else if (cmd == LINK_CMD_STOP) stop_all("orden del S3");
        else if (cmd == LINK_CMD_CAL_PITCH) {
            ESP_LOGI(TAG, "calibrar inclinacion: %s", height_calibrate_axis(&s_hgt) ? "eje delantero aprendido" : "levanta el morro al menos 5 grados");
        } else if (cmd == LINK_CMD_SET_NORTH) {
            map_pose_reset(&s_mp);                       /* el rumbo y la posicion actuales pasan a ser el norte y el origen */
            ++s_map_epoch;
            ESP_LOGI(TAG, "norte fijado: el rumbo actual es 0 y la posicion actual el origen del mapa");
        } else if ((cmd == LINK_CMD_CAL_FRONT || cmd == LINK_CMD_CAL_RIGHT) && !s_armed) lidar_calibrate(cmd);
        else if (cmd == LINK_CMD_START && CONFIG_WRO_WEB_START_ENABLED && !s_armed && !s_start_pending) {
            s_start_pending = true;
            ESP_LOGI(TAG, "orden de iniciar del S3: calibrando giroscopio");
        }
        if (s_armed && s_link_run && CONFIG_WRO_LINK_DEADMAN_S > 0 &&
            esp_timer_get_time() - hw_link_last_alive_us() > (int64_t)CONFIG_WRO_LINK_DEADMAN_S * 1000000)
            stop_all("sin latido de la pagina web");
        if (s_armed && rt.controller.phase == WRO_PHASE_DONE) s_armed = false;   /* mision terminada: motores a 0 */
        if (s_armed && s_mtest.mask) motor_test_request(&s_mtest, MOTOR_TEST_REQ_STOP, now_us(NULL));   /* al iniciar, la prueba se acaba */
        {
            static uint8_t last_mask;                /* solo se escriben los motores cuando cambian */
            motor_test_update(&s_mtest, now_us(NULL));
            if (s_mtest.mask || last_mask) {
                float l, r;
                motor_test_wheels(&s_mtest, &l, &r);
                hw_motors_set_wheels(l, r);
            }
            last_mask = s_mtest.mask;
        }

        hw_encoders_feed(&rt.controller, now_us(NULL));
        p4_runtime_step(&rt);

        const uint64_t t = now_us(NULL);
        {   /* con el robot armado o iniciando: una linea por 0,5 s con lo que decide el controlador y lo que recibe el puente */
            static uint64_t last_run_log;
            if ((s_armed || s_start_pending) && t - last_run_log >= 500000u) {
                last_run_log = t;
                uint32_t d[4];
                hw_motors_last_duty(d);
                ESP_LOGI(TAG, "MARCHA: pend=%d armado=%d fase=%d accion=%d v=%.1f w=%.1f frente=%.0f izq=%.0f der=%.0f pasa=%.0f | PWM IN1..4=%u %u %u %u | scans=%lu muestras_gyro=%lu",
                         s_start_pending, s_armed, (int)rt.controller.phase, (int)rt.last.action, rt.last.speed_cm_s, rt.last.yaw_rate_dps,
                         rt.last.front_cm, rt.last.left_cm, rt.last.right_cm, rt.last.pass_cm, (unsigned)d[0], (unsigned)d[1], (unsigned)d[2], (unsigned)d[3],
                         (unsigned long)rt.scans, (unsigned long)rt.gyro_samples);
            }
        }
        if (t - last_report >= 2000000u) {       /* informe cada 2 s */
            last_report = t;
            uint32_t ok, chk, revs, pts, drop, cam_bytes, cam_lost;
            bool cam_connected;
            hw_lidar_stats(&ok, &chk, &revs, &pts, &drop);
            uint32_t raw;
            uint8_t head[8];
            hw_lidar_raw(&raw, head);
            hw_camera_stats(&cam_bytes, &cam_lost, &cam_connected);
            float l_m, r_m;
            uint32_t imu_n;
            float imu_dps;
            uint32_t imu_err;
            hw_imu_stats(&imu_n, &imu_dps, &imu_err);
            hw_encoders_position_m(&l_m, &r_m);
            /* Diagnostico del mapa: caja que ocupa el ultimo barrido (este-oeste x norte-sur, en el marco del robot). */
            float xmin = 1e9f, xmax = -1e9f, ymin = 1e9f, ymax = -1e9f;
            xSemaphoreTake(s_scan_lock, portMAX_DELAY);
            for (size_t i = 0; i < s_scan.n; ++i) {
                if (!s_scan.pts[i].valid || s_scan.pts[i].distance_cm < 10.0f) continue;
                const float a = s_scan.pts[i].angle_deg * 3.14159265f / 180.0f;
                const float x = s_scan.pts[i].distance_cm * sinf(a), y = s_scan.pts[i].distance_cm * cosf(a);
                if (x < xmin) xmin = x;
                if (x > xmax) xmax = x;
                if (y < ymin) ymin = y;
                if (y > ymax) ymax = y;
            }
            xSemaphoreGive(s_scan_lock);
            ESP_LOGI(TAG, "mapa: rumbo=%.1f pos=(%.0f,%.0f) cm sesgo=%.2f dps alin=%d | barrido x[%.0f,%.0f] y[%.0f,%.0f] cm | altura=%.1f cabeceo=%.1f signo_imu=%+d",
                     s_mp.heading_deg, s_mp.e_cm, s_mp.n_cm, s_mp.bias_dps, s_mp.last_matches, xmin, xmax, ymin, ymax,
                     s_hgt.height_cm, s_hgt.pitch_deg, hw_imu_sign());
            ESP_LOGI(TAG,
                     "LiDAR raw=%lu [%02X %02X %02X %02X] paq=%lu chk_err=%lu vueltas=%lu puntos=%lu (desc %lu) | "
                     "IMU muestras=%lu err=%lu dps=%.2f | S3 %s bytes=%lu perd=%lu tablas=%lu | enc I=%.3f D=%.3f m | scans=%lu fase=%d armado=%d",
                     (unsigned long)raw, head[0], head[1], head[2], head[3], (unsigned long)ok, (unsigned long)chk, (unsigned long)revs, (unsigned long)pts,
                     (unsigned long)drop,
                     (unsigned long)imu_n, (unsigned long)imu_err, imu_dps, cam_connected ? "conectado" : "no conectado", (unsigned long)cam_bytes, (unsigned long)cam_lost, (unsigned long)rt.tables_ok, l_m, r_m,
                     (unsigned long)rt.scans, (int)rt.controller.phase, s_armed);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
