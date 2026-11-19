#ifndef WRO_LINK_PROTOCOL_H
#define WRO_LINK_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Enlace de control P4 <-> S3 por la UART que ya une las dos placas (921600 8N1). Comparte el cable con la
 * tabla de la camara (S3 -> P4, magia A5 5A); estas tramas llevan otra magia, asi que no se mezclan.
 *
 * Trama: A6 5B | tipo | largo(2, LE) | carga | crc16(2, LE). El CRC16-CCITT cubre tipo..carga.
 *   LINK_CMD   (S3 -> P4): orden de iniciar/parar + "la pagina web esta viva" (latido).
 *   LINK_STATE (P4 -> S3): estado del robot, camara vista y el ultimo barrido del LiDAR. */
#define LINK_MAGIC_0 0xA6u
#define LINK_MAGIC_1 0x5Bu
#define LINK_TYPE_CMD 1u
#define LINK_TYPE_STATE 2u
#define LINK_MAX_SCAN_POINTS 512u
#define LINK_MAX_CAM_OBJECTS 12u
#define LINK_MAX_PAYLOAD (96u + LINK_MAX_CAM_OBJECTS * 5u + LINK_MAX_SCAN_POINTS * 4u)
#define LINK_MAX_FRAME (5u + LINK_MAX_PAYLOAD + 2u)

enum { LINK_CMD_NONE = 0, LINK_CMD_START = 1, LINK_CMD_STOP = 2,
       LINK_CMD_CAL_FRONT = 3,      /* con un objeto justo al frente del robot: fija el 0 del LiDAR */
       LINK_CMD_CAL_RIGHT = 4,
       LINK_CMD_SET_NORTH = 5,
       LINK_CMD_CAL_PITCH = 6,
       LINK_CMD_MOTOR_M1 = 7, LINK_CMD_MOTOR_M2 = 8,   /* prueba de motores: una rueda 1 s */
       LINK_CMD_MOTOR_SEQ = 9, LINK_CMD_MOTOR_STOP = 10 };    /* con el morro levantado >= 5 grados: aprende el eje delantero del IMU */    /* el rumbo actual pasa a ser el norte y la posicion actual el origen del mapa */    /* con un objeto a la derecha: comprueba/corrige el sentido del giro */

/* Resultado de la ultima calibracion del LiDAR (link_state_t.cal_msg). */
enum { LINK_CAL_NONE = 0, LINK_CAL_FRONT_OK, LINK_CAL_RIGHT_OK, LINK_CAL_SIGN_FLIPPED, LINK_CAL_TOO_FAR, LINK_CAL_RIGHT_BAD };

/* Que esta haciendo el robot (el texto lo pone quien lo muestra). */
typedef enum {
    LINK_MSG_WAITING = 0, LINK_MSG_CALIBRATING, LINK_MSG_DONE, LINK_MSG_FAULT, LINK_MSG_STOPPED,
    LINK_MSG_PIVOT, LINK_MSG_REVERSE, LINK_MSG_CORNER_RIGHT, LINK_MSG_CORNER_LEFT, LINK_MSG_STRAIGHT,
    LINK_MSG_RED_RIGHT, LINK_MSG_RED_LEFT, LINK_MSG_GREEN_RIGHT, LINK_MSG_GREEN_LEFT,
    LINK_MSG_TURN_RIGHT, LINK_MSG_TURN_LEFT, LINK_MSG_COUNT
} link_msg_t;

typedef struct {
    uint8_t cmd;       /* LINK_CMD_* */
    uint8_t seq;       /* cambia con cada orden nueva del usuario: las calibraciones solo se hacen una vez por cambio */
    bool web_alive;    /* la pagina consulto hace poco: si deja de llegar, el P4 para */
} link_cmd_t;

typedef struct {
    uint8_t phase, msg;                 /* wro_phase_t, link_msg_t */
    bool armed;
    int8_t direction;                   /* +1 horario, -1 antihorario, 0 aun no se sabe */
    uint8_t laps, corners;
    float speed_cm_s, yaw_rate_dps, yaw_deg;
    uint16_t front_cm, left_cm, right_cm;
    float elapsed_s;
    float enc_left_m, enc_right_m;
    float imu_dps;
    uint16_t lidar_age_ms;              /* 0xFFFF = aun sin barrido */
    uint32_t lidar_revs, tables_ok;
    uint8_t cal_msg;                    /* LINK_CAL_* */
    int8_t cal_sign;                    /* sentido del giro del LiDAR (+1/-1) */
    float cal_offset_deg;               /* grados que se suman para que 0 sea el frente del robot */
    float map_heading_deg;              /* rumbo del robot (0 = norte, horario positivo) cuando se tomo el barrido */
    float map_e_cm, map_n_cm;           /* posicion (este, norte) en cm respecto al origen del mapa */
    float gyro_bias_dps;                /* sesgo extra del giroscopio aprendido con el LiDAR en reposo */
    uint8_t map_epoch;                  /* cambia al fijar el norte: el mapa anterior ya no sirve */
    float height_cm;                    /* altura relativa desde que se inicio (IMU + distancia) */
    float pitch_deg;                    /* inclinacion con signo: morro arriba positivo */
    uint8_t level;                      /* cambia cada vez que la altura se aleja 30 cm: una medida (mapa) nueva */
    bool pitch_cal;                     /* el eje delantero del IMU ya se aprendio */
    uint8_t motor_test;                 /* prueba de motores: bit0 M1 encendido, bit1 M2 encendido */
    uint8_t cam_count;
    struct { float bearing_deg; uint16_t distance_cm; uint8_t color; } cam[LINK_MAX_CAM_OBJECTS];
    uint16_t scan_count;
    struct { float angle_deg; uint16_t distance_cm; } scan[LINK_MAX_SCAN_POINTS];
} link_state_t;

size_t link_encode_cmd(const link_cmd_t *cmd, uint8_t *out, size_t capacity);
size_t link_encode_state(const link_state_t *state, uint8_t *out, size_t capacity);
bool link_decode_cmd(const uint8_t *payload, size_t len, link_cmd_t *out);
bool link_decode_state(const uint8_t *payload, size_t len, link_state_t *out);

/* Decodificador de flujo: alimentalo con bytes; devuelve el tipo (1 o 2) cuando hay una trama con CRC valido
 * (la carga queda en payload/payload_len hasta el siguiente feed), o 0. */
typedef struct {
    uint8_t buf[LINK_MAX_FRAME];
    size_t used;
    const uint8_t *payload;
    size_t payload_len;
} link_decoder_t;

void link_decoder_reset(link_decoder_t *d);
int link_decoder_feed(link_decoder_t *d, uint8_t byte);

#endif
