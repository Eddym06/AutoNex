#ifndef WEB_H
#define WEB_H
#include <stdbool.h>
#include <stdint.h>

/* Panel web del robot: pagina con boton Iniciar/Parar, zona del LiDAR y estado. */
typedef struct {
    bool (*get_state_json)(char *out, int cap, int *len);   /* escribe el JSON del estado */
    int (*get_cam_jpeg)(uint8_t *out, int cap);             /* ultimo cuadro JPEG; devuelve su largo (0 = aun no hay) */
    void (*request_start)(void);
    void (*request_stop)(void);
    void (*request_cal)(int kind);                            /* 0 = frente, 1 = derecha, 2 = fijar norte, 3 = calibrar inclinacion */
    void (*request_motor)(int which);                         /* prueba de motores: 1 = M1, 2 = M2, 3 = secuencia M1 y M2 */
} web_api_t;

void web_start(const web_api_t *api);
/* Microsegundos de la ultima consulta de la pagina (0 = nunca): para la parada automatica. */
int64_t web_last_poll_us(void);

#endif
