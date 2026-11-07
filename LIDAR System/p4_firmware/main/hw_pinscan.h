#ifndef HW_PINSCAN_H
#define HW_PINSCAN_H

/* Diagnostico: durante ~0.5 s mide los cambios de nivel de cada GPIO libre y imprime los que se mueven.
 * Sirve para saber en que pin esta realmente conectado el TX del LiDAR. Activar con WRO_PIN_SCAN. */
void hw_pinscan_run(void);

#endif
