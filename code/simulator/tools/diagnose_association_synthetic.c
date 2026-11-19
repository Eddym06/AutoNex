/* Ensayo controlado del asociador: la deteccion de camara se inyecta, no se simula
 * la imagen. Compara una superficie corta y dos largas en la misma direccion. */
#include <math.h>
#include <stdio.h>
#include "local_obstacle_map.h"
#define RAD_TO_DEG 57.29577951308232f
static local_obstacle_map_t map;
static int run(float half_width, float cluster_radius) {
    local_planner_config_t cfg; local_planner_config_default(&cfg);
    cfg.lidar_forward_cm=10.0f; cfg.camera_forward_cm=13.0f;
    cfg.cluster_radius_cm=cluster_radius; cfg.deskew=false;
    local_lidar_point_t pts[81]; size_t n=0;
    for (float x=-half_width; x<=half_width; x+=1.0f) {
        const float forward_from_lidar=53.0f;
        pts[n++]=(local_lidar_point_t){.angle_deg=atan2f(x,forward_from_lidar)*RAD_TO_DEG,
            .distance_cm=hypotf(x,forward_from_lidar),.valid=true};
    }
    const local_camera_object_t object={.bearing_deg=0,.distance_cm=50,.color=3,.valid=true};
    local_obstacle_map_update_cfg(&map,&cfg,pts,n,1000000,&object,1,1000000,NULL);
    return map.colored_count && map.colored[0].confirmed;
}
int main(void) {
    const char *names[]={"pilar_5cm","tope_20cm","pared_40cm"};
    const float half[]={2.5f,10.0f,20.0f};
    for(int i=0;i<3;i++) printf("%s radio6=%d radio10=%d\n",names[i],run(half[i],6),run(half[i],10));
    return 0;
}
