/* Adaptador nativo para evaluar exactamente la implementación C con barridos
 * guardados. La verdad nunca se envía por stdin. */
#include "trace_parking.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
static trace_map_t map;
static local_lidar_point_t points[LOCAL_MAP_MAX_POINTS];
int main(void){
 trace_parking_observation_t c={0};float mount;unsigned nscans;
 if(scanf("%f%f%f%f%f%f%f%f%f%f%u",&c.camera_pose.x_cm,&c.camera_pose.z_cm,&c.camera_pose.theta_deg,
    &c.camera_forward_cm,&c.camera_lateral_cm,&c.min_bearing_deg,&c.max_bearing_deg,&c.distance_cm,
    &c.spacing_cm,&mount,&nscans)!=11)return 2;
 for(unsigned i=0;i<nscans;i++){
  arena_pose_t p={0};unsigned count;
  if(scanf("%f%f%f%u",&p.x_cm,&p.z_cm,&p.theta_deg,&count)!=4||count>LOCAL_MAP_MAX_POINTS)return 3;
  for(unsigned k=0;k<count;k++){int valid;if(scanf("%f%f%d",&points[k].angle_deg,&points[k].distance_cm,&valid)!=3)return 4;points[k].valid=valid!=0;}
  trace_map_update(&map,points,count,NULL,&p,mount);
 }
 const trace_parking_result_t r=trace_parking_estimate(&map,&c);
 if(getenv("BENCH_TRACE_PARKING")){
  const clock_t start=clock();volatile unsigned sink=0;
  for(int i=0;i<100;i++)sink+=(unsigned)trace_parking_estimate(&map,&c).status;
  fprintf(stderr,"100 ejecuciones: %.3f ms/ejecucion (CPU nativa), control=%u\n",
          1000.0*(double)(clock()-start)/CLOCKS_PER_SEC/100.0,sink);
 }
 printf("{\"status\":%d,\"wall_cells\":%u,\"alternatives\":%u,\"spacing_cm\":%.3f,\"points\":[[%.3f,%.3f],[%.3f,%.3f]],\"scans\":%u,\"cells\":%u}\n",
  r.status,r.wall_cells,r.alternatives,r.spacing_cm,r.wall_x[0],r.wall_z[0],r.wall_x[1],r.wall_z[1],map.scans,map.occupied_cells);
 return 0;
}
