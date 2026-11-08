#include <windows.h>
#include <stdio.h>
#include <math.h>
#include "wro_controller.h"
#include "park_ackermann.h"
static double ms(void){ LARGE_INTEGER a,b; QueryPerformanceCounter(&a); QueryPerformanceFrequency(&b); return 1000.0*(double)a.QuadPart/(double)b.QuadPart; }
static wro_controller_t c;
static local_lidar_point_t pts[360];
int main(void){
 wro_controller_config_t cfg; wro_controller_config_default(&cfg); cfg.pivot_enabled=false; cfg.parking_enabled=false; cfg.planner.wheelbase_cm=14;
 wro_controller_init(&c,&cfg); for(int i=0;i<360;i++){float a=(i-180)*0.01745329252f;float s=fabsf(sinf(a));float co=cosf(a);float d=300;if(s>0.1f)d=fminf(d,50/s);if(co>0.1f)d=fminf(d,180/co);pts[i]=(local_lidar_point_t){.angle_deg=(float)(i-180),.distance_cm=d,.valid=true};}
 for(int i=1;i<=100;i++)wro_controller_imu(&c,(uint64_t)i*10000,0,NAN);wro_controller_start(&c,1000001);wro_command_t out; double start=ms();
 for(int i=0;i<150;i++){uint64_t t=1100000+(uint64_t)i*100000;wro_controller_imu(&c,t,0,NAN);wro_controller_scan(&c,pts,360,t,&out);}printf("controller_scan_ms %.6f\n",(ms()-start)/150);
 park_params_t p;park_params_default(&p,14,22,4);p.margin_cm=1;park_ackermann_config_t cfgp={14,30,3.5f,0};park_pose_t pose[]={{-7,12.5,0},{-30,36,0},{-20,32,0},{0,36,0}};
 for(int i=0;i<4;i++){start=ms();park_ackermann_result_t r=park_ackermann_plan(&p,&cfgp,&pose[i],i?PARK_GOAL_IN:PARK_GOAL_OUT);printf("parking_%d_ms %.3f expanded %u found %d steps %u\n",i,ms()-start,r.expanded,r.found,r.count);}
}