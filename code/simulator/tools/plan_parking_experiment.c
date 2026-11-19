/* Adaptador nativo de investigación para consultar el planificador C real.
 * Solo calcula un plan; nunca controla el vehículo. */
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "park_ackermann.h"
int main(int argc,char **argv){
 if(argc!=6){fprintf(stderr,"u w theta longitud_cajon direccion_previa\n");return 2;}
 park_pose_t pose={(float)atof(argv[1]),(float)atof(argv[2]),(float)atof(argv[3])};
 park_params_t p;park_params_default(&p,13,26,3.775f);p.lot_length_cm=(float)atof(argv[4]);p.margin_cm=1;
 park_ackermann_config_t cfg={18.45f,40,3.5f,atoi(argv[5])};
 clock_t start=clock();park_ackermann_result_t a=park_ackermann_plan(&p,&cfg,&pose,PARK_GOAL_IN);
 printf("{\"found\":%s,\"atGoal\":%s,\"expanded\":%u,\"clock_ms\":%.3f,\"arcs\":[",a.found?"true":"false",a.at_goal?"true":"false",a.expanded,1000.0*(clock()-start)/CLOCKS_PER_SEC);
 for(unsigned i=0;i<a.count;i++)printf("%s[%d,%d]",i?",":"",a.directions[i],a.steering[i]);
 puts("]}");return 0;
}
