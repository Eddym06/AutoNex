#include "trace_parking.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#define RAD 0.017453292519943295f
#define N TRACE_MAP_SIZE

static float coord(int index){return TRACE_MAP_ORIGIN_CM+(index+0.5f)*TRACE_MAP_CELL_CM;}
static int observed(const trace_map_t *m,int x,int z){return (m->cells[z][x]>>TRACE_MAP_SUPPORT_SHIFT)>=3u;}
bool trace_parking_consistent(const trace_parking_result_t *a,const trace_parking_result_t *b,float tolerance){
    if(!a||!b||!isfinite(tolerance)||tolerance<0)return false;
    for(int i=0;i<2;i++)if(!isfinite(a->wall_x[i])||!isfinite(a->wall_z[i])||!isfinite(b->wall_x[i])||!isfinite(b->wall_z[i]))return false;
    const float d00=hypotf(a->wall_x[0]-b->wall_x[0],a->wall_z[0]-b->wall_z[0]);
    const float d11=hypotf(a->wall_x[1]-b->wall_x[1],a->wall_z[1]-b->wall_z[1]);
    const float d01=hypotf(a->wall_x[0]-b->wall_x[1],a->wall_z[0]-b->wall_z[1]);
    const float d10=hypotf(a->wall_x[1]-b->wall_x[0],a->wall_z[1]-b->wall_z[0]);
    return fminf(fmaxf(d00,d11),fmaxf(d01,d10))<=tolerance;
}
static int in_view(float x,float z,const trace_parking_observation_t *o){
    const float a=o->camera_pose.theta_deg*RAD,s=sinf(a),c=cosf(a);
    const float ox=o->camera_pose.x_cm+s*o->camera_forward_cm+c*o->camera_lateral_cm;
    const float oz=o->camera_pose.z_cm+c*o->camera_forward_cm-s*o->camera_lateral_cm;
    const float dx=x-ox,dz=z-oz,right=c*dx-s*dz,forward=s*dx+c*dz;
    if(forward<=0)return 0;
    const float angle=atan2f(right,forward)/RAD,dist=hypotf(dx,dz);
    return angle>=o->min_bearing_deg-8 && angle<=o->max_bearing_deg+8 &&
        dist>=fmaxf(5,o->distance_cm*.3f-12) && dist<=o->distance_cm*2+45;
}
static trace_parking_result_t estimate_axis(const trace_map_t *m,const trace_parking_observation_t *o,int tangent_x){
    trace_parking_result_t out;memset(&out,0,sizeof(out));
    if(!m||!o||!isfinite(o->camera_pose.x_cm)||!isfinite(o->camera_pose.z_cm)||
       !isfinite(o->camera_pose.theta_deg)||!isfinite(o->distance_cm)||o->distance_cm<=0||
       !isfinite(o->spacing_cm)||o->spacing_cm<=0||!isfinite(o->min_bearing_deg)||
       !isfinite(o->max_bearing_deg)||o->min_bearing_deg>o->max_bearing_deg)return out;
    int best=-1,best_count=0;uint8_t wall_scores[N]={0};
    for(int n=0;n<N;n++){
        int count=0,lo=N,hi=-1;
        for(int t=0;t<N;t++){
            const int x=tangent_x?t:n,z=tangent_x?n:t;
            if(observed(m,x,z)&&in_view(coord(x),coord(z),o)){count++;if(t<lo)lo=t;if(t>hi)hi=t;}
        }
        if(count>=10&&(hi-lo)*TRACE_MAP_CELL_CM>=25){wall_scores[n]=count;if(count>best_count){best=n;best_count=count;}}
    }
    if(best<0)return out;
    const float camera_n=tangent_x?o->camera_pose.z_cm:o->camera_pose.x_cm;
    const int side=camera_n>=coord(best)?1:-1;
    uint8_t score[N]={0},seen[N]={0};
    for(int t=0;t<N;t++)for(int d=2;d<=9;d++){
        const int n=best+side*d;if(n<0||n>=N)continue;
        const int x=tangent_x?t:n,z=tangent_x?n:t;
        if(observed(m,x,z)){score[t]++;if(in_view(coord(x),coord(z),o))seen[t]=1;}
    }
    /* Combina columnas contiguas; un bloque muy ancho se considera ambiguo. */
    float marker[40];uint8_t camera_hit[40];int markers=0;
    for(int t=0;t<N;){
        if(score[t]<3){t++;continue;}
        int start=t,sum=0,weighted=0,cam=0;
        while(t<N&&score[t]>=3){sum+=score[t];weighted+=t*score[t];cam|=seen[t];t++;}
        if(t-start>6||markers>=40)continue;
        marker[markers]=weighted/(float)sum;camera_hit[markers]=cam;markers++;
    }
    int ia=-1,ib=-1,alternatives=0;
    for(int i=0;i<markers;i++)for(int j=i+1;j<markers;j++){
        const float spacing=(marker[j]-marker[i])*TRACE_MAP_CELL_CM;
        if(fabsf(spacing-o->spacing_cm)>7||(!camera_hit[i]&&!camera_hit[j]))continue;
        alternatives++;ia=i;ib=j;
    }
    out.wall_cells=(unsigned)best_count;out.alternatives=(unsigned)alternatives;
    if(alternatives==0)return out;
    int competing_wall=0;
    for(int n=0;n<N;n++)if(abs(n-best)>=4 && wall_scores[n]*2>=best_count)competing_wall=1;
    if(alternatives>1 || best_count<15 || competing_wall){out.status=TRACE_PARK_AMBIGUOUS;return out;}
    out.status=TRACE_PARK_CANDIDATE;
    const float a=TRACE_MAP_ORIGIN_CM+(marker[ia]+.5f)*TRACE_MAP_CELL_CM;
    const float b=TRACE_MAP_ORIGIN_CM+(marker[ib]+.5f)*TRACE_MAP_CELL_CM;
    out.wall_x[0]=tangent_x?a:coord(best);out.wall_z[0]=tangent_x?coord(best):a;
    out.wall_x[1]=tangent_x?b:coord(best);out.wall_z[1]=tangent_x?coord(best):b;
    out.spacing_cm=(marker[ib]-marker[ia])*TRACE_MAP_CELL_CM;
    return out;
}
trace_parking_result_t trace_parking_estimate(const trace_map_t *m,const trace_parking_observation_t *o){
    trace_parking_result_t empty;memset(&empty,0,sizeof(empty));
    if(!m||!o)return empty;
    /* Durante una esquina el rumbo del vehículo no define el eje de la pared. */
    trace_parking_result_t x=estimate_axis(m,o,1),z=estimate_axis(m,o,0);
    if(x.status==TRACE_PARK_NO_EVIDENCE)return z;
    if(z.status==TRACE_PARK_NO_EVIDENCE)return x;
    empty.status=TRACE_PARK_AMBIGUOUS;
    empty.wall_cells=x.wall_cells>z.wall_cells?x.wall_cells:z.wall_cells;
    empty.alternatives=x.alternatives+z.alternatives;
    return empty;
}
