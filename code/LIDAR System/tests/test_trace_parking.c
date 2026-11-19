#include "trace_parking.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
int main(void){
 trace_parking_result_t first={.wall_x={0,40},.wall_z={-150,-150}};
 trace_parking_result_t swapped={.wall_x={41,1},.wall_z={-149,-149}};
 assert(trace_parking_consistent(&first,&swapped,2));
 swapped.wall_x[1]=15;
 assert(!trace_parking_consistent(&first,&swapped,8));
 swapped.wall_x[1]=NAN;
 assert(!trace_parking_consistent(&first,&swapped,8));
 assert(!trace_parking_consistent(NULL,&first,8));
 assert(!trace_parking_consistent(&first,&first,-1));
 puts("PASS: coherencia de candidatos, inversion de extremos y valores invalidos");
}
