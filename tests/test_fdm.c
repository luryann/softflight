#include "fdm.h"
#include <assert.h>
#include <stdio.h>

int main(){
    fdm f; fdm_init_c172(&f);
    float v0 = fdm_get_airspeed(&f);
    for(int i=0;i<1000;i++) fdm_step(&f, 1.0f/120.0f);
    float v1 = fdm_get_airspeed(&f);
    assert(v1>0.0f);
    assert(f.pos.y > 0.0f); // should fly for a bit
    printf("fdm ok\n");
    return 0;
}
