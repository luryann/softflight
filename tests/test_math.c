#include "math.h"
#include <assert.h>
#include <stdio.h>

int main(){
    vec3 a=v3(1,2,3), b=v3(4,5,6);
    vec3 c=v3_add(a,b);
    assert(c.x==5 && c.y==7 && c.z==9);
    assert((int)roundf(v3_len(v3(3,4,0)))==5);
    quat qx = q_from_axis_angle(v3(1,0,0), (float)M_PI*0.5f);
    vec3 vy = q_rotate(qx, v3(0,1,0));
    assert(fabsf(vy.y) < 1e-4f); // rotated to z
    printf("math ok\n");
    return 0;
}
