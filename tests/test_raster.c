#include "renderer.h"
#include "platform.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

int main(){
    framebuffer fb; int w=64,h=64;
    fb.w=w; fb.h=h; fb.color=(uint32_t*)malloc(w*h*4); fb.z=(float*)malloc(w*h*sizeof(float)); fb.shading=1;
    fb_clear(&fb, 0x0, 1e9f);
    tri t = { .v={{v3(-0.5f,-0.5f,0.5f), v3(0,0,0), v3(1,0,0)},
                   {v3( 0.5f,-0.5f,0.5f), v3(0,0,0), v3(0,1,0)},
                   {v3( 0.0f, 0.5f,0.5f), v3(0,0,0), v3(0,0,1)}} };
    mat4 I=m4_ident();
    draw_triangles(&fb, &t, 1, &I, &I);
    int filled=0;
    for(int i=0;i<w*h;i++) if(fb.z[i]<1e9f) filled++;
    assert(filled>0);
    free(fb.color); free(fb.z);
    printf("raster ok\n");
    return 0;
}
