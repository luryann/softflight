#include "terrain.h"
#include "renderer.h"

static float hash(float x,float z){ return sinf(x*12.9898f+z*78.233f)*43758.5453f; }
static float noise(float x,float z){
    int xi=(int)floorf(x), zi=(int)floorf(z);
    float xf=x-floorf(x), zf=z-floorf(z);
    float v00=hash((float)xi,(float)zi), v10=hash((float)(xi+1),(float)zi), v01=hash((float)xi,(float)(zi+1)), v11=hash((float)(xi+1),(float)(zi+1));
    float uxf=xf*xf*(3-2*xf), uzf=zf*zf*(3-2*zf);
    float a=v00 + (v10-v00)*uxf;
    float b=v01 + (v11-v01)*uxf;
    return a + (b-a)*uzf;
}

float terrain_height(float x, float z){
    float h = 0.0f;
    float f=0.005f, a=50.0f;
    for(int i=0;i<4;i++){
        h += a*fabsf(noise(x*f, z*f));
        f*=2.0f; a*=0.5f;
    }
    return h;
}

vec3 terrain_normal(float x, float z){
    const float eps = 1.0f;
    float h = terrain_height(x, z);
    float hx = terrain_height(x+eps, z) - h;
    float hz = terrain_height(x, z+eps) - h;
    // normal approximately perpendicular to surface: (-dh/dx, 1, -dh/dz)
    vec3 n = v3(-hx/eps, 1.0f, -hz/eps);
    return v3_norm(n);
}

// Clip-space plane test: point is outside a plane if it violates any of
// -w <= x <= w, -w <= y <= w, 0 <= z <= w  (D3D-like depth; we use z in [0, w])
static int aabb_outside_any_clip_plane(const mat4* vp, const vec3* corners, int count){
    for(int p=0;p<6;p++){
        int outside = 1;
        for(int i=0;i<count;i++){
            vec4 c = m4_mul_pos4(*vp, corners[i]);
            switch(p){
                case 0: if(c.x >= -c.w) outside = 0; break; // left
                case 1: if(c.x <=  c.w) outside = 0; break; // right
                case 2: if(c.y >= -c.w) outside = 0; break; // bottom
                case 3: if(c.y <=  c.w) outside = 0; break; // top
                case 4: if(c.z >=  0.0f) outside = 0; break; // near
                case 5: if(c.z <=  c.w) outside = 0; break; // far
            }
            if(!outside) break;
        }
        if(outside) return 1;
    }
    return 0;
}

int terrain_build(const terrain_cfg* cfg, const camera* cam, float aspect, tri* out_tris, int max_tris){
    int count=0;
    float T=cfg->tile_size;
    int R=cfg->tiles_radius;
    int grid=8;
    float step=T/(float)grid;

    int ci = (int)floorf(cam->pos.x / T);
    int cj = (int)floorf(cam->pos.z / T);

    mat4 vp = camera_viewproj(cam, aspect);

    for(int tj=-R;tj<=R;tj++){
        for(int ti=-R;ti<=R;ti++){
            float ox = (float)(ci+ti)*T;
            float oz = (float)(cj+tj)*T;

            const float Hmax = 120.0f;
            vec3 corners[8]={
                v3(ox,      0,   oz),      v3(ox+T,   0,   oz),
                v3(ox,      0,   oz+T),    v3(ox+T,   0,   oz+T),
                v3(ox,      Hmax,oz),      v3(ox+T,   Hmax,oz),
                v3(ox,      Hmax,oz+T),    v3(ox+T,   Hmax,oz+T)
            };
            if(aabb_outside_any_clip_plane(&vp, corners, 8)) continue;

            for(int gz=0; gz<grid; gz++){
                for(int gx=0; gx<grid; gx++){
                    if(count+2 > max_tris) return count;
                    vec3 p00 = v3(ox + gx*step,     terrain_height(ox + gx*step,     oz + gz*step),     oz + gz*step);
                    vec3 p10 = v3(ox + (gx+1)*step, terrain_height(ox + (gx+1)*step, oz + gz*step),     oz + gz*step);
                    vec3 p01 = v3(ox + gx*step,     terrain_height(ox + gx*step,     oz + (gz+1)*step), oz + (gz+1)*step);
                    vec3 p11 = v3(ox + (gx+1)*step, terrain_height(ox + (gx+1)*step, oz + (gz+1)*step), oz + (gz+1)*step);

                    out_tris[count].v[0].p=p00;
                    out_tris[count].v[1].p=p10;
                    out_tris[count].v[2].p=p11;
                    out_tris[count].v[0].color=v3(0.2f,0.8f,0.2f);
                    out_tris[count].v[1].color=v3(0.2f,0.8f,0.2f);
                    out_tris[count].v[2].color=v3(0.2f,0.8f,0.2f);
                    count++;

                    out_tris[count].v[0].p=p00;
                    out_tris[count].v[1].p=p11;
                    out_tris[count].v[2].p=p01;
                    out_tris[count].v[0].color=v3(0.2f,0.75f,0.2f);
                    out_tris[count].v[1].color=v3(0.2f,0.75f,0.2f);
                    out_tris[count].v[2].color=v3(0.2f,0.75f,0.2f);
                    count++;
                }
            }
        }
    }
    return count;
}
