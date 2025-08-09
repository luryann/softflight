#ifndef SOFTFLIGHT_RENDERER_H
#define SOFTFLIGHT_RENDERER_H
#include <stdint.h>
#include <stdbool.h>
#include "math.h"
#include "platform.h"

typedef struct {
    uint32_t* color;
    float*    z;
    int w,h;
    int shading; // 0 flat, 1 gouraud, 2 wire
} framebuffer;

typedef struct {
    vec3 pos;
    vec3 forward, right, up;
    float fov_y;
    float near, far;
} camera;

typedef struct {
    vec3 p;    // position (world for input; NDC for raster stage)
    vec3 n;    // normal
    vec3 color;// 0..1
    float invw; // 1/w from clip stage for perspective-correct interpolation
} vertex;

typedef struct {
    vertex v[3];
} tri;

void fb_clear(framebuffer* fb, uint32_t argb, float zfar);
camera camera_make(vec3 pos, vec3 target, float fov_deg, float nearz, float farz);
mat4 camera_viewproj(const camera* c, float aspect);

// Drawing
typedef struct render_ctx render_ctx;

bool renderer_init_ctx(render_ctx** out, int fbw, int fbh);
void renderer_resize_ctx(render_ctx* rc, int fbw, int fbh);
void renderer_shutdown_ctx(render_ctx* rc);

void draw_triangles_ctx(render_ctx* rc, framebuffer* fb, const tri* tris, int count, const mat4* world, const mat4* vp);
void draw_line(framebuffer* fb, int x0,int y0,int x1,int y1, uint32_t argb);
void draw_hud(framebuffer* fb, float airspeed, float altitude, float pitch_rad, float roll_rad, float throttle);

#endif
