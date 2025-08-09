#ifndef SOFTFLIGHT_SIM_H
#define SOFTFLIGHT_SIM_H
#include "fdm.h"
#include "renderer.h"
#include "terrain.h"
#include "platform.h"

typedef struct {
    fdm model;
    camera cam;
    terrain_cfg tcfg;
    int shading_mode;
    render_ctx* rc;
} sim;

void sim_init(sim* s, int fbw, int fbh);
void sim_update(sim* s, const input_state* in, float dt);
void sim_render(sim* s, framebuffer* fb);

#endif
