#include "platform.h"
#include "renderer.h"
#include "sim.h"
#include "renderer.h"
#include "log.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    int width, height, fullscreen;
    int hz;
    int shading;
    float fov_deg, nearz, farz;
} config;

static void default_config(config* c){
    c->width=1280; c->height=720; c->fullscreen=0;
    c->hz=120; c->shading=1; c->fov_deg=75; c->nearz=0.1f; c->farz=50000.0f;
}

static void parse_ini_line(char* line, config* cfg){
    char sect[64]={0}, key[64]={0}, val[128]={0};
    if(sscanf(line, "[%63[^]]]", sect)==1) return;
    if(sscanf(line, "%63[^=]=%127s", key, val)==2){
        if(strcmp(key,"width")==0) cfg->width=atoi(val);
        else if(strcmp(key,"height")==0) cfg->height=atoi(val);
        else if(strcmp(key,"fullscreen")==0) cfg->fullscreen=atoi(val);
        else if(strcmp(key,"hz")==0) cfg->hz=atoi(val);
        else if(strcmp(key,"shading")==0) cfg->shading=atoi(val);
        else if(strcmp(key,"fov_deg")==0) cfg->fov_deg=(float)atof(val);
        else if(strcmp(key,"near")==0) cfg->nearz=(float)atof(val);
        else if(strcmp(key,"far")==0) cfg->farz=(float)atof(val);
    }
}

static void load_config(const char* path, config* cfg){
    default_config(cfg);
    FILE* f=fopen(path,"rb");
    if(!f){ LOGW("Config %s not found; using defaults.", path); return; }
    char line[256];
    while(fgets(line,sizeof(line),f)){
        if(line[0]=='#'||line[0]==';'||line[0]=='\n') continue;
        parse_ini_line(line, cfg);
    }
    fclose(f);
}

int main(int argc, char** argv){
    (void)argc;(void)argv;
    config cfg; load_config("softflight.ini", &cfg);

    platform p={0};
    if(!platform_init(&p, cfg.width, cfg.height, cfg.fullscreen!=0)) return 1;

    framebuffer fb = { p.frame, p.zbuf, p.width, p.height, cfg.shading };

    sim S; sim_init(&S, fb.w, fb.h);

    const double dt_fixed = 1.0 / (double)sf_max(30, cfg.hz);
    double t_accum=0.0, t_prev=platform_seconds();
    input_state in={0}; in.throttle=0.6f; in.shading_mode=cfg.shading;

    while(!in.quit){
        platform_poll_input(&in);
        double t_now=platform_seconds();
        double dt=t_now - t_prev; t_prev=t_now;
        if(dt>0.25) dt=0.25; // clamp pauses

        t_accum += dt;
        while(t_accum >= dt_fixed){
            sim_update(&S, &in, (float)dt_fixed);
            t_accum -= dt_fixed;
        }
        // Render at current state (no interpolation of S for simplicity)
        sim_render(&S, &fb);
        platform_present(&p);
    }

    renderer_shutdown_ctx(S.rc);
    platform_shutdown(&p);
    return 0;
}
