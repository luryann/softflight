#include "platform.h"
#include "log.h"
#include <stdlib.h>
#include <string.h>

bool platform_init(platform* p, int w, int h, bool fullscreen){
    if(SDL_Init(SDL_INIT_VIDEO|SDL_INIT_EVENTS|SDL_INIT_TIMER) != 0){
        LOGE("SDL_Init failed: %s", SDL_GetError());
        return false;
    }
    Uint32 flags = SDL_WINDOW_SHOWN;
    if(fullscreen) flags |= SDL_WINDOW_FULLSCREEN_DESKTOP;
    p->window = SDL_CreateWindow("softflight-c", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, w, h, flags);
    if(!p->window){ LOGE("CreateWindow: %s", SDL_GetError()); return false; }
    p->renderer = SDL_CreateRenderer(p->window, -1, SDL_RENDERER_PRESENTVSYNC);
    if(!p->renderer){ LOGW("CreateRenderer failed (%s), using software", SDL_GetError()); p->renderer = SDL_CreateRenderer(p->window, -1, 0); }
    p->texture = SDL_CreateTexture(p->renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, w, h);
    if(!p->texture){ LOGE("CreateTexture: %s", SDL_GetError()); return false; }
    p->width=w; p->height=h; p->fullscreen=fullscreen;
    size_t npix=(size_t)w*h;
    p->frame=(uint32_t*)malloc(npix*sizeof(uint32_t));
    p->zbuf=(float*)malloc(npix*sizeof(float));
    if(!p->frame || !p->zbuf){ LOGE("Out of memory for framebuffers"); return false; }
    return true;
}

void platform_shutdown(platform* p){
    if(p->frame) free(p->frame);
    if(p->zbuf) free(p->zbuf);
    if(p->texture) SDL_DestroyTexture(p->texture);
    if(p->renderer) SDL_DestroyRenderer(p->renderer);
    if(p->window) SDL_DestroyWindow(p->window);
    SDL_Quit();
}

void platform_present(platform* p){
    SDL_UpdateTexture(p->texture, NULL, p->frame, p->width*4);
    SDL_RenderClear(p->renderer);
    SDL_RenderCopy(p->renderer, p->texture, NULL, NULL);
    SDL_RenderPresent(p->renderer);
}

double platform_seconds(void){
    return (double)SDL_GetTicks64() / 1000.0;
}

void platform_poll_input(input_state* in){
    SDL_Event e;
    while(SDL_PollEvent(&e)){
        if(e.type == SDL_QUIT) in->quit = true;
        if(e.type == SDL_KEYDOWN || e.type == SDL_KEYUP){
            bool down = (e.type == SDL_KEYDOWN);
            SDL_Keycode k = e.key.keysym.sym;
            if(k == SDLK_ESCAPE && down) in->quit=true;
            if(k == SDLK_p && down) in->pause = !in->pause;
            if(k == SDLK_c && down) in->cam_mode = (in->cam_mode+1)%3;
            if(k == SDLK_1 && down) in->shading_mode = 0;
            if(k == SDLK_2 && down) in->shading_mode = 1;
            if(k == SDLK_3 && down) in->shading_mode = 2;
        }
    }
    const Uint8* ks = SDL_GetKeyboardState(NULL);
    in->axis_pitch = (ks[SDL_SCANCODE_S]?1.0f:0.0f) + (ks[SDL_SCANCODE_W]?-1.0f:0.0f);
    in->axis_roll  = (ks[SDL_SCANCODE_D]?1.0f:0.0f) + (ks[SDL_SCANCODE_A]?-1.0f:0.0f);
    in->axis_yaw   = (ks[SDL_SCANCODE_E]?1.0f:0.0f) + (ks[SDL_SCANCODE_Q]?-1.0f:0.0f);
    if(ks[SDL_SCANCODE_R]) in->throttle = fminf(1.0f, in->throttle + 0.01f);
    if(ks[SDL_SCANCODE_F]) in->throttle = fmaxf(0.0f, in->throttle - 0.01f);
}
