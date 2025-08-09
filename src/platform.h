#ifndef SOFTFLIGHT_PLATFORM_H
#define SOFTFLIGHT_PLATFORM_H

#include <SDL.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int width, height;
    bool fullscreen;
    SDL_Window* window;
    SDL_Renderer* renderer;
    SDL_Texture* texture;
    uint32_t* frame; // ARGB
    float* zbuf;
} platform;

typedef struct {
    float axis_pitch;   // -1..1 (S/W)
    float axis_roll;    // -1..1 (A/D)
    float axis_yaw;     // -1..1 (Q/E)
    float throttle;     // 0..1
    bool pause;
    int  shading_mode;  // 0 flat,1 gouraud,2 wire
    int  cam_mode;      // 0 chase,1 cockpit,2 free
    bool quit;
} input_state;

bool platform_init(platform* p, int w, int h, bool fullscreen);
void platform_shutdown(platform* p);
void platform_present(platform* p);
double platform_seconds(void);
void platform_poll_input(input_state* in);

#endif
