#ifndef SOFTFLIGHT_FDM_H
#define SOFTFLIGHT_FDM_H
#include "math.h"
#include <stdbool.h>

typedef struct {
    // State
    vec3  pos;      // world position (x,z horizontal, y up)
    vec3  vel;      // body-frame? We'll store in world frame for simplicity
    quat  orient;   // body -> world
    vec3  omega;    // body angular rates (p,q,r)

    // Mass/inertia
    float mass;
    vec3  I;        // diagonal inertia (Ix,Iy,Iz) body axes

    // Aircraft params (basic)
    float wing_area;    // m^2
    float wing_span;    // m
    float chord;        // m
    float cl_alpha;     // per rad
    float cd0;          // zero-lift drag
    float k_induced;    // induced drag factor
    float cm_alpha;     // pitching moment derivative
    float thrust_max;   // N
} aircraft;

typedef struct {
    float elevator; // -1..1
    float aileron;  // -1..1
    float rudder;   // -1..1
    float throttle; // 0..1
} controls;

typedef struct {
    float rho; // air density kg/m^3
    float wind_x, wind_y, wind_z; // world wind
    float g; // gravity
} environment;

typedef struct {
    aircraft ac;
    environment env;
    controls u;
} fdm;

void fdm_init_c172(fdm* f);
void fdm_step(fdm* f, float dt);
float fdm_get_airspeed(const fdm* f);
float fdm_get_altitude(const fdm* f);
void  fdm_set_controls(fdm* f, float elev, float ail, float rud, float thr);
void  fdm_reset(fdm* f);

#endif
