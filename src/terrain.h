#ifndef SOFTFLIGHT_TERRAIN_H
#define SOFTFLIGHT_TERRAIN_H
#include "math.h"
#include "renderer.h"

typedef struct {
    float tile_size;
    int   tiles_radius;
    float height_scale;
} terrain_cfg;

int terrain_build(const terrain_cfg* cfg, const camera* cam, float aspect, tri* out_tris, int max_tris);
float terrain_height(float x, float z);
vec3 terrain_normal(float x, float z);

#endif
