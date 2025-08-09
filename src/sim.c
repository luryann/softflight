#include "sim.h"
#include "log.h"

void sim_init(sim* s, int fbw, int fbh){
    fdm_init_c172(&s->model);
    s->cam = camera_make(v3(-30, 10, -30), v3(0,5,0), 75.0f, 0.1f, 50000.0f);
    s->tcfg = (terrain_cfg){ .tile_size=200.0f, .tiles_radius=2, .height_scale=1.0f };
    renderer_init_ctx(&s->rc, fbw, fbh);
    s->shading_mode=1;
    (void)fbw; (void)fbh;
}

void sim_update(sim* s, const input_state* in, float dt){
    // Controls map
    fdm_set_controls(&s->model, in->axis_pitch, in->axis_roll, in->axis_yaw, in->throttle);
    if(dt>0 && !in->pause){
        fdm_step(&s->model, dt);

    // Terrain-aware ground contact resolution (post-integrator)
    {
        float ground_y = terrain_height(s->model.pos.x, s->model.pos.z);
        if(s->model.pos.y < ground_y){
            vec3 n = terrain_normal(s->model.pos.x, s->model.pos.z); // up-ish
            // Project velocity into normal/tangent
            float v_n = v3_dot(s->model.vel, n);
            vec3 v_t = v3_sub(s->model.vel, v3_scale(n, v_n));
            // Restitution and friction
            const float e = 0.2f;     // bounce
            const float mu = 0.02f;   // crude friction
            if(v_n < 0) v_n = -e * v_n;
            v_t = v3_scale(v_t, (1.0f - mu));
            s->model.vel = v3_add(v3_scale(n, v_n), v_t);
            s->model.pos.y = ground_y;
        }
    }
    }
    // Camera: chase
    vec3 forward = q_rotate(s->model.orient, v3(1,0,0));
    vec3 up = q_rotate(s->model.orient, v3(0,1,0));
    vec3 right = q_rotate(s->model.orient, v3(0,0,1));
    vec3 eye = v3_add(s->model.pos, v3_add( v3_scale(forward,-15.0f), v3_scale(up,6.0f) ));
    s->cam = camera_make(eye, v3_add(s->model.pos, v3_scale(forward,5.0f)), 75.0f, 0.1f, 50000.0f);
    s->shading_mode = in->shading_mode;
}

void sim_render(sim* s, framebuffer* fb){
    fb->shading = s->shading_mode;
    fb_clear(fb, 0xFF87CEEBu, 1e9f); // sky blue

    // Build terrain tris around the aircraft
    enum { MAX_TRIS = 20000 };
    static tri tris[MAX_TRIS];
    float aspect = (float)fb->w / (float)fb->h;
    int n = terrain_build(&s->tcfg, &s->cam, aspect, tris, MAX_TRIS);

    mat4 world = m4_ident();
    mat4 vp = camera_viewproj(&s->cam, aspect);

    draw_triangles_ctx(s->rc, fb, tris, n, &world, &vp);

    // Simple aircraft proxy: a colored triangle + line
    tri airplane[2];
    vec3 P = s->model.pos;
    vec3 fwd = q_rotate(s->model.orient, v3(1,0,0));
    vec3 rgt = q_rotate(s->model.orient, v3(0,0,1));
    vec3 up = q_rotate(s->model.orient, v3(0,1,0));
    float scale=2.0f;
    airplane[0].v[0]=(vertex){v3_add(P, v3_scale(fwd, 4*scale)), v3(0,0,0), v3(1,0,0)};
    airplane[0].v[1]=(vertex){v3_add(P, v3_add(v3_scale(rgt, 2*scale), v3_scale(up, -1*scale))), v3(0,0,0), v3(1,1,0)};
    airplane[0].v[2]=(vertex){v3_add(P, v3_add(v3_scale(rgt, -2*scale), v3_scale(up, -1*scale))), v3(0,0,0), v3(1,1,0)};
    airplane[1].v[0]=(vertex){v3_add(P, v3_scale(fwd, -2*scale)), v3(0,0,0), v3(0,0,1)};
    airplane[1].v[1]=(vertex){v3_add(P, v3_scale(fwd,  4*scale)), v3(0,0,0), v3(0,0,1)};
    airplane[1].v[2]=(vertex){v3_add(P, v3_scale(up,  2*scale)),  v3(0,0,0), v3(0,0,1)};

    draw_triangles(fb, airplane, 2, &world, &vp);

    // HUD
    draw_hud(fb, fdm_get_airspeed(&s->model)*1.94384f, fdm_get_altitude(&s->model)*3.28084f,
             0.0f, 0.0f, s->model.u.throttle);
}
