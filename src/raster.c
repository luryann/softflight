
#include "renderer.h"
#include "platform.h"
#include "log.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <SDL.h>

#define TILE_SIZE 32

typedef struct { vec3 p; vec3 color; float invw; } RVtx;
typedef struct { RVtx v[3]; } RTri;

typedef struct {
    int *counts;
    int *offsets;
    int *items; // indices into rt_tris[]
    int tiles_w, tiles_h;
    int items_cap;
} TileBuckets;

struct render_ctx {
    // Persistent triangle buffer (post-clip, NDC)
    RTri* rt_tris;
    int   rt_count;
    int   rt_cap;

    // Persistent tile buckets
    TileBuckets tb;

    // Thread pool and sync
    int threads;
    SDL_Thread** th;
    SDL_mutex* m;
    SDL_cond*  cv_start;
    SDL_cond*  cv_done;
    int go_flag;
    int done_count;
    int quit_flag;

    // Workset for threads (shared queue of tiles)
    framebuffer* fb;
    const RTri* tris;
    const TileBuckets* tb_view;
    int next_tile;
    int total_tiles;
};

static inline uint32_t pack_color(vec3 c){
    int r=(int)(sf_clamp(c.x,0,1)*255.0f);
    int g=(int)(sf_clamp(c.y,0,1)*255.0f);
    int b=(int)(sf_clamp(c.z,0,1)*255.0f);
    return (uint32_t)(0xFF000000u | (uint32_t)r<<16 | (uint32_t)g<<8 | (uint32_t)b);
}

void fb_clear(framebuffer* fb, uint32_t argb, float zfar){
    (void)zfar;
    size_t n=(size_t)fb->w*fb->h;
    for(size_t i=0;i<n;i++){ fb->color[i]=argb; fb->z[i]=FLT_MAX; }
}

camera camera_make(vec3 pos, vec3 target, float fov_deg, float nearz, float farz){
    camera c={0};
    c.pos=pos;
    c.forward=v3_norm(v3_sub(target,pos));
    c.right=v3_norm(v3_cross(c.forward, v3(0,1,0)));
    c.up=v3_cross(c.right, c.forward);
    c.fov_y=sf_deg2rad(fov_deg);
    c.near=nearz; c.far=farz;
    return c;
}

static inline mat4 mul44(const mat4* a, const mat4* b){
    mat4 m=m4_ident();
    for(int r=0;r<4;r++){
        for(int col=0;col<4;col++){
            float s=0.f;
            for(int k=0;k<4;k++){
                s += a->m[r + 4*k]*b->m[k + 4*col];
            }
            m.m[r + 4*col] = s;
        }
    }
    return m;
}

mat4 camera_viewproj(const camera* c, float aspect){
    mat4 view = m4_look_at(c->pos, v3_add(c->pos,c->forward), c->up);
    mat4 proj = m4_perspective(c->fov_y, aspect, c->near, c->far);
    return mul44(&proj, &view);
}

static inline void ndc_to_screen(const framebuffer* fb, vec3* p){
    p->x = (p->x*0.5f + 0.5f) * (float)(fb->w-1);
    p->y = (1.0f - (p->y*0.5f + 0.5f)) * (float)(fb->h-1);
}

void draw_line(framebuffer* fb, int x0,int y0,int x1,int y1, uint32_t argb){
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;
    for(;;){
        if(x0>=0 && x0<fb->w && y0>=0 && y0<fb->h) fb->color[y0*fb->w + x0] = argb;
        if(x0==x1 && y0==y1) break;
        e2 = 2*err;
        if(e2 >= dy){ err += dy; x0 += sx; }
        if(e2 <= dx){ err += dx; y0 += sy; }
    }
}

static inline void draw_wire_tri(framebuffer* fb, vec3 a, vec3 b, vec3 c, vec3 col){
    uint32_t k = pack_color(col);
    draw_line(fb,(int)a.x,(int)a.y,(int)b.x,(int)b.y,k);
    draw_line(fb,(int)b.x,(int)b.y,(int)c.x,(int)c.y,k);
    draw_line(fb,(int)c.x,(int)c.y,(int)a.x,(int)a.y,k);
}

// --------- Near-plane clipper in clip space (z + w >= 0) ---------
typedef struct { vec4 clip; vec3 color; } VClip;

static inline VClip vclip_lerp(const VClip* a, const VClip* b, float t){
    VClip r;
    r.clip.x = a->clip.x + (b->clip.x - a->clip.x)*t;
    r.clip.y = a->clip.y + (b->clip.y - a->clip.y)*t;
    r.clip.z = a->clip.z + (b->clip.z - a->clip.z)*t;
    r.clip.w = a->clip.w + (b->clip.w - a->clip.w)*t;
    r.color.x = a->color.x + (b->color.x - a->color.x)*t;
    r.color.y = a->color.y + (b->color.y - a->color.y)*t;
    r.color.z = a->color.z + (b->color.z - a->color.z)*t;
    return r;
}

static int clip_against_near(const VClip* in, int n_in, VClip* out){
    int n_out=0;
    for(int i=0;i<n_in;i++){
        const VClip *A=&in[i], *B=&in[(i+1)%n_in];
        float da = A->clip.z + A->clip.w; // inside if >=0
        float db = B->clip.z + B->clip.w;
        int A_in = da >= 0.f;
        int B_in = db >= 0.f;
        if(A_in && B_in){
            out[n_out++] = *B;
        }else if(A_in && !B_in){
            float t = da / (da - db);
            out[n_out++] = vclip_lerp(A,B,t);
        }else if(!A_in && B_in){
            float t = da / (da - db);
            out[n_out++] = vclip_lerp(A,B,t);
            out[n_out++] = *B;
        }else{
            // out->out
        }
        if(n_out>=8) break;
    }
    return n_out;
}

// --------- Tile buckets ---------
static inline void buckets_init(TileBuckets* tb, int tiles_w, int tiles_h){
    int n = tiles_w*tiles_h;
    tb->tiles_w = tiles_w; tb->tiles_h = tiles_h;
    tb->counts = (int*)calloc(n, sizeof(int));
    tb->offsets = (int*)calloc(n+1, sizeof(int));
    tb->items = NULL;
    tb->items_cap = 0;
}

static inline void buckets_reset_counts(TileBuckets* tb){
    int n = tb->tiles_w*tb->tiles_h;
    memset(tb->counts, 0, sizeof(int)*n);
    memset(tb->offsets, 0, sizeof(int)*(n+1));
}

static inline void buckets_free(TileBuckets* tb){
    free(tb->counts); free(tb->offsets); free(tb->items);
    tb->counts = tb->offsets = tb->items = NULL;
    tb->items_cap = 0;
}

static inline void tri_screen_bbox(const framebuffer* fb, const RTri* t, int* x0,int* y0,int* x1,int* y1){
    vec3 p0=t->v[0].p, p1=t->v[1].p, p2=t->v[2].p;
    ndc_to_screen(fb, &p0); ndc_to_screen(fb, &p1); ndc_to_screen(fb, &p2);
    float minx=floorf(sf_min(sf_min(p0.x,p1.x),p2.x));
    float maxx=ceilf (sf_max(sf_max(p0.x,p1.x),p2.x));
    float miny=floorf(sf_min(sf_min(p0.y,p1.y),p2.y));
    float maxy=ceilf (sf_max(sf_max(p0.y,p1.y),p2.y));
    *x0=(int)sf_clamp(minx,0,fb->w-1);
    *x1=(int)sf_clamp(maxx,0,fb->w-1);
    *y0=(int)sf_clamp(miny,0,fb->h-1);
    *y1=(int)sf_clamp(maxy,0,fb->h-1);
}

static inline void raster_triangle_tile(framebuffer* fb, const RTri* t, int tx0,int ty0,int tx1,int ty1){
    vec3 p0=t->v[0].p, p1=t->v[1].p, p2=t->v[2].p;
    ndc_to_screen(fb, &p0); ndc_to_screen(fb, &p1); ndc_to_screen(fb, &p2);

    float area = (p1.x - p0.x)*(p2.y - p0.y) - (p1.y - p0.y)*(p2.x - p0.x);
    if(fabsf(area) < 1e-6f) return;
    float inv_area = 1.0f/area;

    int x0 = tx0, y0 = ty0;
    int x1 = tx1, y1 = ty1;

    for(int y=y0; y<=y1; y++){
        for(int x=x0; x<=x1; x++){
            float w0 = ((p1.x - p0.x)*((float)y - p0.y) - (p1.y - p0.y)*((float)x - p0.x))*inv_area;
            float w1 = ((p2.x - p1.x)*((float)y - p1.y) - (p2.y - p1.y)*((float)x - p1.x))*inv_area;
            float w2 = ((p0.x - p2.x)*((float)y - p2.y) - (p0.y - p2.y)*((float)x - p2.x))*inv_area;
            if(w0>=0 && w1>=0 && w2>=0){
                float wa = w0 * t->v[0].invw;
                float wb = w1 * t->v[1].invw;
                float wc = w2 * t->v[2].invw;
                float wsum = wa + wb + wc;
                if(wsum <= 1e-12f) continue;
                float z = (wa*p0.z + wb*p1.z + wc*p2.z) / wsum;

                int idx = y*fb->w + x;
                if(z < fb->z[idx]){
                    fb->z[idx] = z;
                    vec3 col = v3(
                        (wa*t->v[0].color.x + wb*t->v[1].color.x + wc*t->v[2].color.x)/wsum,
                        (wa*t->v[0].color.y + wb*t->v[1].color.y + wc*t->v[2].color.y)/wsum,
                        (wa*t->v[0].color.z + wb*t->v[1].color.z + wc*t->v[2].color.z)/wsum
                    );
                    fb->color[idx] = pack_color(col);
                }
            }
        }
    }
}

// --------- Thread pool ----------
static int worker_proc(void* user){
    render_ctx* rc = (render_ctx*)user;
    for(;;){
        SDL_LockMutex(rc->m);
        while(!rc->go_flag && !rc->quit_flag){
            SDL_CondWait(rc->cv_start, rc->m);
        }
        if(rc->quit_flag){
            SDL_UnlockMutex(rc->m);
            break;
        }
        framebuffer* fb = rc->fb;
        const RTri* tris = rc->tris;
        const TileBuckets* tb = rc->tb_view;
        SDL_UnlockMutex(rc->m);

        int tiles_w = tb->tiles_w;

        for(;;){
            SDL_LockMutex(rc->m);
            int t = rc->next_tile;
            if(t >= rc->total_tiles){
                SDL_UnlockMutex(rc->m);
                break;
            }
            rc->next_tile++;
            SDL_UnlockMutex(rc->m);

            int base = tb->offsets[t];
            int count = tb->offsets[t+1] - base;
            int tile_x = t % tiles_w;
            int tile_y = t / tiles_w;
            int x0 = tile_x * TILE_SIZE;
            int y0 = tile_y * TILE_SIZE;
            int x1 = sf_min(x0 + TILE_SIZE - 1, fb->w - 1);
            int y1 = sf_min(y0 + TILE_SIZE - 1, fb->h - 1);

            for(int i=0;i<count;i++){
                int tri_idx = tb->items[base + i];
                int bx0,by0,bx1,by1;
                tri_screen_bbox(fb, &tris[tri_idx], &bx0,&by0,&bx1,&by1);
                int ix0 = sf_max(x0, bx0), iy0 = sf_max(y0, by0);
                int ix1 = sf_min(x1, bx1), iy1 = sf_min(y1, by1);
                if(ix0<=ix1 && iy0<=iy1){
                    raster_triangle_tile(fb, &tris[tri_idx], ix0,iy0,ix1,iy1);
                }
            }
        }

        SDL_LockMutex(rc->m);
        rc->done_count++;
        if(rc->done_count == rc->threads) SDL_CondSignal(rc->cv_done);
        SDL_UnlockMutex(rc->m);
    }
    return 0;
}

bool renderer_init_ctx(render_ctx** out, int fbw, int fbh){
    render_ctx* rc = (render_ctx*)calloc(1, sizeof(render_ctx));
    if(!rc) return false;
    rc->threads = SDL_GetCPUCount(); if(rc->threads<1) rc->threads=1; if(rc->threads>8) rc->threads=8;
    rc->th = (SDL_Thread**)calloc(rc->threads, sizeof(SDL_Thread*));
    rc->m = SDL_CreateMutex();
    rc->cv_start = SDL_CreateCond();
    rc->cv_done  = SDL_CreateCond();
    buckets_init(&rc->tb, (fbw + TILE_SIZE - 1)/TILE_SIZE, (fbh + TILE_SIZE - 1)/TILE_SIZE);

    for(int i=0;i<rc->threads;i++){
        rc->th[i] = SDL_CreateThread(worker_proc, "raster_worker", rc);
        if(!rc->th[i]){
            LOGW("Failed to start worker %d, continuing single-threaded.", i);
            rc->threads = i;
            break;
        }
    }
    *out = rc;
    return true;
}

void renderer_resize_ctx(render_ctx* rc, int fbw, int fbh){
    int tw = (fbw + TILE_SIZE - 1)/TILE_SIZE;
    int th = (fbh + TILE_SIZE - 1)/TILE_SIZE;
    if(tw!=rc->tb.tiles_w || th!=rc->tb.tiles_h){
        buckets_free(&rc->tb);
        buckets_init(&rc->tb, tw, th);
    }
}

void renderer_shutdown_ctx(render_ctx* rc){
    if(!rc) return;
    SDL_LockMutex(rc->m);
    rc->quit_flag = 1;
    SDL_CondBroadcast(rc->cv_start);
    SDL_UnlockMutex(rc->m);
    for(int i=0;i<rc->threads;i++){
        if(rc->th[i]) SDL_WaitThread(rc->th[i], NULL);
    }
    SDL_DestroyCond(rc->cv_start);
    SDL_DestroyCond(rc->cv_done);
    SDL_DestroyMutex(rc->m);
    buckets_free(&rc->tb);
    free(rc->rt_tris);
    free(rc->th);
    free(rc);
}

static inline void push_rtri(render_ctx* rc, const RTri* rt){
    if(rc->rt_count+1 >= rc->rt_cap){
        rc->rt_cap = rc->rt_cap ? rc->rt_cap*2 : 1024;
        rc->rt_tris = (RTri*)realloc(rc->rt_tris, sizeof(RTri)*rc->rt_cap);
    }
    rc->rt_tris[rc->rt_count++] = *rt;
}

void draw_triangles_ctx(render_ctx* rc, framebuffer* fb, const tri* tris, int count, const mat4* world, const mat4* vp){
    // Build clip -> NDC -> RTri
    rc->rt_count = 0;

    for(int i=0;i<count;i++){
        tri t = tris[i];

        vec3 wp[3];
        for(int k=0;k<3;k++) wp[k] = m4_mul_pos(*world, t.v[k].p);

        VClip in[3];
        for(int k=0;k<3;k++){ vec4 cp = m4_mul_pos4(*vp, wp[k]); in[k].clip=cp; in[k].color=t.v[k].color; }

        VClip poly[8];
        int n = clip_against_near(in, 3, poly);
        if(n < 3) continue;

        for(int k=1;k+1<n;k++){
            VClip a=poly[0], b=poly[k], c=poly[k+1];
            RTri rt;
            rt.v[0].invw = 1.0f / a.clip.w; rt.v[0].p = v3(a.clip.x*rt.v[0].invw, a.clip.y*rt.v[0].invw, a.clip.z*rt.v[0].invw); rt.v[0].color=a.color;
            rt.v[1].invw = 1.0f / b.clip.w; rt.v[1].p = v3(b.clip.x*rt.v[1].invw, b.clip.y*rt.v[1].invw, b.clip.z*rt.v[1].invw); rt.v[1].color=b.color;
            rt.v[2].invw = 1.0f / c.clip.w; rt.v[2].p = v3(c.clip.x*rt.v[2].invw, c.clip.y*rt.v[2].invw, c.clip.z*rt.v[2].invw); rt.v[2].color=c.color;
            float area = (rt.v[1].p.x - rt.v[0].p.x)*(rt.v[2].p.y - rt.v[0].p.y) - (rt.v[1].p.y - rt.v[0].p.y)*(rt.v[2].p.x - rt.v[0].p.x);
            if(area <= 0.f) continue;
            push_rtri(rc, &rt);
        }
    }

    // Binning
    renderer_resize_ctx(rc, fb->w, fb->h);
    TileBuckets* tb = &rc->tb;
    buckets_reset_counts(tb);

    for(int i=0;i<rc->rt_count;i++){
        int x0,y0,x1,y1; tri_screen_bbox(fb, &rc->rt_tris[i], &x0,&y0,&x1,&y1);
        int tx0 = x0 / TILE_SIZE, ty0 = y0 / TILE_SIZE;
        int tx1 = x1 / TILE_SIZE, ty1 = y1 / TILE_SIZE;
        for(int ty=ty0; ty<=ty1; ty++){
            for(int tx=tx0; tx<=tx1; tx++){
                tb->counts[ty*tb->tiles_w + tx]++;
            }
        }
    }

    int ntiles = tb->tiles_w*tb->tiles_h;
    int total = 0;
    for(int t=0;t<ntiles;t++){
        tb->offsets[t] = total;
        total += tb->counts[t];
    }
    tb->offsets[ntiles] = total;
    if(total > tb->items_cap){
        free(tb->items);
        tb->items = (int*)malloc(sizeof(int)*total);
        tb->items_cap = total;
    }
    memset(tb->counts, 0, sizeof(int)*ntiles);

    for(int i=0;i<rc->rt_count;i++){
        int x0,y0,x1,y1; tri_screen_bbox(fb, &rc->rt_tris[i], &x0,&y0,&x1,&y1);
        int tx0 = x0 / TILE_SIZE, ty0 = y0 / TILE_SIZE;
        int tx1 = x1 / TILE_SIZE, ty1 = y1 / TILE_SIZE;
        for(int ty=ty0; ty<=ty1; ty++){
            for(int tx=tx0; tx<=tx1; tx++){
                int t = ty*tb->tiles_w + tx;
                int w = tb->offsets[t] + tb->counts[t]++;
                tb->items[w] = i;
            }
        }
    }

    
    if(rc->threads == 0){
        // Serial fallback
        int tiles_w = tb->tiles_w;
        for(int t=0;t<ntiles;t++){
            int base = tb->offsets[t];
            int count = tb->offsets[t+1] - base;
            int tile_x = t % tiles_w;
            int tile_y = t / tiles_w;
            int x0 = tile_x * TILE_SIZE;
            int y0 = tile_y * TILE_SIZE;
            int x1 = sf_min(x0 + TILE_SIZE - 1, fb->w - 1);
            int y1 = sf_min(y0 + TILE_SIZE - 1, fb->h - 1);
            for(int i=0;i<count;i++){
                int tri_idx = tb->items[base + i];
                int bx0,by0,bx1,by1;
                tri_screen_bbox(fb, &rc->rt_tris[tri_idx], &bx0,&by0,&bx1,&by1);
                int ix0 = sf_max(x0, bx0), iy0 = sf_max(y0, by0);
                int ix1 = sf_min(x1, bx1), iy1 = sf_min(y1, by1);
                if(ix0<=ix1 && iy0<=iy1){
                    raster_triangle_tile(fb, &rc->rt_tris[tri_idx], ix0,iy0,ix1,iy1);
                }
            }
        }
        return;
    }

    // Dispatch to persistent workers
    SDL_LockMutex(rc->m);
    rc->fb = fb;
    rc->tris = rc->rt_tris;
    rc->tb_view = tb;
    rc->done_count = 0;
    rc->next_tile = 0;
    rc->total_tiles = ntiles;
    rc->go_flag = 1;
    SDL_CondBroadcast(rc->cv_start);
    SDL_UnlockMutex(rc->m);

    // Wait for completion
    SDL_LockMutex(rc->m);
    while(rc->done_count < rc->threads){
        SDL_CondWait(rc->cv_done, rc->m);
    }
    rc->go_flag = 0;
    SDL_UnlockMutex(rc->m);
}

static inline void draw_rect(framebuffer* fb, int x0,int y0,int x1,int y1, uint32_t argb){
    for(int y=y0;y<=y1;y++){
        if(y<0||y>=fb->h) continue;
        for(int x=x0;x<=x1;x++){
            if(x<0||x>=fb->w) continue;
            fb->color[y*fb->w+x]=argb;
        }
    }
}

void draw_hud(framebuffer* fb, float airspeed, float altitude, float pitch_rad, float roll_rad, float throttle){
    int w=fb->w, h=fb->h;
    int boxw=160, boxh=60, pad=8;
    draw_rect(fb, 0,0, w-1, 40, 0x40000000u);

    int tw=(int)(sf_clamp(throttle,0,1)*(w-2*pad));
    draw_rect(fb, pad, pad, pad+tw, 30, 0x80FFFF00u);

    int yc = h/2 + (int)(pitch_rad * 180.0f / M_PI);
    if(yc>=0 && yc<h) draw_line(fb, 0, yc, w-1, yc, 0x80FFFFFFu);

    float rlen=50.f;
    float rx = w*0.5f + rlen*sinf(roll_rad);
    float ry = 20.f + rlen*-cosf(roll_rad);
    draw_line(fb, (int)(w*0.5f), 20, (int)rx, (int)ry, 0x80FFFFFFu);

    int asbar=(int)(sf_clamp(airspeed/100.0f,0,1)*boxh);
    draw_rect(fb, pad, h-boxh-pad+ (boxh-asbar), pad+20, h-pad, 0x8000FF00u);
    int altbar=(int)(sf_clamp(altitude/10000.0f,0,1)*boxh);
    draw_rect(fb, w-pad-20, h-boxh-pad+ (boxh-altbar), w-pad, h-pad, 0x8000FFFFu);
}
