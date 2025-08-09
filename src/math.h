#ifndef SOFTFLIGHT_MATH_H
#define SOFTFLIGHT_MATH_H

#include <math.h>
#include <float.h>
#include <stdint.h>
#include <stdbool.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define sf_min(a,b) ((a) < (b) ? (a) : (b))
#define sf_max(a,b) ((a) > (b) ? (a) : (b))
#define sf_clamp(x,a,b) ( ((x) < (a)) ? (a) : ((x) > (b)) ? (b) : (x) )
#define sf_deg2rad(d) ((d) * (float)(M_PI/180.0))
#define sf_rad2deg(r) ((r) * (float)(180.0/M_PI))
#define sf_likely(x)   __builtin_expect(!!(x), 1)
#define sf_unlikely(x) __builtin_expect(!!(x), 0)

typedef struct { float x,y; } vec2;
typedef struct { float x,y,z; } vec3;
typedef struct { float x,y,z,w; } vec4;
typedef struct { float x,y,z,w; } quat;
typedef struct { float m[16]; } mat4; // column-major

static inline vec3 v3(float x,float y,float z){ vec3 v={x,y,z}; return v; }
static inline vec2 v2(float x,float y){ vec2 v={x,y}; return v; }

static inline vec3 v3_add(vec3 a, vec3 b){ return v3(a.x+b.x,a.y+b.y,a.z+b.z); }
static inline vec3 v3_sub(vec3 a, vec3 b){ return v3(a.x-b.x,a.y-b.y,a.z-b.z); }
static inline vec3 v3_scale(vec3 a, float s){ return v3(a.x*s,a.y*s,a.z*s); }
static inline float v3_dot(vec3 a, vec3 b){ return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline vec3 v3_cross(vec3 a, vec3 b){
    return v3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}
static inline float v3_len(vec3 a){ return sqrtf(v3_dot(a,a)); }
static inline vec3 v3_norm(vec3 a){ float L=v3_len(a); return (L>1e-8f)? v3_scale(a,1.f/L):v3(0,0,0); }
static inline vec3 v3_lerp(vec3 a, vec3 b, float t){ return v3(a.x+(b.x-a.x)*t, a.y+(b.y-a.y)*t, a.z+(b.z-a.z)*t); }

static inline quat q_ident(){ quat q={0,0,0,1}; return q; }
static inline quat q(float x,float y,float z,float w){ quat r={x,y,z,w}; return r; }
static inline quat q_mul(quat a, quat b){
    return q(
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z
    );
}
static inline quat q_from_axis_angle(vec3 axis, float rad){
    vec3 n=v3_norm(axis); float s=sinf(rad*0.5f); return q(n.x*s,n.y*s,n.z*s,cosf(rad*0.5f));
}
static inline quat q_from_euler(float roll,float pitch,float yaw){
    // ZYX order: yaw (Z), pitch (Y), roll (X)
    float cr=cosf(roll*0.5f), sr=sinf(roll*0.5f);
    float cp=cosf(pitch*0.5f), sp=sinf(pitch*0.5f);
    float cy=cosf(yaw*0.5f), sy=sinf(yaw*0.5f);
    return q(
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
        cr*cp*cy + sr*sp*sy
    );
}
static inline quat q_norm(quat a){
    float n=sqrtf(a.x*a.x+a.y*a.y+a.z*a.z+a.w*a.w);
    if(n<1e-12f) return q_ident();
    float s=1.f/n;
    return q(a.x*s,a.y*s,a.z*s,a.w*s);
}
static inline vec3 q_rotate(quat q, vec3 v){
    // v' = q * (v,0) * conj(q)
    quat p={v.x,v.y,v.z,0};
    quat qc={-q.x,-q.y,-q.z,q.w};
    quat r=q_mul(q_mul(q,p),qc);
    return v3(r.x,r.y,r.z);
}

static inline mat4 m4_ident(){ mat4 m={{1,0,0,0,   0,1,0,0,   0,0,1,0,   0,0,0,1}}; return m; }
static inline mat4 m4_perspective(float fov_y_rad, float aspect, float near, float far){
    float f=1.0f/tanf(fov_y_rad*0.5f);
    float A=(far+near)/(near-far);
    float B=(2*far*near)/(near-far);
    mat4 m={{f/aspect,0,0,0,  0,f,0,0,  0,0,A,-1,  0,0,B,0}};
    return m;
}
static inline mat4 m4_look_at(vec3 eye, vec3 center, vec3 up){
    vec3 f=v3_norm(v3_sub(center,eye));
    vec3 s=v3_norm(v3_cross(f, up));
    vec3 u=v3_cross(s, f);
    mat4 m={{ s.x, u.x, -f.x, 0,
              s.y, u.y, -f.y, 0,
              s.z, u.z, -f.z, 0,
              -v3_dot(s,eye), -v3_dot(u,eye), v3_dot(f,eye), 1}};
    return m;
}
static inline vec3 m4_mul_pos(mat4 m, vec3 v){
    float x=v.x*m.m[0]+v.y*m.m[4]+v.z*m.m[8]+m.m[12];
    float y=v.x*m.m[1]+v.y*m.m[5]+v.z*m.m[9]+m.m[13];
    float z=v.x*m.m[2]+v.y*m.m[6]+v.z*m.m[10]+m.m[14];
    float w=v.x*m.m[3]+v.y*m.m[7]+v.z*m.m[11]+m.m[15];
    if(fabsf(w) < 1e-8f) w = 1e-8f;
    return v3(x/w,y/w,z/w);
}
static inline vec3 m4_mul_dir(mat4 m, vec3 v){
    float x=v.x*m.m[0]+v.y*m.m[4]+v.z*m.m[8];
    float y=v.x*m.m[1]+v.y*m.m[5]+v.z*m.m[9];
    float z=v.x*m.m[2]+v.y*m.m[6]+v.z*m.m[10];
    return v3(x,y,z);
}


static inline vec4 m4_mul_pos4(mat4 m, vec3 v){
    float x=v.x*m.m[0]+v.y*m.m[4]+v.z*m.m[8]+m.m[12];
    float y=v.x*m.m[1]+v.y*m.m[5]+v.z*m.m[9]+m.m[13];
    float z=v.x*m.m[2]+v.y*m.m[6]+v.z*m.m[10]+m.m[14];
    float w=v.x*m.m[3]+v.y*m.m[7]+v.z*m.m[11]+m.m[15];
    vec4 r={x,y,z,w}; return r;
}
typedef struct { vec3 min,max; } aabb3;
static inline aabb3 aabb3_from_center(vec3 c, vec3 r){ aabb3 b={v3(c.x-r.x,c.y-r.y,c.z-r.z), v3(c.x+r.x,c.y+r.y,c.z+r.z)}; return b; }

#endif // SOFTFLIGHT_MATH_H
