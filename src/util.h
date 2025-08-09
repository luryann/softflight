#ifndef SOFTFLIGHT_UTIL_H
#define SOFTFLIGHT_UTIL_H
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

typedef struct {
    uint8_t* base;
    size_t   cap;
    size_t   off;
} arena;

static inline arena arena_make(void* mem, size_t cap){ arena a={(uint8_t*)mem,cap,0}; return a; }
static inline void* arena_alloc(arena* a, size_t sz, size_t align){
    size_t m=(a->off + (align-1)) & ~(align-1);
    if(m + sz > a->cap) return NULL;
    void* p=a->base + m; a->off = m+sz; return p;
}
static inline void arena_reset(arena* a){ a->off=0; }

typedef struct {
    double dt;
    double now;
} timer_info;

#endif
