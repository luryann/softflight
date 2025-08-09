#ifndef SOFTFLIGHT_LOG_H
#define SOFTFLIGHT_LOG_H
#include <stdio.h>
#include <stdarg.h>

static inline void sf_log(const char* lvl, const char* fmt, ...){
    va_list ap; va_start(ap, fmt);
    fprintf(stderr, "[%s] ", lvl);
    vfprintf(stderr, fmt, ap);
    fprintf(stderr, "\n");
    va_end(ap);
}
#define LOGI(...) sf_log("INFO", __VA_ARGS__)
#define LOGW(...) sf_log("WARN", __VA_ARGS__)
#define LOGE(...) sf_log("ERR", __VA_ARGS__)
#endif
