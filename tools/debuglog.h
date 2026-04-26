/* Stub for XTulator debuglog.h */
#ifndef _DEBUGLOG_H_
#define _DEBUGLOG_H_
#define DEBUG_INFO  0
#define DEBUG_DETAIL 1
#include <stdio.h>
static inline void debug_log(int level, const char *fmt, ...) {
    (void)level; (void)fmt;
}
#endif
