#pragma once

#include <stdarg.h>
#include <stdio.h>

inline int printfAsync(const char *fmt, ...) {
    int len;

    va_list ap;
    va_start(ap, fmt);

    len = printf(fmt, ap);

    va_end(ap);
    return len;
};

