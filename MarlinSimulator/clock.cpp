#include "clock.h"

#ifdef __WIN32__
#include <Windows.h>
#else
#include <time.h>
#endif//__WIN32__

uint64_t getMilliseconds()
{
#ifdef __WIN32__
    return GetTickCount();
#else
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t)t.tv_sec * 1000LL + (uint64_t)t.tv_nsec / 1000000LL;
#endif
}
