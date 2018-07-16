#ifndef __UTILS_H__
#define __UTILS_H__

#include <stdint.h>
#include <sys/time.h>
#include <time.h>
#include <math.h>

#define DEG2mRAD(X) (X*M_PI*1e5)/180
#define DEG2RAD(X)  (X*M_PI)/180

#define mRAD2DEG(X) (X*180)/(M_PI*1e5)
#define mRAD2RAD(X) (X/1e5)
#define RAD2mRAD(X) ((X)*1e5)

#define V2mV(X) (X*1000/2)

#define NSEC_PER_SEC	1000000000


#ifdef __XENO__
    #include <rtdk.h>
    #define DPRINTF rt_printf
#else
    #include <stdio.h>
    #define DPRINTF printf
#endif


inline uint64_t get_time_ns(void)
{
    uint64_t time_ns;
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    time_ns = ts.tv_sec * 1000000000ULL + ts.tv_nsec;
    return time_ns;
}

inline void tsnorm(struct timespec *ts)
{
	while (ts->tv_nsec >= NSEC_PER_SEC) {
		ts->tv_nsec -= NSEC_PER_SEC;
		ts->tv_sec++;
	}
}



#endif
