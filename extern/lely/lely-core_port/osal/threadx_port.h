#ifndef LELY_THREADX_PORT_H_
#define LELY_THREADX_PORT_H_

#include <limits.h>
#include <time.h>

#include "tx_api.h"

#ifndef CLOCK_REALTIME
#define CLOCK_REALTIME 0
#endif

#ifndef CLOCK_MONOTONIC
#define CLOCK_MONOTONIC 1
#endif

#ifndef TIMER_ABSTIME
#define TIMER_ABSTIME 1
#endif

#ifndef TIME_UTC
#define TIME_UTC 1
#endif

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef LELY_THREADX_STACK_SIZE
#define LELY_THREADX_STACK_SIZE 16384u
#endif

#ifndef LELY_THREADX_MAX_TSS_KEYS
#define LELY_THREADX_MAX_TSS_KEYS 16u
#endif

#define LELY_THREADX_NSEC_PER_SEC 1000000000ll

    long long lely_threadx_timespec_to_ns(const struct timespec *ts);
    struct timespec lely_threadx_ns_to_timespec(long long ns);
    ULONG lely_threadx_duration_to_ticks(const struct timespec *duration);
    ULONG lely_threadx_abs_timeout_to_ticks(const struct timespec *abs_time,
                                            int *timedout);
    int lely_threadx_clock_now(clockid_t clock_id, struct timespec *tp);

#ifdef __cplusplus
}
#endif

#endif // LELY_THREADX_PORT_H_