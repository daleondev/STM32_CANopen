#include "threadx_port.h"

#include <errno.h>
#include <stddef.h>

static long long g_realtime_offset_ns = 0;

static long long lely_threadx_monotonic_ns(void)
{
    const unsigned long long ticks = tx_time_get();
    return (long long)((ticks * LELY_THREADX_NSEC_PER_SEC) / TX_TIMER_TICKS_PER_SECOND);
}

long long lely_threadx_timespec_to_ns(const struct timespec *ts)
{
    if (!ts)
        return 0;

    return (long long)ts->tv_sec * LELY_THREADX_NSEC_PER_SEC + ts->tv_nsec;
}

struct timespec lely_threadx_ns_to_timespec(long long ns)
{
    struct timespec ts = {0, 0};

    if (ns <= 0)
        return ts;

    ts.tv_sec = (time_t)(ns / LELY_THREADX_NSEC_PER_SEC);
    ts.tv_nsec = (long)(ns % LELY_THREADX_NSEC_PER_SEC);
    return ts;
}

ULONG lely_threadx_duration_to_ticks(const struct timespec *duration)
{
    const long long ns = lely_threadx_timespec_to_ns(duration);
    unsigned long long ticks;

    if (ns <= 0)
        return 0;

    ticks = ((unsigned long long)ns * TX_TIMER_TICKS_PER_SECOND + (LELY_THREADX_NSEC_PER_SEC - 1)) / LELY_THREADX_NSEC_PER_SEC;

    if (ticks == 0)
        ticks = 1;

    if (ticks >= TX_WAIT_FOREVER)
        ticks = TX_WAIT_FOREVER - 1;

    return (ULONG)ticks;
}

int lely_threadx_clock_now(clockid_t clock_id, struct timespec *tp)
{
    long long ns;

    if (!tp)
        return 0;

    if (clock_id == CLOCK_REALTIME)
    {
        ns = lely_threadx_monotonic_ns() + g_realtime_offset_ns;
        *tp = lely_threadx_ns_to_timespec(ns);
        return 0;
    }

    if (clock_id == CLOCK_MONOTONIC)
    {
        *tp = lely_threadx_ns_to_timespec(lely_threadx_monotonic_ns());
        return 0;
    }

    errno = EINVAL;
    return -1;
}

ULONG lely_threadx_abs_timeout_to_ticks(const struct timespec *abs_time,
                                        int *timedout)
{
    struct timespec now = {0, 0};
    long long remaining_ns;

    if (timedout)
        *timedout = 0;

    if (!abs_time)
        return TX_WAIT_FOREVER;

    if (lely_threadx_clock_now(CLOCK_REALTIME, &now) == -1)
    {
        if (timedout)
            *timedout = 1;
        return 0;
    }

    remaining_ns = lely_threadx_timespec_to_ns(abs_time) - lely_threadx_timespec_to_ns(&now);

    if (remaining_ns <= 0)
    {
        if (timedout)
            *timedout = 1;
        return 0;
    }

    return lely_threadx_duration_to_ticks(
        &(struct timespec){
            .tv_sec = (time_t)(remaining_ns / LELY_THREADX_NSEC_PER_SEC),
            .tv_nsec = (long)(remaining_ns % LELY_THREADX_NSEC_PER_SEC)});
}

int clock_getres(clockid_t clock_id, struct timespec *res)
{
    (void)clock_id;

    if (res)
    {
        res->tv_sec = 0;
        res->tv_nsec = (long)(LELY_THREADX_NSEC_PER_SEC / TX_TIMER_TICKS_PER_SECOND);
    }

    return 0;
}

int clock_gettime(clockid_t clock_id, struct timespec *tp)
{
    return lely_threadx_clock_now(clock_id, tp);
}

int clock_settime(clockid_t clock_id, const struct timespec *tp)
{
    struct timespec now = {0, 0};

    if (clock_id != CLOCK_REALTIME || !tp)
    {
        errno = EINVAL;
        return -1;
    }

    if (lely_threadx_clock_now(CLOCK_MONOTONIC, &now) == -1)
        return -1;

    g_realtime_offset_ns = lely_threadx_timespec_to_ns(tp) - lely_threadx_timespec_to_ns(&now);
    return 0;
}

int clock_nanosleep(clockid_t clock_id, int flags, const struct timespec *rqtp,
                    struct timespec *rmtp)
{
    struct timespec now = {0, 0};
    struct timespec duration = {0, 0};
    long long remaining_ns;
    ULONG ticks;

    if (!rqtp || rqtp->tv_nsec < 0 || rqtp->tv_nsec >= LELY_THREADX_NSEC_PER_SEC)
        return EINVAL;

    if (clock_id != CLOCK_REALTIME && clock_id != CLOCK_MONOTONIC)
        return EINVAL;

    if (flags & TIMER_ABSTIME)
    {
        if (lely_threadx_clock_now(clock_id, &now) == -1)
            return errno;

        remaining_ns = lely_threadx_timespec_to_ns(rqtp) - lely_threadx_timespec_to_ns(&now);
        if (remaining_ns <= 0)
            return 0;

        duration = lely_threadx_ns_to_timespec(remaining_ns);
    }
    else
    {
        duration = *rqtp;
    }

    ticks = lely_threadx_duration_to_ticks(&duration);
    while (ticks > 0)
    {
        ULONG chunk = ticks;
        if (chunk >= TX_WAIT_FOREVER)
            chunk = TX_WAIT_FOREVER - 1;
        tx_thread_sleep(chunk);
        ticks -= chunk;
    }

    if (rmtp)
    {
        rmtp->tv_sec = 0;
        rmtp->tv_nsec = 0;
    }

    return 0;
}

int nanosleep(const struct timespec *rqtp, struct timespec *rmtp)
{
    const int errsv = clock_nanosleep(CLOCK_REALTIME, 0, rqtp, rmtp);
    if (errsv)
    {
        errno = errsv;
        return -1;
    }

    return 0;
}

#ifndef LELY_HAVE_TIMESPEC_GET
int timespec_get(struct timespec *ts, int base)
{
    if (base != TIME_UTC || clock_gettime(CLOCK_REALTIME, ts) == -1)
        return 0;
    return base;
}
#endif