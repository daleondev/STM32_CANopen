#ifndef LELY_THREADX_THREADS_H_
#define LELY_THREADX_THREADS_H_

#include <time.h>

#include "tx_api.h"

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef thread_local
#if __cplusplus >= 201103L
// thread_local is a keyword in C++11 and later.
#else
#define thread_local _Thread_local
#endif
#endif

    typedef struct
    {
        TX_MUTEX mutex;
        int type;
    } mtx_t;

    typedef struct
    {
        TX_MUTEX mutex;
        TX_SEMAPHORE sem;
        unsigned long waiters;
    } cnd_t;

    struct lely_threadx_thread;
    typedef struct lely_threadx_thread *thrd_t;
    typedef unsigned int tss_t;

    typedef struct
    {
        volatile unsigned long state;
    } once_flag;

#define ONCE_FLAG_INIT {0}
#define TSS_DTOR_ITERATIONS 1

    enum
    {
        mtx_plain = 0,
        mtx_recursive = 2,
        mtx_timed = 1
    };

    enum
    {
        thrd_success,
        thrd_error,
        thrd_timedout,
        thrd_busy,
        thrd_nomem
    };

    typedef void (*tss_dtor_t)(void *);
    typedef int (*thrd_start_t)(void *);

    void call_once(once_flag *flag, void (*func)(void));

    int cnd_broadcast(cnd_t *cond);
    void cnd_destroy(cnd_t *cond);
    int cnd_init(cnd_t *cond);
    int cnd_signal(cnd_t *cond);
    int cnd_timedwait(cnd_t *cond, mtx_t *mtx, const struct timespec *ts);
    int cnd_wait(cnd_t *cond, mtx_t *mtx);

    void mtx_destroy(mtx_t *mtx);
    int mtx_init(mtx_t *mtx, int type);
    int mtx_lock(mtx_t *mtx);
    int mtx_timedlock(mtx_t *mtx, const struct timespec *ts);
    int mtx_trylock(mtx_t *mtx);
    int mtx_unlock(mtx_t *mtx);

    int thrd_create(thrd_t *thr, thrd_start_t func, void *arg);
    thrd_t thrd_current(void);
    int thrd_detach(thrd_t thr);
    int thrd_equal(thrd_t thr0, thrd_t thr1);
    void thrd_exit(int res);
    int thrd_join(thrd_t thr, int *res);
    int thrd_sleep(const struct timespec *duration, struct timespec *remaining);
    void thrd_yield(void);

    int tss_create(tss_t *key, tss_dtor_t dtor);
    void tss_delete(tss_t key);
    void *tss_get(tss_t key);
    int tss_set(tss_t key, void *val);

#ifdef __cplusplus
}
#endif

#endif // LELY_THREADX_THREADS_H_