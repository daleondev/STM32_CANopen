#include "threadx_port.h"

#include <threads.h>

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct lely_threadx_thread
{
    TX_THREAD thread;
    TX_SEMAPHORE done;
    thrd_start_t func;
    void *arg;
    void *stack;
    unsigned int stack_size;
    int result;
    unsigned int detached;
    unsigned int completed;
    char name[24];
};

static thread_local thrd_t g_current_thread = NULL;
static thread_local void *g_tss_values[LELY_THREADX_MAX_TSS_KEYS];

static unsigned int g_tss_count = 0;
static unsigned long g_thread_count = 0;
static TX_MUTEX g_once_mutex;
static unsigned int g_once_mutex_ready = 0;

static void lely_threadx_cleanup_thread(thrd_t thr)
{
    if (!thr)
        return;

    tx_thread_delete(&thr->thread);
    tx_semaphore_delete(&thr->done);
    free(thr->stack);
    free(thr);
}

static void lely_threadx_ensure_once_mutex(void)
{
    if (!g_once_mutex_ready)
    {
        tx_mutex_create(&g_once_mutex, "lely_once", TX_NO_INHERIT);
        g_once_mutex_ready = 1;
    }
}

static VOID lely_threadx_thread_entry(ULONG arg)
{
    thrd_t thr = (thrd_t)(uintptr_t)arg;

    g_current_thread = thr;
    thr->result = thr->func(thr->arg);
    thr->completed = 1;
    tx_semaphore_put(&thr->done);
}

void call_once(once_flag *flag, void (*func)(void))
{
    if (!flag || !func)
        return;

    if (flag->state == 2)
        return;

    lely_threadx_ensure_once_mutex();
    tx_mutex_get(&g_once_mutex, TX_WAIT_FOREVER);

    if (flag->state == 0)
    {
        flag->state = 1;
        tx_mutex_put(&g_once_mutex);

        func();

        tx_mutex_get(&g_once_mutex, TX_WAIT_FOREVER);
        flag->state = 2;
    }

    tx_mutex_put(&g_once_mutex);

    while (flag->state != 2)
        tx_thread_relinquish();
}

int cnd_broadcast(cnd_t *cond)
{
    unsigned long waiters;

    if (!cond)
        return thrd_error;

    tx_mutex_get(&cond->mutex, TX_WAIT_FOREVER);
    waiters = cond->waiters;
    cond->waiters = 0;
    tx_mutex_put(&cond->mutex);

    while (waiters--)
        tx_semaphore_put(&cond->sem);

    return thrd_success;
}

void cnd_destroy(cnd_t *cond)
{
    if (!cond)
        return;

    tx_semaphore_delete(&cond->sem);
    tx_mutex_delete(&cond->mutex);
}

int cnd_init(cnd_t *cond)
{
    if (!cond)
        return thrd_error;

    memset(cond, 0, sizeof(*cond));

    if (tx_mutex_create(&cond->mutex, "lely_cnd", TX_NO_INHERIT) != TX_SUCCESS)
        return thrd_error;

    if (tx_semaphore_create(&cond->sem, "lely_cnd_sem", 0) != TX_SUCCESS)
    {
        tx_mutex_delete(&cond->mutex);
        return thrd_error;
    }

    return thrd_success;
}

int cnd_signal(cnd_t *cond)
{
    if (!cond)
        return thrd_error;

    tx_mutex_get(&cond->mutex, TX_WAIT_FOREVER);
    if (cond->waiters > 0)
    {
        cond->waiters--;
        tx_semaphore_put(&cond->sem);
    }
    tx_mutex_put(&cond->mutex);

    return thrd_success;
}

int cnd_timedwait(cnd_t *cond, mtx_t *mtx, const struct timespec *ts)
{
    UINT status;
    int timedout = 0;
    ULONG ticks;

    if (!cond || !mtx || !ts)
        return thrd_error;

    ticks = lely_threadx_abs_timeout_to_ticks(ts, &timedout);
    if (timedout)
        return thrd_timedout;

    tx_mutex_get(&cond->mutex, TX_WAIT_FOREVER);
    cond->waiters++;
    tx_mutex_put(&cond->mutex);

    mtx_unlock(mtx);
    status = tx_semaphore_get(&cond->sem, ticks);
    mtx_lock(mtx);

    if (status == TX_SUCCESS)
        return thrd_success;

    tx_mutex_get(&cond->mutex, TX_WAIT_FOREVER);
    if (cond->waiters > 0)
        cond->waiters--;
    tx_mutex_put(&cond->mutex);

    return status == TX_NO_INSTANCE ? thrd_timedout : thrd_error;
}

int cnd_wait(cnd_t *cond, mtx_t *mtx)
{
    UINT status;

    if (!cond || !mtx)
        return thrd_error;

    tx_mutex_get(&cond->mutex, TX_WAIT_FOREVER);
    cond->waiters++;
    tx_mutex_put(&cond->mutex);

    mtx_unlock(mtx);
    status = tx_semaphore_get(&cond->sem, TX_WAIT_FOREVER);
    mtx_lock(mtx);

    return status == TX_SUCCESS ? thrd_success : thrd_error;
}

void mtx_destroy(mtx_t *mtx)
{
    if (mtx)
        tx_mutex_delete(&mtx->mutex);
}

int mtx_init(mtx_t *mtx, int type)
{
    if (!mtx)
        return thrd_error;

    memset(mtx, 0, sizeof(*mtx));
    mtx->type = type;

    return tx_mutex_create(&mtx->mutex, "lely_mtx", TX_NO_INHERIT) == TX_SUCCESS
               ? thrd_success
               : thrd_error;
}

int mtx_lock(mtx_t *mtx)
{
    if (!mtx)
        return thrd_error;

    return tx_mutex_get(&mtx->mutex, TX_WAIT_FOREVER) == TX_SUCCESS ? thrd_success
                                                                    : thrd_error;
}

int mtx_timedlock(mtx_t *mtx, const struct timespec *ts)
{
    int timedout = 0;

    if (!mtx || !ts)
        return thrd_error;

    if (tx_mutex_get(&mtx->mutex,
                     lely_threadx_abs_timeout_to_ticks(ts, &timedout)) == TX_SUCCESS)
    {
        return thrd_success;
    }

    return timedout ? thrd_timedout : thrd_error;
}

int mtx_trylock(mtx_t *mtx)
{
    UINT status;

    if (!mtx)
        return thrd_error;

    status = tx_mutex_get(&mtx->mutex, TX_NO_WAIT);
    if (status == TX_SUCCESS)
        return thrd_success;
    if (status == TX_NOT_AVAILABLE)
        return thrd_busy;
    return thrd_error;
}

int mtx_unlock(mtx_t *mtx)
{
    if (!mtx)
        return thrd_error;

    return tx_mutex_put(&mtx->mutex) == TX_SUCCESS ? thrd_success : thrd_error;
}

int thrd_create(thrd_t *thr, thrd_start_t func, void *arg)
{
    thrd_t thread;
    UINT status;

    if (!thr || !func)
        return thrd_error;

    thread = calloc(1, sizeof(*thread));
    if (!thread)
        return thrd_nomem;

    thread->stack_size = (unsigned int)LELY_THREADX_STACK_SIZE;
    thread->stack = malloc(thread->stack_size);
    if (!thread->stack)
    {
        free(thread);
        return thrd_nomem;
    }

    thread->func = func;
    thread->arg = arg;

    if (tx_semaphore_create(&thread->done, "lely_thrd_done", 0) != TX_SUCCESS)
    {
        free(thread->stack);
        free(thread);
        return thrd_error;
    }

    snprintf(thread->name, sizeof(thread->name), "lely_%lu",
             (unsigned long)++g_thread_count);

    status = tx_thread_create(&thread->thread, thread->name,
                              lely_threadx_thread_entry, (ULONG)(uintptr_t)thread, thread->stack,
                              thread->stack_size, 15, 15, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        tx_semaphore_delete(&thread->done);
        free(thread->stack);
        free(thread);
        return thrd_error;
    }

    *thr = thread;
    return thrd_success;
}

thrd_t thrd_current(void)
{
    return g_current_thread;
}

int thrd_detach(thrd_t thr)
{
    if (!thr)
        return thrd_error;

    thr->detached = 1;
    if (thr->completed)
        lely_threadx_cleanup_thread(thr);

    return thrd_success;
}

int thrd_equal(thrd_t thr0, thrd_t thr1)
{
    return thr0 == thr1;
}

void thrd_exit(int res)
{
    thrd_t thr = g_current_thread;

    if (thr)
    {
        thr->result = res;
        thr->completed = 1;
        tx_semaphore_put(&thr->done);
    }

    tx_thread_terminate(tx_thread_identify());
    for (;;)
        tx_thread_sleep(TX_WAIT_FOREVER);
}

int thrd_join(thrd_t thr, int *res)
{
    if (!thr || thr->detached)
        return thrd_error;

    if (tx_semaphore_get(&thr->done, TX_WAIT_FOREVER) != TX_SUCCESS)
        return thrd_error;

    if (res)
        *res = thr->result;

    lely_threadx_cleanup_thread(thr);
    return thrd_success;
}

int thrd_sleep(const struct timespec *duration, struct timespec *remaining)
{
    ULONG ticks;

    if (remaining)
    {
        remaining->tv_sec = 0;
        remaining->tv_nsec = 0;
    }

    if (!duration)
        return -1;

    ticks = lely_threadx_duration_to_ticks(duration);
    if (ticks == 0)
        return 0;

    tx_thread_sleep(ticks);
    return 0;
}

void thrd_yield(void)
{
    tx_thread_relinquish();
}

int tss_create(tss_t *key, tss_dtor_t dtor)
{
    (void)dtor;

    if (!key)
        return thrd_error;

    if (g_tss_count >= LELY_THREADX_MAX_TSS_KEYS)
        return thrd_nomem;

    *key = g_tss_count++;
    return thrd_success;
}

void tss_delete(tss_t key)
{
    (void)key;
}

void *tss_get(tss_t key)
{
    if (key >= LELY_THREADX_MAX_TSS_KEYS)
        return NULL;

    return g_tss_values[key];
}

int tss_set(tss_t key, void *val)
{
    if (key >= LELY_THREADX_MAX_TSS_KEYS)
        return thrd_error;

    g_tss_values[key] = val;
    return thrd_success;
}