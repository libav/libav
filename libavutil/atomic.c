/*
 * Copyright (c) 2012 Ronald S. Bultje <rsbultje@gmail.com>
 *
 * This file is part of Libav.
 *
 * Libav is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * Libav is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with Libav; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "atomic.h"

#if !HAVE_MEMORYBARRIER && !HAVE_SYNC_SYNCHRONIZE

#if HAVE_PTHREADS

#include <pthread.h>

static pthread_mutex_t atomic_lock = PTHREAD_MUTEX_INITIALIZER;

int av_atomic_int_get(volatile int *ptr)
{
    int res;

    pthread_mutex_lock(&atomic_lock);
    res = *ptr;
    pthread_mutex_unlock(&atomic_lock);

    return res;
}

void av_atomic_int_set(volatile int *ptr, int val)
{
    pthread_mutex_lock(&atomic_lock);
    *ptr = val;
    pthread_mutex_unlock(&atomic_lock);
}

int av_atomic_int_add_and_fetch(volatile int *ptr, int inc)
{
    int res;

    pthread_mutex_lock(&atomic_lock);
    *ptr += inc;
    res = *ptr;
    pthread_mutex_unlock(&atomic_lock);

    return res;
}

void *av_atomic_ptr_cas(void * volatile *ptr, void *oldval, void *newval)
{
    void *ret;
    pthread_mutex_lock(&atomic_lock);
    ret = *ptr;
    if (*ptr == oldval)
        *ptr = newval;
    pthread_mutex_unlock(&atomic_lock);
    return ret;
}

#else
// assume there is no threading at all, noop implementations of everything

int av_atomic_int_get(volatile int *ptr)
{
    return *ptr;
}

void av_atomic_int_set(volatile int *ptr, int val)
{
    *ptr = val;
}

int av_atomic_int_add_and_fetch(volatile int *ptr, int inc)
{
    *ptr += inc;
    return *ptr;
}

void *av_atomic_ptr_cas(void * volatile *ptr, void *oldval, void *newval)
{
    if (*ptr == oldval) {
        *ptr = newval;
        return oldval;
    }
    return *ptr;
}

#endif // HAVE_PTHREADS

#endif

#ifdef TEST
#include <assert.h>

int main(int argc, char *argv[])
{
    volatile int val = 1;
    int res;

    res = av_atomic_int_add_and_fetch(&val, 1);
    assert(res == 2);
    av_atomic_int_set(&val, 3);
    res = av_atomic_int_get(&val);
    assert(res == 3);

    return 0;
}
#endif
