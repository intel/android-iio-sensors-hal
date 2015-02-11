/*
 * Copyright (C) 2014-2015 Intel Corporation.
 */

#ifndef _LINUX_ATOMIC_H
#define _LINUX_ATOMIC_H

int android_atomic_release_cas(int32_t oldvalue, int32_t newvalue,
			       volatile int32_t* addr)
{
	return !__sync_bool_compare_and_swap(addr, oldvalue, newvalue);
}

#define android_atomic_cmpxchg android_atomic_release_cas


#endif /* _LINUX_ATOMIC_H */
