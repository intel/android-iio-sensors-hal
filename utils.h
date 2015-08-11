/*
 * Copyright (C) 2014-2015 Intel Corporation.
 */

#ifndef __UTILS_H__
#define __UTILS_H__

#include "common.h"

int	sysfs_read_int	 (const char path[PATH_MAX], int *value);
int	sysfs_write_int	 (const char path[PATH_MAX], int value);

int	sysfs_read_str	 (const char path[PATH_MAX], char *buf, int buf_len);
int	sysfs_write_str	 (const char path[PATH_MAX], const char *buf);

int	sysfs_read_float (const char path[PATH_MAX], float *value);
int	sysfs_write_float(const char path[PATH_MAX], float value);

int	sysfs_read_uint64(const char path[PATH_MAX], uint64_t *value);

void	set_timestamp	(struct timespec *out, int64_t target_ns);

int64_t get_timestamp_boot	(void);
int64_t get_timestamp_thread	(void);
int64_t get_timestamp_realtime	(void);
int64_t get_timestamp_monotonic	(void);

#endif



