/*
 * Copyright (C) 2014 Intel Corporation.
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

int	decode_type_spec(const char type_buf[MAX_TYPE_SPEC_LEN],
			 struct datum_info_t *type_info);

int64_t	get_timestamp	(void);

#endif



