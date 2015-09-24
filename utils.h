/*
// Copyright (c) 2015 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
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



