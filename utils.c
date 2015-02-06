/*
 * Copyright (C) 2014-2015 Intel Corporation.
 */

#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <utils/Log.h>
#include <hardware/sensors.h>
#include <utils/Atomic.h>
#include <linux/android_alarm.h>

#include "common.h"
#include "utils.h"


/* Note:
 *
 * Some of these calls are going to fail, because not all sensors expose all
 * possible sysfs attributes. As an optimization we may want to cache which
 * ones are valid and immediately return in error for inexistent entries.
 */
int sysfs_read(const char path[PATH_MAX], void *buf, int buf_len)
{
	int fd, len;

	if (!path[0] || !buf || buf_len < 1)
		return -1;

	fd = open(path, O_RDONLY);

	if (fd == -1) {
		ALOGV("Cannot open %s (%s)\n", path, strerror(errno));
		return -1;
	}

	len = read(fd, buf, buf_len);

	close(fd);

	if (len == -1)
		ALOGW("Cannot read from %s (%s)\n", path, strerror(errno));
	else
		ALOGV("Read %d bytes from %s\n", len, path);

	return len;
}


int sysfs_write(const char path[PATH_MAX], const void *buf, const int buf_len)
{
	int fd, len;

	if (!path[0] || !buf || buf_len < 1)
		return -1;

	fd = open(path, O_WRONLY);

	if (fd == -1) {
		ALOGV("Cannot open %s (%s)\n", path, strerror(errno));
		return -1;
	}

	len = write(fd, buf, buf_len);

	close(fd);

	if (len == -1)
		ALOGW("Cannot write to %s (%s)\n", path, strerror(errno));
	else if (len != buf_len)
		ALOGW("Cannot write %d bytes to %s (%d)\n", buf_len, path, len);
	else
		ALOGV("Wrote %d bytes to %s\n", buf_len, path);

	return len;
}


int sysfs_write_int(const char path[PATH_MAX], int value)
{
	int ret;
	int fd;
	int len;
	char buf[20];

	len = sprintf(buf, "%d", value);

	if (!path[0] || len <= 0) {
		ALOGE("Unexpected condition in sysfs_write_int\n");
		return -1;
	}

	fd = open(path, O_WRONLY);

	if (fd == -1) {
		ALOGV("Cannot open %s (%s)\n", path, strerror(errno));
		return -1;
	}

	ret = write(fd, buf, len);

	if (ret != len) {
		ALOGW("Cannot write %s (%d bytes) to %s (%s)\n", buf, len, path,
		      strerror(errno));
	}

	close(fd);

	return ret;
}


int sysfs_read_int(const char path[PATH_MAX], int *value)
{
	int fd;
	int len;
	char buf[20] = {0};

	if (!path[0] || !value) {
		return -1;
	}

	fd = open(path, O_RDONLY);

	if (fd == -1) {
		ALOGV("Cannot open %s (%s)\n", path, strerror(errno));
		return -1;
	}

	len = read(fd, buf, sizeof(buf));

	close(fd);

	if (len <= 0) {
		ALOGW("Cannot read integer from %s (%s)\n", path,
		      strerror(errno));
		return -1;
	}

	*value = atoi(buf);

	ALOGV("Read %d from %s\n", *value, path);

	return 0;
}


int sysfs_write_str(const char path[PATH_MAX], const char *str)
{
	if (!str || !str[0])
		return -1;

	return sysfs_write(path, str, strlen(str));
}


int sysfs_read_str(const char path[PATH_MAX], char *buf, int buf_len)
{
	int len;

	if (!buf || buf_len < 1)
		return -1;

	len = sysfs_read(path, buf, buf_len);

	if (len == -1) {
		ALOGW("Cannot read string from %s (%s)\n", path,
		      strerror(errno));
		return -1;
	}

	buf[len == 0 ? 0 : len - 1] = '\0';

	ALOGV("Read %s from %s\n", buf, path);

	return len;
}


int sysfs_write_float(const char path[PATH_MAX], float value)
{
	int ret;
	int fd;
	int len;
	char buf[20];

	len = snprintf(buf, sizeof(buf), "%g", value);

	if (!path[0] || len <= 0) {
		ALOGE("Unexpected condition in sysfs_write_float\n");
		return -1;
	}

	fd = open(path, O_WRONLY);

	if (fd == -1) {
		ALOGV("Cannot open %s (%s)\n", path, strerror(errno));
		return -1;
	}

	ret = write(fd, buf, len);

	if (ret != len) {
		ALOGW("Cannot write %s (%d bytes) to %s (%s)\n", buf, len, path,
		      strerror(errno));
	}

	close(fd);

	return ret;
}


int sysfs_read_float(const char path[PATH_MAX], float *value)
{
	int fd;
	int len;
	char buf[20] = {0};

	if (!path[0] || !value) {
		return -1;
	}

	fd = open(path, O_RDONLY);

	if (fd == -1) {
		ALOGV("Cannot open %s (%s)\n", path, strerror(errno));
		return -1;
	}

	len = read(fd, buf, sizeof(buf));

	close(fd);

	if (len <= 0) {
		ALOGW("Cannot read float from %s (%s)\n", path,
		      strerror(errno));
		return -1;
	}

	*value = (float) strtod(buf, NULL);

	ALOGV("Read %g from %s\n", *value, path);

	return 0;
}

int sysfs_read_uint64(const char path[PATH_MAX], uint64_t *value)
{
	int fd;
	int len;
	char buf[20] = {0};

	if (!path[0] || !value) {
		return -1;
	}

	fd = open(path, O_RDONLY);

	if (fd == -1) {
		ALOGV("Cannot open %s (%s)\n", path, strerror(errno));
		return -1;
	}

	len = read(fd, buf, sizeof(buf));

	close(fd);

	if (len <= 0) {
		ALOGW("Cannot read uint64 from %s (%s)\n", path,
		      strerror(errno));
		return -1;
	}

	*value =  atoll(buf);

	ALOGV("Read %llu from %s\n", *value, path);

	return 0;
}



int64_t get_timestamp_realtime (void)
{
	struct timespec ts = {0};
	clock_gettime(CLOCK_REALTIME, &ts);

	return 1000000000LL * ts.tv_sec + ts.tv_nsec;
}


int64_t get_timestamp_boot (void)
{
	struct timespec ts = {0};
	clock_gettime(CLOCK_BOOTTIME, &ts);

	return 1000000000LL * ts.tv_sec + ts.tv_nsec;
}


int64_t get_timestamp_monotonic (void)
{
	struct timespec ts = {0};
	clock_gettime(CLOCK_MONOTONIC, &ts);

	return 1000000000LL * ts.tv_sec + ts.tv_nsec;
}


void set_timestamp (struct timespec *out, int64_t target_ns)
{
	out->tv_sec  = target_ns / 1000000000LL;
	out->tv_nsec = target_ns % 1000000000LL;
}

