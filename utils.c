/*
 * Copyright (C) 2014 Intel Corporation.
 */

#include <stdlib.h>
#include <fcntl.h>
#include <utils/Log.h>
#include <hardware/sensors.h>
#include "common.h"
#include "utils.h"


/* Note:
 *
 * Some of these calls are going to fail, because not all sensors expose all
 * possible sysfs attributes. As an optimization we may want to cache which
 * ones are valid and immediately return in error for inexistent entries.
 */

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
	int ret;
	int fd;
	int len;

	if (!path[0] || !str || !str[0]) {
		return -1;
	}

	fd = open(path, O_WRONLY);

	if (fd == -1) {
		ALOGV("Cannot open %s (%s)\n", path, strerror(errno));
		return -1;
	}

	len = strlen(str);

	ret = write(fd, str, len);

	if (ret != len) {
		ALOGW("Cannot write %s (%d bytes) to %s (%s)\n", str, len, path,
		      strerror(errno));
	}
	else
		ALOGV("Wrote %s to %s\n", str, path);

	close(fd);

	return ret;
}


int sysfs_read_str(const char path[PATH_MAX], char *buf, int buf_len)
{
	int fd;
	int len;

	if (!path[0] || !buf || buf_len < 1)
		return -1;

	fd = open(path, O_RDONLY);

	if (fd == -1) {
		ALOGV("Cannot open %s (%s)\n", path, strerror(errno));
		return -1;
	}

	len = read(fd, buf, buf_len);

	close(fd);

	if (len == -1) {
		ALOGW("Cannot read string from %s (%s)\n", path,
		      strerror(errno));
		return -1;
	}

	buf[len == 0 ? 0 : len-1] = '\0';

	ALOGV("Read %s from %s\n", buf, path);

	return len;
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

	ALOGV("Read %f from %s\n", *value, path);

	return 0;
}


int decode_type_spec(	const char type_buf[MAX_TYPE_SPEC_LEN],
			struct datum_info_t *type_info)
{
	/* Return size in bytes for this type specification, or -1 in error */
	char sign;
	char endianness;
	unsigned int realbits, storagebits, shift;
	int tokens;

	/* Valid specs: "le:u10/16>>0", "le:s16/32>>0" or "le:s32/32>>0" */

	tokens = sscanf(type_buf, "%ce:%c%u/%u>>%u",
			&endianness, &sign, &realbits, &storagebits, &shift);

	if     (tokens != 5 ||
		(endianness != 'b' && endianness != 'l') ||
		(sign != 'u' && sign != 's') ||
		realbits > storagebits ||
		(storagebits != 16 && storagebits != 32 && storagebits != 64)) {
			ALOGE("Invalid iio channel type spec: %s\n", type_buf);
			return -1;
		}

	type_info->endianness	=		endianness;
	type_info->sign		=		sign;
	type_info->realbits	=	(short)	realbits;
	type_info->storagebits	=	(short)	storagebits;
	type_info->shift	=	(short)	shift;

	return storagebits / 8;
}


int64_t get_timestamp(void)
{
	struct timespec ts = {0};

	clock_gettime(CLOCK_MONOTONIC, &ts);

	return 1000000000LL * ts.tv_sec + ts.tv_nsec;
}
