/*
 * Copyright (C) 2014 Intel Corporation.
 */

#include <stdlib.h>
#include <fcntl.h>
#include <utils/Log.h>
#include <hardware/sensors.h>
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

	if (!path[0]) {
		return -1;
	}

	fd = open(path, O_WRONLY);

	if (fd == -1) {
		ALOGV("Cannot open %s (%s)\n", path, strerror(errno));
		return -1;
	}

	len = sprintf(buf, "%d", value) + 1;

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


int64_t sample_as_int64(unsigned char* sample, struct datum_info_t* type)
{
	uint16_t u16;
	uint32_t u32;
	uint64_t u64;
	int i;

	switch (type->storagebits) {
		case 64:
			u64 = 0;

			if (type->endianness == 'b')
				for (i=0; i<8; i++)
					u64 = (u64 << 8) | sample[i];
			else
				for (i=7; i>=0; i--)
					u64 = (u64 << 8) | sample[i];

			if (type->sign == 'u')
				return (int64_t) (u64 >> type->shift);

			return ((int64_t) u64) >> type->shift;

		case 32:
			if (type->endianness == 'b')
				u32 = (sample[0] << 24) | (sample[1] << 16) |
					(sample[2] << 8) | sample[3];
			else
				u32 = (sample[3] << 24) | (sample[2] << 16) |
					(sample[1] << 8) | sample[0];

			if (type->sign == 'u')
				return u32 >> type->shift;

			return ((int32_t) u32) >> type->shift;

		case 16:
			if (type->endianness == 'b')
				u16 = (sample[0] << 8) | sample[1];
			else
				u16 = (sample[1] << 8) | sample[0];

			if (type->sign == 'u')
				return u16 >> type->shift;

			return  ((int16_t) u16) >> type->shift;
	}

	ALOGE("Unhandled sample storage size\n");
	return 0;
}


float transform_sample (int sensor_type, int channel, float val)
{
	/* Last opportunity to alter sample data before it goes to Android */
	switch (sensor_type) {
		case SENSOR_TYPE_ACCELEROMETER:
			/*
			 * Invert x axis orientation from SI units - see
			 * /hardware/libhardware/include/hardware/sensors.h
			 * for a discussion of what Android expects
			 */
			if (channel == 0)
				return -val;
			break;

		case SENSOR_TYPE_GYROSCOPE:
			/* Limit drift */
			if (val > -0.05 && val < 0.05)
				return 0;
			break;
	}

	return val;
}


int64_t get_timestamp(void)
{
	struct timespec ts = {0};

	clock_gettime(CLOCK_MONOTONIC, &ts);

	return 1000000000LL * ts.tv_sec + ts.tv_nsec;
}


