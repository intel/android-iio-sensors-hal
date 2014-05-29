/*
 * Copyright (C) 2014 Intel Corporation.
 */

#include <stdio.h>
#include <math.h>
#include <sys/time.h>
#include <utils/Log.h>
#include <hardware/sensors.h>
#include "common.h"
#include "calibration.h"


void gyro_cal_init(struct sensor_info_t* info)
{
	info->calibrated = 0;

	struct gyro_cal* gyro_data = (struct gyro_cal*) info->cal_data;

	if (gyro_data == NULL)
		return;

	gyro_data->start = 0;
	gyro_data->count = 0;
	gyro_data->bias[0] = gyro_data->bias[1] = gyro_data->bias[2] = 0;
}

/* Collect points - circular queue for the last GYRO_DS_SIZE
   elements. If the points are sufficiently close compute bias */
static void gyro_collect(float x, float y, float z, struct sensor_info_t* info)
{
	int offset, k;
	float min[3], max[3];

	struct gyro_cal* gyro_data = (struct gyro_cal*) info->cal_data;

	if (gyro_data == NULL)
		return;

	int pos = (gyro_data->start + gyro_data->count) % GYRO_DS_SIZE;
	gyro_data->sample[pos][0] = x;
	gyro_data->sample[pos][1] = y;
	gyro_data->sample[pos][2] = z;

	if (gyro_data->count < GYRO_DS_SIZE)
		gyro_data->count++;
	else
		gyro_data->start = (gyro_data->start + 1) % GYRO_DS_SIZE;

	for (k = 0; k < 3; k++)
		min[k] = max[k] = gyro_data->sample[gyro_data->start][k];

	/* Search for min-max values */
	for (offset = 1; offset < gyro_data->count; offset++) {
		int pos2 = (gyro_data->start + offset) % GYRO_DS_SIZE;
		for (k = 0; k < 3; k++) {
			if (min[k] > gyro_data->sample[pos2][k])
				min[k] = gyro_data->sample[pos2][k];
			else if (max[k] < gyro_data->sample[pos2][k])
				max[k] = gyro_data->sample[pos2][k];
		}
	}

	if (gyro_data->count == GYRO_DS_SIZE &&
		fabs(max[0] - min[0]) < GYRO_MAX_ERR &&
		fabs(max[1] - min[1]) < GYRO_MAX_ERR &&
		fabs(max[2] - min[2]) < GYRO_MAX_ERR) {
		info->calibrated = 1;
		gyro_data->bias[0] = (max[0] + min[0]) / 2;
		gyro_data->bias[1] = (max[1] + min[1]) / 2;
		gyro_data->bias[2] = (max[2] + min[2]) / 2;
	}
}

void calibrate_gyro(struct sensors_event_t* event, struct sensor_info_t* info)
{
	if (!info->calibrated) {
		gyro_collect(event->data[0], event->data[1], event->data[2], info);
	}
	else {
		struct gyro_cal* gyro_data = (struct gyro_cal*) info->cal_data;

		if (gyro_data == NULL)
			return;

		event->data[0] = event->gyro.x = event->data[0] - gyro_data->bias[0];
		event->data[1] = event->gyro.y = event->data[1] - gyro_data->bias[1];
		event->data[2] = event->gyro.z = event->data[2] - gyro_data->bias[2];
	}
}