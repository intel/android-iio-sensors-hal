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

static void reset (struct gyro_cal* cal_data)
{
	cal_data->count = 0;

	cal_data->bias_x = cal_data->bias_y = cal_data->bias_z = 0;

	cal_data->min_x  = cal_data->min_y  = cal_data->min_z  = 1.0;
	cal_data->max_x  = cal_data->max_y  = cal_data->max_z  =-1.0;
}


void gyro_cal_init (struct sensor_info_t* info)
{
	info->cal_level = 0;

	if (info->cal_data)
		reset((struct gyro_cal*) info->cal_data);
}


static int gyro_collect (float x, float y, float z, struct gyro_cal* cal_data)
{
	/* Analyze gyroscope data */

	if (fabs(x) >= 1 || fabs(y) >= 1 || fabs(z) >= 1) {

		/* We're supposed to be standing still ; start over */
		reset(cal_data);

		return 0; /* Uncalibrated */
	}

	if (cal_data->count < GYRO_DS_SIZE) {

		if (x < cal_data->min_x)
			cal_data->min_x = x;

		if (y < cal_data->min_y)
			cal_data->min_y = y;

		if (z < cal_data->min_z)
			cal_data->min_z = z;

		if (x > cal_data->max_x)
			cal_data->max_x = x;

		if (y > cal_data->max_y)
			cal_data->max_y = y;

		if (z > cal_data->max_z)
			cal_data->max_z = z;

		if (fabs(cal_data->max_x - cal_data->min_x) <= GYRO_MAX_ERR &&
		    fabs(cal_data->max_y - cal_data->min_y) <= GYRO_MAX_ERR &&
		    fabs(cal_data->max_z - cal_data->min_z) <= GYRO_MAX_ERR)
			cal_data->count++; /* One more conformant sample */
		else
			reset(cal_data); /* Out of spec sample ; start over */

		return 0; /* Still uncalibrated */
	}

	/* We got enough stable samples to estimate gyroscope bias */
	cal_data->bias_x = (cal_data->max_x + cal_data->min_x) / 2;
	cal_data->bias_y = (cal_data->max_y + cal_data->min_y) / 2;
	cal_data->bias_z = (cal_data->max_z + cal_data->min_z) / 2;

	return 1; /* Calibrated! */
}

void calibrate_gyro(struct sensors_event_t* event, struct sensor_info_t* info)
{
	struct gyro_cal* cal_data = (struct gyro_cal*) info->cal_data;

	event->gyro.status = SENSOR_STATUS_ACCURACY_MEDIUM;

	if (cal_data == NULL)
		return;

	/* Attempt gyroscope calibration if we have not reached this state */
	if (info->cal_level == 0)
		info->cal_level = gyro_collect(event->data[0], event->data[1],
					       event->data[2], cal_data);

	if (info->cal_level)
		event->gyro.status = SENSOR_STATUS_ACCURACY_HIGH;

	switch (event->type) {
		case SENSOR_TYPE_GYROSCOPE:
			/* For the gyroscope apply the bias */
			event->data[0] = event->data[0] - cal_data->bias_x;
			event->data[1] = event->data[1] - cal_data->bias_y;
			event->data[2] = event->data[2] - cal_data->bias_z;
			break;

		case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
			/*
			 * For the uncalibrated gyroscope don't apply the bias,
			 * but tell he Android framework what we think it is.
			 */
			event->uncalibrated_gyro.bias[0] = cal_data->bias_x;
			event->uncalibrated_gyro.bias[1] = cal_data->bias_y;
			event->uncalibrated_gyro.bias[2] = cal_data->bias_z;
			break;
	}
}
