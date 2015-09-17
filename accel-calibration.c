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

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <utils/Log.h>
#include <hardware/sensors.h>
#include "common.h"
#include "calibration.h"
#include "utils.h"

/*
 * This implements a crude way of estimating the accelerometer manufacturing mounting bias. We monitor x y and z distribution around a few strategic spots that
 * should represent most hits when the device is stable (on earth!). We then try to derive an estimation of the accelerometer bias for each of these axes from
 * that data, continuously. This is a very rough method that should only be used as a last resort for now.
 */

static float bucket_center[BUCKET_COUNT] = { -9.8, 0, 9.8 };	/* The spots we are most interested in */

#define ACCEL_CALIB_DATA_VERSION	1			/* Update this whenever the stored data structure changes */

#define ACCEL_CALIBRATION_PATH    "/data/accel.conf"		/* Location of saved calibration data */

#define REFRESH_INTERVAL	1000*1000*1000			/* Recompute bias estimation every second */


static void ascribe_sample (accel_cal_t* cal_data, int channel, float value)
{
	/* Check if this falls within one of our ranges of interest */
	float range_min;
	float range_max;
	int i;
	int slice;

	for (i=0; i<BUCKET_COUNT; i++) {
		range_min = bucket_center[i] - BUCKET_TOLERANCE;
		range_max = bucket_center[i] + BUCKET_TOLERANCE;

		if (value >= range_min && value <= range_max) {
			/* Find suitable bucket */
			slice = (int) ((value-range_min) / (range_max-range_min) * (SLICES-1));

			/* Increment counters */
			cal_data->bucket[channel][i][slice]++;
			cal_data->bucket_usage[channel][i]++;
			return;
		}
	}
}


static float estimate_bias (accel_cal_t* cal_data, int channel)
{
	/*
	 * The long term distribution within the bucket, for each of the buckets, should be centered (samples evenly distributed).
	 * Try to determine the position in the bucket that separates it in two portions holding as many samples, then compute an estimated bias for that axis
	 * (channel) based on that data.
	 */

	int i;
	uint64_t half_of_the_samples;
	uint64_t count;
	float median;
	float estimated_bucket_bias[BUCKET_COUNT] = {0};
	uint64_t bias_weight[BUCKET_COUNT];
	uint64_t total_weight;
	float range_min;
	float range_max;
	float estimated_bias;
	int slice;

	for (i=0; i<BUCKET_COUNT; i++) {
		half_of_the_samples = cal_data->bucket_usage[channel][i] / 2;
		count = 0;

		for (slice = 0; slice < SLICES; slice++) {
			count += cal_data->bucket[channel][i][slice];

			if (count >= half_of_the_samples) {
				range_min = bucket_center[i] - BUCKET_TOLERANCE;
				range_max = bucket_center[i] + BUCKET_TOLERANCE;

				median = range_min + ((float) slice) / (SLICES-1) * (range_max-range_min);

				estimated_bucket_bias[i] = median - bucket_center[i];

				bias_weight[i] = count;
				break;
			}
		}
	}

	/* Weight each of the estimated bucket bias values based on the number of samples collected */

	total_weight = 0;

	for (i=0; i<BUCKET_COUNT; i++)
		total_weight += bias_weight[i];

	if (total_weight == 0)
		return 0.0;

	estimated_bias = 0;

	for (i=0; i<BUCKET_COUNT; i++)
		if (bias_weight[i])
			estimated_bias += estimated_bucket_bias[i] * (float) bias_weight[i] / (float) total_weight;

	return estimated_bias;
}


void calibrate_accel (int s, sensors_event_t* event)
{
	accel_cal_t* cal_data = (accel_cal_t*) sensor[s].cal_data;
	uint64_t current_ts;
	float x, y, z;

	if (cal_data == NULL)
		return;

	x = event->data[0];
	y = event->data[1];
	z = event->data[2];

	/* Analyze sample */
	ascribe_sample(cal_data, 0, x);
	ascribe_sample(cal_data, 1, y);
	ascribe_sample(cal_data, 2, z);

	current_ts = get_timestamp_boot();

	/* Estimate bias using accumulated data, from time to time*/
	if (current_ts >= cal_data->last_estimation_ts + REFRESH_INTERVAL) {
		cal_data->last_estimation_ts = current_ts;

		cal_data->accel_bias_x = estimate_bias(cal_data, 0);
		cal_data->accel_bias_y = estimate_bias(cal_data, 1);
		cal_data->accel_bias_z = estimate_bias(cal_data, 2);
	}

	ALOGV("Compensating for estimated accelerometer bias: x=%g, y=%g, z=%g\n", cal_data->accel_bias_x, cal_data->accel_bias_y, cal_data->accel_bias_z);

	/* Apply compensation */
	event->data[0] = x - cal_data->accel_bias_x;
	event->data[1] = y - cal_data->accel_bias_y;
	event->data[2] = z - cal_data->accel_bias_z;
}


void accel_cal_init (int s)
{
	int fd;
	int n;

	accel_cal_t* cal_data = (accel_cal_t*) sensor[s].cal_data;

	if (cal_data == NULL)
		return;

	if (cal_data->last_estimation_ts)
		return;	/* No need to overwrite perfectly good data at reenable time */

	fd = open(ACCEL_CALIBRATION_PATH, O_RDONLY);

	if (fd != -1) {
		n = read(fd, cal_data, sizeof(accel_cal_t));

		close(fd);

		if (n == sizeof(accel_cal_t) &&
			cal_data->version == ((ACCEL_CALIB_DATA_VERSION << 16) + sizeof(accel_cal_t)) &&
			cal_data->bucket_count == BUCKET_COUNT &&
			cal_data->slices == SLICES &&
			cal_data->bucket_tolerance == BUCKET_TOLERANCE) {
				cal_data->last_estimation_ts = 0;
				return; /* We successfully loaded previously saved accelerometer calibration data */
			}
	}

	/* Fall back to initial values */
	memset(cal_data, 0, sizeof(accel_cal_t));

	/* Store the parameters that are used with that data set, so we can check them against future version of the code to prevent inadvertent reuse */
	cal_data->version	   = (ACCEL_CALIB_DATA_VERSION << 16) + sizeof(accel_cal_t);
	cal_data->bucket_count	   = BUCKET_COUNT;
	cal_data->slices	   = SLICES;
	cal_data->bucket_tolerance = BUCKET_TOLERANCE;
}


void accel_cal_store (int s)
{
	int fd;
	accel_cal_t* cal_data = (accel_cal_t*) sensor[s].cal_data;

	if (cal_data == NULL)
		return;

	fd = open(ACCEL_CALIBRATION_PATH, O_WRONLY | O_TRUNC | O_CREAT, S_IRUSR);

	if (fd != -1) {
		write(fd, cal_data, sizeof(accel_cal_t));
		close(fd);
	}
}
