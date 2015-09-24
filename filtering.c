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

#include <ctype.h>
#include <stdlib.h>
#include <hardware/sensors.h>
#include <utils/Log.h>
#include "common.h"
#include "filtering.h"
#include "description.h"

typedef struct
{
	float* buff;
	unsigned int idx;
	unsigned int count;
	unsigned int sample_size;
}
filter_median_t;


typedef struct
{
	int max_samples;	/* Maximum averaging window size	      */
	int num_fields;		/* Number of fields per sample (usually 3)    */
	float *history;		/* Working buffer containing recorded samples */
	float *history_sum;	/* The current sum of the history elements    */
	int history_size;	/* Number of recorded samples		      */
	int history_entries;	/* How many of these are initialized	      */
	int history_index;	/* Index of sample to evict next time	      */
}
filter_average_t;


static unsigned int partition (float* list, unsigned int left, unsigned int right, unsigned int pivot_index)
{
	unsigned int i;
	unsigned int store_index = left;
	float aux;
	float pivot_value = list[pivot_index];

	/* Swap list[pivotIndex] and list[right] */
	aux = list[pivot_index];
	list[pivot_index] = list[right];
	list[right] = aux;

	for (i = left; i < right; i++)
	{
		if (list[i] < pivot_value)
		{
			/* Swap list[store_index] and list[i] */
			aux = list[store_index];
			list[store_index] = list[i];
			list[i] = aux;
			store_index++;
		}
	}

	/* Swap list[right] and list[store_index] */
	aux = list[right];
	list[right] = list[store_index];
	list[store_index] = aux;
	return store_index;
}


static float median (float* queue, unsigned int size)
{
	/* http://en.wikipedia.org/wiki/Quickselect */

	unsigned int left = 0;
	unsigned int right = size - 1;
	unsigned int pivot_index;
	unsigned int median_index = (right / 2);
	float temp[size];

	memcpy(temp, queue, size * sizeof(float));

	/* If the list has only one element return it */
	if (left == right)
		return temp[left];

	while (left < right) {
		pivot_index = (left + right) / 2;
		pivot_index = partition(temp, left, right, pivot_index);
		if (pivot_index == median_index)
			return temp[median_index];
		else if (pivot_index > median_index)
			right = pivot_index - 1;
		else
			left = pivot_index + 1;
	}

	return temp[left];
}


static void denoise_median_init (int s, unsigned int num_fields, unsigned int max_samples)
{
	filter_median_t* f_data = (filter_median_t*) malloc(sizeof(filter_median_t));

	f_data->buff = (float*) calloc(max_samples, sizeof(float) * num_fields);
	f_data->sample_size = max_samples;
	f_data->count = 0;
	f_data->idx = 0;
	sensor[s].filter = f_data;
}


static void denoise_average_init (int s, unsigned int num_fields, unsigned int max_samples)
{
	filter_average_t* filter = (filter_average_t*) malloc(sizeof(filter_average_t));

	if (filter) {
		memset(filter, 0, sizeof(filter_average_t));
		filter->max_samples = max_samples;
		filter->num_fields = num_fields;
	}

	sensor[s].filter = filter;
}


static void denoise_median_reset (sensor_info_t* info)
{
	filter_median_t* f_data = (filter_median_t*) info->filter;

	if (!f_data)
		return;

	f_data->count = 0;
	f_data->idx = 0;
}


static void denoise_median (sensor_info_t* info, sensors_event_t* data, unsigned int num_fields)
{
	unsigned int field, offset;

	filter_median_t* f_data = (filter_median_t*) info->filter;
	if (!f_data)
		return;

	/* If we are at event count 1 reset the indices */
	if (info->event_count == 1)
		denoise_median_reset(info);

	if (f_data->count < f_data->sample_size)
		f_data->count++;

	for (field = 0; field < num_fields; field++) {
		offset = f_data->sample_size * field;
		f_data->buff[offset + f_data->idx] = data->data[field];

		data->data[field] = median(f_data->buff + offset, f_data->count);
	}

	f_data->idx = (f_data->idx + 1) % f_data->sample_size;
}


static void denoise_average (sensor_info_t* si, sensors_event_t* data)
{
	/*
	 * Smooth out incoming data using a moving average over a number of
	 * samples. We accumulate one second worth of samples, or max_samples,
	 * depending on which is lower.
	 */

	int f;
	int sampling_rate = (int) si->sampling_rate;
	int history_size;
	int history_full = 0;
	filter_average_t* filter;

	/* Don't denoise anything if we have less than two samples per second */
	if (sampling_rate < 2)
		return;

	filter = (filter_average_t*) si->filter;

	if (!filter)
		return;

	/* Restrict window size to the min of sampling_rate and max_samples */
	if (sampling_rate > filter->max_samples)
		history_size = filter->max_samples;
	else
		history_size = sampling_rate;

	/* Reset history if we're operating on an incorrect window size */
	if (filter->history_size != history_size) {
		filter->history_size = history_size;
		filter->history_entries = 0;
		filter->history_index = 0;
		filter->history = (float*) realloc(filter->history, filter->history_size * filter->num_fields * sizeof(float));
		if (filter->history) {
			filter->history_sum = (float*) realloc(filter->history_sum, filter->num_fields * sizeof(float));
			if (filter->history_sum)
				memset(filter->history_sum, 0, filter->num_fields * sizeof(float));
		}
	}

	if (!filter->history || !filter->history_sum)
		return;	/* Unlikely, but still... */

	/* Update initialized samples count */
	if (filter->history_entries < filter->history_size)
		filter->history_entries++;
	else
		history_full = 1;

	/* Record new sample and calculate the moving sum */
	for (f=0; f < filter->num_fields; f++) {
		/** A field is going to be overwritten if history is full, so decrease the history sum */
		if (history_full)
			filter->history_sum[f] -= filter->history[filter->history_index * filter->num_fields + f];

		filter->history[filter->history_index * filter->num_fields + f] = data->data[f];
		filter->history_sum[f] += data->data[f];

		/* For now simply compute a mobile mean for each field and output filtered data */
		data->data[f] = filter->history_sum[f] / filter->history_entries;
	}

	/* Update our rolling index (next evicted cell) */
	filter->history_index = (filter->history_index + 1) % filter->history_size;
}


void setup_noise_filtering (int s)
{
	char filter_buf[MAX_NAME_SIZE];
	int num_fields;
	char* cursor;
	int window_size = 0;

	/* By default, don't apply filtering */
	sensor[s].filter_type = FILTER_TYPE_NONE;

	/* Restrict filtering to a few sensor types for now */
	switch (sensor[s].type) {
			case SENSOR_TYPE_ACCELEROMETER:
			case SENSOR_TYPE_GYROSCOPE:
			case SENSOR_TYPE_MAGNETIC_FIELD:
				num_fields = 3 /* x,y,z */;
				break;

			default:
				return;	/* No filtering */
	}

	/* If noisy, start with default filter for sensor type */
	if (sensor[s].quirks & QUIRK_NOISY)
		switch (sensor[s].type) {
			case SENSOR_TYPE_GYROSCOPE:
				sensor[s].filter_type = FILTER_TYPE_MEDIAN;
				break;

			case SENSOR_TYPE_MAGNETIC_FIELD:
				sensor[s].filter_type = FILTER_TYPE_MOVING_AVERAGE;
				break;
		}

	/* Use whatever was specified if there's an explicit configuration choice for this sensor */

	filter_buf[0] = '\0';
	sensor_get_st_prop(s, "filter", filter_buf);

	cursor = strstr(filter_buf, "median");
	if (cursor)
		sensor[s].filter_type = FILTER_TYPE_MEDIAN;
	else {
		cursor = strstr(filter_buf, "average");
		if (cursor)
			sensor[s].filter_type = FILTER_TYPE_MOVING_AVERAGE;
	}

	/* Check if an integer is part of the string, and use it as window size */
	if (cursor) {
		while (*cursor && !isdigit(*cursor))
			cursor++;

		if (*cursor)
			window_size = atoi(cursor);
	}

	switch (sensor[s].filter_type) {

		case FILTER_TYPE_MEDIAN:
			denoise_median_init(s, num_fields, window_size ? window_size : 5);
			break;

		case FILTER_TYPE_MOVING_AVERAGE:
			denoise_average_init(s, num_fields, window_size ? window_size: 20);
			break;
	}
}


void denoise (int s, sensors_event_t* data)
{
	switch (sensor[s].filter_type) {

		case FILTER_TYPE_MEDIAN:
			denoise_median(&sensor[s], data, 3);
			break;

		case FILTER_TYPE_MOVING_AVERAGE:
			denoise_average(&sensor[s], data);
			break;
	}
}


void release_noise_filtering_data (int s)
{
	void *buf;

	if (!sensor[s].filter)
		return;

	switch (sensor[s].filter_type) {

		case FILTER_TYPE_MEDIAN:
			buf = ((filter_median_t*) sensor[s].filter)->buff;
			if (buf)
				free(buf);
			break;

		case FILTER_TYPE_MOVING_AVERAGE:
			buf = ((filter_average_t*) sensor[s].filter)->history;
			if (buf)
				free(buf);

			buf = ((filter_average_t*) sensor[s].filter)->history_sum;
			if (buf)
				free(buf);
			break;
	}

	free(sensor[s].filter);
	sensor[s].filter = NULL;
}


#define GLOBAL_HISTORY_SIZE 100

typedef struct
{
	int sensor;
	int motion_trigger;
	sensors_event_t data;
}
recorded_sample_t;

/*
 * This is a circular buffer holding the last GLOBAL_HISTORY_SIZE events, covering the entire sensor collection. It is intended as a way to correlate
 * data coming from active sensors, no matter the sensor type, over a recent window of time. The array is not sorted ; we simply evict the oldest cell
 * (by insertion time) and replace its contents. Timestamps don't necessarily grow monotonically as they tell the data acquisition type, and that there
 * can be a delay between acquisition and insertion into this table.
 */

static recorded_sample_t global_history[GLOBAL_HISTORY_SIZE];

static int initialized_entries;	/* How many of these are initialized	      */
static int insertion_index;	/* Index of sample to evict next time	      */


void record_sample (int s, const sensors_event_t* event)
{
	recorded_sample_t *cell;
	int i;

	/* Don't record duplicate samples, as they are not useful for filters */
	if (sensor[s].report_pending == DATA_DUPLICATE)
		return;

	if (initialized_entries == GLOBAL_HISTORY_SIZE) {
		i = insertion_index;
		insertion_index = (insertion_index+1) % GLOBAL_HISTORY_SIZE;
	} else {
		i = initialized_entries;
		initialized_entries++;
	}

	cell = &global_history[i];

	cell->sensor = s;

	cell->motion_trigger = (sensor[s].selected_trigger == sensor[s].motion_trigger_name);

	memcpy(&cell->data, event, sizeof(sensors_event_t));
}
