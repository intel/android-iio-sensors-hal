#include <stdlib.h>
#include <hardware/sensors.h>
#include <math.h>
#include <pthread.h>
#include <utils/Log.h>
#include "common.h"
#include "filtering.h"


struct filter_median
{
	float* buff;
	unsigned int idx;
	unsigned int count;
	unsigned int sample_size;
};

static unsigned int partition(float* list, unsigned int left,
	unsigned int right, unsigned int pivot_index)
{
	unsigned int i;
	unsigned int store_index = left;
	float aux;
	float pivot_value = list[pivot_index];

	// swap list[pivotIndex] and list[right]
	aux = list[pivot_index];
	list[pivot_index] = list[right];
	list[right] = aux;

	for (i = left; i < right; i++)
	{
		if (list[i] < pivot_value)
		{
			// swap list[store_index] and list[i]
			aux = list[store_index];
			list[store_index] = list[i];
			list[i] = aux;
			store_index++;
		}
	}
	//swap list[right] and list[store_index]
	aux = list[right];
	list[right] = list[store_index];
	list[store_index] = aux;
	return store_index;
}

/* http://en.wikipedia.org/wiki/Quickselect */
float median(float* queue, unsigned int size)
{
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

void denoise_median_init(int s, unsigned int num_fields,
	unsigned int max_samples)
{
	struct filter_median* f_data = (struct filter_median*) calloc(1, sizeof(struct filter_median));
	f_data->buff = (float*)calloc(max_samples,
		sizeof(float) * num_fields);
	f_data->sample_size = max_samples;
	f_data->count = 0;
	f_data->idx = 0;
	sensor_info[s].filter = f_data;
}

void denoise_median_release(int s)
{
	if (!sensor_info[s].filter)
		return;

	free(((struct filter_median*)sensor_info[s].filter)->buff);
	free(sensor_info[s].filter);
	sensor_info[s].filter = NULL;
}
void denoise_median(struct sensor_info_t* info, struct sensors_event_t* data,
					unsigned int num_fields)
{
	float x, y, z;
	float scale;
	unsigned int field, offset;

	struct filter_median* f_data = (struct filter_median*) info->filter;
	if (!f_data)
		return;


	if (f_data->count < f_data->sample_size)
		f_data->count++;

	for (field = 0; field < num_fields; field++) {
		offset = f_data->sample_size * field;
		f_data->buff[offset + f_data->idx] = data->data[field];

		data->data[field] = median(f_data->buff + offset, f_data->count);
	}

	f_data->idx = (f_data->idx + 1) % f_data->sample_size;
}

