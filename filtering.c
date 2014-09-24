#include <stdlib.h>
#include <hardware/sensors.h>
#include <math.h>
#include <pthread.h>
#include "common.h"
#include "filtering.h"

void add_to_buff(struct circ_buff* circ_buff, float val)
{
	if (circ_buff->count < circ_buff->size)
	{
		circ_buff->buff[circ_buff->count] = val;
		circ_buff->count++;
		return;
	}

	circ_buff->idx = circ_buff->idx % circ_buff->size;
	circ_buff->buff[circ_buff->idx] = val;
	circ_buff->idx++;
	return;
}

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

void denoise_median(struct sensors_event_t* data, struct sensor_info_t* info)
{
	float x, y, z;
	float scale;

	struct filter* f_data = (struct filter*) info->filter;
	if (!f_data)
		return;

	x = data->data[0];
	y = data->data[1];
	z = data->data[2];

	add_to_buff(f_data->x_buff, x);
	add_to_buff(f_data->y_buff, y);
	add_to_buff(f_data->z_buff, z);

	x = median(f_data->x_buff->buff, f_data->x_buff->count);
	y = median(f_data->y_buff->buff, f_data->y_buff->count);
	z = median(f_data->z_buff->buff, f_data->z_buff->count);

	data->data[0] = x;
	data->data[1] = y;
	data->data[2] = z;
}

