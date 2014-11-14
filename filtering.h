#ifndef FILTERING_H
#define FILTERING_H

void denoise_median(struct sensor_info_t* info, struct sensors_event_t* event,
	unsigned int num_fields);
void denoise_median_init(int s, unsigned int num_fields,
	unsigned int max_samples);
void denoise_median_release(int s);

void denoise_average (	struct sensor_info_t* si, struct sensors_event_t* data,
			int num_fields, int max_samples);

void record_sample(int s, const sensors_event_t* data);
#endif
