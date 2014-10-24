#ifndef FILTERING_H
#define FILTERING_H

void denoise_median(struct sensor_info_t* info, struct sensors_event_t* event,
	unsigned int num_fields);
void denoise_median_init(int s, unsigned int sample_size,
	unsigned int num_fields);
void denoise_median_release(int s);

void record_sample(int s, const sensors_event_t* data);
#endif
