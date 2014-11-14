#ifndef FILTERING_H
#define FILTERING_H

void setup_noise_filtering		(int s);
void release_noise_filtering_data	(int s);
void denoise				(int s, struct sensors_event_t* event);
void record_sample			(int s, const sensors_event_t* data);

#endif
