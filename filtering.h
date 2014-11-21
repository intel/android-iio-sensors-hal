#ifndef FILTERING_H
#define FILTERING_H

#define	GYRO_MIN_SAMPLES 5 /* Drop first few gyro samples after enable */

void setup_noise_filtering		(int s);
void release_noise_filtering_data	(int s);
void denoise				(int s, struct sensors_event_t* event);
void record_sample			(int s, const sensors_event_t* data);

#endif
