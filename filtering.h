/*
 * Copyright (C) 2014-2015 Intel Corporation.
 */

#ifndef FILTERING_H
#define FILTERING_H

void setup_noise_filtering		(int s);
void release_noise_filtering_data	(int s);
void denoise				(int s, sensors_event_t* event);
void record_sample			(int s, const sensors_event_t* data);

#endif
