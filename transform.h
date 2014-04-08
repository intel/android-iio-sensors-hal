/*
 * Copyright (C) 2014 Intel Corporation.
 */

#ifndef __TRANSFORM_H__
#define __TRANSFORM_H__

#include "common.h"

float transform_sample	(int sensor, int channel, unsigned char* sample_data);

void finalize_sample	(int s, struct sensors_event_t* data);


#endif



