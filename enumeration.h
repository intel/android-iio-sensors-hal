/*
 * Copyright (C) 2014-2015 Intel Corporation.
 */

#ifndef __ENUMERATION_H__
#define __ENUMERATION_H__

#include "common.h"

int	get_sensors_list	(struct sensors_module_t* module,
				 struct sensor_t const** list);

void	enumerate_sensors	(void);
void	delete_enumeration_data (void);

/*
 * These are fine-grained type definitions that are used internally, in the sensor array, but mapped to an Android sensor type in the processing pipeline.
 * The sensor array uses these, not the desc array.
 */
#define SENSOR_TYPE_INTERNAL_ILLUMINANCE	-1	/* Global illuminance, in lux				 */
#define SENSOR_TYPE_INTERNAL_INTENSITY		-2	/* Global intensity, in sensor specific units		 */

#endif
