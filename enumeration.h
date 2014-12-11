/*
 * Copyright (C) 2014 Intel Corporation.
 */

#ifndef __ENUMERATION_H__
#define __ENUMERATION_H__

#include "common.h"

/*
 * Macros associating iio sysfs entries to to sensor types ; see
 * linux/kernel/drivers/iio/industrialio-core.c and
 * hardware/libhardware/include/hardware/sensor.h
 */

#define DECLARE_CHANNEL(tag, spacer, name)		\
			name,				\
			"in_"tag spacer name"_en",	\
			"in_"tag spacer name"_type",	\
			"in_"tag spacer name"_index",	\
			"in_"tag spacer name"_raw",	\
			"in_"tag spacer name"_input",	\
			"in_"tag spacer name"_scale",	\

#define DECLARE_NAMED_CHANNEL(tag, name)	DECLARE_CHANNEL(tag, "_", name)

#define DECLARE_GENERIC_CHANNEL(tag)		DECLARE_CHANNEL(tag, "", "")

int	get_sensors_list	(struct sensors_module_t* module,
				 struct sensor_t const** list);

void	enumerate_sensors	(void);
void	delete_enumeration_data (void);

#endif
