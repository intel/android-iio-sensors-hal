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

#endif
