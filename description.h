/*
 * Copyright (C) 2014 Intel Corporation.
 */

#ifndef __DESCRIPTION_H__
#define __DESCRIPTION_H__

#include "common.h"

char*	sensor_get_name		(int handle);
char*	sensor_get_vendor	(int handle);
int	sensor_get_version	(int handle);
float	sensor_get_max_range	(int handle);
float	sensor_get_resolution	(int handle);
float	sensor_get_power	(int handle);
float	sensor_get_illumincalib (int handle);

#endif
