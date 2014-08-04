/*
 * Copyright (C) 2014 Intel Corporation.
 */

#ifndef __DESCRIPTION_H__
#define __DESCRIPTION_H__

#include "common.h"

#define QUIRKS_ALREADY_DECODED	0x01  /* Sensor quirks have been read	    */
#define QUIRKS_INITIAL_RATE	0x02  /* Force initial sensor sampling rate */
#define QUIRK_TERSE_DRIVER	0x08  /* Force duplicate events generation  */
#define QUIRK_NOISY		0x10  /* High noise level on readings	    */

char*	sensor_get_name		(int handle);
char*	sensor_get_vendor	(int handle);
int	sensor_get_version	(int handle);
float	sensor_get_max_range	(int handle);
float	sensor_get_resolution	(int handle);
float	sensor_get_power	(int handle);
float	sensor_get_illumincalib (int handle);

int		sensor_get_fl_prop (int s, const char* sel, float* val);

int		sensor_get_order	(int s, unsigned char map[MAX_CHANNELS]);

#endif
