/*
 * Copyright (C) 2014 Intel Corporation.
 */

#ifndef __DESCRIPTION_H__
#define __DESCRIPTION_H__

#include "common.h"

#define QUIRK_ALREADY_DECODED	0x01  /* Sensor quirks have been read	      */
#define QUIRK_INITIAL_RATE	0x02  /* Force initial sensor sampling rate   */
#define QUIRK_FIELD_ORDERING	0x04  /* Do field remapping for this sensor   */
#define QUIRK_TERSE_DRIVER	0x08  /* Force duplicate events generation    */
#define QUIRK_NOISY		0x10  /* High noise level on readings	      */
#define QUIRK_FORCE_CONTINUOUS	0x20  /* Force usage of continuous trigger    */

char*		sensor_get_name		(int s);
char*		sensor_get_vendor	(int s);
int		sensor_get_version	(int s);
float		sensor_get_max_range	(int s);
float		sensor_get_resolution	(int s);
float		sensor_get_power	(int s);
flag_t		sensor_get_flags	(int s);
int32_t		sensor_get_min_delay	(int s);
max_delay_t	sensor_get_max_delay	(int s);
float		sensor_get_illumincalib (int s);
uint32_t	sensor_get_quirks	(int s);
int		sensor_get_prop		(int s, const char* sel, int* val);
int		sensor_get_fl_prop	(int s, const char* sel, float* val);
int		sensor_get_order	(int s,unsigned char map[MAX_CHANNELS]);
int		sensor_get_cal_steps	(int s);
char*		sensor_get_string_type	(int s);

#endif
