/*
// Copyright (c) 2015 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#ifndef __DESCRIPTION_H__
#define __DESCRIPTION_H__

#include "common.h"

#define QUIRK_ALREADY_DECODED	0x01	/* Sensor quirks have been read				*/
#define QUIRK_INITIAL_RATE	0x02	/* Force initial sensor sampling rate			*/
#define QUIRK_FIELD_ORDERING	0x04	/* Do field remapping for this sensor			*/
#define QUIRK_TERSE_DRIVER	0x08	/* Force duplicate events generation			*/
#define QUIRK_NOISY		0x10	/* High noise level on readings				*/
#define QUIRK_FORCE_CONTINUOUS	0x20	/* Force usage of continuous trigger			*/
#define QUIRK_BIASED		0x40	/* Biased sensor, requires compensation			*/
#define QUIRK_SPOTTY		0x80	/* Driver may lose events				*/
#define QUIRK_NO_EVENT_MODE	0x100	/* Disable event mode					*/
#define QUIRK_NO_TRIG_MODE	0x200	/* Disable trigger mode					*/
#define QUIRK_NO_POLL_MODE	0x400	/* Disable poll mode					*/
#define QUIRK_MOUNTING_MATRIX	0x800 	/* Mounting information present				*/
#define QUIRK_HRTIMER		0x1000	/* We may use a hrtimer if there is no other trigger	*/
#define QUIRK_SECONDARY		0x2000	/* List after other sensors of the same type		*/

#ifdef __LP64__
	typedef uint64_t	flag_t;
	typedef int64_t		max_delay_t;
#else
	typedef uint32_t	flag_t;
	typedef int32_t		max_delay_t;
#endif

char*		sensor_get_name		(int s);
char*		sensor_get_vendor	(int s);
int		sensor_get_version	(int s);
float		sensor_get_max_range	(int s);
void 		sensor_update_max_range	(int s);
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
int		sensor_get_mounting_matrix(int s,float mounting_matrix[9]);
int		sensor_get_available_frequencies(int s);
int		sensor_get_cal_steps	(int s);
char*		sensor_get_string_type	(int s);
int 		sensor_get_st_prop	(int s, const char* sel, char val[MAX_NAME_SIZE]);

#endif
