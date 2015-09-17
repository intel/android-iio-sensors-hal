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

#ifndef __CALIBRATION_H__
#define __CALIBRATION_H__

#include "common.h"

#define MAGN_DS_SIZE 32


typedef struct {
    /* hard iron offsets */
    double offset[3][1];

    /* soft iron matrix */
    double w_invert[3][3];

    /* geomagnetic strength */
    double bfield;

    /* selection data */
    float sample[MAGN_DS_SIZE][3];
    unsigned int sample_count;
    float average[3];
}
compass_cal_t;


typedef struct {
    float bias_x, bias_y, bias_z;
    int count;
    float min_x, min_y, min_z;
    float max_x, max_y, max_z;
}
gyro_cal_t;

/* Accelerometer bias estimation and compensation */

#define BUCKET_COUNT 3
#define BUCKET_TOLERANCE 2.5 	/* Maximum monitoring distance from value of interest, in m/s² */
#define SLICES 100		    /* We currently have 3 buckets per axis, and 100 slices per bucket ; then we distribute incoming samples among slices */

typedef struct
{
    int version;
    int bucket_count;
    int slices;
    float bucket_tolerance;

    uint64_t bucket[3][BUCKET_COUNT][SLICES];	/* How many samples fell in each of the slices, per axis */
    uint64_t bucket_usage[3][BUCKET_COUNT];	/* How many samples fell in each of the buckets (= sum of the slice counts) */

    /* Estimated bias, according to accumulated data */
    float accel_bias_x;
    float accel_bias_y;
    float accel_bias_z;

    uint64_t last_estimation_ts;
}
accel_cal_t;


typedef double mat_input_t[MAGN_DS_SIZE][3];


void calibrate_compass  (int s, sensors_event_t* event);
void compass_read_data  (int s);
void compass_store_data (int s);

void calibrate_gyro     (int s, sensors_event_t* event);
void gyro_cal_init      (int s);
void gyro_store_data    (int s);

void calibrate_accel    (int s, sensors_event_t* event);
void accel_cal_init     (int s);
void accel_cal_store	(int s);

#endif
