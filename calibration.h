/*
 * Copyright (C) 2014 Intel Corporation.
 */

#ifndef __CALIBRATION_H__
#define __CALIBRATION_H__

#include "common.h"

/* compass defines */
#define COMPASS_CALIBRATION_PATH "/data/compass.conf"
#define DS_SIZE 32
#define EPSILON 0.000000001
#define CAL_STEPS 4

/* We'll have multiple calibration levels
*  so that we can provide an estimation as fast as possible
*/
static const float min_diffs[CAL_STEPS] =  { 0.2, 0.4, 0.6, 1.0 };
static const float max_sqr_errs[CAL_STEPS] = { 10.0, 8.0, 5.0, 3.5 };
static const unsigned int lookback_counts[CAL_STEPS] = { 3, 4, 5, 6 };


#ifdef DBG_RAW_DATA
#define RAW_DATA_FULL_PATH "/data/raw_compass_data_full_%d.txt"
#define RAW_DATA_SELECTED_PATH "/data/raw_compass_data_selected_%d.txt"
#endif

/* gyro defines */
#define GYRO_MAX_ERR 0.05f
#define GYRO_DS_SIZE 8

struct compass_cal {
    /* hard iron offsets */
    double offset[3][1];

    /* soft iron matrix */
    double w_invert[3][3];

    /* geomagnetic strength */
    double bfield;

    /* selection data */
    float sample[DS_SIZE][3];
    unsigned int sample_count;
};

struct gyro_cal {
    float bias[3];
    int start;
    int count;
    float sample[GYRO_DS_SIZE][3];
};

typedef double mat_input_t[DS_SIZE][3];

void calibrate_compass (struct sensors_event_t* event, struct sensor_info_t* info, int64_t time);
void compass_read_data (struct sensor_info_t* info);
void compass_store_data (struct sensor_info_t* info);

void calibrate_gyro(struct sensors_event_t* event, struct sensor_info_t* info);
void gyro_cal_init(struct sensor_info_t* info);
#endif
