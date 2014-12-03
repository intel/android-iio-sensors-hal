/*
 * Copyright (C) 2014 Intel Corporation.
 */

#ifndef __CALIBRATION_H__
#define __CALIBRATION_H__

#include "common.h"

#define MAGN_DS_SIZE 32

struct compass_cal {
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
};

struct gyro_cal {
    float bias_x, bias_y, bias_z;
    int count;
    float min_x, min_y, min_z;
    float max_x, max_y, max_z;
};

typedef double mat_input_t[MAGN_DS_SIZE][3];

void calibrate_compass (struct sensors_event_t* event, struct sensor_info_t* info);
void compass_read_data (struct sensor_info_t* info);
void compass_store_data (struct sensor_info_t* info);

void calibrate_gyro(struct sensors_event_t* event, struct sensor_info_t* info);
void gyro_cal_init(struct sensor_info_t* info);
void gyro_store_data (struct sensor_info_t* info);
#endif
