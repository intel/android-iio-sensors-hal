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

#define MAGNETIC_LOW 960 /* 31 micro tesla squared */
#define MAGNETIC_HIGH 3600 /* 60 micro tesla squared */

#ifdef DBG_RAW_DATA
#define RAW_DATA_FULL_PATH "/data/raw_compass_data_full_%d.txt"
#define RAW_DATA_SELECTED_PATH "/data/raw_compass_data_selected_%d.txt"
#endif

/* gyro defines */
#define GYRO_MAX_ERR 0.05f
#define GYRO_DS_SIZE 100
#define GYRO_CALIBRATION_PATH "/data/gyro.conf"

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
    float bias_x, bias_y, bias_z;
    int count;
    float min_x, min_y, min_z;
    float max_x, max_y, max_z;
};

typedef double mat_input_t[DS_SIZE][3];

void calibrate_compass (struct sensors_event_t* event, struct sensor_info_t* info);
void compass_read_data (struct sensor_info_t* info);
void compass_store_data (struct sensor_info_t* info);

void calibrate_gyro(struct sensors_event_t* event, struct sensor_info_t* info);
void gyro_cal_init(struct sensor_info_t* info);
void gyro_store_data (struct sensor_info_t* info);
#endif
