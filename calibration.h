/*
 * Copyright (C) 2014 Intel Corporation.
 */

#ifndef __CALIBRATION_H__
#define __CALIBRATION_H__

#define COMPASS_CALIBRATION_PATH "/data/compass.conf"
#define DS_SIZE 48
#define EPSILON 0.000000001


/* If no cal data is present - first calibration will
   use a more relaxed set of values to get an initial
   calibration faster */
#define FIRST_MIN_DIFF 1.0f
#define FIRST_MAX_SQR_ERR 3.5f
#define FIRST_LOOKBACK_COUNT 4

#define MIN_DIFF 1.5f
#define MAX_SQR_ERR 2.5f
#define LOOKBACK_COUNT 6


#ifdef DBG_RAW_DATA
#define RAW_DATA_FULL_PATH "/data/raw_compass_data_full_%d.txt"
#define RAW_DATA_SELECTED_PATH "/data/raw_compass_data_selected_%d.txt"
#endif

typedef struct {
    /* hard iron offsets */
    double offset[3][1];

    /* soft iron matrix */
    double w_invert[3][3];

    /* geomagnetic strength */
    double bfield;

} calibration_data;

typedef double mat_input_t[DS_SIZE][3];

void calibrate_compass (struct sensors_event_t* event, int64_t time);
void compass_read_data (const char* config_path);
void compass_store_data (const char* config_path);

#endif
