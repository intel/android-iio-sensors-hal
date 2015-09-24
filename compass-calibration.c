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

#include <math.h>
#include <hardware/sensors.h>
#include <stdio.h>
#include <utils/Log.h>
#include "calibration.h"
#include "matrix-ops.h"
#include "description.h"


/* Compass defines */
#define COMPASS_CALIBRATION_PATH    "/data/compass.conf"
#define EPSILON                     0.000000001

#define MAGNETIC_LOW                960 /* 31 micro tesla squared */
#define CAL_STEPS                   5
#define CAL_VERSION 1.0

/* We'll have multiple calibration levels so that we can provide an estimation as fast as possible */
static const float          min_diffs       [CAL_STEPS] = {0.2,  0.25, 0.4, 0.6, 1.0};
static const float          max_sqr_errs    [CAL_STEPS] = {10.0, 10.0, 8.0, 5.0, 3.5};
static const unsigned int   lookback_counts [CAL_STEPS] = {2,    3,    4,   5,   6  };


/* Reset calibration algorithm */
static void reset_sample (compass_cal_t* data)
{
    int i,j;
    data->sample_count = 0;
    for (i = 0; i < MAGN_DS_SIZE; i++)
        for (j=0; j < 3; j++)
            data->sample[i][j] = 0;

    data->average[0] = data->average[1] = data->average[2] = 0;
}


static double calc_square_err (compass_cal_t* data)
{
    double err = 0;
    double raw[3][1], result[3][1], mat_diff[3][1];
    int i;
    float stdev[3] = {0,0,0};
    double diff;

    for (i = 0; i < MAGN_DS_SIZE; i++) {
        raw[0][0] = data->sample[i][0];
        raw[1][0] = data->sample[i][1];
        raw[2][0] = data->sample[i][2];

        stdev[0] += (raw[0][0] - data->average[0]) * (raw[0][0] - data->average[0]);
        stdev[1] += (raw[1][0] - data->average[1]) * (raw[1][0] - data->average[1]);
        stdev[2] += (raw[2][0] - data->average[2]) * (raw[2][0] - data->average[2]);

        substract (3, 1, raw, data->offset, mat_diff);
        multiply(3, 3, 1, data->w_invert, mat_diff, result);

        diff = sqrt(result[0][0] * result[0][0] + result[1][0] * result[1][0] + result[2][0] * result[2][0]) - data->bfield;

        err += diff * diff;
    }

    stdev[0] = sqrt(stdev[0] / MAGN_DS_SIZE);
    stdev[1] = sqrt(stdev[1] / MAGN_DS_SIZE);
    stdev[2] = sqrt(stdev[2] / MAGN_DS_SIZE);

    /* A sanity check - if we have too little variation for an axis it's best to reject the calibration than risking a wrong calibration */
    if (stdev[0] <= 1 || stdev[1] <= 1 || stdev[2] <= 1)
        return max_sqr_errs[0];

    err /= MAGN_DS_SIZE;
    return err;
}


/* Given an real symmetric 3x3 matrix A, compute the eigenvalues */
static void compute_eigenvalues (double mat[3][3], double* eig1, double* eig2, double* eig3)
{
    double p = mat[0][1] * mat[0][1] + mat[0][2] * mat[0][2] + mat[1][2] * mat[1][2];

    if (p < EPSILON) {
        *eig1 = mat[0][0];
        *eig2 = mat[1][1];
        *eig3 = mat[2][2];
        return;
    }

    double q = (mat[0][0] + mat[1][1] + mat[2][2]) / 3;
    double temp1 = mat[0][0] - q;
    double temp2 = mat[1][1] - q;
    double temp3 = mat[2][2] - q;

    p = temp1 * temp1 + temp2 * temp2 + temp3 * temp3 + 2 * p;
    p = sqrt(p / 6);

    double mat2[3][3];
    assign(3, 3, mat, mat2);
    mat2[0][0] -= q;
    mat2[1][1] -= q;
    mat2[2][2] -= q;
    multiply_scalar_inplace(3, 3, mat2, 1/p);

    double r = (mat2[0][0] * mat2[1][1] * mat2[2][2] + mat2[0][1] * mat2[1][2] * mat2[2][0]
        + mat2[0][2] * mat2[1][0] * mat2[2][1] - mat2[0][2] * mat2[1][1] * mat2[2][0]
        - mat2[0][0] * mat2[1][2] * mat2[2][1] - mat2[0][1] * mat2[1][0] * mat2[2][2]) / 2;

    double phi;
    if (r <= -1.0)
        phi = M_PI/3;
    else if (r >= 1.0)
            phi = 0;
        else
            phi = acos(r) / 3;

    *eig3 = q + 2 * p * cos(phi);
    *eig1 = q + 2 * p * cos(phi + 2 * M_PI / 3);
    *eig2 = 3 * q - *eig1 - *eig3;
}


static void calc_evector (double mat[3][3], double eig, double vec[3][1])
{
    double h[3][3];
    double x_tmp[2][2];
    assign(3, 3, mat, h);
    h[0][0] -= eig;
    h[1][1] -= eig;
    h[2][2] -= eig;

    double x[2][2];
    x[0][0] = h[1][1];
    x[0][1] = h[1][2];
    x[1][0] = h[2][1];
    x[1][1] = h[2][2];
    invert(2, x, x_tmp);
    assign(2, 2, x_tmp, x);

    double temp1 = x[0][0] * (-h[1][0]) + x[0][1] * (-h[2][0]);
    double temp2 = x[1][0] * (-h[1][0]) + x[1][1] * (-h[2][0]);
    double norm = sqrt(1 + temp1 * temp1 + temp2 * temp2);

    vec[0][0] = 1.0 / norm;
    vec[1][0] = temp1 / norm;
    vec[2][0] = temp2 / norm;
}


static int ellipsoid_fit (mat_input_t m, double offset[3][1], double w_invert[3][3], double* bfield)
{
    int i;
    double h[MAGN_DS_SIZE][9];
    double w[MAGN_DS_SIZE][1];
    double h_trans[9][MAGN_DS_SIZE];
    double p_temp1[9][9];
    double p_temp2[9][MAGN_DS_SIZE];
    double temp1[3][3], temp[3][3];
    double temp1_inv[3][3];
    double temp2[3][1];
    double result[9][9];
    double p[9][1];
    double a[3][3], sqrt_evals[3][3], evecs[3][3], evecs_trans[3][3];
    double evec1[3][1], evec2[3][1], evec3[3][1];

    for (i = 0; i < MAGN_DS_SIZE; i++) {
        w[i][0] = m[i][0] * m[i][0];
        h[i][0] = m[i][0];
        h[i][1] = m[i][1];
        h[i][2] = m[i][2];
        h[i][3] = -1 * m[i][0] * m[i][1];
        h[i][4] = -1 * m[i][0] * m[i][2];
        h[i][5] = -1 * m[i][1] * m[i][2];
        h[i][6] = -1 * m[i][1] * m[i][1];
        h[i][7] = -1 * m[i][2] * m[i][2];
        h[i][8] = 1;
    }

    transpose (MAGN_DS_SIZE, 9, h, h_trans);
    multiply (9, MAGN_DS_SIZE, 9, h_trans, h, result);
    invert (9, result, p_temp1);
    multiply (9, 9, MAGN_DS_SIZE, p_temp1, h_trans, p_temp2);
    multiply (9, MAGN_DS_SIZE, 1, p_temp2, w, p);

    temp1[0][0] = 2;
    temp1[0][1] = p[3][0];
    temp1[0][2] = p[4][0];
    temp1[1][0] = p[3][0];
    temp1[1][1] = 2 * p[6][0];
    temp1[1][2] = p[5][0];
    temp1[2][0] = p[4][0];
    temp1[2][1] = p[5][0];
    temp1[2][2] = 2 * p[7][0];

    temp2[0][0] = p[0][0];
    temp2[1][0] = p[1][0];
    temp2[2][0] = p[2][0];

    invert(3, temp1, temp1_inv);
    multiply(3, 3, 1, temp1_inv, temp2, offset);
    double off_x = offset[0][0];
    double off_y = offset[1][0];
    double off_z = offset[2][0];

    a[0][0] = 1.0 / (p[8][0] + off_x * off_x + p[6][0] * off_y * off_y
            + p[7][0] * off_z * off_z + p[3][0] * off_x * off_y
            + p[4][0] * off_x * off_z + p[5][0] * off_y * off_z);

    a[0][1] = p[3][0] * a[0][0] / 2;
    a[0][2] = p[4][0] * a[0][0] / 2;
    a[1][2] = p[5][0] * a[0][0] / 2;
    a[1][1] = p[6][0] * a[0][0];
    a[2][2] = p[7][0] * a[0][0];
    a[2][1] = a[1][2];
    a[1][0] = a[0][1];
    a[2][0] = a[0][2];

    double eig1 = 0, eig2 = 0, eig3 = 0;
    compute_eigenvalues(a, &eig1, &eig2, &eig3);

    if (eig1 <=0 || eig2 <= 0 || eig3 <= 0)
        return 0;

    sqrt_evals[0][0] = sqrt(eig1);
    sqrt_evals[1][0] = 0;
    sqrt_evals[2][0] = 0;
    sqrt_evals[0][1] = 0;
    sqrt_evals[1][1] = sqrt(eig2);
    sqrt_evals[2][1] = 0;
    sqrt_evals[0][2] = 0;
    sqrt_evals[1][2] = 0;
    sqrt_evals[2][2] = sqrt(eig3);

    calc_evector(a, eig1, evec1);
    calc_evector(a, eig2, evec2);
    calc_evector(a, eig3, evec3);

    evecs[0][0] = evec1[0][0];
    evecs[1][0] = evec1[1][0];
    evecs[2][0] = evec1[2][0];
    evecs[0][1] = evec2[0][0];
    evecs[1][1] = evec2[1][0];
    evecs[2][1] = evec2[2][0];
    evecs[0][2] = evec3[0][0];
    evecs[1][2] = evec3[1][0];
    evecs[2][2] = evec3[2][0];

    multiply (3, 3, 3, evecs, sqrt_evals, temp1);
    transpose(3, 3, evecs, evecs_trans);
    multiply (3, 3, 3, temp1, evecs_trans, temp);
    transpose (3, 3, temp, w_invert);

    *bfield = pow(sqrt(1/eig1) * sqrt(1/eig2) * sqrt(1/eig3), 1.0/3.0);

    if (*bfield < 0)
        return 0;

    multiply_scalar_inplace(3, 3, w_invert, *bfield);

    return 1;
}


static void compass_cal_init (FILE* data_file, sensor_info_t* info)
{
    compass_cal_t* cal_data = (compass_cal_t*) info->cal_data;
    int cal_steps = (info->max_cal_level && info->max_cal_level <= CAL_STEPS) ? info->max_cal_level : CAL_STEPS;
    float version;

    if (cal_data == NULL)
        return;

    int data_count = 15;
    reset_sample(cal_data);

    if (!info->cal_level && data_file != NULL) {
       int ret = fscanf(data_file, "%f %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
            &version, &info->cal_level,
            &cal_data->offset[0][0], &cal_data->offset[1][0], &cal_data->offset[2][0],
            &cal_data->w_invert[0][0], &cal_data->w_invert[0][1], &cal_data->w_invert[0][2],
            &cal_data->w_invert[1][0], &cal_data->w_invert[1][1], &cal_data->w_invert[1][2],
            &cal_data->w_invert[2][0], &cal_data->w_invert[2][1], &cal_data->w_invert[2][2],
            &cal_data->bfield);

        if (ret != data_count || info->cal_level >= cal_steps || version != CAL_VERSION)
            info->cal_level = 0;
    }

    if (info->cal_level) {
        ALOGV("CompassCalibration: load old data, caldata: %f %f %f %f %f %f %f %f %f %f %f %f %f",
            cal_data->offset[0][0], cal_data->offset[1][0], cal_data->offset[2][0],
            cal_data->w_invert[0][0], cal_data->w_invert[0][1], cal_data->w_invert[0][2], cal_data->w_invert[1][0],
            cal_data->w_invert[1][1], cal_data->w_invert[1][2], cal_data->w_invert[2][0], cal_data->w_invert[2][1],
            cal_data->w_invert[2][2], cal_data->bfield);
    } else {
        cal_data->offset[0][0] = 0;
        cal_data->offset[1][0] = 0;
        cal_data->offset[2][0] = 0;

        cal_data->w_invert[0][0] = 1;
        cal_data->w_invert[1][0] = 0;
        cal_data->w_invert[2][0] = 0;
        cal_data->w_invert[0][1] = 0;
        cal_data->w_invert[1][1] = 1;
        cal_data->w_invert[2][1] = 0;
        cal_data->w_invert[0][2] = 0;
        cal_data->w_invert[1][2] = 0;
        cal_data->w_invert[2][2] = 1;

        cal_data->bfield = 0;
    }
}


static void compass_store_result (FILE* data_file, sensor_info_t* info)
{
    compass_cal_t* cal_data = (compass_cal_t*) info->cal_data;

    if (data_file == NULL || cal_data == NULL)
        return;

    int ret = fprintf(data_file, "%f %d %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
        CAL_VERSION, info->cal_level,
        cal_data->offset[0][0], cal_data->offset[1][0], cal_data->offset[2][0],
        cal_data->w_invert[0][0], cal_data->w_invert[0][1], cal_data->w_invert[0][2],
        cal_data->w_invert[1][0], cal_data->w_invert[1][1], cal_data->w_invert[1][2],
        cal_data->w_invert[2][0], cal_data->w_invert[2][1], cal_data->w_invert[2][2],
        cal_data->bfield);

    if (ret < 0)
        ALOGE ("Compass calibration - store data failed!");
}


static int compass_collect (sensors_event_t* event, sensor_info_t* info)
{
    float data[3] = {event->magnetic.x, event->magnetic.y, event->magnetic.z};
    unsigned int index,j;
    unsigned int lookback_count;
    float min_diff;

    compass_cal_t* cal_data = (compass_cal_t*) info->cal_data;

    if (cal_data == NULL)
        return -1;

    /* Discard the point if not valid */
    if (data[0] == 0 || data[1] == 0 || data[2] == 0)
        return -1;

    lookback_count = lookback_counts[info->cal_level];
    min_diff = min_diffs[info->cal_level];

    /* For the current point to be accepted, each x/y/z value must be different enough to the last several collected points */
    if (cal_data->sample_count > 0 && cal_data->sample_count < MAGN_DS_SIZE) {
        unsigned int lookback = lookback_count < cal_data->sample_count ? lookback_count : cal_data->sample_count;
        for (index = 0; index < lookback; index++)
            for (j = 0; j < 3; j++)
                if (fabsf(data[j] - cal_data->sample[cal_data->sample_count-1-index][j]) < min_diff) {
                    ALOGV("CompassCalibration:point reject: [%f,%f,%f], selected_count=%d", data[0], data[1], data[2], cal_data->sample_count);
                    return 0;
                }
    }

    if (cal_data->sample_count < MAGN_DS_SIZE) {
        memcpy(cal_data->sample[cal_data->sample_count], data, sizeof(float) * 3);
        cal_data->sample_count++;
        cal_data->average[0] += data[0];
        cal_data->average[1] += data[1];
        cal_data->average[2] += data[2];
        ALOGV("CompassCalibration:point collected [%f,%f,%f], selected_count=%d", (double)data[0], (double)data[1], (double)data[2], cal_data->sample_count);
    }
    return 1;
}


static void scale_event (sensors_event_t* event)
{
    float sqr_norm = 0;
    float sanity_norm = 0;
    float scale = 1;

    sqr_norm = (event->magnetic.x * event->magnetic.x +
                event->magnetic.y * event->magnetic.y +
                event->magnetic.z * event->magnetic.z);

    if (sqr_norm < MAGNETIC_LOW)
        sanity_norm = MAGNETIC_LOW;

    if (sanity_norm && sqr_norm) {
        scale = sanity_norm / sqr_norm;
        scale = sqrt(scale);
        event->magnetic.x = event->magnetic.x * scale;
        event->magnetic.y = event->magnetic.y * scale;
        event->magnetic.z = event->magnetic.z * scale;
    }
}


static void compass_compute_cal (sensors_event_t* event, sensor_info_t* info)
{
    compass_cal_t* cal_data = (compass_cal_t*) info->cal_data;
    double result[3][1], raw[3][1], diff[3][1];

    if (!info->cal_level || cal_data == NULL)
        return;

    raw[0][0] = event->magnetic.x;
    raw[1][0] = event->magnetic.y;
    raw[2][0] = event->magnetic.z;

    substract(3, 1, raw, cal_data->offset, diff);
    multiply (3, 3, 1, cal_data->w_invert, diff, result);

    event->magnetic.x = event->data[0] = result[0][0];
    event->magnetic.y = event->data[1] = result[1][0];
    event->magnetic.z = event->data[2] = result[2][0];

    scale_event(event);
}


static int compass_ready (sensor_info_t* info)
{
    mat_input_t mat;
    int i;
    float max_sqr_err;

    compass_cal_t* cal_data = (compass_cal_t*) info->cal_data;
    compass_cal_t new_cal_data;

    /*
     * Some sensors take unrealistically long to calibrate at higher levels. We'll use a max_cal_level if we have such a property setup,
     * or go with the default settings if not.
     */
    int cal_steps = (info->max_cal_level && info->max_cal_level <= CAL_STEPS) ? info->max_cal_level : CAL_STEPS;

    if (cal_data->sample_count < MAGN_DS_SIZE)
        return info->cal_level;

    max_sqr_err = max_sqr_errs[info->cal_level];

    /* Enough points have been collected, do the ellipsoid calibration */

    /* Compute average per axis */
    cal_data->average[0] /= MAGN_DS_SIZE;
    cal_data->average[1] /= MAGN_DS_SIZE;
    cal_data->average[2] /= MAGN_DS_SIZE;

    for (i = 0; i < MAGN_DS_SIZE; i++) {
       mat[i][0] = cal_data->sample[i][0];
       mat[i][1] = cal_data->sample[i][1];
       mat[i][2] = cal_data->sample[i][2];
    }

    /* Check if result is good. The sample data must remain the same */
    new_cal_data = *cal_data;

    if (ellipsoid_fit(mat, new_cal_data.offset, new_cal_data.w_invert, &new_cal_data.bfield)) {
        double new_err = calc_square_err (&new_cal_data);
        ALOGI("new err is %f, max sqr err id %f", new_err,max_sqr_err);
        if (new_err < max_sqr_err) {
            double err = calc_square_err(cal_data);
            if (new_err < err) {
                /* New cal data is better, so we switch to the new */
                memcpy(cal_data->offset, new_cal_data.offset, sizeof(cal_data->offset));
                memcpy(cal_data->w_invert, new_cal_data.w_invert, sizeof(cal_data->w_invert));
                cal_data->bfield = new_cal_data.bfield;
                if (info->cal_level < (cal_steps - 1))
                    info->cal_level++;
                ALOGV("CompassCalibration: ready check success, caldata: %f %f %f %f %f %f %f %f %f %f %f %f %f, err %f",
                    cal_data->offset[0][0], cal_data->offset[1][0], cal_data->offset[2][0], cal_data->w_invert[0][0],
                    cal_data->w_invert[0][1], cal_data->w_invert[0][2], cal_data->w_invert[1][0], cal_data->w_invert[1][1],
                    cal_data->w_invert[1][2], cal_data->w_invert[2][0], cal_data->w_invert[2][1], cal_data->w_invert[2][2],
                    cal_data->bfield, new_err);
            }
        }
    }
    reset_sample(cal_data);
    return info->cal_level;
}


void calibrate_compass (int s, sensors_event_t* event)
{
    int cal_level;

    /* Calibration is continuous */
    compass_collect (event, &sensor[s]);

    cal_level = compass_ready(&sensor[s]);

    switch (cal_level) {
        case 0:
            scale_event(event);
            event->magnetic.status = SENSOR_STATUS_UNRELIABLE;
            break;

        case 1:
            compass_compute_cal (event, &sensor[s]);
            event->magnetic.status = SENSOR_STATUS_ACCURACY_LOW;
            break;

        case 2:
            compass_compute_cal (event, &sensor[s]);
            event->magnetic.status = SENSOR_STATUS_ACCURACY_MEDIUM;
            break;

        default:
            compass_compute_cal (event, &sensor[s]);
            event->magnetic.status = SENSOR_STATUS_ACCURACY_HIGH;
            break;
    }
}

void compass_read_data (int s)
{
    FILE* data_file = fopen (COMPASS_CALIBRATION_PATH, "r");

    compass_cal_init(data_file, &sensor[s]);

    if (data_file)
        fclose(data_file);
}


void compass_store_data (int s)
{
    FILE* data_file = fopen (COMPASS_CALIBRATION_PATH, "w");

    compass_store_result(data_file, &sensor[s]);

    if (data_file)
        fclose(data_file);
}
