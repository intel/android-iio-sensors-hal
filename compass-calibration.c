#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <hardware/sensors.h>
#include <sys/stat.h>
#include <string.h>
#include <utils/Log.h>
#include "calibration.h"
#include "matrix-ops.h"


#ifdef DBG_RAW_DATA
#define MAX_RAW_DATA_COUNT 2000
static FILE *raw_data = NULL;
static FILE *raw_data_selected = NULL;
static FILE *compensation_data = NULL;
static int raw_data_count = 0;
int file_no = 0;
#endif

static float select_points[DS_SIZE][3];
static int select_point_count = 0;

static int calibrated = 0;
static calibration_data cal_data;

/* reset calibration algorithm */
static void reset_calibration ()
{
    int i,j;
    select_point_count = 0;
    for (i = 0; i < DS_SIZE; i++)
        for (j=0; j < 3; j++)
            select_points[i][j] = 0;
}

static double calc_square_err (calibration_data data)
{
    double err = 0;
    double raw[3][1], result[3][1], mat_diff[3][1];
    int i;

    for (i = 0; i < DS_SIZE; i++) {
        raw[0][0] = select_points[i][0];
        raw[1][0] = select_points[i][1];
        raw[2][0] = select_points[i][2];

        substract (3, 1, raw, data.offset, mat_diff);
        multiply(3, 3, 1, data.w_invert, mat_diff, result);

        double diff = sqrt(result[0][0] * result[0][0] + result[1][0] * result[1][0]
             + result[2][0] * result[2][0]) - data.bfield;

        err += diff * diff;
    }
    err /= DS_SIZE;
    return err;
}

// Given an real symmetric 3x3 matrix A, compute the eigenvalues
static void compute_eigenvalues(double mat[3][3], double* eig1, double* eig2, double* eig3)
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

static void calc_evector(double mat[3][3], double eig, double vec[3][1])
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
    int i,j;
    double h[DS_SIZE][9];
    double w[DS_SIZE][1];
    double h_trans[9][DS_SIZE];
    double p_temp1[9][9];
    double p_temp2[9][DS_SIZE];
    double temp1[3][3], temp[3][3];
    double temp1_inv[3][3];
    double temp2[3][1];
    double result[9][9];
    double p[9][1];
    double a[3][3], sqrt_evals[3][3], evecs[3][3], evecs_trans[3][3];
    double evec1[3][1], evec2[3][1], evec3[3][1];

    for (i = 0; i < DS_SIZE; i++) {
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
    transpose (DS_SIZE, 9, h, h_trans);
    multiply (9, DS_SIZE, 9, h_trans, h, result);
    invert (9, result, p_temp1);
    multiply (9, 9, DS_SIZE, p_temp1, h_trans, p_temp2);
    multiply (9, DS_SIZE, 1, p_temp2, w, p);

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
    multiply_scalar_inplace(3, 3, w_invert, *bfield);

    return 1;
}

static void compass_cal_init (FILE* data_file)
{

#ifdef DBG_RAW_DATA
    if (raw_data) {
        fclose(raw_data);
        raw_data = NULL;
    }

    if (raw_data_selected) {
        fclose(raw_data_selected);
        raw_data_selected = NULL;
    }

    char path[64];
    snprintf(path, 64, RAW_DATA_FULL_PATH, file_no);
    raw_data = fopen(path,"w+");
    snprintf(path, 64, RAW_DATA_SELECTED_PATH, file_no);
    raw_data_selected = fopen(path,"w+");
    file_no++;
    raw_data_count = 0;
#endif

    int data_count = 14;
    reset_calibration();
    calibrated = 0;

    if (data_file != NULL) {
       int ret = fscanf(data_file, "%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
            &calibrated, &cal_data.offset[0][0], &cal_data.offset[1][0], &cal_data.offset[2][0],
            &cal_data.w_invert[0][0], &cal_data.w_invert[0][1], &cal_data.w_invert[0][2],
            &cal_data.w_invert[1][0], &cal_data.w_invert[1][1], &cal_data.w_invert[1][2],
            &cal_data.w_invert[2][0], &cal_data.w_invert[2][1], &cal_data.w_invert[2][2],
            &cal_data.bfield);

        if (ret != data_count) {
            calibrated = 0;
        }
    }

    if (calibrated) {
        ALOGI("CompassCalibration: load old data, caldata: %f %f %f %f %f %f %f %f %f %f %f %f %f",
            cal_data.offset[0][0], cal_data.offset[1][0], cal_data.offset[2][0],
            cal_data.w_invert[0][0], cal_data.w_invert[0][1],cal_data.w_invert[0][2],cal_data.w_invert[1][0],
            cal_data.w_invert[1][1], cal_data.w_invert[1][2],cal_data.w_invert[2][0],cal_data.w_invert[2][1],
            cal_data.w_invert[2][2], cal_data.bfield);

    } else {
        cal_data.offset[0][0] = 0;
        cal_data.offset[1][0] = 0;
        cal_data.offset[2][0] = 0;

        cal_data.w_invert[0][0] = 1;
        cal_data.w_invert[1][0] = 0;
        cal_data.w_invert[2][0] = 0;
        cal_data.w_invert[0][1] = 0;
        cal_data.w_invert[1][1] = 1;
        cal_data.w_invert[2][1] = 0;
        cal_data.w_invert[0][2] = 0;
        cal_data.w_invert[1][2] = 0;
        cal_data.w_invert[2][2] = 1;

        cal_data.bfield = 0;
    }

}

static void compass_store_result(FILE* data_file)
{
    if (data_file == NULL)
        return;
    int ret = fprintf(data_file, "%d %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
        calibrated, cal_data.offset[0][0], cal_data.offset[1][0], cal_data.offset[2][0],
        cal_data.w_invert[0][0], cal_data.w_invert[0][1], cal_data.w_invert[0][2],
        cal_data.w_invert[1][0], cal_data.w_invert[1][1], cal_data.w_invert[1][2],
        cal_data.w_invert[2][0], cal_data.w_invert[2][1], cal_data.w_invert[2][2],
        cal_data.bfield);

    if (ret < 0)
        ALOGE ("compass calibration - store data failed!");
}

static int compass_collect (struct sensors_event_t* event, int64_t current_time)
{
    float data[3] = {event->magnetic.x, event->magnetic.y, event->magnetic.z};
    int index,j;
    int lookback_count;
    float min_diff;

#ifdef DBG_RAW_DATA
    if (raw_data && raw_data_count < MAX_RAW_DATA_COUNT) {
        fprintf(raw_data, "%f %f %f\n", (double)data[0], (double)data[1],
                 (double)data[2]);
        raw_data_count++;
    }

    if (raw_data && raw_data_count >= MAX_RAW_DATA_COUNT) {
        fclose(raw_data);
        raw_data = NULL;
    }
#endif

    lookback_count = calibrated ? LOOKBACK_COUNT : FIRST_LOOKBACK_COUNT;
    min_diff = calibrated ? MIN_DIFF : FIRST_MIN_DIFF;

    // For the current point to be accepted, each x/y/z value must be different enough
    // to the last several collected points
    if (select_point_count > 0 && select_point_count < DS_SIZE) {
        int lookback = lookback_count < select_point_count ? lookback_count : select_point_count;
        for (index = 0; index < lookback; index++){
            for (j = 0; j < 3; j++) {
                if (fabsf(data[j] - select_points[select_point_count-1-index][j]) < min_diff) {
                    ALOGV("CompassCalibration:point reject: [%f,%f,%f], selected_count=%d",
                       data[0], data[1], data[2], select_point_count);
                       return 0;
                }
            }
        }
    }

    if (select_point_count < DS_SIZE) {
        memcpy(select_points[select_point_count], data, sizeof(float) * 3);
        select_point_count++;
        ALOGV("CompassCalibration:point collected [%f,%f,%f], selected_count=%d",
            (double)data[0], (double)data[1], (double)data[2], select_point_count);
#ifdef DBG_RAW_DATA
        if (raw_data_selected) {
            fprintf(raw_data_selected, "%f %f %f\n", (double)data[0], (double)data[1], (double)data[2]);
        }
#endif
    }
    return 1;
}

static void compass_compute_cal (struct sensors_event_t* event)
{
    if (!calibrated)
        return;

    double result[3][1], raw[3][1], diff[3][1];

    raw[0][0] = event->magnetic.x;
    raw[1][0] = event->magnetic.y;
    raw[2][0] = event->magnetic.z;

    substract(3, 1, raw, cal_data.offset, diff);
    multiply (3, 3, 1, cal_data.w_invert, diff, result);

    event->magnetic.x = event->data[0] = result[0][0];
    event->magnetic.y = event->data[1] = result[1][0];
    event->magnetic.z = event->data[2] = result[2][0];
}

static int compass_ready ()
{
    mat_input_t mat;
    int i;
    float max_sqr_err;

    if (select_point_count < DS_SIZE)
        return calibrated;

    max_sqr_err = calibrated ? MAX_SQR_ERR : FIRST_MAX_SQR_ERR;

    /* enough points have been collected, do the ellipsoid calibration */
    for (i = 0; i < DS_SIZE; i++) {
       mat[i][0] = select_points[i][0];
       mat[i][1] = select_points[i][1];
       mat[i][2] = select_points[i][2];
    }

    /* check if result is good */
    calibration_data new_cal_data;
    if (ellipsoid_fit(mat, new_cal_data.offset, new_cal_data.w_invert, &new_cal_data.bfield)) {
        double new_err = calc_square_err (new_cal_data);
        ALOGV("new err is %f, max sqr err id %f", new_err,max_sqr_err);
        if (new_err < max_sqr_err) {
            double err = calc_square_err(cal_data);
            if (new_err < err) {
                /* new cal data is better, so we switch to the new */
                cal_data = new_cal_data;
                calibrated = 1;
                ALOGV("CompassCalibration: ready check success, caldata: %f %f %f %f %f %f %f %f %f %f %f %f %f, err %f",
                    cal_data.offset[0][0], cal_data.offset[1][0], cal_data.offset[2][0], cal_data.w_invert[0][0],
                    cal_data.w_invert[0][1], cal_data.w_invert[0][2], cal_data.w_invert[1][0],cal_data.w_invert[1][1],
                    cal_data.w_invert[1][2], cal_data.w_invert[2][0], cal_data.w_invert[2][1], cal_data.w_invert[2][2],
                    cal_data.bfield, new_err);
            }
        }
    }
    reset_calibration();
    return calibrated;
}

void calibrate_compass (struct sensors_event_t* event, int64_t current_time)
{
    long current_time_ms = current_time / 1000000;
    compass_collect (event, current_time_ms);
    if (compass_ready()) {
        compass_compute_cal (event);
        event->magnetic.status = SENSOR_STATUS_ACCURACY_HIGH;
    } else {
        event->magnetic.status = SENSOR_STATUS_ACCURACY_LOW;
    }
}

void compass_read_data (const char* config_file)
{
    FILE* data_file = fopen (config_file, "r");

    compass_cal_init(data_file);
    if (data_file)
        fclose(data_file);
}

void compass_store_data (const char* config_file)
{
    FILE* data_file = fopen (config_file, "w");

    compass_store_result(data_file);
    if (data_file)
        fclose(data_file);

}
