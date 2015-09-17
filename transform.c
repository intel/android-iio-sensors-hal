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

#include <stdlib.h>
#include <math.h>
#include <utils/Log.h>
#include <cutils/properties.h>
#include <hardware/sensors.h>
#include "calibration.h"
#include "common.h"
#include "description.h"
#include "transform.h"
#include "utils.h"
#include "filtering.h"
#include "enumeration.h"

#define	GYRO_MIN_SAMPLES 5 /* Drop first few gyro samples after enable */


/*----------------------------------------------------------------------------*/


/* Macros related to Intel Sensor Hub */

#define GRAVITY 	9.80665

/* 720 LSG = 1G */
#define LSG		1024.0
#define NUMOFACCDATA	8.0

/* Conversion of acceleration data to SI units (m/s^2) */
#define CONVERT_A	(GRAVITY_EARTH / LSG / NUMOFACCDATA)
#define CONVERT_A_X(x)	((float(x) / 1000) * (GRAVITY * -1.0))
#define CONVERT_A_Y(x)	((float(x) / 1000) * (GRAVITY * 1.0))
#define CONVERT_A_Z(x)	((float(x) / 1000) * (GRAVITY * 1.0))

/* Conversion of magnetic data to uT units */
#define CONVERT_M	(1.0 / 6.6)
#define CONVERT_M_X	(-CONVERT_M)
#define CONVERT_M_Y	(-CONVERT_M)
#define CONVERT_M_Z	(CONVERT_M)

/* Conversion of orientation data to degree units */
#define CONVERT_O	(1.0 / 64)
#define CONVERT_O_A	(CONVERT_O)
#define CONVERT_O_P	(CONVERT_O)
#define CONVERT_O_R	(-CONVERT_O)

/* Conversion of gyro data to SI units (radian/sec) */
#define CONVERT_GYRO	(2000.0 / 32767 * M_PI / 180)
#define CONVERT_GYRO_X	(-CONVERT_GYRO)
#define CONVERT_GYRO_Y	(-CONVERT_GYRO)
#define CONVERT_GYRO_Z	(CONVERT_GYRO)

#define BIT(x) (1 << (x))

#define PROXIMITY_THRESHOLD 1

inline unsigned int set_bit_range (int start, int end)
{
	int i;
	unsigned int value = 0;

	for (i = start; i < end; ++i)
		value |= BIT(i);

	return value;
}

inline float convert_from_vtf_format (int size, int exponent, unsigned int value)
{
	int divider = 1;
	int i;
	float sample;
	float mul = 1.0;

	value = value & set_bit_range(0, size * 8);

	if (value & BIT(size*8-1)) {
		value =  ((1LL << (size * 8)) - value);
		mul = -1.0;
	}

	sample = value * 1.0;

	if (exponent < 0) {
		exponent = abs(exponent);
		for (i = 0; i < exponent; ++i)
			divider = divider * 10;

		return mul * sample/divider;
	}

	return mul * sample * pow(10.0, exponent);
}

/* Platform sensor orientation */
#define DEF_ORIENT_ACCEL_X	-1
#define DEF_ORIENT_ACCEL_Y	-1
#define DEF_ORIENT_ACCEL_Z	-1

#define DEF_ORIENT_GYRO_X	1
#define DEF_ORIENT_GYRO_Y	1
#define DEF_ORIENT_GYRO_Z	1

/* G to m/s^2 */
#define CONVERT_FROM_VTF16(s,d,x)      convert_from_vtf_format(s,d,x)
#define CONVERT_A_G_VTF16E14_X(s,d,x)  (DEF_ORIENT_ACCEL_X * convert_from_vtf_format(s,d,x) * GRAVITY)
#define CONVERT_A_G_VTF16E14_Y(s,d,x)  (DEF_ORIENT_ACCEL_Y * convert_from_vtf_format(s,d,x) * GRAVITY)
#define CONVERT_A_G_VTF16E14_Z(s,d,x)  (DEF_ORIENT_ACCEL_Z * convert_from_vtf_format(s,d,x) * GRAVITY)

/* Degree/sec to radian/sec */
#define CONVERT_G_D_VTF16E14_X(s,d,x)  (DEF_ORIENT_GYRO_X * convert_from_vtf_format(s,d,x) * M_PI / 180)
#define CONVERT_G_D_VTF16E14_Y(s,d,x)  (DEF_ORIENT_GYRO_Y * convert_from_vtf_format(s,d,x) * M_PI / 180)
#define CONVERT_G_D_VTF16E14_Z(s,d,x)  (DEF_ORIENT_GYRO_Z * convert_from_vtf_format(s,d,x) * M_PI / 180)

/* Milli gauss to micro tesla */
#define CONVERT_M_MG_VTF16E14_X(s,d,x) (convert_from_vtf_format(s,d,x) / 10)
#define CONVERT_M_MG_VTF16E14_Y(s,d,x) (convert_from_vtf_format(s,d,x) / 10)
#define CONVERT_M_MG_VTF16E14_Z(s,d,x) (convert_from_vtf_format(s,d,x) / 10)


static int64_t sample_as_int64 (unsigned char* sample, datum_info_t* type)
{
	uint64_t u64;
	int i;
	int zeroed_bits = type->storagebits - type->realbits;
	uint64_t sign_mask;
	uint64_t value_mask;

	u64 = 0;

	if (type->endianness == 'b')
		for (i=0; i<type->storagebits/8; i++)
			u64 = (u64 << 8) | sample[i];
	else
		for (i=type->storagebits/8 - 1; i>=0; i--)
			u64 = (u64 << 8) | sample[i];

	u64 = (u64 >> type->shift) & (~0ULL >> zeroed_bits);

	if (type->sign == 'u')
		return (int64_t) u64; /* We don't handle unsigned 64 bits int */

	/* Signed integer */

	switch (type->realbits) {
		case 0 ... 1:
			return 0;

		case 8:
			return (int64_t) (int8_t) u64;

		case 16:
			return (int64_t) (int16_t) u64;

		case 32:
			return (int64_t) (int32_t) u64;

		case 64:
			return (int64_t) u64;

		default:
			sign_mask = 1 << (type->realbits-1);
			value_mask = sign_mask - 1;

			if (u64 & sign_mask)
				return - ((~u64 & value_mask) + 1);	/* Negative value: return 2-complement */
			else
				return (int64_t) u64;			/* Positive value */
	}
}


static void reorder_fields (float* data, unsigned char map[MAX_CHANNELS])
{
	int i;
	float temp[MAX_CHANNELS];

	for (i=0; i<MAX_CHANNELS; i++)
		temp[i] = data[map[i]];

	for (i=0; i<MAX_CHANNELS; i++)
		data[i] = temp[i];
}

static void mount_correction (float* data, float mm[9])
{
	int i;
	float temp[3];

	for (i=0; i<3; i++)
		temp[i] = data[0] * mm[i * 3] + data[1] * mm[i * 3 + 1] + data[2] * mm[i * 3 + 2];

	for (i=0; i<3; i++)
		data[i] = temp[i];
}

static void clamp_gyro_readings_to_zero (int s, sensors_event_t* data)
{
	float x, y, z;
	float near_zero;

	x = data->data[0];
	y = data->data[1];
	z = data->data[2];

	/* If we're calibrated, don't filter out as much */
	if (sensor[s].cal_level > 0)
		near_zero = 0.02; /* rad/s */
	else
		near_zero = 0.1;

	/* If motion on all axes is small enough */
	if (fabs(x) < near_zero && fabs(y) < near_zero && fabs(z) < near_zero) {

		/*
		 * Report that we're not moving at all... but not exactly zero as composite sensors (orientation, rotation vector) don't
		 * seem to react very well to it.
		 */

		data->data[0] *= 0.000001;
		data->data[1] *= 0.000001;
		data->data[2] *= 0.000001;
	}
}


static void process_event_gyro_uncal (int s, int i, sensors_event_t* data)
{
	gyro_cal_t* gyro_data;

	if (sensor[s].type == SENSOR_TYPE_GYROSCOPE) {
		gyro_data = (gyro_cal_t*) sensor[s].cal_data;

		memcpy(&sensor[i].sample, data, sizeof(sensors_event_t));

		sensor[i].sample.type = SENSOR_TYPE_GYROSCOPE_UNCALIBRATED;
		sensor[i].sample.sensor = s;

		sensor[i].sample.data[0] = data->data[0] + gyro_data->bias_x;
		sensor[i].sample.data[1] = data->data[1] + gyro_data->bias_y;
		sensor[i].sample.data[2] = data->data[2] + gyro_data->bias_z;

		sensor[i].sample.uncalibrated_gyro.bias[0] = gyro_data->bias_x;
		sensor[i].sample.uncalibrated_gyro.bias[1] = gyro_data->bias_y;
		sensor[i].sample.uncalibrated_gyro.bias[2] = gyro_data->bias_z;

		sensor[i].report_pending = 1;
	}
}

static void process_event_magn_uncal (int s, int i, sensors_event_t* data)
{
	compass_cal_t* magn_data;

	if (sensor[s].type == SENSOR_TYPE_MAGNETIC_FIELD) {
		magn_data = (compass_cal_t*) sensor[s].cal_data;

		memcpy(&sensor[i].sample, data, sizeof(sensors_event_t));

		sensor[i].sample.type = SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED;
		sensor[i].sample.sensor = s;

		sensor[i].sample.data[0] = data->data[0] + magn_data->offset[0][0];
		sensor[i].sample.data[1] = data->data[1] + magn_data->offset[1][0];
		sensor[i].sample.data[2] = data->data[2] + magn_data->offset[2][0];

		sensor[i].sample.uncalibrated_magnetic.bias[0] = magn_data->offset[0][0];
		sensor[i].sample.uncalibrated_magnetic.bias[1] = magn_data->offset[1][0];
		sensor[i].sample.uncalibrated_magnetic.bias[2] = magn_data->offset[2][0];

		sensor[i].report_pending = 1;
	}
}

static void process_event (int s, sensors_event_t* data)
{
	/*
	 * This gets the real event (post process - calibration, filtering & co.) and makes it into a virtual one.
	 * The specific processing function for each sensor will populate the necessary fields and set up the report pending flag.
	 */

	 int i;

	 /* Go through out virtual sensors and check if we can use this event */
	 for (i = 0; i < sensor_count; i++)
		switch (sensor[i].type) {
			case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
				process_event_gyro_uncal(s, i, data);
				break;
			case SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED:
				process_event_magn_uncal(s, i, data);
				break;
 			default:
				break;
		}
}


static int finalize_sample_default (int s, sensors_event_t* data)
{
	/* Swap fields if we have a custom channel ordering on this sensor */
	if (sensor[s].quirks & QUIRK_FIELD_ORDERING)
		reorder_fields(data->data, sensor[s].order);
	if (sensor[s].quirks & QUIRK_MOUNTING_MATRIX)
		mount_correction(data->data, sensor[s].mounting_matrix);

	sensor[s].event_count++;

	switch (sensor[s].type) {
		case SENSOR_TYPE_ACCELEROMETER:
			/* Always consider the accelerometer accurate */
			data->acceleration.status = SENSOR_STATUS_ACCURACY_HIGH;
			if (sensor[s].quirks & QUIRK_BIASED)
				calibrate_accel(s, data);
			denoise(s, data);
			break;

		case SENSOR_TYPE_MAGNETIC_FIELD:
			calibrate_compass (s, data);
			denoise(s, data);
			break;

		case SENSOR_TYPE_GYROSCOPE:

			/* Report medium accuracy by default ; higher accuracy levels will be reported once, and if, we achieve  calibration. */
			data->gyro.status = SENSOR_STATUS_ACCURACY_MEDIUM;

			/*
			 * We're only trying to calibrate data from continuously firing gyroscope drivers, as motion based ones use
			 * movement thresholds that may lead us to incorrectly estimate bias.
			 */
			if (sensor[s].selected_trigger !=
				sensor[s].motion_trigger_name)
					calibrate_gyro(s, data);

			/*
			 * For noisy sensors drop a few samples to make sure we have at least GYRO_MIN_SAMPLES events in the
			 * filtering queue. This improves mean and std dev.
			 */
			if (sensor[s].filter_type) {
				if (sensor[s].selected_trigger !=
				    sensor[s].motion_trigger_name &&
				    sensor[s].event_count < GYRO_MIN_SAMPLES)
						return 0;

				denoise(s, data);
			}

			/* Clamp near zero moves to (0,0,0) if appropriate */
			clamp_gyro_readings_to_zero(s, data);
			break;

		case SENSOR_TYPE_PROXIMITY:
			/*
			 * See iio spec for in_proximity* - depending on the device
			 * this value is either in meters either unit-less and cannot
			 * be translated to SI units. Where the translation is not possible
			 * lower values indicate something is close and higher ones indicate distance.
			 */
			if (data->data[0] > PROXIMITY_THRESHOLD)
				data->data[0] = PROXIMITY_THRESHOLD;

			/* ... fall through ... */
		case SENSOR_TYPE_LIGHT:
		case SENSOR_TYPE_AMBIENT_TEMPERATURE:
		case SENSOR_TYPE_TEMPERATURE:
		case SENSOR_TYPE_INTERNAL_ILLUMINANCE:
		case SENSOR_TYPE_INTERNAL_INTENSITY:
			/* Only keep two decimals for these readings */
			data->data[0] = 0.01 * ((int) (data->data[0] * 100));

			/* These are on change sensors ; drop the sample if it has the same value as the previously reported one. */
			if (data->data[0] == sensor[s].prev_val.data)
				return 0;

			sensor[s].prev_val.data = data->data[0];
			break;
		case SENSOR_TYPE_STEP_COUNTER:
			if (data->u64.step_counter == sensor[s].prev_val.data64)
				return 0;
			sensor[s].prev_val.data64 = data->u64.data[0];
			break;

	}

	/* If there are active virtual sensors depending on this one - process the event */
	if (sensor[s].ref_count)
		process_event(s, data);


	return 1; /* Return sample to Android */
}


static float transform_sample_default (int s, int c, unsigned char* sample_data)
{
	datum_info_t* sample_type = &sensor[s].channel[c].type_info;
	int64_t s64 = sample_as_int64(sample_data, sample_type);
	float scale = sensor[s].scale ? sensor[s].scale : sensor[s].channel[c].scale;

	/* In case correction has been requested using properties, apply it */
	float correction = sensor[s].channel[c].opt_scale;

	/* Correlated with "acquire_immediate_value" method */
	if (sensor[s].type == SENSOR_TYPE_MAGNETIC_FIELD)
                return CONVERT_GAUSS_TO_MICROTESLA((sensor[s].offset + s64) * scale) * correction;

	/* Apply default scaling rules */
	return (sensor[s].offset + s64) * scale * correction;
}


static int finalize_sample_ISH (int s, sensors_event_t* data)
{
	float pitch, roll, yaw;

	/* Swap fields if we have a custom channel ordering on this sensor */
	if (sensor[s].quirks & QUIRK_FIELD_ORDERING)
		reorder_fields(data->data, sensor[s].order);

	if (sensor[s].type == SENSOR_TYPE_ORIENTATION) {

		pitch = data->data[0];
		roll = data->data[1];
		yaw = data->data[2];

		data->data[0] = 360.0 - yaw;
		data->data[1] = -pitch;
		data->data[2] = -roll;
	}

	/* Add this event to our global records, for filtering purposes */
	record_sample(s, data);

	return 1; /* Return sample to Android */
}


static float transform_sample_ISH (int s, int c, unsigned char* sample_data)
{
	datum_info_t* sample_type = &sensor[s].channel[c].type_info;
	int val		= (int) sample_as_int64(sample_data, sample_type);
	float correction;
	int data_bytes  = (sample_type->realbits)/8;
	int exponent    = sensor[s].offset;

	/* In case correction has been requested using properties, apply it */
	correction = sensor[s].channel[c].opt_scale;

	switch (sensor_desc[s].type) {
		case SENSOR_TYPE_ACCELEROMETER:
			switch (c) {
				case 0:
					return correction * CONVERT_A_G_VTF16E14_X(data_bytes, exponent, val);

				case 1:
					return correction * CONVERT_A_G_VTF16E14_Y(data_bytes, exponent, val);

				case 2:
					return	correction * CONVERT_A_G_VTF16E14_Z(data_bytes, exponent, val);
			}
			break;

		case SENSOR_TYPE_GYROSCOPE:
			switch (c) {
				case 0:
					return correction * CONVERT_G_D_VTF16E14_X(data_bytes, exponent, val);

				case 1:
					return correction * CONVERT_G_D_VTF16E14_Y(data_bytes, exponent, val);

				case 2:
					return	correction * CONVERT_G_D_VTF16E14_Z(data_bytes, exponent, val);
			}
			break;

		case SENSOR_TYPE_MAGNETIC_FIELD:
			switch (c) {
				case 0:
					return correction * CONVERT_M_MG_VTF16E14_X(data_bytes, exponent, val);

				case 1:
					return correction * CONVERT_M_MG_VTF16E14_Y(data_bytes, exponent, val);

				case 2:
					return correction * CONVERT_M_MG_VTF16E14_Z(data_bytes, exponent, val);
			}
			break;

		case SENSOR_TYPE_LIGHT:
			return (float) val;

		case SENSOR_TYPE_ORIENTATION:
			return correction * convert_from_vtf_format(data_bytes, exponent, val);

		case SENSOR_TYPE_ROTATION_VECTOR:
			return correction * convert_from_vtf_format(data_bytes, exponent, val);
	}

	return 0;
}


void select_transform (int s)
{
	char prop_name[PROP_NAME_MAX];
	char prop_val[PROP_VALUE_MAX];
	int i			= sensor[s].catalog_index;
	const char *prefix	= sensor_catalog[i].tag;

	sprintf(prop_name, PROP_BASE, prefix, "transform");

	if (property_get(prop_name, prop_val, ""))
		if (!strcmp(prop_val, "ISH")) {
			ALOGI(	"Using Intel Sensor Hub semantics on %s\n", sensor[s].friendly_name);

			sensor[s].ops.transform = transform_sample_ISH;
			sensor[s].ops.finalize = finalize_sample_ISH;
			return;
		}

	sensor[s].ops.transform = transform_sample_default;
	sensor[s].ops.finalize = finalize_sample_default;
}


float acquire_immediate_float_value (int s, int c)
{
	char sysfs_path[PATH_MAX];
	float val;
	int ret;
	int dev_num = sensor[s].dev_num;
	int i = sensor[s].catalog_index;
	const char* raw_path = sensor_catalog[i].channel[c].raw_path;
	const char* input_path = sensor_catalog[i].channel[c].input_path;
	float scale = sensor[s].scale ? sensor[s].scale : sensor[s].channel[c].scale;
	float offset = sensor[s].offset;
	float correction;

	/* In case correction has been requested using properties, apply it */
	correction = sensor[s].channel[c].opt_scale;

	/* Acquire a sample value for sensor s / channel c through sysfs */

	if (sensor[s].channel[c].input_path_present) {
		sprintf(sysfs_path, BASE_PATH "%s", dev_num, input_path);
		ret = sysfs_read_float(sysfs_path, &val);

		if (!ret) {
			if (sensor[s].type == SENSOR_TYPE_MAGNETIC_FIELD)
				return CONVERT_GAUSS_TO_MICROTESLA (val * correction);
			return val * correction;
		}
	}

	if (!sensor[s].channel[c].raw_path_present)
		return 0;

	sprintf(sysfs_path, BASE_PATH "%s", dev_num, raw_path);
	ret = sysfs_read_float(sysfs_path, &val);

	if (ret == -1)
		return 0;

	/*
	 * There is no transform ops defined yet for raw sysfs values.
         * Use this function to perform transformation as well.
	 */
	if (sensor[s].type == SENSOR_TYPE_MAGNETIC_FIELD)
                return CONVERT_GAUSS_TO_MICROTESLA ((val + offset) * scale) * correction;

	return (val + offset) * scale * correction;
}

uint64_t acquire_immediate_uint64_value (int s, int c)
{
	char sysfs_path[PATH_MAX];
	uint64_t val;
	int ret;
	int dev_num = sensor[s].dev_num;
	int i = sensor[s].catalog_index;
	const char* raw_path = sensor_catalog[i].channel[c].raw_path;
	const char* input_path = sensor_catalog[i].channel[c].input_path;
	float scale = sensor[s].scale ? sensor[s].scale : sensor[s].channel[c].scale;
	float offset = sensor[s].offset;
	int sensor_type = sensor_catalog[i].type;
	float correction;

	/* In case correction has been requested using properties, apply it */
	correction = sensor[s].channel[c].opt_scale;

	/* Acquire a sample value for sensor s / channel c through sysfs */

	if (sensor[s].channel[c].input_path_present) {
		sprintf(sysfs_path, BASE_PATH "%s", dev_num, input_path);
		ret = sysfs_read_uint64(sysfs_path, &val);

		if (!ret)
			return val * correction;
	};

	if (!sensor[s].channel[c].raw_path_present)
		return 0;

	sprintf(sysfs_path, BASE_PATH "%s", dev_num, raw_path);
	ret = sysfs_read_uint64(sysfs_path, &val);

	if (ret == -1)
		return 0;

	return (val + offset) * scale * correction;
}
