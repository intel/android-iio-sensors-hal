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
#include <ctype.h>
#include <utils/Log.h>
#include <cutils/properties.h>
#include <hardware/sensors.h>
#include "common.h"
#include "enumeration.h"
#include "description.h"
#include "utils.h"
#include "transform.h"

#define IIO_SENSOR_HAL_VERSION	1

#define MIN_ON_CHANGE_SAMPLING_PERIOD_US   200000 /* For on change sensors (temperature, proximity, ALS, etc.) report we support 5 Hz max (0.2 s min period) */
#define MAX_ON_CHANGE_SAMPLING_PERIOD_US 10000000 /* 0.1 Hz min (10 s max period)*/
#define ANDROID_MAX_FREQ 1000 /* 1000 Hz - This is how much Android requests for the fastest frequency */

/*
 * About properties
 *
 * We acquire a number of parameters about sensors by reading properties.
 * The idea here is that someone (either a script, or daemon, sets them
 * depending on the set of sensors present on the machine.
 *
 * There are fallback paths in case the properties are not defined, but it is
 * highly desirable to at least have the following for each sensor:
 *
 * ro.iio.anglvel.name = Gyroscope
 * ro.iio.anglvel.vendor = Intel
 * ro.iio.anglvel.max_range = 35
 * ro.iio.anglvel.resolution = 0.002
 * ro.iio.anglvel.power = 6.1
 *
 * Besides these, we have a couple of knobs initially used to cope with Intel
 * Sensor Hub oddities, such as HID inspired units or firmware bugs:
 *
 * ro.iio.anglvel.transform = ISH
 * ro.iio.anglvel.quirks = init-rate
 *
 * The "terse" quirk indicates that the underlying driver only sends events
 * when the sensor reports a change. The HAL then periodically generates
 * duplicate events so the sensor behaves as a continously firing one.
 *
 * The "noisy" quirk indicates that the underlying driver has a unusually high
 * level of noise in its readings, and that the HAL has to accomodate it
 * somehow, e.g. in the magnetometer calibration code path.
 *
 * This one is used specifically to pass a calibration scale to ALS drivers:
 *
 * ro.iio.illuminance.name = CPLM3218x Ambient Light Sensor
 * ro.iio.illuminance.vendor = Capella Microsystems
 * ro.iio.illuminance.max_range = 167000
 * ro.iio.illuminance.resolution = 1
 * ro.iio.illuminance.power = .001
 * ro.iio.illuminance.illumincalib = 7400
 *
 * There's a 'opt_scale' specifier, documented as follows:
 *
 *  This adds support for a scaling factor that can be expressed
 *  using properties, for all sensors, on a channel basis. That
 *  scaling factor is applied after all other transforms have been
 *  applied, and is intended as a way to compensate for problems
 *  such as an incorrect axis polarity for a given sensor.
 *
 *  The syntax is <usual property prefix>.<channel>.opt_scale, e.g.
 *  ro.iio.accel.y.opt_scale = -1 to negate the sign of the y readings
 *  for the accelerometer.
 *
 *  For sensors using a single channel - and only those - the channel
 *  name is implicitly void and a syntax such as ro.iio.illuminance.
 *  opt_scale = 3 has to be used.
 *
 * 'panel' and 'rotation' specifiers can be used to express ACPI PLD placement
 * information ; if found they will be used in priority over the actual ACPI
 * data. That is intended as a way to verify values during development.
 *
 * It's possible to use the contents of the iio device name as a way to
 * discriminate between sensors. Several sensors of the same type can coexist:
 * e.g. ro.iio.temp.bmg160.name = BMG160 Thermometer will be used in priority
 * over ro.iio.temp.name = BMC150 Thermometer if the sensor for which we query
 * properties values happen to have its iio device name set to bmg160.
 */

int sensor_get_st_prop (int s, const char* sel, char val[MAX_NAME_SIZE])
{
	char prop_name[PROP_NAME_MAX];
	char prop_val[PROP_VALUE_MAX];
	char extended_sel[PROP_VALUE_MAX];

	int i			= sensor[s].catalog_index;
	const char *prefix	= sensor_catalog[i].tag;
	const char *shorthand = sensor_catalog[i].shorthand;

	/* First try most specialized form, like ro.iio.anglvel.bmg160.name */

	snprintf(extended_sel, PROP_NAME_MAX, "%s.%s",
		 sensor[s].internal_name, sel);

	snprintf(prop_name, PROP_NAME_MAX, PROP_BASE, prefix, extended_sel);

	if (property_get(prop_name, prop_val, "")) {
		strncpy(val, prop_val, MAX_NAME_SIZE-1);
		val[MAX_NAME_SIZE-1] = '\0';
		return 0;
	}

	if (shorthand[0] != '\0') {
		/* Try with shorthand instead of prefix */
		snprintf(prop_name, PROP_NAME_MAX, PROP_BASE, shorthand, extended_sel);

		if (property_get(prop_name, prop_val, "")) {
			strncpy(val, prop_val, MAX_NAME_SIZE-1);
			val[MAX_NAME_SIZE-1] = '\0';
			return 0;
		}
	}
	/* Fall back to simple form, like ro.iio.anglvel.name */

	snprintf(prop_name, PROP_NAME_MAX, PROP_BASE, prefix, sel);

	if (property_get(prop_name, prop_val, "")) {
		strncpy(val, prop_val, MAX_NAME_SIZE-1);
		val[MAX_NAME_SIZE-1] = '\0';
		return 0;
	}

	return -1;
}


int sensor_get_prop (int s, const char* sel, int* val)
{
	char buf[MAX_NAME_SIZE];

	if (sensor_get_st_prop(s, sel, buf))
		return -1;

	*val = atoi(buf);
	return 0;
}


int sensor_get_fl_prop (int s, const char* sel, float* val)
{
	char buf[MAX_NAME_SIZE];

	if (sensor_get_st_prop(s, sel, buf))
		return -1;

	*val = (float) strtod(buf, NULL);
	return 0;
}


char* sensor_get_name (int s)
{
	char buf[MAX_NAME_SIZE];

	if (sensor[s].is_virtual) {
		switch (sensor[s].type) {
			case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
			case SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED:
				strcpy(buf, sensor[sensor[s].base[0]].friendly_name);
				snprintf(sensor[s].friendly_name,
					 MAX_NAME_SIZE,
					 "%s %s", "Uncalibrated", buf);
				return sensor[s].friendly_name;

			default:
				return "";
		}
	}

	if (sensor[s].friendly_name[0] != '\0' ||
		!sensor_get_st_prop(s, "name", sensor[s].friendly_name))
			return sensor[s].friendly_name;

	/* If we got a iio device name from sysfs, use it */
	if (sensor[s].internal_name[0]) {
		snprintf(sensor[s].friendly_name, MAX_NAME_SIZE, "S%d-%s",
			 s, sensor[s].internal_name);
	} else {
		sprintf(sensor[s].friendly_name, "S%d", s);
	}

	return sensor[s].friendly_name;
}


char* sensor_get_vendor (int s)
{
	if (sensor[s].is_virtual) {
		switch (sensor[s].type) {
			case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
			case SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED:
				return sensor[sensor[s].base[0]].vendor_name;
			break;

			default:
				return "";

		}
	}

	if (sensor[s].vendor_name[0] ||
		!sensor_get_st_prop(s, "vendor", sensor[s].vendor_name))
			return sensor[s].vendor_name;

	return "";
}


int sensor_get_version (__attribute__((unused)) int s)
{
	return IIO_SENSOR_HAL_VERSION;
}

void sensor_update_max_range(int s)
{
	if (sensor[s].max_range)
		return;

	if (sensor[s].num_channels && sensor[s].channel[0].type_info.realbits) {
		switch (sensor[s].type) {
		case SENSOR_TYPE_MAGNETIC_FIELD:
			sensor[s].max_range = (1ULL << sensor[s].channel[0].type_info.realbits) *
					CONVERT_MICROTESLA_TO_GAUSS(sensor[s].resolution) +
					(sensor[s].offset || sensor[s].channel[0].offset);
			sensor[s].max_range = CONVERT_GAUSS_TO_MICROTESLA(sensor[s].max_range);
			break;
		case SENSOR_TYPE_PROXIMITY:
			break;
		default:
			sensor[s].max_range =  (1ULL << sensor[s].channel[0].type_info.realbits) *
				sensor[s].resolution + (sensor[s].offset || sensor[s].channel[0].offset);
			break;
		}
	}

	if (!sensor[s].max_range) {
		/* Try returning a sensible value given the sensor type */
		/* We should cap returned samples accordingly... */
		switch (sensor[s].type) {
		case SENSOR_TYPE_ACCELEROMETER:		/* m/s^2	*/
			sensor[s].max_range = 50;
			break;
		case SENSOR_TYPE_MAGNETIC_FIELD:	/* micro-tesla	*/
			sensor[s].max_range = 500;
			break;
		case SENSOR_TYPE_ORIENTATION:		/* degrees	*/
			sensor[s].max_range = 360;
			break;
		case SENSOR_TYPE_GYROSCOPE:		/* radians/s	*/
			sensor[s].max_range = 10;
			break;
		case SENSOR_TYPE_LIGHT:			/* SI lux units */
			sensor[s].max_range = 50000;
			break;
		case SENSOR_TYPE_AMBIENT_TEMPERATURE:	/* °C		*/
		case SENSOR_TYPE_TEMPERATURE:		/* °C		*/
		case SENSOR_TYPE_PROXIMITY:		/* centimeters	*/
		case SENSOR_TYPE_PRESSURE:		/* hecto-pascal */
		case SENSOR_TYPE_RELATIVE_HUMIDITY:	/* percent */
			sensor[s].max_range = 100;
			break;
		}
	}

	if (sensor[s].max_range)
		sensor_desc[s].maxRange = sensor[s].max_range;
}

float sensor_get_max_range (int s)
{
	if (sensor[s].is_virtual) {
		switch (sensor[s].type) {
			case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
			case SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED:
				return sensor[sensor[s].base[0]].max_range;

			default:
				return 0.0;
		}
	}

	if (sensor[s].max_range != 0.0 ||
		!sensor_get_fl_prop(s, "max_range", &sensor[s].max_range))
			return sensor[s].max_range;

	return 0;
}

float sensor_get_min_freq (int s)
{
	/*
	 * Check if a low cap has been specified for this sensor sampling rate.
	 * In some case, even when the driver supports lower rate, we still
	 * wish to receive a certain number of samples per seconds for various
	 * reasons (calibration, filtering, no change in power consumption...).
	 */

	float min_freq;

	if (!sensor_get_fl_prop(s, "min_freq", &min_freq))
		return min_freq;

	return 0;
}


float sensor_get_max_freq (int s)
{
	float max_freq;

	if (!sensor_get_fl_prop(s, "max_freq", &max_freq))
		return max_freq;

	return ANDROID_MAX_FREQ;
}

int sensor_get_cal_steps (int s)
{
	int cal_steps;
	if (!sensor_get_prop(s, "cal_steps", &cal_steps))
		return cal_steps;

	return 0;
}

float sensor_get_resolution (int s)
{
	if (sensor[s].is_virtual) {
		switch (sensor[s].type) {
			case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
			case SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED:
				return sensor[sensor[s].base[0]].resolution;

			default:
				return 0;
		}
	}

	if (sensor[s].resolution != 0.0 ||
	    !sensor_get_fl_prop(s, "resolution", &sensor[s].resolution)) {
		return sensor[s].resolution;
	}

	sensor[s].resolution = sensor[s].scale;
	if (!sensor[s].resolution && sensor[s].num_channels)
		sensor[s].resolution = sensor[s].channel[0].scale;

	if (sensor[s].type == SENSOR_TYPE_MAGNETIC_FIELD)
		sensor[s].resolution = CONVERT_GAUSS_TO_MICROTESLA(sensor[s].resolution);

	return sensor[s].resolution ? : 1;
}


float sensor_get_power (int s)
{

	if (sensor[s].is_virtual) {
		switch (sensor[s].type) {
			case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
			case SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED:
				return sensor[sensor[s].base[0]].power;

			default:
				return 0;
		}
	}

	/* mA used while sensor is in use ; not sure about volts :) */
	if (sensor[s].power != 0.0 ||
		!sensor_get_fl_prop(s, "power",	&sensor[s].power))
			return sensor[s].power;

	return 0;
}


float sensor_get_illumincalib (int s)
{
	/* calibrating the ALS Sensor*/
	if (sensor[s].illumincalib != 0.0 ||
		!sensor_get_fl_prop(s, "illumincalib", &sensor[s].illumincalib)) {
			return sensor[s].illumincalib;
	}

	return 0;
}


uint32_t sensor_get_quirks (int s)
{
	char quirks_buf[MAX_NAME_SIZE];

	/* Read and decode quirks property on first reference */
	if (!(sensor[s].quirks & QUIRK_ALREADY_DECODED)) {
		quirks_buf[0] = '\0';
		sensor_get_st_prop(s, "quirks", quirks_buf);

		if (strstr(quirks_buf, "init-rate"))
			sensor[s].quirks |= QUIRK_INITIAL_RATE;

		if (strstr(quirks_buf, "continuous"))
			sensor[s].quirks |= QUIRK_FORCE_CONTINUOUS;

		if (strstr(quirks_buf, "terse"))
			sensor[s].quirks |= QUIRK_TERSE_DRIVER;

		if (strstr(quirks_buf, "noisy"))
			sensor[s].quirks |= QUIRK_NOISY;

		if (strstr(quirks_buf, "biased"))
			sensor[s].quirks |= QUIRK_BIASED;

		if (strstr(quirks_buf, "spotty"))
			sensor[s].quirks |= QUIRK_SPOTTY;

		if (strstr(quirks_buf, "no-event"))
			sensor[s].quirks |= QUIRK_NO_EVENT_MODE;

		if (strstr(quirks_buf, "no-trig"))
			sensor[s].quirks |= QUIRK_NO_TRIG_MODE;

		if (strstr(quirks_buf, "no-poll"))
			sensor[s].quirks |= QUIRK_NO_POLL_MODE;

		if (strstr(quirks_buf, "hrtimer"))
			sensor[s].quirks |= QUIRK_HRTIMER;

		if (strstr(quirks_buf, "secondary"))
			sensor[s].quirks |= QUIRK_SECONDARY;

		sensor[s].quirks |= QUIRK_ALREADY_DECODED;
	}

	return sensor[s].quirks;
}


int sensor_get_order (int s, unsigned char map[MAX_CHANNELS])
{
	char buf[MAX_NAME_SIZE];
	int i;
	int count = sensor_catalog[sensor[s].catalog_index].num_channels;

	if (sensor_get_st_prop(s, "order", buf))
		return 0; /* No order property */

	/* Assume ASCII characters, in the '0'..'9' range */

	for (i=0; i<count; i++)
		if (buf[i] - '0' >= count) {
			ALOGE("Order index out of range for sensor %d\n", s);
			return 0;
		}

	for (i=0; i<count; i++)
		map[i] = buf[i] - '0';

	return 1;	/* OK to use modified ordering map */
}

int sensor_get_available_frequencies (int s)
{
	int dev_num = sensor[s].dev_num, err, i;
	char avail_sysfs_path[PATH_MAX], freqs_buf[100];
	char *p, *end;
	float f;

	sensor[s].avail_freqs_count = 0;
	sensor[s].avail_freqs = 0;

	sprintf(avail_sysfs_path, DEVICE_AVAIL_FREQ_PATH, dev_num);

	err = sysfs_read_str(avail_sysfs_path, freqs_buf, sizeof(freqs_buf));
	if (err < 0)
		return 0;

	for (p = freqs_buf, f = strtof(p, &end); p != end; p = end, f = strtof(p, &end))
		sensor[s].avail_freqs_count++;

	if (sensor[s].avail_freqs_count) {
		sensor[s].avail_freqs = (float*) calloc(sensor[s].avail_freqs_count, sizeof(float));

		for (p = freqs_buf, f = strtof(p, &end), i = 0; p != end; p = end, f = strtof(p, &end), i++)
			sensor[s].avail_freqs[i] = f;
	}

	return 0;
}

int sensor_get_mounting_matrix (int s, float mm[9])
{
	int dev_num = sensor[s].dev_num, err, i;
	char mm_path[PATH_MAX], mm_buf[100];
	char *tmp1 = mm_buf, *tmp2;

	switch (sensor[s].type) {
		case SENSOR_TYPE_ACCELEROMETER:
		case SENSOR_TYPE_MAGNETIC_FIELD:
		case SENSOR_TYPE_GYROSCOPE:
		case SENSOR_TYPE_PROXIMITY:
			break;
		default:
			return 0;
	}

	sprintf(mm_path, MOUNTING_MATRIX_PATH, dev_num);

	err = sysfs_read_str(mm_path, mm_buf, sizeof(mm_buf));
	if (err < 0)
		return 0;

	for(i = 0; i < 9; i++) {
		float f;

		f = strtof(tmp1, &tmp2);
		if (!f && tmp1 == tmp2)
			return 0;
		mm[i] = f;
		tmp1 = tmp2 + 1;
	}

	/*
	 * For proximity sensors, interpret a negative final z value as a hint that the sensor is back mounted. In that case, mark the sensor as secondary to
	 * ensure that it gets listed after other sensors of same type that would be front-mounted. Most applications will only ask for the default proximity
	 * sensor and it makes more sense to point to, say, the IR based proximity sensor rather than SAR based one if we have both, as on SoFIA LTE MRD boards.
	 */
	 if (sensor[s].type == SENSOR_TYPE_PROXIMITY) {
		if (mm[8] < 0) {
			sensor[s].quirks |= QUIRK_SECONDARY;
		}
		return 0;
	}

	ALOGI("%s: %f %f %f %f %f %f %f %f %f\n", __func__, mm[0], mm[1], mm[2], mm[3], mm[4], mm[5], mm[6], mm[7], mm[8]);
	return 1;
}


char* sensor_get_string_type (int s)
{
	switch (sensor_desc[s].type) {
		case SENSOR_TYPE_ACCELEROMETER:
			return SENSOR_STRING_TYPE_ACCELEROMETER;

		case SENSOR_TYPE_MAGNETIC_FIELD:
			return SENSOR_STRING_TYPE_MAGNETIC_FIELD;

		case SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED:
			return SENSOR_STRING_TYPE_MAGNETIC_FIELD_UNCALIBRATED;

		case SENSOR_TYPE_ORIENTATION:
			return SENSOR_STRING_TYPE_ORIENTATION;

		case SENSOR_TYPE_GYROSCOPE:
			return SENSOR_STRING_TYPE_GYROSCOPE;

		case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
			return SENSOR_STRING_TYPE_GYROSCOPE_UNCALIBRATED;

		case SENSOR_TYPE_LIGHT:
			return SENSOR_STRING_TYPE_LIGHT;

		case SENSOR_TYPE_AMBIENT_TEMPERATURE:
			return SENSOR_STRING_TYPE_AMBIENT_TEMPERATURE;

		case SENSOR_TYPE_TEMPERATURE:
			return SENSOR_STRING_TYPE_TEMPERATURE;

		case SENSOR_TYPE_PROXIMITY:
			return SENSOR_STRING_TYPE_PROXIMITY;

		case SENSOR_TYPE_PRESSURE:
			return SENSOR_STRING_TYPE_PRESSURE;

		case SENSOR_TYPE_RELATIVE_HUMIDITY:
			return SENSOR_STRING_TYPE_RELATIVE_HUMIDITY;

		default:
			return "";
		}
}


flag_t sensor_get_flags (int s)
{
	flag_t flags = 0;

	switch (sensor_desc[s].type) {
		case SENSOR_TYPE_LIGHT:
		case SENSOR_TYPE_AMBIENT_TEMPERATURE:
		case SENSOR_TYPE_TEMPERATURE:
		case SENSOR_TYPE_RELATIVE_HUMIDITY:
		case SENSOR_TYPE_STEP_COUNTER:
			flags |= SENSOR_FLAG_ON_CHANGE_MODE;
			break;


		case SENSOR_TYPE_PROXIMITY:
			flags |= SENSOR_FLAG_WAKE_UP;
			flags |= SENSOR_FLAG_ON_CHANGE_MODE;
			break;
		case SENSOR_TYPE_STEP_DETECTOR:
			flags |= SENSOR_FLAG_SPECIAL_REPORTING_MODE;
			break;
		default:
			break;
		}
	return flags;
}


static int get_cdd_freq (int s, int must)
{
	switch (sensor_desc[s].type) {
		case SENSOR_TYPE_ACCELEROMETER:
			return (must ? 100 : 200); /* must 100 Hz, should 200 Hz, CDD compliant */

		case SENSOR_TYPE_GYROSCOPE:
			return (must ? 200 : 200); /* must 200 Hz, should 200 Hz, CDD compliant */

		case SENSOR_TYPE_MAGNETIC_FIELD:
			return (must ? 10 : 50);   /* must 10 Hz, should 50 Hz, CDD compliant */

		case SENSOR_TYPE_LIGHT:
		case SENSOR_TYPE_AMBIENT_TEMPERATURE:
		case SENSOR_TYPE_TEMPERATURE:
			return (must ? 1 : 2);     /* must 1 Hz, should 2Hz, not mentioned in CDD */

		default:
			return 1; /* Use 1 Hz by default, e.g. for proximity */
	}
}

/*
 * This value is defined only for continuous mode and on-change sensors. It is the delay between two sensor events corresponding to the lowest frequency that
 * this sensor supports. When lower frequencies are requested through batch()/setDelay() the events will be generated at this frequency instead. It can be used
 * by the framework or applications to estimate when the batch FIFO may be full. maxDelay should always fit within a 32 bit signed integer. It is declared as
 * 64 bit on 64 bit architectures only for binary compatibility reasons. Availability: SENSORS_DEVICE_API_VERSION_1_3
 */
max_delay_t sensor_get_max_delay (int s)
{
	int dev_num = sensor[s].dev_num, i;
	float min_supported_rate;
	float rate_cap;

	/*
	 * continuous, on-change: maximum sampling period allowed in microseconds.
	 * one-shot, special : 0
	 */
	switch (REPORTING_MODE(sensor_desc[s].flags)) {
		case SENSOR_FLAG_ONE_SHOT_MODE:
		case SENSOR_FLAG_SPECIAL_REPORTING_MODE:
			return 0;

		case SENSOR_FLAG_ON_CHANGE_MODE:
			return MAX_ON_CHANGE_SAMPLING_PERIOD_US;

		default:
			break;
	}

	if (sensor[s].is_virtual) {
		switch (sensor[s].type) {
			case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
			case SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED:
				return sensor_desc[sensor[s].base[0]].maxDelay;
			default:
				return 0;
		}
	}

	switch (sensor[s].mode) {
		case MODE_TRIGGER:
			/* For interrupt-based devices, obey the list of supported sampling rates */
			if (sensor[s].avail_freqs_count) {
				min_supported_rate = 1000;
				for (i = 0; i < sensor[s].avail_freqs_count; i++) {
					if (sensor[s].avail_freqs[i] < min_supported_rate)
						min_supported_rate = sensor[s].avail_freqs[i];
				}
				break;
			}
			/* Fall through ... */

		default:
			/* Report 1 Hz */
			min_supported_rate = 1;
			break;
	}

	/* Check if a minimum rate was specified for this sensor */
	rate_cap = sensor_get_min_freq(s);

	if (min_supported_rate < rate_cap)
		min_supported_rate = rate_cap;

	/* return 0 for wrong values */
	if (min_supported_rate < 0.1)
		return 0;

	/* Return microseconds */
	return (max_delay_t) (1000000.0 / min_supported_rate);
}

float sensor_get_max_static_freq(int s)
{
	float max_from_prop = sensor_get_max_freq(s);

	/* If we have max specified via a property use it */
	if (max_from_prop != ANDROID_MAX_FREQ) {
		return max_from_prop;
	} else {
		/* The should rate */
		return get_cdd_freq(s, 0);
	}
}

int32_t sensor_get_min_delay (int s)
{
	int dev_num = sensor[s].dev_num, i;
	float max_supported_rate = 0;
	float max_from_prop = sensor_get_max_freq(s);

	/* continuous, on change: minimum sampling period allowed in microseconds.
	 * special : 0, unless otherwise noted
	 * one-shot:-1
	 */
	switch (REPORTING_MODE(sensor_desc[s].flags)) {
		case SENSOR_FLAG_ON_CHANGE_MODE:
			return MIN_ON_CHANGE_SAMPLING_PERIOD_US;

		case SENSOR_FLAG_SPECIAL_REPORTING_MODE:
			return 0;

		case SENSOR_FLAG_ONE_SHOT_MODE:
			return -1;

		default:
			break;
	}

	if (sensor[s].is_virtual) {
		switch (sensor[s].type) {
			case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
			case SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED:
				return sensor_desc[sensor[s].base[0]].minDelay;
			default:
				return 0;
		}
	}

	if (!sensor[s].avail_freqs_count) {
		if (sensor[s].mode == MODE_POLL) {
			/* If we have max specified via a property use it */
			if (max_from_prop != ANDROID_MAX_FREQ)
				max_supported_rate = max_from_prop;
			else
				/* The should rate */
				max_supported_rate = get_cdd_freq(s, 0);
		}
	} else {
		for (i = 0; i < sensor[s].avail_freqs_count; i++) {
			if (sensor[s].avail_freqs[i] > max_supported_rate &&
				sensor[s].avail_freqs[i] <= max_from_prop) {
				max_supported_rate = sensor[s].avail_freqs[i];
			}
		}
	}

	/* return 0 for wrong values */
	if (max_supported_rate < 0.1)
		return 0;

	/* Return microseconds */
	return (int32_t) (1000000.0 / max_supported_rate);
}
