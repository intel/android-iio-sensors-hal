/*
 * Copyright (C) 2014 Intel Corporation.
 */

#include <stdlib.h>
#include <utils/Log.h>
#include <cutils/properties.h>
#include <hardware/sensors.h>
#include "enumeration.h"
#include "description.h"

#define IIO_SENSOR_HAL_VERSION	1

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
 * Finally there's a 'opt_scale' specifier, documented as follows:
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
 */

static int sensor_get_st_prop (int s, const char* sel, char val[MAX_NAME_SIZE])
{
	char prop_name[PROP_NAME_MAX];
	char prop_val[PROP_VALUE_MAX];
	int i			= sensor_info[s].catalog_index;
	const char *prefix	= sensor_catalog[i].tag;

	sprintf(prop_name, PROP_BASE, prefix, sel);

	if (property_get(prop_name, prop_val, "")) {
		strncpy(val, prop_val, MAX_NAME_SIZE-1);
		val[MAX_NAME_SIZE-1] = '\0';
		return 0;
	}

	return -1;
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
	if (sensor_info[s].friendly_name[0] != '\0' ||
		!sensor_get_st_prop(s, "name", sensor_info[s].friendly_name))
			return sensor_info[s].friendly_name;

	/* If we got a iio device name from sysfs, use it */
	if (sensor_info[s].internal_name[0]) {
		snprintf(sensor_info[s].friendly_name, MAX_NAME_SIZE, "S%d-%s",
			 s, sensor_info[s].internal_name);
	} else {
		sprintf(sensor_info[s].friendly_name, "S%d", s);
	}

	return sensor_info[s].friendly_name;
}


char* sensor_get_vendor (int s)
{
	if (sensor_info[s].vendor_name[0] ||
		!sensor_get_st_prop(s, "vendor", sensor_info[s].vendor_name))
			return sensor_info[s].vendor_name;

	return "";
}


int sensor_get_version (int s)
{
	return IIO_SENSOR_HAL_VERSION;
}


float sensor_get_max_range (int s)
{
	int catalog_index;
	int sensor_type;

	if (sensor_info[s].max_range != 0.0 ||
		!sensor_get_fl_prop(s, "max_range", &sensor_info[s].max_range))
			return sensor_info[s].max_range;

	/* Try returning a sensible value given the sensor type */

	/* We should cap returned samples accordingly... */

	catalog_index = sensor_info[s].catalog_index;
	sensor_type = sensor_catalog[catalog_index].type;

	switch (sensor_type) {
		case SENSOR_TYPE_ACCELEROMETER:		/* m/s^2	*/
			return 50;

		case SENSOR_TYPE_MAGNETIC_FIELD:	/* micro-tesla	*/
			return 500;

		case SENSOR_TYPE_ORIENTATION:		/* degrees	*/
			return 360;

		case SENSOR_TYPE_GYROSCOPE:		/* radians/s	*/
			return 10;

		case SENSOR_TYPE_LIGHT:			/* SI lux units */
			return 50000;

		case SENSOR_TYPE_AMBIENT_TEMPERATURE:	/* °C		*/
		case SENSOR_TYPE_TEMPERATURE:		/* °C		*/
		case SENSOR_TYPE_PROXIMITY:		/* centimeters	*/
		case SENSOR_TYPE_PRESSURE:		/* hecto-pascal */
		case SENSOR_TYPE_RELATIVE_HUMIDITY:	/* percent */
			return 100;

		default:
			return 0.0;
		}
}


float sensor_get_resolution (int s)
{
	if (sensor_info[s].resolution != 0.0 ||
		!sensor_get_fl_prop(s, "resolution", &sensor_info[s].resolution))
			return sensor_info[s].resolution;

	return 0;
}


float sensor_get_power (int s)
{
	/* mA used while sensor is in use ; not sure about volts :) */
	if (sensor_info[s].power != 0.0 ||
		!sensor_get_fl_prop(s, "power",	&sensor_info[s].power))
			return sensor_info[s].power;

	return 0;
}


float sensor_get_illumincalib (int s)
{
	/* calibrating the ALS Sensor*/
	if (sensor_info[s].illumincalib != 0.0 ||
		!sensor_get_fl_prop(s, "illumincalib", &sensor_info[s].illumincalib)) {
			return sensor_info[s].illumincalib;
	}

	return 0;
}


uint32_t sensor_get_quirks (int s)
{
	char quirks_buf[MAX_NAME_SIZE];

	/* Read and decode quirks property on first reference */
	if (!(sensor_info[s].quirks & QUIRK_ALREADY_DECODED)) {
		quirks_buf[0] = '\0';
		sensor_get_st_prop(s, "quirks", quirks_buf);

		if (strstr(quirks_buf, "init-rate"))
			sensor_info[s].quirks |= QUIRK_INITIAL_RATE;

		if (strstr(quirks_buf, "terse"))
			sensor_info[s].quirks |= QUIRK_TERSE_DRIVER;

		if (strstr(quirks_buf, "noisy"))
			sensor_info[s].quirks |= QUIRK_NOISY;

		sensor_info[s].quirks |= QUIRK_ALREADY_DECODED;
	}

	return sensor_info[s].quirks;
}


int sensor_get_order (int s, unsigned char map[MAX_CHANNELS])
{
	char buf[MAX_NAME_SIZE];
	int i;
	int count = sensor_catalog[sensor_info[s].catalog_index].num_channels;

	memset(map, 0, MAX_CHANNELS);

	if  (sensor_get_st_prop(s, "order", buf))
		return 0; /* No order property */

	/* Assume ASCII characters, in the '0'..'9' range */

	for (i=0; i<count; i++)
		map[i] = buf[i] - '0';

	/* Check that our indices are in range */
	for (i=0; i<count; i++)
		if (map[i] >= count) {
			ALOGE("Order index out of range for sensor %d\n", s);
			return 0;
		}

	return 1;	/* OK to use modified ordering map */
}

char* sensor_get_string_type(int s)
{
	int catalog_index;
	int sensor_type;

	catalog_index = sensor_info[s].catalog_index;
	sensor_type = sensor_catalog[catalog_index].type;

	switch (sensor_type) {
		case SENSOR_TYPE_ACCELEROMETER:
			return SENSOR_STRING_TYPE_ACCELEROMETER;

		case SENSOR_TYPE_MAGNETIC_FIELD:
			return SENSOR_STRING_TYPE_MAGNETIC_FIELD;

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
	int catalog_index;
	int sensor_type;

	flag_t flags = 0x0;
	catalog_index = sensor_info[s].catalog_index;
	sensor_type = sensor_catalog[catalog_index].type;

	switch (sensor_type) {
		case SENSOR_TYPE_ACCELEROMETER:
		case SENSOR_TYPE_MAGNETIC_FIELD:
		case SENSOR_TYPE_ORIENTATION:
		case SENSOR_TYPE_GYROSCOPE:
		case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
		case SENSOR_TYPE_PRESSURE:
			flags |= SENSOR_FLAG_CONTINUOUS_MODE;
			break;

		case SENSOR_TYPE_LIGHT:
		case SENSOR_TYPE_AMBIENT_TEMPERATURE:
		case SENSOR_TYPE_TEMPERATURE:
		case SENSOR_TYPE_RELATIVE_HUMIDITY:
			flags |= SENSOR_FLAG_ON_CHANGE_MODE;
			break;


		case SENSOR_TYPE_PROXIMITY:
			flags |= SENSOR_FLAG_WAKE_UP;
			flags |= SENSOR_FLAG_ON_CHANGE_MODE;
			break;

		default:
			ALOGI("Unknown sensor");
		}
	return flags;
}
