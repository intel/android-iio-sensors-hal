/*
 * Copyright (C) 2014 Intel Corporation.
 */

#include <stdlib.h>
#include <utils/Log.h>
#include <cutils/properties.h>
#include <hardware/sensors.h>
#include "enumeration.h"

#define IIO_SENSOR_HAL_VERSION	1


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
