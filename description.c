/*
 * Copyright (C) 2014 Intel Corporation.
 */

#include <utils/Log.h>
#include <hardware/sensors.h>
#include "enumeration.h"

/*
 * This information should be provided on a sensor basis through a configuration
 * file, or we should build a catalog of known sensors.
 */

#define IIO_SENSOR_HAL_VERSION	1


char* sensor_get_name (int s)
{
	if (sensor_info[s].friendly_name[0] == '\0') {

		/* If we got a iio device name from sysfs, use it */
		if (sensor_info[s].internal_name[0])
			sprintf(sensor_info[s].friendly_name, "S%d-%s", s,
				sensor_info[s].internal_name);
		else
			sprintf(sensor_info[s].friendly_name, "S%d", s);
	}

	return sensor_info[s].friendly_name;
}


char* sensor_get_vendor (int s)
{
	if (sensor_info[s].vendor_name[0])
		return sensor_info[s].vendor_name;

	return "<unknown>";
}


int sensor_get_version (int handle)
{
	return IIO_SENSOR_HAL_VERSION;
}


float sensor_get_max_range (int s)
{
	int catalog_index;
	int sensor_type;

	if (sensor_info[s].max_range != 0.0)
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
	return sensor_info[s].resolution;
}


float sensor_get_power (int s)
{
	/* mA used while sensor is in use ; not sure about volts :) */
	return sensor_info[s].power;
}
