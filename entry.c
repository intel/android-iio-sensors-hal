/*
 * Copyright (C) 2014 Intel Corporation.
 */

#include <hardware/sensors.h>
#include <utils/Log.h>
#include "enumeration.h"
#include "control.h"

/* This is the IIO Sensors HAL module entry points file */

static int init_count;

static int activate(struct sensors_poll_device_t* dev, int handle, int enabled)
{
	if (init_count == 0 || handle < 0 || handle >= sensor_count)
		return -EINVAL;

	return sensor_activate(handle, enabled);
}


static int set_delay(struct sensors_poll_device_t* dev, int handle, int64_t ns)
{
	if (init_count == 0 || handle < 0 || handle >= sensor_count)
		return -EINVAL;

	return sensor_set_delay(handle, ns);
}


static int poll(struct sensors_poll_device_t* dev, sensors_event_t* data,
		int count)
{
	if (init_count == 0 || !data || count < 1)
		return -EINVAL;

	return sensor_poll(data, count);
}


static int close_module(hw_device_t *device)
{
	if (init_count == 0)
		return -EINVAL;

	init_count--;

	if (init_count == 0) {
		ALOGI("Closing IIO sensors HAL module\n");
		delete_enumeration_data();
		delete_control_data();
	}

	return 0;
}


static int initialize_module(const struct hw_module_t *module, const char *id,
				struct hw_device_t** device)
{
	static struct sensors_poll_device_t poll_device;

	if (strcmp(id, SENSORS_HARDWARE_POLL))
                return -EINVAL;

	poll_device.common.tag		= HARDWARE_DEVICE_TAG;
	poll_device.common.version	= 0;
	poll_device.common.module	= (struct hw_module_t*) module;
	poll_device.common.close	= close_module;

	poll_device.activate		= activate;
	poll_device.setDelay		= set_delay;
	poll_device.poll		= poll;

	*device = &poll_device.common;

        if (init_count == 0) {
		ALOGI("Initializing IIO sensors HAL module\n");
		allocate_control_data();
		enumerate_sensors();
	}

	init_count++;
	return 0;
}


static struct hw_module_methods_t module_methods = {
	.open = initialize_module
};


/* Externally visible module descriptor */
struct sensors_module_t __attribute__ ((visibility ("default")))
	HAL_MODULE_INFO_SYM = {
		.common = {
			.tag = HARDWARE_MODULE_TAG,
			.version_major = 1,
			.version_minor = 0,
			.id = SENSORS_HARDWARE_MODULE_ID,
			.name = "IIO sensors HAL",
			.author = "Intel",
			.methods = &module_methods,
		},
		.get_sensors_list = get_sensors_list
};

