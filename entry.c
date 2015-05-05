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

#include <hardware/sensors.h>
#include <utils/Log.h>
#include "enumeration.h"
#include "control.h"
#include "description.h"
#include "utils.h"

#include <errno.h>

/* This is the IIO Sensors HAL module entry points file */

static int init_count;

static int activate (__attribute__((unused)) struct sensors_poll_device_t* dev,
		     int handle, int enabled)
{
	int64_t entry_ts;
	int ret;
	int elapsed_ms;

	if (init_count == 0 || handle < 0 || handle >= sensor_count)
		return -EINVAL;

	entry_ts = get_timestamp_thread();

	/*
	 * The Intel sensor hub seems to have trouble enabling sensors before
	 * a sampling rate has been configured, and setting the sampling rate
	 * after it's been enabled does not seem to revive affected sensors.
	 * The issue does not show up with an up to date ISH firmware but as the
	 * updater is a Windows only tool and is not widely available, implement
	 * a workaround for this behavior. We set the initial sampling rate to
	 * 10 events per second when the sensor is enabled for the first time.
	 */

	if (enabled && sensor_get_quirks(handle) & QUIRK_INITIAL_RATE) {
		ALOGI("Forcing initial sampling rate\n");
		sensor_activate(handle, 1, 0);
		sensor_set_delay(handle, 100000000);	/* Start with 100 ms */
		sensor_activate(handle, 0, 0);

		/* Clear flag for this sensor as do this only once */
		sensor[handle].quirks ^= QUIRK_INITIAL_RATE;
	}

	ret = sensor_activate(handle, enabled, 0);

	elapsed_ms = (int) ((get_timestamp_thread() - entry_ts) / 1000000);

	if (elapsed_ms) {
		if (enabled)
			ALOGI("Activation of sensor %s took %d ms\n", sensor[handle].friendly_name, elapsed_ms);
		else
			ALOGI("Deactivation of sensor %s took %d ms\n", sensor[handle].friendly_name, elapsed_ms);
	}

	return ret;
}


static int set_delay (__attribute__((unused)) struct sensors_poll_device_t* dev,
		      int handle, int64_t ns)
{
	int i;

	if (init_count == 0 || handle < 0 || handle >= sensor_count)
		return -EINVAL;

	/*
	 * If this sensor relies on other sensors, try to propagate the
	 * requested sampling rate to the base sensors.
	 */

	for (i=0; i<sensor[handle].base_count; i++)
		sensor_set_delay(sensor[handle].base[i], ns);

	return sensor_set_delay(handle, ns);
}


static int poll (__attribute__((unused)) struct sensors_poll_device_t* dev,
		 sensors_event_t* data, int count)
{
	if (init_count == 0 || !data || count < 1)
		return -EINVAL;

	return sensor_poll(data, count);
}


static int batch (__attribute__((unused)) struct sensors_poll_device_1* dev,
		  int sensor_handle, __attribute__((unused)) int flags,
		  int64_t sampling_period_ns,
		  __attribute__((unused)) int64_t max_report_latency_ns)
{
	return set_delay ((struct sensors_poll_device_t*)dev,
		sensor_handle, sampling_period_ns);
}

static int flush (__attribute__((unused)) struct sensors_poll_device_1* dev,
		  int handle)
{
	return sensor_flush (handle);
}


static int close_module (__attribute__((unused)) hw_device_t *device)
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
	static struct sensors_poll_device_1 poll_device;

	if (strcmp(id, SENSORS_HARDWARE_POLL))
                return -EINVAL;

	poll_device.common.tag		= HARDWARE_DEVICE_TAG;
	poll_device.common.version	= SENSORS_DEVICE_API_VERSION_1_3;
	poll_device.common.module	= (struct hw_module_t*) module;
	poll_device.common.close	= close_module;

	poll_device.activate		= activate;
	poll_device.setDelay		= set_delay;
	poll_device.poll		= poll;
	poll_device.batch		= batch;
	poll_device.flush		= flush;

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
			.version_minor = 3,
			.id = SENSORS_HARDWARE_MODULE_ID,
			.name = "IIO sensors HAL",
			.author = "Intel",
			.methods = &module_methods,
		},
		.get_sensors_list = get_sensors_list
};

