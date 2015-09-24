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

/*
 * This file represents the entry point for the activity recognition HAL module.
 */

#include <utils/Log.h>
#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <pthread.h>
#include <fcntl.h>
#include <hardware/sensors.h>

#include "common.h"
#include "activity_event_utils.h"
#include <linux/iio/events.h>

#define MODULE_VERSION		1
#define HAL_VERSION		0

#define MODULE_NAME		"Activity recognition HAL"
#define MODULE_AUTHOR		"Intel"

#define CONTROL_FD		(-1)
#define EXIT_FD			(-2)

/*
 * This table maps syfs entries in scan_elements directories to sensor types,
 * and will also be used to determine other sysfs names as well as the iio
 * device number associated to a specific sensor.
 */
sensor_catalog_entry_t sensor_catalog[] = {
	{
		.tag		= "activity",
		.type		= SENSOR_TYPE_SIGNIFICANT_MOTION,
		.num_channels	= 3,
		.is_virtual	= 0,
		.channel	= {
			{
				DECLARE_VOID_CHANNEL("still")
				.num_events = 2,
				.event = {
					{ DECLARE_GENERIC_EVENT("activity", "still", "thresh", "rising") },
					{ DECLARE_GENERIC_EVENT("activity", "still", "thresh", "falling") },
				},
			},
			{
				DECLARE_VOID_CHANNEL("walking")
				.num_events = 2,
				.event = {
					{ DECLARE_GENERIC_EVENT("activity", "walking", "thresh", "rising") },
					{ DECLARE_GENERIC_EVENT("activity", "walking", "thresh", "falling") },
				},
			},
			{
				DECLARE_VOID_CHANNEL("running")
				.num_events = 2,
				.event = {
					{ DECLARE_GENERIC_EVENT("activity", "running", "thresh", "rising") },
					{ DECLARE_GENERIC_EVENT("activity", "running", "thresh", "falling") },
				},
			},
		},
	},
};

unsigned int catalog_size = ARRAY_SIZE(sensor_catalog);

/* All possible activities - see activity_recognition.h */
static char const* sysfs_activity_names[MAX_ACTIVITIES] = {
	"in_vehicle",
	"on_bicycle",
	"walking",
	"running",
	"still",
	"tilting",
};

/* Internal HAL info */
static struct activity_event_info supported_activities[MAX_ACTIVITIES + 1];
/* Android framework level description (activities' name). The element on the
 * first position (0) is reserved for the FLUSH COMPLETE event, thus we have
 * (MAX_ACTIVITIES + 1) possible activities.
 */
static char const* supported_activity_names[MAX_ACTIVITIES + 1];
/* Supported activities count */
static unsigned int count;

static int poll_fd, control_fd, exit_fd;
static pthread_t control_thread;

static activity_recognition_callback_procs_t activity_dev_callback;
static pthread_mutex_t callback_mutex;

static int instances_count;

static void register_activity_callback(const struct activity_recognition_device *dev __attribute((unused)),
				       const activity_recognition_callback_procs_t* callback)
{
	pthread_mutex_lock(&callback_mutex);
	activity_dev_callback.activity_callback = callback->activity_callback;
	pthread_mutex_unlock(&callback_mutex);
}

static bool check_activity_event(uint32_t activity_handle, uint32_t event_type)
{
	if (activity_handle > count)
		return false;

	/* Also return false if the handle is 0 - this is reserved for flush
	 * event, that is currently not supported. */
	if (activity_handle == 0)
		return 0;

	switch (event_type) {
		case ACTIVITY_EVENT_ENTER:
		case ACTIVITY_EVENT_EXIT:
			return true;
		case ACTIVITY_EVENT_FLUSH_COMPLETE:
			/* Not supported yet */
		default:
			return false;
	}
}

static int set_activity_event_state(const struct activity_recognition_device *dev __attribute((unused)),
				    uint32_t activity_handle, uint32_t event_type,
				    char const *action)
{
	uint64_t control_code;
	int ret;

	/* Check received index boundaries */
	if (!check_activity_event(activity_handle, event_type)) {
		ALOGE("Received invalid <activity %d, event %d> %s request\n",
		      activity_handle, event_type, action);
		return -EINVAL;
	}

	control_code = get_control_code((uint8_t) 1,
					(uint8_t) activity_handle,
					(uint8_t) event_type);

	ret = write(control_fd, &control_code, sizeof(control_code));
	if (ret < 0) {
		ALOGE("Error writing to control fd to %s activity event\n", action);
		return errno;
	}

	ALOGI("Sent %s <%s, %i> request\n", action, supported_activity_names[activity_handle], event_type);

	return 0;
}

static int enable_activity_event(const struct activity_recognition_device *dev,
				 uint32_t activity_handle, uint32_t event_type,
				 int64_t max_batch_report_latency_ns __attribute((unused)))
{
	return set_activity_event_state(dev, activity_handle, event_type, "enable");
}

static int disable_activity_event(const struct activity_recognition_device *dev,
				  uint32_t activity_handle, uint32_t event_type)
{
	return set_activity_event_state(dev, activity_handle, event_type, "disable");
}

/**
 * For now, just report that the function call has been made, since we yet do
 * not have a batch FIFO device.
 */
static int flush(const struct activity_recognition_device *dev __attribute((unused)))
{
	ALOGV("Flushing...\n");

	return 0;
}

static void process_disabling_activity_ev(uint8_t activity, uint8_t event);

static int close_device(struct hw_device_t *device __attribute((unused)))
{
	int j, ret, exit_ping;
	unsigned int i;

	if (!instances_count)
		return -EINVAL;

	instances_count--;

	if (instances_count)
		return 0;

	/* Send exit request to the worker thread and close resources. We can
	 * write anything to the exit fd, we just need the event.
	 */
	exit_ping = 1;
	write(exit_fd, &exit_ping, sizeof(exit_ping));

	/* Wait for worker thread to finish in order to release shared
	 * resources.
	 */
	pthread_join(control_thread, NULL);

	/* Close exit fd after sending the canceling request. */
	ret = epoll_ctl(poll_fd, EPOLL_CTL_DEL, exit_fd, NULL);
	if (ret == -1)
		ALOGE("Error deleting exit fd from polling pool\n");
	close(exit_fd);

	/* Clean control data. */
	ret = epoll_ctl(poll_fd, EPOLL_CTL_DEL, control_fd, NULL);
	if (ret == -1)
		ALOGE("Error deleting control fd from polling pool\n");
	close(control_fd);

	/* Disable all monitored <activity, event> pairs. This step should be
	 * the last one, after worker thread has ended in order to avoid
	 * supported_activities data corruption and avoid using another lock.
	 */
	for (i = 1; i <= count; i++)
		for (j = 0; j < MAX_EVENTS_PER_ACTIVITY; j++)
			if (supported_activities[i].monitored[j])
				process_disabling_activity_ev(
							      (uint8_t) i,
							      supported_activities[i].event[j]->event_type);

	close(poll_fd);

	pthread_mutex_destroy(&callback_mutex);

	ALOGI("Successfully closed device\n");

	return 0;
}

/*
 * Finds the event given by type in the sensor's channel structure and retrieves
 * its index.
 * Equivalence (activity HAL naming - sensor HAL naming):
 *	ACTIVITY_EVENT_ENTER - "rising"
 *	ACTIVITY_EVENT_EXIT - "falling"
 */
static int get_ev_index(int ev_type, channel_descriptor_t *chann)
{
	int i;
	char const *ev_dir;

	switch (ev_type) {
		case ACTIVITY_EVENT_ENTER:
			ev_dir = "rising";
			break;
		case ACTIVITY_EVENT_EXIT:
			ev_dir = "falling";
			break;
		default:
			ev_dir = NULL;
			return -1;
	}

	for (i = 0; i < chann->num_events; i++) {
		if (strcmp(ev_dir, chann->event[i].dir) == 0)
			return i;
	}

	return -1;
}

static int set_event_enabling(int dev_num, const char *en_path, int value)
{
	char path[PATH_MAX];
	int ret;

	ret = snprintf(path, sizeof(EVENTS_PATH) + sizeof(en_path), EVENTS_PATH "%s", dev_num, en_path);
	if (ret < 0)
		return ret;

	return sysfs_write_int(path, value);

}

static void process_enabling_activity_ev(uint8_t activity, uint8_t event)
{
	struct activity_event_info *activ = supported_activities + activity;
	channel_descriptor_t *chann;
	struct activity_event *ev;
	char path[PATH_MAX];
	struct epoll_event ev_data;
	int dev_fd, ev_index, ret;
	unsigned int i;
	bool open_now = false;

	/* Allocate event structure and populate it */
	ev = malloc(sizeof(*ev));
	if (!ev) {
		ALOGE("Error allocating activity event for enabling\n");
		return;
	}

	ev->event_type	= (uint32_t) event;
	ev->activity	= (uint32_t) activity;
	ev->timestamp	= -1;

	/* The event fd is one per device, so we need to check if we have not
	 * retrieved it already when monitoring another <activity, event> pair.
	 * If it has not been retrieved, get it and update all other activities
	 * associated with the same device.
	 */
	if (activ->event_fd == -1) {
		ret = snprintf(path, sizeof(DEV_FILE_PATH), DEV_FILE_PATH, activ->dev_num);
		if (ret < 0)
			goto dev_err;

		dev_fd = open(path, O_RDONLY | O_NONBLOCK);
		if (dev_fd < 0)
			goto dev_err;

		ret = ioctl(dev_fd, IIO_GET_EVENT_FD_IOCTL, &activ->event_fd);
		close(dev_fd);
		if (ret < 0)
			goto dev_err;

		open_now = true;

		ev_data.events	= EPOLLIN;
		ev_data.data.fd	= activ->event_fd;
		ret = epoll_ctl(poll_fd, EPOLL_CTL_ADD, activ->event_fd, &ev_data);
		if (ret == -1)
			goto event_err;

		/* Update all other activities generated by this device */
		for (i = 1; i <= count; i++)
			if (supported_activities[i].dev_num == activ->dev_num)
				supported_activities[i].event_fd = activ->event_fd;
	}

	/* Activate the event */
	chann = sensor_catalog[activ->sensor_catalog_index].channel + activ->channel_index;
	ev_index = get_ev_index((int)event, chann);
	if (ev_index < 0) {
		ALOGE("Invalid event index: %d\n", ev_index);
		goto event_err;
	}
	ret = set_event_enabling(activ->dev_num, chann->event[ev_index].ev_en_path, 1);
	if (ret < 0)
		goto event_err;

	/* Internally mark that the <activity, event> pair is being monitored.
	 * We keep the same event index in our activity structure as is in the
	 * channel descriptor structure.
	 */
	activ->event[ev_index]		= ev;
	activ->monitored[ev_index]	= true;
	activ->event_count++;

	return;

event_err:
	if (open_now) {
		close(activ->event_fd);
		for (i = 1; i <= count; i++)
			if (supported_activities[i].dev_num == activ->dev_num)
				supported_activities[i].event_fd = -1;
	}
dev_err:
	free(ev);
}

static bool device_not_monitored(int dev_num)
{
	unsigned int i;

	for (i = 1; i <= count; i++)
		if (supported_activities[i].dev_num == dev_num &&
		    supported_activities[i].event_count > 0) {
			return false;
		}

	return true;
}

static void process_disabling_activity_ev(uint8_t activity, uint8_t event)
{
	struct activity_event_info *activ = supported_activities + activity;
	channel_descriptor_t *chann;
	int ev_index, ret;
	unsigned int i;

	/* Deactivate the event. */
	chann = sensor_catalog[activ->sensor_catalog_index].channel + activ->channel_index;
	ev_index = get_ev_index((int)event, chann);
	if (ev_index < 0)
		ALOGE("Invalid event index: %d\n", ev_index);
	else {
		ret = set_event_enabling(activ->dev_num, chann->event[ev_index].ev_en_path, 0);
		if (ret < 0)
			ALOGE("Could not deactivate event - writing error\n");
	}

	/* Mark that the <activity, event> pair is not monitored any longer. */
	activ->monitored[ev_index] = false;
	activ->event_count--;

	/* Close the event fd if this is the last pair monitored for the given
	 * device and remove it from the polling pool.
	 */
	if (device_not_monitored(activ->dev_num)) {
		ret = epoll_ctl(poll_fd, EPOLL_CTL_DEL, activ->event_fd, NULL);
		if (ret == -1) {
			ALOGE("Error removing event fd from polling pool\n");
			return;
		}

		close(activ->event_fd);
		for (i = 1; i <= count; i++)
			if (supported_activities[i].dev_num == activ->dev_num)
				supported_activities[i].event_fd = -1;
	}

	/* Free resources. */
	free(activ->event[ev_index]);
	activ->event[ev_index] = NULL;
}

static void process_control_event(void)
{
	struct control_event_data control_data;
	uint64_t control_code;
	ssize_t ret;

	/* Read control data from the control fd and interpret it */
	ret = read(control_fd, &control_code, sizeof(control_code));
	if (ret < 0) {
		ALOGW("Error reading from control fd\n");
		return;
	}

	get_control_data(control_code, &control_data);

	if (control_data.enable)
		process_enabling_activity_ev(control_data.activity, control_data.event);
	else
		process_disabling_activity_ev(control_data.activity, control_data.event);
}

static int get_activity_index(int modifier)
{
	unsigned int i;

	/* Start from 1 since 0 is reserved for FLUSH_COMPLETE event. */
	for (i = 1; i <= count; i++)
		if (supported_activities[i].modifier == modifier)
			return i;

	return -1;
}

static void process_activity_event(int fd, struct activity_event events[], int *count)
{
	struct iio_event_data event;
	int ret, chann_type, ev_type, ev_dir, ev_modifier, index, activity_index;

	/* Retrieve event. */
	ret = read(fd, &event, sizeof(event));
	if (ret < 0) {
		ALOGE("Error reading event\n");
		return;
	}

	/* Extract fields we are interested in and check the generated event. */
	chann_type = IIO_EVENT_CODE_EXTRACT_CHAN_TYPE(event.id);
	if (chann_type != IIO_ACTIVITY) {
		ALOGW("Event came from other than an activity channel\n");
		return;
	}

	ev_modifier = IIO_EVENT_CODE_EXTRACT_MODIFIER(event.id);
	switch (ev_modifier) {
		case IIO_MOD_STILL:
		case IIO_MOD_WALKING:
		case IIO_MOD_RUNNING:
			activity_index = get_activity_index(ev_modifier);
			if (activity_index >= 0)
				break;
		default:
			ALOGW("Incompatible modifier - none of the supported activities is present\n");
			return;
	}

	ev_type = IIO_EVENT_CODE_EXTRACT_TYPE(event.id);
	if (ev_type != IIO_EV_TYPE_THRESH) {
		ALOGW("Event type is not threshold\n");
		return;
	}

	ev_dir = IIO_EVENT_CODE_EXTRACT_DIR(event.id);
	switch (ev_dir) {
		case IIO_EV_DIR_RISING:
			ev_dir = ACTIVITY_EVENT_ENTER;
			break;
		case IIO_EV_DIR_FALLING:
			ev_dir = ACTIVITY_EVENT_EXIT;
			break;
		default:
			ALOGW("Incompatible event direction - only RISING and FALLING supported\n");
			return;
	}

	/* Add the activity event to the array for further processing. */
	index = *count;
	events[index].event_type	= ev_dir;
	events[index].activity		= activity_index;
	events[index].timestamp		= event.timestamp;

	index++;
	*count = index;
}

static void* events_routine(void *arg __attribute((unused)))
{
	struct epoll_event events[MAX_ACTIVITIES + 2];
	struct activity_event data_events[MAX_ACTIVITIES];
	int no_events, i, no_activity_events;

	while (1) {
		ALOGV("Waiting for sensor events ...\n");

		no_activity_events = 0;
		no_events = epoll_wait(poll_fd, events, MAX_ACTIVITIES + 2, -1);
		if (no_events == -1) {
			ALOGE("epoll_wait error %s\n", strerror(errno));
			continue;
		}

		for (i = 0; i < no_events; i++)
			if (events[i].events == EPOLLIN) {
				int data = events[i].data.fd;

				if (data >= 0)
					process_activity_event(data,
							       data_events,
							       &no_activity_events);
				else switch (data) {
					case CONTROL_FD:
						process_control_event();
						break;
					case EXIT_FD:
						return NULL;
					default:
						ALOGW("Invalid event user data: %d \n", events[i].data.fd);
						break;
				}
			} else
				ALOGW("Epoll events %i not expected\n", events[i].events);

		/* Call the callback function for the retrieved events (if it
		 * has been set).
		 */
		pthread_mutex_lock(&callback_mutex);
		if (activity_dev_callback.activity_callback) {
			activity_dev_callback.activity_callback(
								&activity_dev_callback,
								data_events,
								no_activity_events);
		}
		pthread_mutex_unlock(&callback_mutex);
	}
}

static int set_up_control_data(void)
{
	struct epoll_event control_ev, exit_ev;
	int ret = 0;

	ret = pthread_mutex_init(&callback_mutex, NULL);
	if (ret)
		return ret;

	/* Maximum fds is maximum activities + 1 control fd + 1 exit fd */
	poll_fd = epoll_create(MAX_ACTIVITIES + 2);
	if (poll_fd == -1)
		return errno;
	if (ret)
		goto poll_control_err;

	control_fd = eventfd(0, 0);
	if (control_fd == -1) {
		ret = errno;
		goto poll_control_err;
	}

	control_ev.events = EPOLLIN;
	/* Set data field to event file descriptor */
	control_ev.data.fd = CONTROL_FD;
	ret = epoll_ctl(poll_fd, EPOLL_CTL_ADD, control_fd, &control_ev);
	if (ret == -1)
		goto control_data_err;

	exit_fd = eventfd(0, 0);
	if (exit_fd == -1) {
		ALOGE("Error allocating exit fd\n");
		goto exit_control_err;
	}
	exit_ev.events	= EPOLLIN;
	exit_ev.data.fd	= EXIT_FD;
	ret = epoll_ctl(poll_fd, EPOLL_CTL_ADD, exit_fd, &exit_ev);
	if (ret == -1) {
		ALOGE("Error adding exit fd to the polling pool\n");
		goto exit_err;
	}

	/* Start worker thread to wait on all event sources */
	ret = pthread_create(&control_thread, NULL, events_routine, NULL);
	if (ret)
		goto thread_err;

	return 0;

thread_err:
	epoll_ctl(poll_fd, EPOLL_CTL_DEL, exit_fd, NULL);
exit_err:
	close(exit_fd);
exit_control_err:
	epoll_ctl(poll_fd, EPOLL_CTL_DEL, control_fd, NULL);
control_data_err:
	close(control_fd);
poll_control_err:
	close(poll_fd);

	return ret;
}

/* Returns the IIO_MOD_* equivalent to the given name. */
static int get_modifier_as_int(const char* mod)
{
	if (strncmp(mod, "still", sizeof("still")) == 0)
		return IIO_MOD_STILL;

	if (strncmp(mod, "walking", sizeof("walking")) == 0)
		return IIO_MOD_WALKING;

	if (strncmp(mod, "running", sizeof("running")) == 0)
		return IIO_MOD_RUNNING;

	return -1;
}

static void add_activity(int sensor_catalog_index, int channel_index,
			 int dev_num, const char *name)
{
	int index, i, modifier;

	if (count == MAX_ACTIVITIES) {
		ALOGE("Trying to add more than supported activities!\n");
		return;
	}

	modifier = get_modifier_as_int(name);
	if (modifier < 0) {
		ALOGE("Invalid channel name as modifier: %s\n", name);
		return;
	}

	index = ++count;

	for (i = 0; i < MAX_EVENTS_PER_ACTIVITY; i++) {
		supported_activities[index].event[i]		= NULL;
		supported_activities[index].monitored[i]	= false;
	}
	supported_activities[index].modifier			= modifier;
	supported_activities[index].event_count			= 0;
	supported_activities[index].sensor_catalog_index	= sensor_catalog_index;
	supported_activities[index].channel_index		= channel_index;
	supported_activities[index].dev_num			= dev_num;
	supported_activities[index].event_fd			= -1;

	supported_activity_names[index] = name;
}

static bool is_activity_valid(const char *activity_name)
{
	unsigned int i;

	/* Look if this activity has not been already added */
	for (i = 1; i <= count; i++)
		if (strcmp(supported_activity_names[i], activity_name) == 0)
			return false;

	/* Check that the found activity is recognized by this API */
	for (i = 0; i < MAX_ACTIVITIES; i++)
		if (strcmp(sysfs_activity_names[i], activity_name) == 0)
			return true;

	return false;
}

/* Get all possible activities provided by the IIO sensors */
static void discover_activity_events(void)
{
	channel_descriptor_t *chann;
	int i, num_channels, dev_num;
	unsigned int index;
	char event_sensors[catalog_size];

	/* Discover event sensors */
	for (dev_num = 0; dev_num < MAX_DEVICES; dev_num++) {
		discover_sensors(dev_num, EVENTS_PATH, event_sensors, check_event_sensors);
		for (index = 0; index < catalog_size; index++) {
			if (!event_sensors[index])
				continue;

			num_channels = sensor_catalog[index].num_channels;
			for (i = 0; i < num_channels; i++) {
				chann = sensor_catalog[index].channel + i;

				if (is_activity_valid(chann->name))
					add_activity(index, i, dev_num, chann->name);
			}
		}
	}

	ALOGI("Discovered %d activities\n", count);
}

static int open_module(const struct hw_module_t *module, const char *id,
		       struct hw_device_t **device)
{
	static struct activity_recognition_device activity_dev;
	int ret = 0;

	if (strncmp(id, ACTIVITY_RECOGNITION_HARDWARE_INTERFACE, sizeof(ACTIVITY_RECOGNITION_HARDWARE_INTERFACE)) != 0)
		return -EINVAL;

	activity_dev.common.tag		= HARDWARE_DEVICE_TAG;
	activity_dev.common.version	= ACTIVITY_RECOGNITION_API_VERSION_0_1;
	activity_dev.common.module	= (struct hw_module_t *) module;
	activity_dev.common.close	= close_device;

	activity_dev.register_activity_callback	= register_activity_callback;
	activity_dev.enable_activity_event	= enable_activity_event;
	activity_dev.disable_activity_event	= disable_activity_event;
	activity_dev.flush			= flush;

	*device = &activity_dev.common;

	if (instances_count == 0) {
		discover_activity_events();
		ret = set_up_control_data();

		ALOGI("Initialized activity recognition HAL (exit code %i)\n", ret);
	}

	instances_count++;

	return ret;
}

static struct hw_module_methods_t module_methods = {
	.open = open_module
};


static int get_supported_activities_list(struct activity_recognition_module *module __attribute((unused)),
					 char const* const* *activity_list)
{
	*activity_list = supported_activity_names + 1;

	return count;
}

/* Module descriptor visible to the Android framework. */
struct activity_recognition_module __attribute__ ((visibility ("default")))
	HAL_MODULE_INFO_SYM = {
		.common = {
			.tag			= HARDWARE_MODULE_TAG,
			.module_api_version	= MODULE_VERSION,
			.hal_api_version	= HAL_VERSION,
			.id 			= ACTIVITY_RECOGNITION_HARDWARE_MODULE_ID,
			.name			= MODULE_NAME,
			.author			= MODULE_AUTHOR,
			.methods		= &module_methods,
		},
		.get_supported_activities_list = get_supported_activities_list

};
