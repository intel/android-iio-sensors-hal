/*
 * Copyright (C) 2014 Intel Corporation.
 */

#include <stdlib.h>
#include <ctype.h>
#include <fcntl.h>
#include <pthread.h>
#include <time.h>
#include <math.h>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <utils/Log.h>
#include <hardware/sensors.h>
#include "control.h"
#include "enumeration.h"
#include "utils.h"
#include "transform.h"
#include "calibration.h"
#include "description.h"
#include "filtering.h"

/* Currently active sensors count, per device */
static int poll_sensors_per_dev[MAX_DEVICES];	/* poll-mode sensors */
static int trig_sensors_per_dev[MAX_DEVICES];	/* trigger, event based */

static int device_fd[MAX_DEVICES];   /* fd on the /dev/iio:deviceX file */
static int has_iio_ts[MAX_DEVICES];  /* ts channel available on this iio dev */
static int expected_dev_report_size[MAX_DEVICES]; /* expected iio scan len */
static int poll_fd; /* epoll instance covering all enabled sensors */

static int active_poll_sensors; /* Number of enabled poll-mode sensors */

/* We use pthread condition variables to get worker threads out of sleep */
static pthread_condattr_t thread_cond_attr	[MAX_SENSORS];
static pthread_cond_t     thread_release_cond	[MAX_SENSORS];
static pthread_mutex_t    thread_release_mutex	[MAX_SENSORS];

/*
 * We associate tags to each of our poll set entries. These tags have the
 * following values:
 * - a iio device number if the fd is a iio character device fd
 * - THREAD_REPORT_TAG_BASE + sensor handle if the fd is the receiving end of a
 *   pipe used by a sysfs data acquisition thread
 */
#define THREAD_REPORT_TAG_BASE	0x00010000

#define ENABLE_BUFFER_RETRIES 10
#define ENABLE_BUFFER_RETRY_DELAY_MS 10


inline int is_enabled (int s)
{
	return sensor[s].directly_enabled || sensor[s].ref_count;
}


static int check_state_change (int s, int enabled, int from_virtual)
{
	if (enabled) {
		if (sensor[s].directly_enabled)
					/*
					 * We're being enabled but already were
					 * directly activated: no change.
					 */
					return 0;

		if (!from_virtual)
			/* We're being directly enabled */
			sensor[s].directly_enabled = 1;

		if (sensor[s].ref_count)
			/* We were already indirectly enabled */
			return 0;

		return 1; /* Do continue enabling this sensor */
	}

	if (!is_enabled(s))
		/* We are being disabled but already were: no change */
		return 0;

	if (from_virtual && sensor[s].directly_enabled)
		/* We're indirectly disabled but the base is still active */
		return 0;

	/* We're now directly disabled */
	sensor[s].directly_enabled = 0;

	if (!from_virtual && sensor[s].ref_count)
		return 0; /* We still have ref counts */

	return 1; /* Do continue disabling this sensor */
}


static int enable_buffer(int dev_num, int enabled)
{
	char sysfs_path[PATH_MAX];
	int ret, retries, millisec;
	struct timespec req = {0};

	retries = ENABLE_BUFFER_RETRIES;
	millisec = ENABLE_BUFFER_RETRY_DELAY_MS;
	req.tv_sec = 0;
	req.tv_nsec = millisec * 1000000L;

	sprintf(sysfs_path, ENABLE_PATH, dev_num);

	while (retries--) {
		/* Low level, non-multiplexed, enable/disable routine */
		ret = sysfs_write_int(sysfs_path, enabled);
		if (ret > 0)
			break;

		ALOGE("Failed enabling buffer, retrying");
		nanosleep(&req, (struct timespec *)NULL);
	}

	if (ret < 0) {
		ALOGE("Could not enable buffer\n");
		return -EIO;
	}

	return 0;
}


static int setup_trigger (int s, const char* trigger_val)
{
	char sysfs_path[PATH_MAX];
	int ret = -1, attempts = 5;

	sprintf(sysfs_path, TRIGGER_PATH, sensor[s].dev_num);

	if (trigger_val[0] != '\n')
		ALOGI("Setting S%d (%s) trigger to %s\n", s,
			sensor[s].friendly_name, trigger_val);

	while (ret == -1 && attempts) {
		ret = sysfs_write_str(sysfs_path, trigger_val);
		attempts--;
	}

	if (ret != -1)
		sensor[s].selected_trigger = trigger_val;
	else
		ALOGE("Setting S%d (%s) trigger to %s FAILED.\n", s,
			sensor[s].friendly_name, trigger_val);
	return ret;
}


static void enable_iio_timestamp (int dev_num, int known_channels)
{
	/* Check if we have a dedicated iio timestamp channel */

	char spec_buf[MAX_TYPE_SPEC_LEN];
	char sysfs_path[PATH_MAX];
	int n;

	sprintf(sysfs_path, CHANNEL_PATH "%s", dev_num, "in_timestamp_type");

	n = sysfs_read_str(sysfs_path, spec_buf, sizeof(spec_buf));

	if (n <= 0)
		return;

	if (strcmp(spec_buf, "le:s64/64>>0"))
		return;

	/* OK, type is int64_t as expected, in little endian representation */

	sprintf(sysfs_path, CHANNEL_PATH"%s", dev_num, "in_timestamp_index");

	if (sysfs_read_int(sysfs_path, &n))
		return;

	/* Check that the timestamp comes after the other fields we read */
	if (n != known_channels)
		return;

	/* Try enabling that channel */
	sprintf(sysfs_path, CHANNEL_PATH "%s", dev_num, "in_timestamp_en");

	sysfs_write_int(sysfs_path, 1);

	if (sysfs_read_int(sysfs_path, &n))
		return;

	if (n) {
		ALOGI("Detected timestamp channel on iio device %d\n", dev_num);
		has_iio_ts[dev_num] = 1;
	}
}


static int decode_type_spec (const char type_buf[MAX_TYPE_SPEC_LEN],
			     struct datum_info_t *type_info)
{
	/* Return size in bytes for this type specification, or -1 in error */
	char sign;
	char endianness;
	unsigned int realbits, storagebits, shift;
	int tokens;

	/* Valid specs: "le:u10/16>>0", "le:s16/32>>0" or "le:s32/32>>0" */

	tokens = sscanf(type_buf, "%ce:%c%u/%u>>%u",
			&endianness, &sign, &realbits, &storagebits, &shift);

	if     (tokens != 5 ||
		(endianness != 'b' && endianness != 'l') ||
		(sign != 'u' && sign != 's') ||
		realbits > storagebits ||
		(storagebits != 16 && storagebits != 32 && storagebits != 64)) {
			ALOGE("Invalid iio channel type spec: %s\n", type_buf);
			return -1;
		}

	type_info->endianness	=		endianness;
	type_info->sign		=		sign;
	type_info->realbits	=	(short)	realbits;
	type_info->storagebits	=	(short)	storagebits;
	type_info->shift	=	(short)	shift;

	return storagebits / 8;
}


void build_sensor_report_maps (int dev_num)
{
	/*
	 * Read sysfs files from a iio device's scan_element directory, and
	 * build a couple of tables from that data. These tables will tell, for
	 * each sensor, where to gather relevant data in a device report, i.e.
	 * the structure that we read from the /dev/iio:deviceX file in order to
	 * sensor report, itself being the data that we return to Android when a
	 * sensor poll completes. The mapping should be straightforward in the
	 * case where we have a single sensor active per iio device but, this is
	 * not the general case. In general several sensors can be handled
	 * through a single iio device, and the _en, _index and _type syfs
	 * entries all concur to paint a picture of what the structure of the
	 * device report is.
	 */

	int s;
	int c;
	int n;
	int i;
	int ch_index;
	char* ch_spec;
	char spec_buf[MAX_TYPE_SPEC_LEN];
	struct datum_info_t* ch_info;
	int size;
	char sysfs_path[PATH_MAX];
	int known_channels;
	int offset;
	int channel_size_from_index[MAX_SENSORS * MAX_CHANNELS] = { 0 };
	int sensor_handle_from_index[MAX_SENSORS * MAX_CHANNELS] = { 0 };
	int channel_number_from_index[MAX_SENSORS * MAX_CHANNELS] = { 0 };

	known_channels = 0;

	/* For each sensor that is linked to this device */
	for (s=0; s<sensor_count; s++) {
		if (sensor[s].dev_num != dev_num)
			continue;

		i = sensor[s].catalog_index;

		/* Read channel details through sysfs attributes */
		for (c=0; c<sensor[s].num_channels; c++) {

			/* Read _type file */
			sprintf(sysfs_path, CHANNEL_PATH "%s",
				sensor[s].dev_num,
				sensor_catalog[i].channel[c].type_path);

			n = sysfs_read_str(sysfs_path, spec_buf, 
						sizeof(spec_buf));

			if (n == -1) {
					ALOGW(	"Failed to read type: %s\n",
					sysfs_path);
					continue;
				}

			ch_spec = sensor[s].channel[c].type_spec;

			memcpy(ch_spec, spec_buf, sizeof(spec_buf));

			ch_info = &sensor[s].channel[c].type_info;

			size = decode_type_spec(ch_spec, ch_info);

			/* Read _index file */
			sprintf(sysfs_path, CHANNEL_PATH "%s",
				sensor[s].dev_num,
				sensor_catalog[i].channel[c].index_path);

			n = sysfs_read_int(sysfs_path, &ch_index);

			if (n == -1) {
					ALOGW(	"Failed to read index: %s\n",
						sysfs_path);
					continue;
				}

			if (ch_index >= MAX_SENSORS) {
				ALOGE("Index out of bounds!: %s\n", sysfs_path);
				continue;
			}

			/* Record what this index is about */

			sensor_handle_from_index [ch_index] = s;
			channel_number_from_index[ch_index] = c;
			channel_size_from_index  [ch_index] = size;

			known_channels++;
		}

		/* Stop sampling - if we are recovering from hal restart */
                enable_buffer(dev_num, 0);
                setup_trigger(s, "\n");

		/* Turn on channels we're aware of */
		for (c=0;c<sensor[s].num_channels; c++) {
			sprintf(sysfs_path, CHANNEL_PATH "%s",
				sensor[s].dev_num,
				sensor_catalog[i].channel[c].en_path);
			sysfs_write_int(sysfs_path, 1);
		}
	}

	ALOGI("Found %d channels on iio device %d\n", known_channels, dev_num);

	/*
	 * Now that we know which channels are defined, their sizes and their
	 * ordering, update channels offsets within device report. Note: there
	 * is a possibility that several sensors share the same index, with
	 * their data fields being isolated by masking and shifting as specified
	 * through the real bits and shift values in type attributes. This case
	 * is not currently supported. Also, the code below assumes no hole in
	 * the sequence of indices, so it is dependent on discovery of all
	 * sensors.
	 */
	 offset = 0;
	 for (i=0; i<MAX_SENSORS * MAX_CHANNELS; i++) {
		s =	sensor_handle_from_index[i];
		c =	channel_number_from_index[i];
		size = 	channel_size_from_index[i];

		if (!size)
			continue;

		ALOGI("S%d C%d : offset %d, size %d, type %s\n",
		      s, c, offset, size, sensor[s].channel[c].type_spec);

		sensor[s].channel[c].offset	= offset;
		sensor[s].channel[c].size		= size;

		offset += size;
	 }

	/* Enable the timestamp channel if there is one available */
	enable_iio_timestamp(dev_num, known_channels);

	/* Add padding and timestamp size if it's enabled on this iio device */
	if (has_iio_ts[dev_num])
		offset = (offset+7)/8*8 + sizeof(int64_t);

	expected_dev_report_size[dev_num] = offset;
	ALOGI("Expecting %d scan length on iio dev %d\n", offset, dev_num);

	if (expected_dev_report_size[dev_num] > MAX_DEVICE_REPORT_SIZE) {
		ALOGE("Unexpectedly large scan buffer on iio dev%d: %d bytes\n",
		      dev_num, expected_dev_report_size[dev_num]);

		expected_dev_report_size[dev_num] = MAX_DEVICE_REPORT_SIZE;
	}
}


int adjust_counters (int s, int enabled, int from_virtual)
{
	/*
	 * Adjust counters based on sensor enable action. Return values are:
	 *  0 if the operation was completed and we're all set
	 *  1 if we toggled the state of the sensor and there's work left
	 */

	int dev_num = sensor[s].dev_num;

	if (!check_state_change(s, enabled, from_virtual))
		/* The state of the sensor remains the same: we're done */
		return 0;

	if (enabled) {
		ALOGI("Enabling sensor %d (iio device %d: %s)\n",
			s, dev_num, sensor[s].friendly_name);

		switch (sensor[s].type) {
			case SENSOR_TYPE_MAGNETIC_FIELD:
				compass_read_data(&sensor[s]);
				break;

			case SENSOR_TYPE_GYROSCOPE:
				gyro_cal_init(&sensor[s]);
				break;
		}
	} else {
		ALOGI("Disabling sensor %d (iio device %d: %s)\n", s, dev_num,
		      sensor[s].friendly_name);

		/* Sensor disabled, lower report available flag */
		sensor[s].report_pending = 0;

		if (sensor[s].type == SENSOR_TYPE_MAGNETIC_FIELD)
			compass_store_data(&sensor[s]);

		if (sensor[s].type == SENSOR_TYPE_GYROSCOPE)
			gyro_store_data(&sensor[s]);
	}

	/* We changed the state of a sensor: adjust device ref counts */

	if (sensor[s].num_channels) {

			if (enabled)
				trig_sensors_per_dev[dev_num]++;
			else
				trig_sensors_per_dev[dev_num]--;

			return 1;
	}

	if (enabled) {
		active_poll_sensors++;
		poll_sensors_per_dev[dev_num]++;
		return 1;
	}

	active_poll_sensors--;
	poll_sensors_per_dev[dev_num]--;
	return 1;
}


static int get_field_count (int s)
{
	switch (sensor[s].type) {
		case SENSOR_TYPE_ACCELEROMETER:		/* m/s^2	*/
		case SENSOR_TYPE_MAGNETIC_FIELD:	/* micro-tesla	*/
		case SENSOR_TYPE_ORIENTATION:		/* degrees	*/
		case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
		case SENSOR_TYPE_GYROSCOPE:		/* radians/s	*/
			return 3;

		case SENSOR_TYPE_LIGHT:			/* SI lux units */
		case SENSOR_TYPE_AMBIENT_TEMPERATURE:	/* °C		*/
		case SENSOR_TYPE_TEMPERATURE:		/* °C		*/
		case SENSOR_TYPE_PROXIMITY:		/* centimeters	*/
		case SENSOR_TYPE_PRESSURE:		/* hecto-pascal */
		case SENSOR_TYPE_RELATIVE_HUMIDITY:	/* percent */
			return 1;

		case SENSOR_TYPE_ROTATION_VECTOR:
			return 4;

		default:
			ALOGE("Unknown sensor type!\n");
			return 0;			/* Drop sample */
	}
}


static void* acquisition_routine (void* param)
{
	/*
	 * Data acquisition routine run in a dedicated thread, covering a single
	 * sensor. This loop will periodically retrieve sampling data through
	 * sysfs, then package it as a sample and transfer it to our master poll
	 * loop through a report fd. Checks for a cancellation signal quite
	 * frequently, as the thread may be disposed of at any time. Note that
	 * Bionic does not provide pthread_cancel / pthread_testcancel...
	 */

	int s = (int) (size_t) param;
	int num_fields, sample_size;
	struct sensors_event_t data = {0};
	int c;
	int ret;
	struct timespec target_time;
	int64_t timestamp, period, start, stop;

	if (s < 0 || s >= sensor_count) {
		ALOGE("Invalid sensor handle!\n");
		return NULL;
	}

	ALOGI("Entering data acquisition thread S%d (%s), rate:%g\n",
	      s, sensor[s].friendly_name, sensor[s].sampling_rate);

	if (sensor[s].sampling_rate <= 0) {
		ALOGE("Invalid rate in acquisition routine for sensor %d: %g\n",
			s, sensor[s].sampling_rate);
		return NULL;
	}

	num_fields = get_field_count(s);
	sample_size = sizeof(int64_t) + num_fields * sizeof(float);

	/*
	 * Each condition variable is associated to a mutex that has to be
	 * locked by the thread that's waiting on it. We use these condition
	 * variables to get the acquisition threads out of sleep quickly after
	 * the sampling rate is adjusted, or the sensor is disabled.
	 */
	pthread_mutex_lock(&thread_release_mutex[s]);

	/* Pinpoint the moment we start sampling */
	timestamp = get_timestamp_monotonic();

	/* Check and honor termination requests */
	while (sensor[s].thread_data_fd[1] != -1) {
		start = get_timestamp_boot();
		/* Read values through sysfs */
		for (c=0; c<num_fields; c++) {
			data.data[c] = acquire_immediate_value(s, c);
			/* Check and honor termination requests */
			if (sensor[s].thread_data_fd[1] == -1)
				goto exit;
		}
		stop = get_timestamp_boot();
		data.timestamp = start/2 + stop/2;

		/* If the sample looks good */
		if (sensor[s].ops.finalize(s, &data)) {

			/* Pipe it for transmission to poll loop */
			ret = write(	sensor[s].thread_data_fd[1],
					&data.timestamp, sample_size);

			if (ret != sample_size)
				ALOGE("S%d write failure: wrote %d, got %d\n",
				      s, sample_size, ret);
		}

		/* Check and honor termination requests */
		if (sensor[s].thread_data_fd[1] == -1)
			goto exit;

		/* Recalculate period asumming sensor[s].sampling_rate
		 * can be changed dynamically during the thread run */
		if (sensor[s].sampling_rate <= 0) {
			ALOGE("Unexpected sampling rate for sensor %d: %g\n",
				s, sensor[s].sampling_rate);
			goto exit;
		}

		period = (int64_t) (1000000000.0 / sensor[s].sampling_rate);
		timestamp += period;
		set_timestamp(&target_time, timestamp);

		/*
		 * Wait until the sampling time elapses, or a rate change is
		 * signaled, or a thread exit is requested.
		 */
		ret = pthread_cond_timedwait(	&thread_release_cond[s],
						&thread_release_mutex[s],
						&target_time);
	}

exit:
	ALOGV("Acquisition thread for S%d exiting\n", s);
	pthread_mutex_unlock(&thread_release_mutex[s]);
	pthread_exit(0);
	return NULL;
}


static void start_acquisition_thread (int s)
{
	int incoming_data_fd;
	int ret;

	struct epoll_event ev = {0};

	ALOGV("Initializing acquisition context for sensor %d\n", s);

	/* Create condition variable and mutex for quick thread release */
	ret = pthread_condattr_init(&thread_cond_attr[s]);
	ret = pthread_condattr_setclock(&thread_cond_attr[s], CLOCK_MONOTONIC);
	ret = pthread_cond_init(&thread_release_cond[s], &thread_cond_attr[s]);
	ret = pthread_mutex_init(&thread_release_mutex[s], NULL);

	/* Create a pipe for inter thread communication */
	ret = pipe(sensor[s].thread_data_fd);

	incoming_data_fd = sensor[s].thread_data_fd[0];

	ev.events = EPOLLIN;
	ev.data.u32 = THREAD_REPORT_TAG_BASE + s;

	/* Add incoming side of pipe to our poll set, with a suitable tag */
	ret = epoll_ctl(poll_fd, EPOLL_CTL_ADD, incoming_data_fd , &ev);

	/* Create and start worker thread */
	ret = pthread_create(	&sensor[s].acquisition_thread,
				NULL,
				acquisition_routine,
				(void*) (size_t) s);
}


static void stop_acquisition_thread (int s)
{
	int incoming_data_fd = sensor[s].thread_data_fd[0];
	int outgoing_data_fd = sensor[s].thread_data_fd[1];

	ALOGV("Tearing down acquisition context for sensor %d\n", s);

	/* Delete the incoming side of the pipe from our poll set */
	epoll_ctl(poll_fd, EPOLL_CTL_DEL, incoming_data_fd, NULL);

	/* Mark the pipe ends as invalid ; that's a cheap exit flag */
	sensor[s].thread_data_fd[0] = -1;
	sensor[s].thread_data_fd[1] = -1;

	/* Close both sides of our pipe */
	close(incoming_data_fd);
	close(outgoing_data_fd);

	/* Stop acquisition thread and clean up thread handle */
	pthread_cond_signal(&thread_release_cond[s]);
	pthread_join(sensor[s].acquisition_thread, NULL);

	/* Clean up our sensor descriptor */
	sensor[s].acquisition_thread = -1;

	/* Delete condition variable and mutex */
	pthread_cond_destroy(&thread_release_cond[s]);
	pthread_mutex_destroy(&thread_release_mutex[s]);
}


static int is_fast_accelerometer (int s)
{
	/*
	 * Some games don't react well to accelerometers using any-motion
	 * triggers. Even very low thresholds seem to trip them, and they tend
	 * to request fairly high event rates. Favor continuous triggers if the
	 * sensor is an accelerometer and uses a sampling rate of at least 25.
	 */

	if (sensor[s].type != SENSOR_TYPE_ACCELEROMETER)
		return 0;

	if (sensor[s].sampling_rate < 25)
		return 0;

	return 1;
}


static void tentative_switch_trigger (int s)
{
	/*
	 * Under certain situations it may be beneficial to use an alternate
	 * trigger:
	 *
	 * - for applications using the accelerometer with high sampling rates,
	 *   prefer the continuous trigger over the any-motion one, to avoid
	 *   jumps related to motion thresholds
	 */

	if (is_fast_accelerometer(s) &&
		!(sensor[s].quirks & QUIRK_TERSE_DRIVER) &&
			sensor[s].selected_trigger ==
				sensor[s].motion_trigger_name)
		setup_trigger(s, sensor[s].init_trigger_name);
}


static int sensor_set_rate (int s, float requested_rate)
{
	/* Set the rate at which a specific sensor should report events */
	/* See Android sensors.h for indication on sensor trigger modes */

	char sysfs_path[PATH_MAX];
	char avail_sysfs_path[PATH_MAX];
	int dev_num		=	sensor[s].dev_num;
	int i			=	sensor[s].catalog_index;
	const char *prefix	=	sensor_catalog[i].tag;
	int per_sensor_sampling_rate;
	int per_device_sampling_rate;
	int32_t min_delay_us = sensor_desc[s].minDelay;
	max_delay_t max_delay_us = sensor_desc[s].maxDelay;
	float min_supported_rate = max_delay_us ? 1000000.0/max_delay_us : 1;
	float max_supported_rate =
		min_delay_us && min_delay_us != -1 ? 1000000.0/min_delay_us : 0;
	char freqs_buf[100];
	char* cursor;
	int n;
	float sr;
	float cur_sampling_rate; /* Currently used sampling rate	      */
	float arb_sampling_rate; /* Granted sampling rate after arbitration   */

	ALOGV("Sampling rate %g requested on sensor %d (%s)\n", requested_rate,
	      s, sensor[s].friendly_name);

	sensor[s].requested_rate = requested_rate;

	arb_sampling_rate = requested_rate;

	if (arb_sampling_rate < min_supported_rate) {
		ALOGV("Sampling rate %g too low for %s, using %g instead\n",
		       arb_sampling_rate, sensor[s].friendly_name,
		       min_supported_rate);

		arb_sampling_rate = min_supported_rate;
	}

	if (max_supported_rate && arb_sampling_rate > max_supported_rate) {
		ALOGV("Sampling rate %g too high for %s, using %g instead\n",
		arb_sampling_rate, sensor[s].friendly_name, max_supported_rate);
		arb_sampling_rate = max_supported_rate;
	}

	sensor[s].sampling_rate = arb_sampling_rate;

	/* If we're dealing with a poll-mode sensor */
	if (!sensor[s].num_channels) {
		if (is_enabled(s))
			/* Wake up thread so the new sampling rate gets used */
			pthread_cond_signal(&thread_release_cond[s]);
		return 0;
	}

	sprintf(sysfs_path, SENSOR_SAMPLING_PATH, dev_num, prefix);

	if (sysfs_read_float(sysfs_path, &cur_sampling_rate) != -1) {
		per_sensor_sampling_rate = 1;
		per_device_sampling_rate = 0;
	} else {
		per_sensor_sampling_rate = 0;

		sprintf(sysfs_path, DEVICE_SAMPLING_PATH, dev_num);

		if (sysfs_read_float(sysfs_path, &cur_sampling_rate) != -1)
			per_device_sampling_rate = 1;
		else
			per_device_sampling_rate = 0;
	}

	if (!per_sensor_sampling_rate && !per_device_sampling_rate) {
		ALOGE("No way to adjust sampling rate on sensor %d\n", s);
		return -ENOSYS;
	}

	/* Check if we have contraints on allowed sampling rates */

	sprintf(avail_sysfs_path, DEVICE_AVAIL_FREQ_PATH, dev_num);

	if (sysfs_read_str(avail_sysfs_path, freqs_buf, sizeof(freqs_buf)) > 0){
		cursor = freqs_buf;

		/* Decode allowed sampling rates string, ex: "10 20 50 100" */

		/* While we're not at the end of the string */
		while (*cursor && cursor[0]) {

			/* Decode a single value */
			sr = strtod(cursor, NULL);

			/*
			 * If this matches the selected rate, we're happy.
			 * Have some tolerance to counter rounding errors and
			 * avoid needless jumps to higher rates.
			 */
			if (fabs(arb_sampling_rate - sr) <= 0.001) {
				arb_sampling_rate = sr;
				break;
			}

			/*
			 * If we reached a higher value than the desired rate,
			 * adjust selected rate so it matches the first higher
			 * available one and stop parsing - this makes the
			 * assumption that rates are sorted by increasing value
			 * in the allowed frequencies string.
			 */
			if (sr > arb_sampling_rate) {
				arb_sampling_rate = sr;
				break;
			}

			/* Skip digits */
			while (cursor[0] && !isspace(cursor[0]))
				cursor++;

			/* Skip spaces */
			while (cursor[0] && isspace(cursor[0]))
					cursor++;
		}
	}

	if (max_supported_rate &&
		arb_sampling_rate > max_supported_rate) {
		arb_sampling_rate = max_supported_rate;
	}


	/* Coordinate with others active sensors on the same device, if any */
	if (per_device_sampling_rate)
		for (n=0; n<sensor_count; n++)
			if (n != s && sensor[n].dev_num == dev_num &&
			    sensor[n].num_channels && is_enabled(n) &&
			    sensor[n].sampling_rate > arb_sampling_rate) {
	ALOGV("Sampling rate shared between %s and %s, using %g instead of %g\n"
	      , sensor[s].friendly_name, sensor[n].friendly_name,
	      sensor[n].sampling_rate, arb_sampling_rate);
				arb_sampling_rate = sensor[n].sampling_rate;
			}

	sensor[s].sampling_rate = arb_sampling_rate;

	/*
	 * Update actual sampling rate field for this sensor and others which
	 * may be sharing the same sampling rate.
	 */
	if (per_device_sampling_rate)
		for (n=0; n<sensor_count; n++)
			if (sensor[n].dev_num == dev_num && n != s &&
			    sensor[n].num_channels)
				sensor[n].sampling_rate = arb_sampling_rate;

	/* If the desired rate is already active we're all set */
	if (arb_sampling_rate == cur_sampling_rate)
		return 0;

	ALOGI("Sensor %d (%s) sampling rate set to %g\n",
	      s, sensor[s].friendly_name, arb_sampling_rate);

	if (trig_sensors_per_dev[dev_num])
		enable_buffer(dev_num, 0);

	sysfs_write_float(sysfs_path, arb_sampling_rate);

	/* Check if it makes sense to use an alternate trigger */
	tentative_switch_trigger(s);

	if (trig_sensors_per_dev[dev_num])
		enable_buffer(dev_num, 1);

	return 0;
}


/*
 * We go through all the virtual sensors of the base - and the base itself
 * in order to recompute the maximum requested delay of the group and setup the base
 * at that specific delay.
 */
static int arbitrate_bases (int s)
{
	int i, vidx;

	float arbitrated_rate = 0;

	if (sensor[s].directly_enabled)
		arbitrated_rate = sensor[s].requested_rate;

	 for (i = 0; i < sensor_count; i++) {
			for (vidx = 0; vidx < sensor[i].base_count; vidx++)
			/* If we have a virtual sensor depending on this one - handle it */
				if (sensor[i].base[vidx] == s &&
					sensor[i].directly_enabled &&
					sensor[i].requested_rate > arbitrated_rate)
						arbitrated_rate = sensor[i].requested_rate;
		}

	return sensor_set_rate(s, arbitrated_rate);
}


/*
 * Re-assesment for delays. We need to re-asses delays for all related groups
 * of sensors everytime a sensor enables / disables / changes frequency.
 */
int arbitrate_delays (int s)
{
	int i;

	if (!sensor[s].is_virtual) {
		return arbitrate_bases(s);
	}
	/* Is virtual sensor - go through bases */
	for (i = 0; i < sensor[s].base_count; i++)
		arbitrate_bases(sensor[s].base[i]);

	return 0;
}


static int sensor_activate_virtual (int s, int enabled, int from_virtual)
{
	int i, base;

	sensor[s].event_count = 0;
	sensor[s].meta_data_pending = 0;

	if (!check_state_change(s, enabled, from_virtual))
		/* The state of the sensor remains the same ; we're done */
		return 0;

	if (enabled)
		ALOGI("Enabling sensor %d (%s)\n", s, sensor[s].friendly_name);
	else
		ALOGI("Disabling sensor %d (%s)\n", s, sensor[s].friendly_name);

	sensor[s].report_pending = 0;

	for (i=0; i<sensor[s].base_count; i++) {

		base = sensor[s].base[i];
		sensor_activate(base, enabled, 1);

		if (enabled)
			sensor[base].ref_count++;
		else
			sensor[base].ref_count--;
	}

	return 0;
}


int sensor_activate (int s, int enabled, int from_virtual)
{
	char device_name[PATH_MAX];
	struct epoll_event ev = {0};
	int dev_fd;
	int ret;
	int dev_num = sensor[s].dev_num;
	int is_poll_sensor = !sensor[s].num_channels;

	if (sensor[s].is_virtual) {
		sensor_activate_virtual(s, enabled, from_virtual);
		arbitrate_delays(s);
		return 0;
	}

	/* Prepare the report timestamp field for the first event, see set_report_ts method */
	sensor[s].report_ts = 0;

	ret = adjust_counters(s, enabled, from_virtual);

	/* If the operation was neutral in terms of state, we're done */
	if (ret <= 0)
		return ret;

	arbitrate_delays(s);

	sensor[s].event_count = 0;
	sensor[s].meta_data_pending = 0;

	if (enabled && (sensor[s].quirks & QUIRK_NOISY))
		/* Initialize filtering data if required */
		setup_noise_filtering(s);

	if (!is_poll_sensor) {

		/* Stop sampling */
		enable_buffer(dev_num, 0);
		setup_trigger(s, "\n");

		/* If there's at least one sensor enabled on this iio device */
		if (trig_sensors_per_dev[dev_num]) {

			/* Start sampling */
			setup_trigger(s, sensor[s].init_trigger_name);
			enable_buffer(dev_num, 1);
		}
	}

	/*
	 * Make sure we have a fd on the character device ; conversely, close
	 * the fd if no one is using associated sensors anymore. The assumption
	 * here is that the underlying driver will power on the relevant
	 * hardware block while someone holds a fd on the device.
	 */
	dev_fd = device_fd[dev_num];

	if (!enabled) {
		if (is_poll_sensor)
			stop_acquisition_thread(s);

		if (dev_fd != -1 && !poll_sensors_per_dev[dev_num] &&
			!trig_sensors_per_dev[dev_num]) {
				/*
				 * Stop watching this fd. This should be a no-op
				 * in case this fd was not in the poll set.
				 */
				epoll_ctl(poll_fd, EPOLL_CTL_DEL, dev_fd, NULL);

				close(dev_fd);
				device_fd[dev_num] = -1;
			}

		/* Release any filtering data we may have accumulated */
		release_noise_filtering_data(s);

		return 0;
	}

	if (dev_fd == -1) {
		/* First enabled sensor on this iio device */
		sprintf(device_name, DEV_FILE_PATH, dev_num);
		dev_fd = open(device_name, O_RDONLY | O_NONBLOCK);

		device_fd[dev_num] = dev_fd;

		if (dev_fd == -1) {
			ALOGE("Could not open fd on %s (%s)\n",
			      device_name, strerror(errno));
			adjust_counters(s, 0, from_virtual);
			return -1;
		}

		ALOGV("Opened %s: fd=%d\n", device_name, dev_fd);

		if (!is_poll_sensor) {

			/* Add this iio device fd to the set of watched fds */
			ev.events = EPOLLIN;
			ev.data.u32 = dev_num;

			ret = epoll_ctl(poll_fd, EPOLL_CTL_ADD, dev_fd, &ev);

			if (ret == -1) {
				ALOGE(	"Failed adding %d to poll set (%s)\n",
					dev_fd, strerror(errno));
				return -1;
			}

			/* Note: poll-mode fds are not readable */
		}
	}

	/* Ensure that on-change sensors send at least one event after enable */
	sensor[s].prev_val = -1;

	if (is_poll_sensor)
		start_acquisition_thread(s);

	return 0;
}


static void enable_motion_trigger (int dev_num)
{
	/*
	 * In the ideal case, we enumerate two triggers per iio device ; the
	 * default (periodically firing) trigger, and another one (the motion
	 * trigger) that only fires up when motion is detected. This second one
	 * allows for lesser energy consumption, but requires periodic sample
	 * duplication at the HAL level for sensors that Android defines as
	 * continuous. This "duplicate last sample" logic can only be engaged
	 * once we got a first sample for the driver, so we start with the
	 * default trigger when an iio device is first opened, then adjust the
	 * trigger when we got events for all active sensors. Unfortunately in
	 * the general case several sensors can be associated to a given iio
	 * device, they can independently be controlled, and we have to adjust
	 * the trigger in use at the iio device level depending on whether or
	 * not appropriate conditions are met at the sensor level.
	 */

	int s;
	int i;
	int active_sensors = trig_sensors_per_dev[dev_num];
	int candidate[MAX_SENSORS];
	int candidate_count = 0;

	if  (!active_sensors)
		return;

	/* Check that all active sensors are ready to switch */

	for (s=0; s<MAX_SENSORS; s++)
		if (sensor[s].dev_num == dev_num &&
		    is_enabled(s) &&
		    sensor[s].num_channels &&
		    (!sensor[s].motion_trigger_name[0] ||
		     !sensor[s].report_initialized ||
		     is_fast_accelerometer(s) ||
		     (sensor[s].quirks & QUIRK_FORCE_CONTINUOUS))
		    )
			return; /* Nope */

	/* Record which particular sensors need to switch */

	for (s=0; s<MAX_SENSORS; s++)
		if (sensor[s].dev_num == dev_num &&
		    is_enabled(s) &&
		    sensor[s].num_channels &&
		    sensor[s].selected_trigger !=
			sensor[s].motion_trigger_name)
				candidate[candidate_count++] = s;

	if (!candidate_count)
		return;

	/* Now engage the motion trigger for sensors which aren't using it */

	enable_buffer(dev_num, 0);

	for (i=0; i<candidate_count; i++) {
		s = candidate[i];
		setup_trigger(s, sensor[s].motion_trigger_name);
	}

	enable_buffer(dev_num, 1);
}


/*
 *  CTS acceptable thresholds:
 *	EventGapVerification.java: (th <= 1.8)
 *	FrequencyVerification.java: (0.9)*(expected freq) => (th <= 1.1111)
 */
#define THRESHOLD 1.10
#define MAX_DELAY 500000000 /* 500 ms */

void set_report_ts(int s, int64_t ts)
{
	int64_t maxTs, period;

	/*
	*  A bit of a hack to please a bunch of cts tests. They
	*  expect the timestamp to be exacly according to the set-up
	*  frequency but if we're simply getting the timestamp at hal level
	*  this may not be the case. Perhaps we'll get rid of this when
	*  we'll be reading the timestamp from the iio channel for all sensors
	*/
	if (sensor[s].report_ts && sensor[s].sampling_rate &&
		REPORTING_MODE(sensor_desc[s].flags) == SENSOR_FLAG_CONTINUOUS_MODE)
	{
		period = (int64_t) (1000000000.0 / sensor[s].sampling_rate);
		maxTs = sensor[s].report_ts + THRESHOLD * period;
		/* If we're too far behind get back on track */
		if (ts - maxTs >= MAX_DELAY)
			maxTs = ts;
		sensor[s].report_ts = (ts < maxTs ? ts : maxTs);
	} else {
		sensor[s].report_ts = ts;
	}
}


static void stamp_reports (int dev_num, int64_t ts)
{
	int s;

	for (s=0; s<MAX_SENSORS; s++)
			if (sensor[s].dev_num == dev_num &&
				is_enabled(s))
					set_report_ts(s, ts);
}


static int integrate_device_report (int dev_num)
{
	int len;
	int s,c;
	unsigned char buf[MAX_DEVICE_REPORT_SIZE] = { 0 };
	int sr_offset;
	unsigned char *target;
	unsigned char *source;
	int size;
	int64_t ts = 0;
	int ts_offset = 0;	/* Offset of iio timestamp, if provided */
	int64_t boot_to_rt_delta;

	/* There's an incoming report on the specified iio device char dev fd */

	if (dev_num < 0 || dev_num >= MAX_DEVICES) {
		ALOGE("Event reported on unexpected iio device %d\n", dev_num);
		return -1;
	}

	if (device_fd[dev_num] == -1) {
		ALOGE("Ignoring stale report on iio device %d\n", dev_num);
		return -1;
	}

	len = read(device_fd[dev_num], buf, expected_dev_report_size[dev_num]);

	if (len == -1) {
		ALOGE("Could not read report from iio device %d (%s)\n",
		      dev_num, strerror(errno));
		return -1;
	}

	ALOGV("Read %d bytes from iio device %d\n", len, dev_num);

	/* Map device report to sensor reports */

	for (s=0; s<MAX_SENSORS; s++)
		if (sensor[s].dev_num == dev_num &&
		    is_enabled(s)) {

			sr_offset = 0;

			/* Copy data from device to sensor report buffer */
			for (c=0; c<sensor[s].num_channels; c++) {

				target = sensor[s].report_buffer +
					sr_offset;

				source = buf + sensor[s].channel[c].offset;

				size = sensor[s].channel[c].size;

				memcpy(target, source, size);

				sr_offset += size;
			}

			ALOGV("Sensor %d report available (%d bytes)\n", s,
			      sr_offset);

			sensor[s].report_pending = DATA_TRIGGER;
			sensor[s].report_initialized = 1;

			ts_offset += sr_offset;
		}

	/* Tentatively switch to an any-motion trigger if conditions are met */
	enable_motion_trigger(dev_num);

	/* If no iio timestamp channel was detected for this device, bail out */
	if (!has_iio_ts[dev_num]) {
		stamp_reports(dev_num, get_timestamp_boot());
		return 0;
	}

	/* Don't trust the timestamp channel in any-motion mode */
	for (s=0; s<MAX_SENSORS; s++)
		if (sensor[s].dev_num == dev_num &&
		    is_enabled(s) &&
		    sensor[s].selected_trigger ==
					sensor[s].motion_trigger_name) {
		stamp_reports(dev_num, get_timestamp_boot());
		return 0;
	}

	/* Align on a 64 bits boundary */
	ts_offset = (ts_offset + 7)/8*8;

	/* If we read an amount of data consistent with timestamp presence */
	if (len == expected_dev_report_size[dev_num])
		ts = *(int64_t*) (buf + ts_offset);

	if (ts == 0) {
		ALOGV("Unreliable timestamp channel on iio dev %d\n", dev_num);
		stamp_reports(dev_num, get_timestamp_boot());
		return 0;
	}

	ALOGV("Driver timestamp on iio device %d: ts=%lld\n", dev_num, ts);

	boot_to_rt_delta = get_timestamp_boot() - get_timestamp_realtime();

	stamp_reports(dev_num, ts + boot_to_rt_delta);

	return 0;
}


static int propagate_vsensor_report (int s, struct sensors_event_t  *data)
{
	/* There's a new report stored in sensor.sample for this sensor; transmit it */

	memcpy(data, &sensor[s].sample, sizeof(struct sensors_event_t));

	data->sensor	= s;
	data->type	= sensor[s].type;
	return 1;
}


static int propagate_sensor_report (int s, struct sensors_event_t  *data)
{
	/* There's a sensor report pending for this sensor ; transmit it */

	int num_fields	  = get_field_count(s);
	int c;
	unsigned char* current_sample;

	/* If there's nothing to return... we're done */
	if (!num_fields)
		return 0;

	memset(data, 0, sizeof(sensors_event_t));

	data->version	= sizeof(sensors_event_t);
	data->sensor	= s;
	data->type	= sensor[s].type;
	data->timestamp = sensor[s].report_ts;

	ALOGV("Sample on sensor %d (type %d):\n", s, sensor[s].type);

	current_sample = sensor[s].report_buffer;

	/* If this is a poll sensor */
	if (!sensor[s].num_channels) {
		/* Use the data provided by the acquisition thread */
		ALOGV("Reporting data from worker thread for S%d\n", s);
		memcpy(data->data, current_sample, num_fields * sizeof(float));
		return 1;
	}

	/* Convert the data into the expected Android-level format */
	for (c=0; c<num_fields; c++) {

		data->data[c] = sensor[s].ops.transform (s, c, current_sample);

		ALOGV("\tfield %d: %g\n", c, data->data[c]);
		current_sample += sensor[s].channel[c].size;
	}

	/*
	 * The finalize routine, in addition to its late sample processing duty,
	 * has the final say on whether or not the sample gets sent to Android.
	 */
	return sensor[s].ops.finalize(s, data);
}


static void synthetize_duplicate_samples (void)
{
	/*
	 * Some sensor types (ex: gyroscope) are defined as continuously firing
	 * by Android, despite the fact that we can be dealing with iio drivers
	 * that only report events for new samples. For these we generate
	 * reports periodically, duplicating the last data we got from the
	 * driver. This is not necessary for polling sensors.
	 */

	int s;
	int64_t current_ts;
	int64_t target_ts;
	int64_t period;

	for (s=0; s<sensor_count; s++) {

		/* Ignore disabled sensors */
		if (!is_enabled(s))
			continue;

		/* If the sensor is continuously firing, leave it alone */
		if (sensor[s].selected_trigger !=
		    sensor[s].motion_trigger_name)
			continue;

		/* If we haven't seen a sample, there's nothing to duplicate */
		if (!sensor[s].report_initialized)
			continue;

		/* If a sample was recently buffered, leave it alone too */
		if (sensor[s].report_pending)
			continue;

		/* We also need a valid sampling rate to be configured */
		if (!sensor[s].sampling_rate)
			continue;

		period = (int64_t) (1000000000.0 / sensor[s].sampling_rate);

		current_ts = get_timestamp_boot();
		target_ts = sensor[s].report_ts + period;

		if (target_ts <= current_ts) {
			/* Mark the sensor for event generation */
			set_report_ts(s, current_ts);
			sensor[s].report_pending = DATA_DUPLICATE;
		}
	}
}


static void integrate_thread_report (uint32_t tag)
{
	int s = tag - THREAD_REPORT_TAG_BASE;
	int len;
	int expected_len;
	int64_t timestamp;
	unsigned char current_sample[MAX_SENSOR_REPORT_SIZE];

	expected_len = sizeof(int64_t) + get_field_count(s) * sizeof(float);

	len = read(sensor[s].thread_data_fd[0],
		   current_sample,
		   expected_len);

	memcpy(&timestamp, current_sample, sizeof(int64_t));
	memcpy(sensor[s].report_buffer, sizeof(int64_t) + current_sample,
			expected_len - sizeof(int64_t));

	if (len == expected_len) {
		set_report_ts(s, timestamp);
		sensor[s].report_pending = DATA_SYSFS;
	}
}


static int get_poll_wait_timeout (void)
{
	/*
	 * Compute an appropriate timeout value, in ms, for the epoll_wait
	 * call that's going to await for iio device reports and incoming
	 * reports from our sensor sysfs data reader threads.
	 */

	int s;
	int64_t target_ts = INT64_MAX;
	int64_t ms_to_wait;
	int64_t period;

	/*
	 * Check if we're dealing with a driver that only send events when
	 * there is motion, despite the fact that the associated Android sensor
	 * type is continuous rather than on-change. In that case we have to
	 * duplicate events. Check deadline for the nearest upcoming event.
	 */
	for (s=0; s<sensor_count; s++)
		if (is_enabled(s) &&
		    sensor[s].selected_trigger ==
		    sensor[s].motion_trigger_name &&
		    sensor[s].sampling_rate) {
			period = (int64_t) (1000000000.0 /
						sensor[s].sampling_rate);

			if (sensor[s].report_ts + period < target_ts)
				target_ts = sensor[s].report_ts + period;
		}

	/* If we don't have such a driver to deal with */
	if (target_ts == INT64_MAX)
		return -1; /* Infinite wait */

	ms_to_wait = (target_ts - get_timestamp_boot()) / 1000000;

	/* If the target timestamp is already behind us, don't wait */
	if (ms_to_wait < 1)
		return 0;

	return ms_to_wait;
}


int sensor_poll (struct sensors_event_t* data, int count)
{
	int s;
	int i;
	int nfds;
	struct epoll_event ev[MAX_DEVICES];
	int returned_events;
	int event_count;
	int uncal_start;

	/* Get one or more events from our collection of sensors */
return_available_sensor_reports:

	/* Synthetize duplicate samples if needed */
	synthetize_duplicate_samples();

	returned_events = 0;
	/* Check our sensor collection for available reports */
	for (s=0; s<sensor_count && returned_events < count; s++) {
		if (sensor[s].report_pending) {
			event_count = 0;

			if (sensor[s].is_virtual)
				event_count = propagate_vsensor_report(s, &data[returned_events]);
			else {
				/* Report this event if it looks OK */
				event_count = propagate_sensor_report(s, &data[returned_events]);
			}

			/* Lower flag */
			sensor[s].report_pending = 0;
			returned_events += event_count;
			/*
			 * If the sample was deemed invalid or unreportable,
			 * e.g. had the same value as the previously reported
			 * value for a 'on change' sensor, silently drop it.
			 */
		}
		while (sensor[s].meta_data_pending) {
			/* See sensors.h on these */
			data[returned_events].version = META_DATA_VERSION;
			data[returned_events].sensor = 0;
			data[returned_events].type = SENSOR_TYPE_META_DATA;
			data[returned_events].reserved0 = 0;
			data[returned_events].timestamp = 0;
			data[returned_events].meta_data.sensor = s;
			data[returned_events].meta_data.what = META_DATA_FLUSH_COMPLETE;
			returned_events++;
			sensor[s].meta_data_pending--;
		}
	}
	if (returned_events)
		return returned_events;

await_event:

	ALOGV("Awaiting sensor data\n");

	nfds = epoll_wait(poll_fd, ev, MAX_DEVICES, get_poll_wait_timeout());

	if (nfds == -1) {
		ALOGE("epoll_wait returned -1 (%s)\n", strerror(errno));
		goto await_event;
	}

	ALOGV("%d fds signalled\n", nfds);

	/* For each of the signalled sources */
	for (i=0; i<nfds; i++)
		if (ev[i].events == EPOLLIN)
			switch (ev[i].data.u32) {
				case 0 ... MAX_DEVICES-1:
					/* Read report from iio char dev fd */
					integrate_device_report(ev[i].data.u32);
					break;

				case THREAD_REPORT_TAG_BASE ...
				     THREAD_REPORT_TAG_BASE + MAX_SENSORS-1:
					/* Get report from acquisition thread */
					integrate_thread_report(ev[i].data.u32);
					break;

				default:
					ALOGW("Unexpected event source!\n");
					break;
			}

	goto return_available_sensor_reports;
}


int sensor_set_delay (int s, int64_t ns)
{
	float requested_sampling_rate;

	if (ns <= 0) {
		ALOGE("Invalid delay requested on sensor %d: %lld\n", s, ns);
		return -EINVAL;
	}

	requested_sampling_rate = 1000000000.0 / ns;

	ALOGV("Entering set delay S%d (%s): current rate: %g, requested: %g\n",
		s, sensor[s].friendly_name, sensor[s].sampling_rate,
		requested_sampling_rate);

	sensor[s].requested_rate = requested_sampling_rate;

	return arbitrate_delays(s);
}


int sensor_flush (int s)
{
	/* If one shot or not enabled return -EINVAL */
	if (sensor_desc[s].flags & SENSOR_FLAG_ONE_SHOT_MODE || !is_enabled(s))
		return -EINVAL;

	sensor[s].meta_data_pending++;
	return 0;
}


int allocate_control_data (void)
{
	int i;

	for (i=0; i<MAX_DEVICES; i++)
		device_fd[i] = -1;

	poll_fd = epoll_create(MAX_DEVICES);

	if (poll_fd == -1) {
		ALOGE("Can't create epoll instance for iio sensors!\n");
		return -1;
	}

	return poll_fd;
}


void delete_control_data (void)
{
}
