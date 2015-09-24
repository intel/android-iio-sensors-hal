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

#ifndef __COMMON_H__
#define __COMMON_H__

#include <string.h>

#define MAX_DEVICES	9	/* Check iio devices 0 to MAX_DEVICES-1 */
#define MAX_SENSORS	12	/* We can handle as many sensors */
#define MAX_CHANNELS	4	/* We can handle as many channels per sensor */
#define MAX_EVENTS	2	/* We can handle as many events per channel */
#define MAX_TRIGGERS	8	/* Check for triggers 0 to MAX_TRIGGERS-1 */

#define DEV_FILE_PATH		"/dev/iio:device%d"
#define BASE_PATH		"/sys/bus/iio/devices/iio:device%d/"
#define TRIGGER_FILE_PATH	"/sys/bus/iio/devices/trigger%d/name"
#define IIO_DEVICES		"/sys/bus/iio/devices/"

#define CHANNEL_PATH		BASE_PATH "scan_elements/"
#define ENABLE_PATH		BASE_PATH "buffer/enable"
#define BUFFER_LENGTH_PATH	BASE_PATH "buffer/length"
#define NAME_PATH		BASE_PATH "name"
#define TRIGGER_PATH		BASE_PATH "trigger/current_trigger"
#define EVENTS_PATH		BASE_PATH "events/"
#define SENSOR_ENABLE_PATH	BASE_PATH "in_%s_en"
#define SENSOR_OFFSET_PATH	BASE_PATH "in_%s_offset"
#define SENSOR_SCALE_PATH	BASE_PATH "in_%s_scale"
#define SENSOR_SAMPLING_PATH	BASE_PATH "in_%s_sampling_frequency"
#define DEVICE_SAMPLING_PATH	BASE_PATH "sampling_frequency"
#define DEVICE_AVAIL_FREQ_PATH	BASE_PATH "sampling_frequency_available"
#define ILLUMINATION_CALIBPATH	BASE_PATH "in_illuminance_calibscale"
#define SENSOR_CALIB_BIAS_PATH	BASE_PATH "in_%s_calibbias"
#define MOUNTING_MATRIX_PATH	BASE_PATH "mounting_matrix"

#define CONFIGFS_TRIGGER_PATH	"/sys/kernel/config/iio/triggers/"

#define PROP_BASE		"ro.iio.%s.%s" /* Note: PROPERTY_KEY_MAX is small */

#define MAX_TYPE_SPEC_LEN	32	/* Channel type spec len; ex: "le:u10/16>>0" */
#define MAX_SENSOR_REPORT_SIZE	32	/* Sensor report buffer size */
#define MAX_DEVICE_REPORT_SIZE	32	/* iio device scan buffer size */

#define MAX_NAME_SIZE		64

#define MAX_SENSOR_BASES	3	/* Max number of base sensors a sensor can rely on */

#define ARRAY_SIZE(x) sizeof(x)/sizeof(x[0])
#define REPORTING_MODE(x)	((x) & 0x06)

#define FILTER_TYPE_NONE		0
#define FILTER_TYPE_MOVING_AVERAGE	1
#define FILTER_TYPE_MEDIAN		2

#define MODE_AUTO	0 /* autodetect */
#define MODE_POLL	1
#define MODE_TRIGGER	2
#define MODE_EVENT	3


typedef struct
{
	const char *type; /* event type; e.g: transition */
	const char *dir;  /* event direction; e.g: rising */

	/* sysfs entries located in /sys/bus/iio/devices/iio:deviceX/events/ */
	const char *ev_en_path;
	const char *ev_value_path;
}
event_descriptor_t;


typedef struct
{
	const char *name;	/* channel name ; ex: x */

	/* sysfs entries located under scan_elements */
	const char *en_path;	/* Enabled sysfs file name ; ex: "in_temp_en" */
	const char *type_path;	/* _type sysfs file name  */
	const char *index_path;	/* _index sysfs file name */

	/* sysfs entries located in /sys/bus/iio/devices/iio:deviceX */
	const char *raw_path;	/* _raw sysfs file name  */
	const char *input_path;	/* _input sysfs file name */
	const char *scale_path;	/* _scale sysfs file name */

	const int num_events;
	event_descriptor_t event[MAX_EVENTS];
}
channel_descriptor_t;


typedef struct
{
	const char *tag;	/* Prefix such as "accel", "gyro", "temp"... */
	const char *shorthand;
	const int type;		/* Sensor type ; ex: SENSOR_TYPE_ACCELEROMETER */
	const int num_channels;	/* Expected iio channels for this sensor */
	const int is_virtual; 	/* Is the sensor virtual or not */
	channel_descriptor_t channel[MAX_CHANNELS];
}
sensor_catalog_entry_t;


typedef struct
{
	char sign;
	char endianness;
	short realbits;
	short storagebits;
	short shift;
}
datum_info_t;


typedef struct
{
	int offset; 	/* Offset in bytes within the iio character device report */
	int size;	/* Field size in bytes */
	float scale;	/* Scale for each channel */
	char type_spec[MAX_TYPE_SPEC_LEN];	/* From driver; ex: le:u10/16>>0 */
	datum_info_t type_info;	   		/* Decoded contents of type spec */
	float opt_scale; /*
			  * Optional correction scale read from a property such as iio.accel.x.scale, allowing late compensation of
			  * problems such as misconfigured axes ; set to 1 by default. Applied at the end of the scaling process.
			  */
	int raw_path_present;   /* Flag signalling the presence of in_<sens>_<axis>_raw file */
	int input_path_present; /* Flag signalling the presence of in_<sens>_input file */
}
channel_info_t;


typedef struct
{
	/* Conversion function called once per channel */
	float (*transform) (int s, int c, unsigned char* sample_data);

	/* Function called once per sample */
	int (*finalize) (int s, sensors_event_t* data);
}
sample_ops_t;


/*
 * Whenever we have sensor data recorded for a sensor in the associated
 * sensor cell, its report_pending field is set to a non-zero value
 * indicating how we got this data.
 */
#define DATA_TRIGGER	1	/* From /dev/iio:device fd		*/
#define DATA_SYSFS	2	/* Through polling    			*/
#define DATA_DUPLICATE	3	/* Duplicate of triggered motion sample	*/


typedef struct
{
	char friendly_name[MAX_NAME_SIZE];	/* ex: Accelerometer	     */
	char internal_name[MAX_NAME_SIZE];	/* ex: accel_3d	             */
	char vendor_name[MAX_NAME_SIZE];	/* ex: Intel 		     */
	char init_trigger_name[MAX_NAME_SIZE];	/* ex: accel-name-dev1	     */
	char motion_trigger_name[MAX_NAME_SIZE];/* ex: accel-any-motion-dev1 */
	char hrtimer_trigger_name[MAX_NAME_SIZE]; /*ex: accel-hr-dev1 */
	int trigger_nr;	/* trigger number associated with this device */
	float max_range;
	float resolution;
	float power;

	/*
	 * Currently active trigger - either a pointer to the initial (default) trigger name buffer, or a pointer to the motion trigger name buffer,
	 * or something else (typically NULL or a pointer to some static "\n". This is used to determine if the conditions are met to switch from
	 * the default trigger to the motion trigger for a sensor, or rather for the interrupt-driven sensors associated to a given iio device.
	 */
	const char* selected_trigger;

	float offset;		/* (cooked = raw + offset) * scale			*/
	float scale;		/* default:1. when set to 0, use channel specific value */
	float illumincalib;	/* to set the calibration for the ALS			*/

	float requested_rate;   /* requested events / second				*/
	float sampling_rate;	/* setup events / second				*/

	float min_supported_rate;
	float max_supported_rate;

	int dev_num;		/* Associated iio dev num, ex: 3 for /dev/iio:device3	*/

	int catalog_index;	/* Associated entry within the sensor_catalog array	*/
	int type;	  	/* Sensor type, such as SENSOR_TYPE_GYROSCOPE		*/

	int num_channels;	/* Actual channel count ; 0 for poll mode sensors	*/

	int mode;	/* Usage mode, ex: poll, trigger ... */

	/*
	 * The array below indicates where to gather report data for this sensor inside the reports that we read from the iio character device.
	 * It is updated whenever channels are enabled or disabled on the same device. Channel size indicates the size in bytes of fields, and
	 * should be zero for disabled channels. The type field indicates how a specific channel data item is structured.
	 */
	channel_info_t channel[MAX_CHANNELS];

	/*
	 * This flag is set if we acquired data from the sensor but did not forward it to upper layers (i.e. Android) yet. If so, report_buffer
	 * contains that data. Valid values are 0: empty, 1: normal, 2: repeat of last acquired value after timeout.
	 */
	int report_pending;

	/* This flag is set if we have a meta data event pending */
	int meta_data_pending;

	/*
	 * Timestamp closely matching the date of sampling, preferably retrieved from a iio channel alongside sample data. Value zero indicates that
	 * we couldn't get such a closely correlated timestamp, and that one has to be generated before the report gets sent up to Android.
	 */
	int64_t report_ts;

	/* Buffer containing the last generated sensor report for this sensor */
	unsigned char report_buffer[MAX_SENSOR_REPORT_SIZE];

	/* Whether or not the above buffer contains data from a device report */
	int report_initialized;

	/* Channel and sample finalization callbacks for this sensor */
	sample_ops_t ops;

	int cal_level; /* 0 means not calibrated */

	/*
	 * Depending on the sensor, calibration may take too much time at higher levels. Allow optional capping to a certain level.
	 */
	int max_cal_level;

	void *cal_data;	/* Sensor calibration data, e.g. for magnetometer */

	void* filter;	/* Filtering data for noisy sensors */
	int filter_type;/* FILTER_ specification for this sensor ; default is FILTER_NONE */

	/* Previously reported value, for on-change sensors */
	union {
		float data;
		uint64_t data64;
	} prev_val;
	/*
	 * Certain sensors expose their readings through sysfs files that have a long response time (100-200 ms for ALS). Rather than block our
	 * global control loop for several hundred ms each second, offload those lengthy blocking reads to dedicated threads, which will then report
	 * their data through a fd that we can add to our poll fd set.
	 */
	int thread_data_fd[2];
	pthread_t acquisition_thread;

	int base_count;	/* How many base sensors is the sensor depending on */
	int base[MAX_SENSOR_BASES];

	uint32_t quirks; /* Bit mask expressing the need for special tweaks */

	/* Note: we may have to explicitely serialize access to some fields */

	int is_virtual;			/* Composite sensor, exposed from data acquired through other sensors */

	uint32_t ref_count;		/* Dependency count - for a real sensor how many active virtual sensors are depending on it */

	uint32_t directly_enabled;	/* Flag showing if a sensor was enabled directly by Android */

	/*
	 * Current sample for a virtual sensor - when a report is ready we'll keep the data here until it's finally processed. Can be modified for
	 * more than one at a later time.
	 */
	sensors_event_t sample;
	uint64_t event_id;

	/*
	 * If the QUIRK_FIELD_ORDERING bit is set in quirks, the contents of this array are used in the finalization stage to swap sample fields
	 * before transmitting them to Android ; they form a mapping between the indices of the input and output arrays: ex: 0123 is identity for
	 * a sample containing 4 fields.
	 */
	unsigned char order[MAX_CHANNELS];

	/*
	 * If the QUIRK_MOUNTING_MATRIX bit is set in quirks, the contents of this matrix is used to correct the sample values so that it takes
	 * into account the way the sensor has been mounted on the PCB.
	 */
	float mounting_matrix[9];

	/** Count of available frequencies */
	int avail_freqs_count;

	/** Array of available frequencies */
	float* avail_freqs;

	/*
	 * Event counter - will be used to check if we have a significant sample for noisy sensors. We want to make sure we do not send any wrong
	 * events before filtering kicks in. We can also use it for statistics.
	 */
	uint64_t event_count;

	/* Some polled sensors need to be first enabled so that they start
	 * computing a set of values in hardware (e.g step counter). Enabling
	 * is done through a sysfs attribute in_<tag>_en
	 */
	int needs_enable;

	float semi_arbitrated_rate;	/* Arbitrated sampling rate before we considered other sensors co-located on the same iio device */
}
sensor_info_t;


/* Reference a few commonly used variables... */
extern int			sensor_count;
extern struct sensor_t		sensor_desc[MAX_SENSORS];
extern sensor_info_t		sensor[MAX_SENSORS];
extern sensor_catalog_entry_t	sensor_catalog[];
extern unsigned int		catalog_size;

/* Needed both in sensors and activity HALs */
void check_trig_sensors (int i, char *sysfs_file, char map[catalog_size]);
void check_poll_sensors (int i, char *sysfs_file, char map[catalog_size]);
void check_event_sensors (int i, char *sysfs_file, char map[catalog_size]);
void discover_sensors(int dev_num, char *sysfs_base_path, char map[catalog_size],
		      void (*discover_sensor)(int, char*, char*));

/*
 * Macros associating iio sysfs entries to to sensor types ; see
 * linux/kernel/drivers/iio/industrialio-core.c and
 * hardware/libhardware/include/hardware/sensor.h
 */

#define DECLARE_VOID_CHANNEL(tag)	\
			tag,	\
			"",	\
			"",	\
			"",	\
			"",	\
			"",	\
			"",	\

#define DECLARE_CHANNEL(tag, spacer, name)		\
			name,				\
			"in_"tag spacer name"_en",	\
			"in_"tag spacer name"_type",	\
			"in_"tag spacer name"_index",	\
			"in_"tag spacer name"_raw",	\
			"in_"tag spacer name"_input",	\
			"in_"tag spacer name"_scale",	\
			0, {{0}},

#define DECLARE_NAMED_CHANNEL(tag, name)	DECLARE_CHANNEL(tag, "_", name)

#define DECLARE_GENERIC_CHANNEL(tag)		DECLARE_CHANNEL(tag, "", "")

#define DECLARE_EVENT(tag, spacer1, name, spacer2, type, spacer3, dir)		\
		      type, dir,						\
		      "in_"tag spacer1 name spacer2 type spacer3 dir"_en",	\
		      "in_"tag spacer1 name spacer2 type spacer3 dir"_value",	\

#define DECLARE_GENERIC_EVENT(tag, name, type, dir) \
		DECLARE_EVENT(tag, "_", name, "_", type, "_", dir)
#define DECLARE_NAMED_EVENT(tag, name) \
		DECLARE_EVENT(tag, "_", name, "","","","")

#endif
