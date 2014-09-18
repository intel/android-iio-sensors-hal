/*
 * Copyright (C) 2014 Intel Corporation.
 */

#ifndef __COMMON_H__
#define __COMMON_H__

#define MAX_DEVICES	8	/* Check iio devices 0 to MAX_DEVICES-1 */
#define MAX_SENSORS	10	/* We can handle as many sensors */
#define MAX_CHANNELS	4	/* We can handle as many channels per sensor */
#define MAX_TRIGGERS	8	/* Check for triggers 0 to MAX_TRIGGERS-1 */

#define DEV_FILE_PATH		"/dev/iio:device%d"
#define BASE_PATH		"/sys/bus/iio/devices/iio:device%d/"
#define TRIGGER_FILE_PATH	"/sys/bus/iio/devices/trigger%d/name"

#define CHANNEL_PATH		BASE_PATH "scan_elements/"
#define ENABLE_PATH		BASE_PATH "buffer/enable"
#define NAME_PATH		BASE_PATH "name"
#define TRIGGER_PATH		BASE_PATH "trigger/current_trigger"
#define SENSOR_OFFSET_PATH	BASE_PATH "in_%s_offset"
#define SENSOR_SCALE_PATH	BASE_PATH "in_%s_scale"
#define SENSOR_SAMPLING_PATH	BASE_PATH "in_%s_sampling_frequency"
#define DEVICE_SAMPLING_PATH	BASE_PATH "sampling_frequency"
#define DEVICE_AVAIL_FREQ_PATH	BASE_PATH "sampling_frequency_available"
#define ILLUMINATION_CALIBPATH	BASE_PATH "in_illuminance_calibscale"

#define PROP_BASE	"ro.iio.%s.%s" /* Note: PROPERTY_KEY_MAX is small */

#define MAX_EVENTS 800		/* 800 hz */
#define MAX_TYPE_SPEC_LEN 32	/* Channel type spec len; ex: "le:u10/16>>0" */
#define MAX_SENSOR_REPORT_SIZE	32	/* Sensor report buffer size */

#define MAX_NAME_SIZE		32

#define ARRAY_SIZE(x) sizeof(x)/sizeof(x[0])

#ifdef __LP64__
	typedef uint64_t flag_t;
	typedef int64_t max_delay_t;
#else
	typedef uint32_t flag_t;
	typedef int32_t max_delay_t;
#endif

struct channel_descriptor_t
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
};

struct sensor_catalog_entry_t
{
	const char *tag; /* Prefix such as "accel", "gyro", "temp"... */
	const int type;	 /* Sensor type ; ex: SENSOR_TYPE_ACCELEROMETER */
	const int num_channels;	/* Expected iio channels for this sensor */
	struct channel_descriptor_t channel[MAX_CHANNELS];
};

struct datum_info_t
{
	char sign;
	char endianness;
	short realbits;
	short storagebits;
	short shift;
};

struct channel_info_t
{
	int offset; /* Offset in bytes within the iio character device report */
	int size;			/* Field size in bytes */
	float scale;			/* scale for each channel */
	char type_spec[MAX_TYPE_SPEC_LEN]; /* From driver; ex: le:u10/16>>0 */
	struct datum_info_t type_info;	   /* Decoded contents of type spec */
	float opt_scale; /* Optional correction scale read from a property such
			  * as iio.accel.x.scale, allowing late compensation of
			  * problems such as misconfigured axes ; set to 1 by
			  * default. Applied at the end of the scaling process.
			  */
};

struct sample_ops_t
{
	/* Conversion function called once per channel */
	float (*transform)(int s, int c, unsigned char* sample_data);

	/* Function called once per sample */
	int (*finalize)(int s, struct sensors_event_t* data);
};

struct sensor_info_t
{
	char friendly_name[MAX_NAME_SIZE];	/* ex: Accelerometer	     */
	char internal_name[MAX_NAME_SIZE];	/* ex: accel_3d	             */
	char vendor_name[MAX_NAME_SIZE];	/* ex: Intel 		     */
	char init_trigger_name[MAX_NAME_SIZE];	/* ex: accel-name-dev1	     */
	char motion_trigger_name[MAX_NAME_SIZE];/* ex: accel-any-motion-dev1 */
	float max_range;
	float resolution;
	float power;

	/*
	 * Currently active trigger - either a pointer to the initial (default)
	 * trigger name buffer, or a pointer to the motion trigger name buffer,
	 * or something else (typically NULL or a pointer to some static "\n".
	 * This is used to determine if the conditions are met to switch from
	 * the default trigger to the motion trigger for a sensor, or rather for
	 * the interrupt-driven sensors associated to a given iio device.
	 */
	const char* selected_trigger;

	float offset;	/* (cooked = raw + offset) * scale */
	float scale;    /*default: 1. when set to 0, use channel specific value*/
	float illumincalib;	/* to set the calibration for the ALS */

	float sampling_rate;	/* requested events / second */

	int dev_num;	/* Associated iio dev num, ex: 3 for /dev/iio:device3 */
	int enable_count;

	int catalog_index;/* Associated entry within the sensor_catalog array */

	int num_channels; /* Actual channel count ; 0 for poll mode sensors */

	/*
	 * The array below indicates where to gather report data for this
	 * sensor inside the reports that we read from the iio character device.
	 * It is updated whenever channels are enabled or disabled on the same
	 * device. Channel size indicates the size in bytes of fields, and
	 * should be zero for disabled channels. The type field indicates how a
	 * specific channel data item is structured.
	 */
	struct channel_info_t channel[MAX_CHANNELS];

	/*
	 * This flag is set if we acquired data from the sensor but did not
	 * forward it to upper layers (i.e. Android) yet. If so, report_buffer
	 * contains that data.
	 */
	int report_pending;

	/*
	 * Timestamp closely matching the date of sampling, preferably retrieved
	 * from a iio channel alongside sample data. Value zero indicates that
	 * we couldn't get such a closely correlated timestamp, and that one
	 * has to be generated before the report gets sent up to Android.
	 */
	int64_t report_ts;

	/* Buffer containing the last generated sensor report for this sensor */
	unsigned char report_buffer[MAX_SENSOR_REPORT_SIZE];

	/* Whether or not the above buffer contains data from a device report */
	int report_initialized;

	struct sample_ops_t ops;

	int cal_level; /* 0 means not calibrated */
	void* cal_data;

	float prev_val; /* Previously reported value, for on-change sensors */

	/*
	 * Certain sensors expose their readings through sysfs files that have
	 * a long response time (100-200 ms for ALS). Rather than block our
	 * global control loop for several hundred ms each second, offload those
	 * lengthy blocking reads to dedicated threads, which will then report
	 * their data through a fd that we can add to our poll fd set.
	 */
	int thread_data_fd[2];
	pthread_t acquisition_thread;

	/* For cal-uncal sensor pairs - index to the pair sensor in sensor_info */
	int pair_idx;

	uint32_t quirks; /* Bit mask expressing the need for special tweaks */

	/* Note: we may have to explicitely serialize access to some fields */


	/*
	 * If the QUIRK_FIELD_ORDERING bit is set in quirks, the contents of
	 * this array are used in the finalization stage to swap sample fields
	 * before transmitting them to Android ; they form a mapping between
	 * the indices of the input and output arrays: ex: 0123 is identity for
	 * a sample containing 4 fields.
	 */
	unsigned char order[MAX_CHANNELS];

	/* A few variables used for data filtering */
	float *history;		/* Working buffer containing recorded samples */
	int history_size;	/* Number of recorded samples		      */
	int history_entries;	/* How many of these are initialized	      */
	int history_index;	/* Index of sample to evict next time	      */
};

/* Reference a few commonly used variables... */
extern int				sensor_count;
extern struct sensor_t      sensor_desc[MAX_SENSORS];
extern struct sensor_info_t		sensor_info[MAX_SENSORS];
extern struct sensor_catalog_entry_t	sensor_catalog[];

#endif
