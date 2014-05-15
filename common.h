/*
 * Copyright (C) 2014 Intel Corporation.
 */

#ifndef __COMMON_H__
#define __COMMON_H__

#define MAX_DEVICES	8	/* Check iio devices 0 to MAX_DEVICES-1 */
#define MAX_SENSORS	10	/* We can handle as many sensors */
#define MAX_CHANNELS	4	/* We can handle as many channels per sensor */

#define DEV_FILE_PATH	"/dev/iio:device%d"

#define BASE_PATH	"/sys/bus/iio/devices/iio:device%d/"

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

#define MAX_TYPE_SPEC_LEN 32	/* Channel type spec len; ex: "le:u10/16>>0" */
#define MAX_SENSOR_REPORT_SIZE	32	/* Sensor report buffer size */

#define MAX_NAME_SIZE		32

#define ARRAY_SIZE(x) sizeof(x)/sizeof(x[0])

struct channel_descriptor_t
{
	/* sysfs entries located under scan_elements */
	const char *en_path;	/* Enabled sysfs file name ; ex: "in_temp_en" */
	const char *type_path;	/* _type sysfs file name  */
	const char *index_path;	/* _index sysfs file name */

	/* sysfs entries located in /sys/bus/iio/devices/iio:deviceX */
	const char *raw_path;	/* _raw sysfs file name  */
	const char *input_path;	/* _input sysfs file name */
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
	char type_spec[MAX_TYPE_SPEC_LEN]; /* From driver; ex: le:u10/16>>0 */
	struct datum_info_t type_info;	   /* Decoded contents of type spec */
};

struct sample_ops_t
{
	/* Conversion function called once per channel */
	float (*transform)(int s, int c, unsigned char* sample_data);

	/* Function called once per sample */
	void (*finalize)(int s, struct sensors_event_t* data);
};

struct sensor_info_t
{
	char friendly_name[MAX_NAME_SIZE];	/* ex: Accelerometer */
	char internal_name[MAX_NAME_SIZE];	/* ex: accel_3d */
	char vendor_name[MAX_NAME_SIZE];	/* ex: Intel */

	float max_range;
	float resolution;
	float power;

	float offset;	/* (cooked = raw + offset) * scale */
	float scale;
	float illumincalib;	/* to set the calibration for the ALS */

	int sampling_rate;	/* requested events / second */

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

	unsigned char report_buffer[MAX_SENSOR_REPORT_SIZE];

	int64_t last_integration_ts; /* Last time an event was reported */

	struct sample_ops_t ops;

	/* Note: we may have to explicitely serialize access to some fields */
};

/* Reference a few commonly used variables... */
extern int				sensor_count;
extern struct sensor_info_t		sensor_info[MAX_SENSORS];
extern struct sensor_catalog_entry_t	sensor_catalog[];

#endif
