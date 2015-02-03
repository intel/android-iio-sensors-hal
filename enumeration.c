/*
 * Copyright (C) 2014 Intel Corporation.
 */

#include <ctype.h>
#include <dirent.h>
#include <stdlib.h>
#include <fcntl.h>
#include <utils/Log.h>
#include <hardware/sensors.h>
#include "enumeration.h"
#include "description.h"
#include "utils.h"
#include "transform.h"
#include "description.h"
#include "control.h"
#include "calibration.h"

/*
 * This table maps syfs entries in scan_elements directories to sensor types,
 * and will also be used to determine other sysfs names as well as the iio
 * device number associated to a specific sensor.
 */

 /*
  * We duplicate entries for the uncalibrated types after their respective base
  * sensor. This is because all sensor entries must have an associated catalog entry
  * and also because when only the uncal sensor is active it needs to take it's data
  * from the same iio device as the base one.
  */

sensor_catalog_entry_t sensor_catalog[] = {
	{
		.tag		= "accel",
		.type		= SENSOR_TYPE_ACCELEROMETER,
		.num_channels	= 3,
		.is_virtual	= 0,
		.channel = {
			{ DECLARE_NAMED_CHANNEL("accel", "x") },
			{ DECLARE_NAMED_CHANNEL("accel", "y") },
			{ DECLARE_NAMED_CHANNEL("accel", "z") },
		},
	},
	{
		.tag		= "anglvel",
		.type		= SENSOR_TYPE_GYROSCOPE,
		.num_channels	= 3,
		.is_virtual	= 0,
		.channel = {
			{ DECLARE_NAMED_CHANNEL("anglvel", "x") },
			{ DECLARE_NAMED_CHANNEL("anglvel", "y") },
			{ DECLARE_NAMED_CHANNEL("anglvel", "z") },
		},
	},
	{
		.tag		= "magn",
		.type		= SENSOR_TYPE_MAGNETIC_FIELD,
		.num_channels	= 3,
		.is_virtual	= 0,
		.channel = {
			{ DECLARE_NAMED_CHANNEL("magn", "x") },
			{ DECLARE_NAMED_CHANNEL("magn", "y") },
			{ DECLARE_NAMED_CHANNEL("magn", "z") },
		},
	},
	{
		.tag		= "intensity",
		.type		= SENSOR_TYPE_LIGHT,
		.num_channels	= 1,
		.is_virtual	= 0,
		.channel = {
			{ DECLARE_NAMED_CHANNEL("intensity", "both") },
		},
	},
	{
		.tag		= "illuminance",
		.type		= SENSOR_TYPE_LIGHT,
		.num_channels	= 1,
		.is_virtual	= 0,
		.channel = {
			{ DECLARE_GENERIC_CHANNEL("illuminance") },
		},
	},
	{
		.tag		= "incli",
		.type		= SENSOR_TYPE_ORIENTATION,
		.num_channels	= 3,
		.is_virtual	= 0,
		.channel = {
			{ DECLARE_NAMED_CHANNEL("incli", "x") },
			{ DECLARE_NAMED_CHANNEL("incli", "y") },
			{ DECLARE_NAMED_CHANNEL("incli", "z") },
		},
	},
	{
		.tag		= "rot",
		.type		= SENSOR_TYPE_ROTATION_VECTOR,
		.num_channels	= 4,
		.is_virtual	= 0,
		.channel = {
			{ DECLARE_NAMED_CHANNEL("rot", "quat_x") },
			{ DECLARE_NAMED_CHANNEL("rot", "quat_y") },
			{ DECLARE_NAMED_CHANNEL("rot", "quat_z") },
			{ DECLARE_NAMED_CHANNEL("rot", "quat_w") },
		},
	},
	{
		.tag		= "temp",
		.type		= SENSOR_TYPE_AMBIENT_TEMPERATURE,
		.num_channels	= 1,
		.is_virtual	= 0,
		.channel = {
			{ DECLARE_GENERIC_CHANNEL("temp") },
		},
	},
	{
		.tag		= "proximity",
		.type		= SENSOR_TYPE_PROXIMITY,
		.num_channels	= 1,
		.is_virtual	= 0,
		.channel = {
			{ DECLARE_GENERIC_CHANNEL("proximity") },
		},
	},
	{
		.tag		= "",
		.type		= SENSOR_TYPE_GYROSCOPE_UNCALIBRATED,
		.num_channels	= 0,
		.is_virtual	= 1,
		.channel = {
			{ DECLARE_GENERIC_CHANNEL("") },
		},

	},
	{
		.tag		= "",
		.type		= SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED,
		.num_channels	= 0,
		.is_virtual	= 1,
		.channel = {
			{ DECLARE_GENERIC_CHANNEL("") },
		},
	},
	{
		.tag		= "steps",
		.type		= SENSOR_TYPE_STEP_COUNTER,
		.num_channels	= 1,
		.is_virtual	= 0,
		.channel = {
			{ DECLARE_GENERIC_CHANNEL("steps") },
		},
	},
	{
		.tag		= "steps",
		.type		= SENSOR_TYPE_STEP_DETECTOR,
		.num_channels	= 1,
		.is_virtual	= 0,
		.channel = {
			{
				DECLARE_VOID_CHANNEL("steps")
				.num_events = 1,
				.event = {
					{ DECLARE_NAMED_EVENT("steps", "change") },
				},
			},
		},
	},
};

#define CATALOG_SIZE	ARRAY_SIZE(sensor_catalog)

/* ACPI PLD (physical location of device) definitions, as used with sensors */

#define PANEL_FRONT	4
#define PANEL_BACK	5

/* We equate sensor handles to indices in these tables */

struct sensor_t	sensor_desc[MAX_SENSORS];	/* Android-level descriptors */
sensor_info_t	sensor[MAX_SENSORS];		/* Internal descriptors      */
int		sensor_count;			/* Detected sensors 	     */


/* if the sensor has an _en attribute, we need to enable it */
int get_needs_enable(int dev_num, const char *tag)
{
	char sysfs_path[PATH_MAX];
	int fd;

	sprintf(sysfs_path, SENSOR_ENABLE_PATH, dev_num, tag);

	fd = open(sysfs_path, O_RDWR);
	if (fd == -1)
		return 0;

	close(fd);
	return 1;
}

static void setup_properties_from_pld (int s, int panel, int rotation,
				       int num_channels)
{
	/*
	 * Generate suitable order and opt_scale directives from the PLD panel
	 * and rotation codes we got. This can later be superseded by the usual
	 * properties if necessary. Eventually we'll need to replace these
	 * mechanisms by a less convoluted one, such as a 3x3 placement matrix.
	 */

	int x = 1;
	int y = 1;
	int z = 1;
	int xy_swap = 0;
	int angle = rotation * 45;

	/* Only deal with 3 axis chips for now */
	if (num_channels < 3)
		return;

	if (panel == PANEL_BACK) {
		/* Chip placed on the back panel ; negate x and z */
		x = -x;
		z = -z;
	}

	switch (angle) {
		case 90: /* 90° clockwise: negate y then swap x,y */
			xy_swap = 1;
			y = -y;
			break;

		case 180: /* Upside down: negate x and y */
			x = -x;
			y = -y;
			break;

		case 270: /* 90° counter clockwise: negate x then swap x,y */
			x = -x;
			xy_swap = 1;
			break;
	}

	if (xy_swap) {
		sensor[s].order[0] = 1;
		sensor[s].order[1] = 0;
		sensor[s].order[2] = 2;
		sensor[s].quirks |= QUIRK_FIELD_ORDERING;
	}

	sensor[s].channel[0].opt_scale = x;
	sensor[s].channel[1].opt_scale = y;
	sensor[s].channel[2].opt_scale = z;
}


static int is_valid_pld (int panel, int rotation)
{
	if (panel != PANEL_FRONT && panel != PANEL_BACK) {
		ALOGW("Unhandled PLD panel spec: %d\n", panel);
		return 0;
	}

	/* Only deal with 90° rotations for now */
	if (rotation < 0 || rotation > 7 || (rotation & 1)) {
		ALOGW("Unhandled PLD rotation spec: %d\n", rotation);
		return 0;
	}

	return 1;
}


static int read_pld_from_properties (int s, int* panel, int* rotation)
{
	int p, r;

	if (sensor_get_prop(s, "panel", &p))
		return -1;

	if (sensor_get_prop(s, "rotation", &r))
		return -1;

	if (!is_valid_pld(p, r))
		return -1;

	*panel = p;
	*rotation = r;

	ALOGI("S%d PLD from properties: panel=%d, rotation=%d\n", s, p, r);

	return 0;
}


static int read_pld_from_sysfs (int s, int dev_num, int* panel, int* rotation)
{
	char sysfs_path[PATH_MAX];
	int p,r;

	sprintf(sysfs_path, BASE_PATH "../firmware_node/pld/panel", dev_num);

	if (sysfs_read_int(sysfs_path, &p))
		return -1;

	sprintf(sysfs_path, BASE_PATH "../firmware_node/pld/rotation", dev_num);

	if (sysfs_read_int(sysfs_path, &r))
		return -1;

	if (!is_valid_pld(p, r))
		return -1;

	*panel = p;
	*rotation = r;

	ALOGI("S%d PLD from sysfs: panel=%d, rotation=%d\n", s, p, r);

	return 0;
}


static void decode_placement_information (int dev_num, int num_channels, int s)
{
	/*
	 * See if we have optional "physical location of device" ACPI tags.
	 * We're only interested in panel and rotation specifiers. Use the
	 * .panel and .rotation properties in priority, and the actual ACPI
	 * values as a second source.
	 */

	int panel;
	int rotation;

	if (read_pld_from_properties(s, &panel, &rotation) &&
		read_pld_from_sysfs(s, dev_num, &panel, &rotation))
			return; /* No PLD data available */

	/* Map that to field ordering and scaling mechanisms */
	setup_properties_from_pld(s, panel, rotation, num_channels);
}


static void populate_descriptors (int s, int sensor_type)
{
	int32_t		min_delay_us;
	max_delay_t	max_delay_us;

	/* Initialize Android-visible descriptor */
	sensor_desc[s].name		= sensor_get_name(s);
	sensor_desc[s].vendor		= sensor_get_vendor(s);
	sensor_desc[s].version		= sensor_get_version(s);
	sensor_desc[s].handle		= s;
	sensor_desc[s].type		= sensor_type;

	sensor_desc[s].maxRange		= sensor_get_max_range(s);
	sensor_desc[s].resolution	= sensor_get_resolution(s);
	sensor_desc[s].power		= sensor_get_power(s);
	sensor_desc[s].stringType	= sensor_get_string_type(s);

	/* None of our supported sensors requires a special permission */
	sensor_desc[s].requiredPermission = "";

	sensor_desc[s].flags = sensor_get_flags(s);
	sensor_desc[s].minDelay = sensor_get_min_delay(s);
	sensor_desc[s].maxDelay = sensor_get_max_delay(s);

	ALOGV("Sensor %d (%s) type(%d) minD(%d) maxD(%d) flags(%2.2x)\n",
		s, sensor[s].friendly_name, sensor_desc[s].type,
		sensor_desc[s].minDelay, sensor_desc[s].maxDelay,
		sensor_desc[s].flags);

	/* We currently do not implement batching */
	sensor_desc[s].fifoReservedEventCount = 0;
	sensor_desc[s].fifoMaxEventCount = 0;

	min_delay_us = sensor_desc[s].minDelay;
	max_delay_us = sensor_desc[s].maxDelay;

	sensor[s].min_supported_rate = max_delay_us ? 1000000.0 / max_delay_us : 1;
	sensor[s].max_supported_rate = min_delay_us && min_delay_us != -1 ? 1000000.0 / min_delay_us : 0;
}


static void add_virtual_sensor (int catalog_index)
{
	int s;
	int sensor_type;

	if (sensor_count == MAX_SENSORS) {
		ALOGE("Too many sensors!\n");
		return;
	}

	sensor_type = sensor_catalog[catalog_index].type;

	s = sensor_count;

	sensor[s].is_virtual = 1;
	sensor[s].catalog_index	= catalog_index;
	sensor[s].type		= sensor_type;

	populate_descriptors(s, sensor_type);

	/* Initialize fields related to sysfs reads offloading */
	sensor[s].thread_data_fd[0]  = -1;
	sensor[s].thread_data_fd[1]  = -1;
	sensor[s].acquisition_thread = -1;

	sensor_count++;
}


static void add_sensor (int dev_num, int catalog_index, int mode)
{
	int s;
	int sensor_type;
	int retval;
	char sysfs_path[PATH_MAX];
	const char* prefix;
        float scale;
	int c;
	float opt_scale;
	const char* ch_name;
	int num_channels;
	char suffix[MAX_NAME_SIZE + 8];
	int calib_bias;

	if (sensor_count == MAX_SENSORS) {
		ALOGE("Too many sensors!\n");
		return;
	}

	sensor_type = sensor_catalog[catalog_index].type;

	/*
	 * At this point we could check that the expected sysfs attributes are
	 * present ; that would enable having multiple catalog entries with the
	 * same sensor type, accomodating different sets of sysfs attributes.
	 */

	s = sensor_count;

	sensor[s].dev_num	= dev_num;
	sensor[s].catalog_index	= catalog_index;
	sensor[s].type		= sensor_type;
	sensor[s].mode		= mode;

        num_channels = sensor_catalog[catalog_index].num_channels;

        if (mode == MODE_POLL)
                sensor[s].num_channels = 0;
        else
                sensor[s].num_channels = num_channels;

	prefix = sensor_catalog[catalog_index].tag;

	/*
	 * receiving the illumination sensor calibration inputs from
	 * the Android properties and setting it within sysfs
	 */
	if (sensor_type == SENSOR_TYPE_LIGHT) {
		retval = sensor_get_illumincalib(s);
                if (retval > 0) {
			sprintf(sysfs_path, ILLUMINATION_CALIBPATH, dev_num);
			sysfs_write_int(sysfs_path, retval);
                }
	}

        /*
         * See if we have optional calibration biases for each of the channels of this sensor. These would be expressed using properties like
         * iio.accel.y.calib_bias = -1, or possibly something like iio.temp.calib_bias if the sensor has a single channel. This value gets stored in the
         * relevant calibbias sysfs file if that file can be located and then used internally by the iio sensor driver.
         */

        if (num_channels) {
		for (c = 0; c < num_channels; c++) {
			ch_name = sensor_catalog[catalog_index].channel[c].name;
			sprintf(suffix, "%s.calib_bias", ch_name);
			if (!sensor_get_prop(s, suffix, &calib_bias) && calib_bias) {
				sprintf(suffix, "%s_%s", prefix, sensor_catalog[catalog_index].channel[c].name);
				sprintf(sysfs_path, SENSOR_CALIB_BIAS_PATH, dev_num, suffix);
				sysfs_write_int(sysfs_path, calib_bias);
			}
		}
        } else
		if (!sensor_get_prop(s, "calib_bias", &calib_bias) && calib_bias) {
				sprintf(sysfs_path, SENSOR_CALIB_BIAS_PATH, dev_num, prefix);
				sysfs_write_int(sysfs_path, calib_bias);
			}

	/* Read name attribute, if available */
	sprintf(sysfs_path, NAME_PATH, dev_num);
	sysfs_read_str(sysfs_path, sensor[s].internal_name, MAX_NAME_SIZE);

	/* See if we have general offsets and scale values for this sensor */

	sprintf(sysfs_path, SENSOR_OFFSET_PATH, dev_num, prefix);
	sysfs_read_float(sysfs_path, &sensor[s].offset);

	sprintf(sysfs_path, SENSOR_SCALE_PATH, dev_num, prefix);
	if (!sensor_get_fl_prop(s, "scale", &scale)) {
		/*
		 * There is a chip preferred scale specified,
		 * so try to store it in sensor's scale file
		 */
		if (sysfs_write_float(sysfs_path, scale) == -1 && errno == ENOENT) {
			ALOGE("Failed to store scale[%g] into %s - file is missing", scale, sysfs_path);
			/* Store failed, try to store the scale into channel specific file */
	                for (c = 0; c < num_channels; c++)
		        {
			        sprintf(sysfs_path, BASE_PATH "%s", dev_num,
					sensor_catalog[catalog_index].channel[c].scale_path);
				if (sysfs_write_float(sysfs_path, scale) == -1)
					ALOGE("Failed to store scale[%g] into %s", scale, sysfs_path);
                        }
                }
	}

	sprintf(sysfs_path, SENSOR_SCALE_PATH, dev_num, prefix);
	if (!sysfs_read_float(sysfs_path, &scale)) {
                sensor[s].scale = scale;
		ALOGV("Scale path:%s scale:%g dev_num:%d\n",
                                        sysfs_path, scale, dev_num);
	} else {
                sensor[s].scale = 1;

                /* Read channel specific scale if any*/
                for (c = 0; c < num_channels; c++)
                {
                        sprintf(sysfs_path, BASE_PATH "%s", dev_num,
                           sensor_catalog[catalog_index].channel[c].scale_path);

                        if (!sysfs_read_float(sysfs_path, &scale)) {
                                sensor[s].channel[c].scale = scale;
			        sensor[s].scale = 0;

			        ALOGV(  "Scale path:%s "
					"channel scale:%g dev_num:%d\n",
                                        sysfs_path, scale, dev_num);
                        }
                }
        }

        /* Set default scaling - if num_channels is zero, we have one channel */

	sensor[s].channel[0].opt_scale = 1;

	for (c = 1; c < num_channels; c++)
		sensor[s].channel[c].opt_scale = 1;

	/* Read ACPI _PLD attributes for this sensor, if there are any */
	decode_placement_information(dev_num, num_channels, s);

        /*
         * See if we have optional correction scaling factors for each of the
         * channels of this sensor. These would be expressed using properties
         * like iio.accel.y.opt_scale = -1. In case of a single channel we also
         * support things such as iio.temp.opt_scale = -1. Note that this works
         * for all types of sensors, and whatever transform is selected, on top
         * of any previous conversions.
         */

        if (num_channels) {
		for (c = 0; c < num_channels; c++) {
			ch_name = sensor_catalog[catalog_index].channel[c].name;
			sprintf(suffix, "%s.opt_scale", ch_name);
			if (!sensor_get_fl_prop(s, suffix, &opt_scale))
				sensor[s].channel[c].opt_scale = opt_scale;
		}
        } else
		if (!sensor_get_fl_prop(s, "opt_scale", &opt_scale))
			sensor[s].channel[0].opt_scale = opt_scale;

	populate_descriptors(s, sensor_type);

	/* Populate the quirks array */
	sensor_get_quirks(s);

	if (sensor[s].internal_name[0] == '\0') {
		/*
		 * In case the kernel-mode driver doesn't expose a name for
		 * the iio device, use (null)-dev%d as the trigger name...
		 * This can be considered a kernel-mode iio driver bug.
		 */
		ALOGW("Using null trigger on sensor %d (dev %d)\n", s, dev_num);
		strcpy(sensor[s].internal_name, "(null)");
	}

	switch (sensor_type) {
		case SENSOR_TYPE_GYROSCOPE:
			sensor[s].cal_data = malloc(sizeof(gyro_cal_t));
			break;

		case SENSOR_TYPE_MAGNETIC_FIELD:
			sensor[s].cal_data = malloc(sizeof(compass_cal_t));
			break;
	}

	sensor[s].max_cal_level = sensor_get_cal_steps(s);

	/* Select one of the available sensor sample processing styles */
	select_transform(s);

	/* Initialize fields related to sysfs reads offloading */
	sensor[s].thread_data_fd[0]  = -1;
	sensor[s].thread_data_fd[1]  = -1;
	sensor[s].acquisition_thread = -1;

	/* Check if we have a special ordering property on this sensor */
	if (sensor_get_order(s, sensor[s].order))
		sensor[s].quirks |= QUIRK_FIELD_ORDERING;

	sensor[s].needs_enable = get_needs_enable(dev_num, sensor_catalog[catalog_index].tag);

	sensor_count++;
}


static void discover_sensors (int dev_num, char *sysfs_base_path, char map[CATALOG_SIZE],
			      void (*discover_sensor)(int, char*, char*))
{
	char sysfs_dir[PATH_MAX];
	DIR *dir;
	struct dirent *d;
	unsigned int i;

	memset(map, 0, CATALOG_SIZE);

	snprintf(sysfs_dir, sizeof(sysfs_dir), sysfs_base_path, dev_num);

	dir = opendir(sysfs_dir);
	if (!dir) {
		return;
	}

	/* Enumerate entries in this iio device's base folder */

	while ((d = readdir(dir))) {
		if (!strcmp(d->d_name, ".") || !strcmp(d->d_name, ".."))
			continue;

		/* If the name matches a catalog entry, flag it */
		for (i = 0; i < CATALOG_SIZE; i++) {

			/* No discovery for virtual sensors */
			if (sensor_catalog[i].is_virtual)
				continue;
			discover_sensor(i, d->d_name, map);
		}
	}

	closedir(dir);
}

static void check_poll_sensors (int i, char *sysfs_file, char map[CATALOG_SIZE])
{
        int c;

	for (c = 0; c < sensor_catalog[i].num_channels; c++)
		if (!strcmp(sysfs_file, sensor_catalog[i].channel[c].raw_path) ||
		    !strcmp(sysfs_file, sensor_catalog[i].channel[c].input_path)) {
			map[i] = 1;
			break;
		}
}
static void check_trig_sensors (int i, char *sysfs_file, char map[CATALOG_SIZE])
{

	if (!strcmp(sysfs_file, sensor_catalog[i].channel[0].en_path)) {
		map[i] = 1;
		return;
	}
}

static void check_event_sensors(int i, char *sysfs_file, char map[CATALOG_SIZE])
{
	int j, k;

	for (j = 0; j < sensor_catalog[i].num_channels; j++)
		for (k = 0; k < sensor_catalog[i].channel[j].num_events; k++)
			if (!strcmp(sysfs_file, sensor_catalog[i].channel[j].event[k].ev_en_path)) {
				map[i] = 1;
				return ;
			}
}

static void virtual_sensors_check (void)
{
	int i;
	int has_acc = 0;
	int has_gyr = 0;
	int has_mag = 0;
	int has_rot = 0;
	int has_ori = 0;
	int catalog_size = CATALOG_SIZE;
	int gyro_cal_idx = 0;
	int magn_cal_idx = 0;

	for (i=0; i<sensor_count; i++)
		switch (sensor[i].type) {
			case SENSOR_TYPE_ACCELEROMETER:
				has_acc = 1;
				break;
			case SENSOR_TYPE_GYROSCOPE:
				has_gyr = 1;
				gyro_cal_idx = i;
				break;
			case SENSOR_TYPE_MAGNETIC_FIELD:
				has_mag = 1;
				magn_cal_idx = i;
				break;
			case SENSOR_TYPE_ORIENTATION:
				has_ori = 1;
				break;
			case SENSOR_TYPE_ROTATION_VECTOR:
				has_rot = 1;
				break;
		}

	for (i=0; i<catalog_size; i++)
		switch (sensor_catalog[i].type) {
			/*
	 		* If we have accel + gyro + magn but no rotation vector sensor,
	 		* SensorService replaces the HAL provided orientation sensor by the
	 		* AOSP version... provided we report one. So initialize a virtual
	 		* orientation sensor with zero values, which will get replaced. See:
	 		* frameworks/native/services/sensorservice/SensorService.cpp, looking
	 		* for SENSOR_TYPE_ROTATION_VECTOR; that code should presumably fall
	 		* back to mUserSensorList.add instead of replaceAt, but accommodate it.
	 		*/

			case SENSOR_TYPE_ORIENTATION:
				if (has_acc && has_gyr && has_mag && !has_rot && !has_ori)
					add_sensor(0, i, MODE_POLL);
				break;
			case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
				if (has_gyr) {
					sensor[sensor_count].base_count = 1;
					sensor[sensor_count].base[0] = gyro_cal_idx;
					add_virtual_sensor(i);
				}
				break;
			case SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED:
				if (has_mag) {
					sensor[sensor_count].base_count = 1;
					sensor[sensor_count].base[0] = magn_cal_idx;
					add_virtual_sensor(i);
				}
				break;
			default:
			break;
		}
}


static void propose_new_trigger (int s, char trigger_name[MAX_NAME_SIZE],
				 int sensor_name_len)
{
	/*
	 * A new trigger has been enumerated for this sensor. Check if it makes sense to use it over the currently selected one,
	 *  and select it if it is so. The format is something like sensor_name-dev0.
	 */

	const char *suffix = trigger_name + sensor_name_len + 1;

	/* dev is the default, and lowest priority; no need to update */
	if (!memcmp(suffix, "dev", 3))
		return;

	/* If we found any-motion trigger, record it */

	if (!memcmp(suffix, "any-motion-", 11)) {
		strcpy(sensor[s].motion_trigger_name, trigger_name);
		return;
	}

	/*
	 * It's neither the default "dev" nor an "any-motion" one. Make sure we use this though, as we may not have any other indication of the name
	 * of the trigger to use with this sensor.
	 */
	strcpy(sensor[s].init_trigger_name, trigger_name);
}


static void update_sensor_matching_trigger_name (char name[MAX_NAME_SIZE])
{
	/*
	 * Check if we have a sensor matching the specified trigger name, which should then begin with the sensor name, and end with a number
	 * equal to the iio device number the sensor is associated to. If so, update the string we're going to write to trigger/current_trigger
	 * when enabling this sensor.
	 */

	int s;
	int dev_num;
	int len;
	char* cursor;
	int sensor_name_len;

	/*
	 * First determine the iio device number this trigger refers to. We expect the last few characters (typically one) of the trigger name
	 * to be this number, so perform a few checks.
	 */
	len = strnlen(name, MAX_NAME_SIZE);

	if (len < 2)
		return;

	cursor = name + len - 1;

	if (!isdigit(*cursor))
		return;

	while (len && isdigit(*cursor)) {
		len--;
		cursor--;
	}

	dev_num = atoi(cursor+1);

	/* See if that matches a sensor */
	for (s=0; s<sensor_count; s++)
		if (sensor[s].dev_num == dev_num) {

			sensor_name_len = strlen(sensor[s].internal_name);

			if (!strncmp(name, sensor[s].internal_name, sensor_name_len))
				/* Switch to new trigger if appropriate */
				propose_new_trigger(s, name, sensor_name_len);
		}
}


static void setup_trigger_names (void)
{
	char filename[PATH_MAX];
	char buf[MAX_NAME_SIZE];
	int s;
	int trigger;
	int ret;

	/* By default, use the name-dev convention that most drivers use */
	for (s=0; s<sensor_count; s++)
		snprintf(sensor[s].init_trigger_name, MAX_NAME_SIZE, "%s-dev%d", sensor[s].internal_name, sensor[s].dev_num);

	/* Now have a look to /sys/bus/iio/devices/triggerX entries */

	for (trigger=0; trigger<MAX_TRIGGERS; trigger++) {

		snprintf(filename, sizeof(filename), TRIGGER_FILE_PATH,trigger);

		ret = sysfs_read_str(filename, buf, sizeof(buf));

		if (ret < 0)
			break;

		/* Record initial and any-motion triggers names */
		update_sensor_matching_trigger_name(buf);
	}

	/*
	 * Certain drivers expose only motion triggers even though they should be continous. For these, use the default trigger name as the motion
	 * trigger. The code generating intermediate events is dependent on motion_trigger_name being set to a non empty string.
	 */

	for (s=0; s<sensor_count; s++)
		if ((sensor[s].quirks & QUIRK_TERSE_DRIVER) && sensor[s].motion_trigger_name[0] == '\0')
			strcpy(sensor[s].motion_trigger_name, sensor[s].init_trigger_name);

	for (s=0; s<sensor_count; s++)
		if (sensor[s].mode == MODE_TRIGGER) {
			ALOGI("Sensor %d (%s) default trigger: %s\n", s, sensor[s].friendly_name, sensor[s].init_trigger_name);
			if (sensor[s].motion_trigger_name[0])
				ALOGI("Sensor %d (%s) motion trigger: %s\n", s, sensor[s].friendly_name, sensor[s].motion_trigger_name);
		}
}

void enumerate_sensors (void)
{
	/*
	 * Discover supported sensors and allocate control structures for them. Multiple sensors can potentially rely on a single iio device (each
	 * using their own channels). We can't have multiple sensors of the same type on the same device. In case of detection as both a poll-mode
	 * and trigger-based sensor, use the trigger usage mode.
	 */
	char poll_sensors[CATALOG_SIZE];
	char trig_sensors[CATALOG_SIZE];
	char event_sensors[CATALOG_SIZE];
	int dev_num;
	unsigned int i;
	int trig_found;

	for (dev_num=0; dev_num<MAX_DEVICES; dev_num++) {
		trig_found = 0;

		discover_sensors(dev_num, BASE_PATH, poll_sensors, check_poll_sensors);
		discover_sensors(dev_num, CHANNEL_PATH, trig_sensors, check_trig_sensors);
		discover_sensors(dev_num, EVENTS_PATH, event_sensors, check_event_sensors);

		for (i=0; i<CATALOG_SIZE; i++) {
			if (event_sensors[i]) {
				add_sensor(dev_num, i, MODE_EVENT);
				continue;
			}
			if (trig_sensors[i]) {
				add_sensor(dev_num, i, MODE_TRIGGER);
				trig_found = 1;
				continue;
			}
			if (poll_sensors[i]) {
				add_sensor(dev_num, i, MODE_POLL);
				continue;
			}
		}

		if (trig_found)
			build_sensor_report_maps(dev_num);
	}

	ALOGI("Discovered %d sensors\n", sensor_count);

	/* Set up default - as well as custom - trigger names */
	setup_trigger_names();

	virtual_sensors_check();
}


void delete_enumeration_data (void)
{
	int i;
	for (i = 0; i < sensor_count; i++)
		if (sensor[i].cal_data) {
			free(sensor[i].cal_data);
			sensor[i].cal_data = NULL;
			sensor[i].cal_level = 0;
		}

	/* Reset sensor count */
	sensor_count = 0;
}


int get_sensors_list (__attribute__((unused)) struct sensors_module_t* module,
		      struct sensor_t const** list)
{
	*list = sensor_desc;
	return sensor_count;
}

