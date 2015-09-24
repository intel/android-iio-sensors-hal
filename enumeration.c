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

#include <ctype.h>
#include <dirent.h>
#include <stdlib.h>
#include <fcntl.h>
#include <utils/Log.h>
#include <sys/stat.h>
#include <hardware/sensors.h>
#include "enumeration.h"
#include "description.h"
#include "utils.h"
#include "transform.h"
#include "description.h"
#include "control.h"
#include "calibration.h"

#include <errno.h>

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
		.shorthand	= "",
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
		.shorthand	= "",
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
		.shorthand	= "",
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
		.shorthand	= "",
		.type		= SENSOR_TYPE_INTERNAL_INTENSITY,
		.num_channels	= 1,
		.is_virtual	= 0,
		.channel = {
			{ DECLARE_NAMED_CHANNEL("intensity", "both") },
		},
	},
	{
		.tag		= "illuminance",
		.shorthand	= "",
		.type		= SENSOR_TYPE_INTERNAL_ILLUMINANCE,
		.num_channels	= 1,
		.is_virtual	= 0,
		.channel = {
			{ DECLARE_GENERIC_CHANNEL("illuminance") },
		},
	},
	{
		.tag		= "incli",
		.shorthand	= "",
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
		.shorthand	= "",
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
		.shorthand	= "",
		.type		= SENSOR_TYPE_AMBIENT_TEMPERATURE,
		.num_channels	= 1,
		.is_virtual	= 0,
		.channel = {
			{ DECLARE_GENERIC_CHANNEL("temp") },
		},
	},
	{
		.tag		= "proximity",
		.shorthand	= "prox",
		.type		= SENSOR_TYPE_PROXIMITY,
		.num_channels	= 1,
		.is_virtual	= 0,
		.channel = {
			{ DECLARE_GENERIC_CHANNEL("proximity") },
		},
	},
	{
		.tag		= "",
		.shorthand	= "",
		.type		= SENSOR_TYPE_GYROSCOPE_UNCALIBRATED,
		.num_channels	= 0,
		.is_virtual	= 1,
		.channel = {
			{ DECLARE_GENERIC_CHANNEL("") },
		},

	},
	{
		.tag		= "",
		.shorthand	= "",
		.type		= SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED,
		.num_channels	= 0,
		.is_virtual	= 1,
		.channel = {
			{ DECLARE_GENERIC_CHANNEL("") },
		},
	},
	{
		.tag		= "steps",
		.shorthand	= "",
		.type		= SENSOR_TYPE_STEP_COUNTER,
		.num_channels	= 1,
		.is_virtual	= 0,
		.channel = {
			{ DECLARE_GENERIC_CHANNEL("steps") },
		},
	},
	{
		.tag		= "steps",
		.shorthand	= "",
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
	{
		.tag		= "proximity",
		.shorthand	= "prox",
		.type		= SENSOR_TYPE_PROXIMITY,
		.num_channels	= 4,
		.is_virtual	= 0,
		.channel = {
			{
				DECLARE_VOID_CHANNEL("proximity0")
				.num_events = 1,
				.event = {
					{ DECLARE_EVENT("proximity0", "_", "", "", "thresh", "_", "either") },
				},
			},
			{
				DECLARE_VOID_CHANNEL("proximity1")
				.num_events = 1,
				.event = {
					{ DECLARE_EVENT("proximity1", "_", "", "", "thresh", "_", "either") },
				},
			},
			{
				DECLARE_VOID_CHANNEL("proximity2")
				.num_events = 1,
				.event = {
					{ DECLARE_EVENT("proximity2", "_", "", "", "thresh", "_", "either") },
				},
			},
			{
				DECLARE_VOID_CHANNEL("proximity3")
				.num_events = 1,
				.event = {
					{ DECLARE_EVENT("proximity3", "_", "", "", "thresh", "_", "either") },
				},
			},
		},
	},
};

unsigned int catalog_size = ARRAY_SIZE(sensor_catalog);

/* ACPI PLD (physical location of device) definitions, as used with sensors */

#define PANEL_FRONT	4
#define PANEL_BACK	5

/* Buffer default length */
#define BUFFER_LENGTH	16

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


static int map_internal_to_external_type (int sensor_type)
{
	/* Most sensors are internally identified using the Android type, but for some we use a different type specification internally */

	switch (sensor_type) {
		case SENSOR_TYPE_INTERNAL_ILLUMINANCE:
		case SENSOR_TYPE_INTERNAL_INTENSITY:
			return SENSOR_TYPE_LIGHT;

		default:
			return sensor_type;
	}
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
	sensor_desc[s].type		= map_internal_to_external_type(sensor_type);

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


static int add_sensor (int dev_num, int catalog_index, int mode)
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
	int buffer_length;

	if (sensor_count == MAX_SENSORS) {
		ALOGE("Too many sensors!\n");
		return -1;
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
	sensor[s].trigger_nr = -1;	/* -1 means no trigger - we'll populate these at a later time */

        num_channels = sensor_catalog[catalog_index].num_channels;

        if (mode == MODE_POLL)
                sensor[s].num_channels = 0;
        else
                sensor[s].num_channels = num_channels;

	/* Populate the quirks array */
	sensor_get_quirks(s);

	/* Reject interfaces that may have been disabled through a quirk for this driver */
	if ((mode == MODE_EVENT   && (sensor[s].quirks & QUIRK_NO_EVENT_MODE)) ||
	    (mode == MODE_TRIGGER && (sensor[s].quirks & QUIRK_NO_TRIG_MODE )) ||
            (mode == MODE_POLL    && (sensor[s].quirks & QUIRK_NO_POLL_MODE ))) {
		memset(&sensor[s], 0, sizeof(sensor[0]));
		return -1;
	}

	prefix = sensor_catalog[catalog_index].tag;

	/*
	 * receiving the illumination sensor calibration inputs from
	 * the Android properties and setting it within sysfs
	 */
	if (sensor_type == SENSOR_TYPE_INTERNAL_ILLUMINANCE) {
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

	/* Change buffer length according to the property or use default value */
	if (mode == MODE_TRIGGER) {
                if (sensor_get_prop(s, "buffer_length", &buffer_length)) {
                        buffer_length = BUFFER_LENGTH;
                }

                sprintf(sysfs_path, BUFFER_LENGTH_PATH, dev_num);

                if (sysfs_write_int(sysfs_path, buffer_length) <= 0) {
                        ALOGE("Failed to set buffer length on dev%d", dev_num);
                }
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

	for (c = 0; c < num_channels; c++) {
		/* Check the presence of the channel's input_path */
		sprintf(sysfs_path, BASE_PATH "%s", dev_num,
			sensor_catalog[catalog_index].channel[c].input_path);
		sensor[s].channel[c].input_path_present = (access(sysfs_path, R_OK) != -1);
		/* Check the presence of the channel's raw_path */
		sprintf(sysfs_path, BASE_PATH "%s", dev_num,
			sensor_catalog[catalog_index].channel[c].raw_path);
		sensor[s].channel[c].raw_path_present = (access(sysfs_path, R_OK) != -1);
	}

	sensor_get_available_frequencies(s);

	if (sensor_get_mounting_matrix(s, sensor[s].mounting_matrix))
		sensor[s].quirks |= QUIRK_MOUNTING_MATRIX;
	else
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
        } else {
		if (!sensor_get_fl_prop(s, "opt_scale", &opt_scale))
			sensor[s].channel[0].opt_scale = opt_scale;
        }

	populate_descriptors(s, sensor_type);

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
		case SENSOR_TYPE_ACCELEROMETER:
			/* Only engage accelerometer bias compensation if really needed */
			if (sensor_get_quirks(s) & QUIRK_BIASED)
				sensor[s].cal_data = calloc(1, sizeof(accel_cal_t));
			break;

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
	return 0;
}

static void virtual_sensors_check (void)
{
	int i;
	int has_acc = 0;
	int has_gyr = 0;
	int has_mag = 0;
	int has_rot = 0;
	int has_ori = 0;
	int gyro_cal_idx = 0;
	int magn_cal_idx = 0;
	unsigned int j;

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

	for (j=0; j<catalog_size; j++)
		switch (sensor_catalog[j].type) {
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
					add_sensor(0, j, MODE_POLL);
				break;
			case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
				if (has_gyr) {
					sensor[sensor_count].base_count = 1;
					sensor[sensor_count].base[0] = gyro_cal_idx;
					add_virtual_sensor(j);
				}
				break;
			case SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED:
				if (has_mag) {
					sensor[sensor_count].base_count = 1;
					sensor[sensor_count].base[0] = magn_cal_idx;
					add_virtual_sensor(j);
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

	/* If we found a hrtimer trigger, record it */
	if (!memcmp(suffix, "hr-dev", 6)) {
		strcpy(sensor[s].hrtimer_trigger_name, trigger_name);
		return;
	}
	/*
	 * It's neither the default "dev" nor an "any-motion" one. Make sure we use this though, as we may not have any other indication of the name
	 * of the trigger to use with this sensor.
	 */
	strcpy(sensor[s].init_trigger_name, trigger_name);
}


static void update_sensor_matching_trigger_name (char name[MAX_NAME_SIZE], int* updated, int trigger)
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
				updated[s] = 1;
				sensor[s].trigger_nr = trigger;
		}
}

extern float sensor_get_max_static_freq(int s);
extern float sensor_get_min_freq (int s);

static int create_hrtimer_trigger(int s, int trigger)
{
	struct stat dir_status;
	char buf[MAX_NAME_SIZE];
	char hrtimer_path[PATH_MAX];
	char hrtimer_name[MAX_NAME_SIZE];
	float min_supported_rate = 1, min_rate_cap, max_supported_rate;

	snprintf(buf, MAX_NAME_SIZE, "hrtimer-%s-hr-dev%d", sensor[s].internal_name, sensor[s].dev_num);
	snprintf(hrtimer_name, MAX_NAME_SIZE, "%s-hr-dev%d", sensor[s].internal_name, sensor[s].dev_num);
	snprintf(hrtimer_path, PATH_MAX, "%s%s", CONFIGFS_TRIGGER_PATH, buf);

	/* Get parent dir status */
	if (stat(CONFIGFS_TRIGGER_PATH, &dir_status))
		return -1;

	/* Create hrtimer with the same access rights as it's parent */
	if (mkdir(hrtimer_path, dir_status.st_mode))
		if (errno != EEXIST)
			return -1;

	strncpy (sensor[s].hrtimer_trigger_name, hrtimer_name, MAX_NAME_SIZE);
	sensor[s].trigger_nr = trigger;

	max_supported_rate = sensor_get_max_static_freq(s);

	/* set 0 for wrong values */
	if (max_supported_rate < 0.1) {
		max_supported_rate = 0;
	}

	sensor[s].max_supported_rate = max_supported_rate;
	sensor_desc[s].minDelay = max_supported_rate ? (int32_t) (1000000.0 / max_supported_rate) : 0;

	/* Check if a minimum rate was specified for this sensor */
	min_rate_cap = sensor_get_min_freq(s);

	if (min_supported_rate < min_rate_cap) {
		min_supported_rate = min_rate_cap;
	}

	sensor[s].min_supported_rate = min_supported_rate;
	sensor_desc[s].maxDelay = (max_delay_t) (1000000.0 / min_supported_rate);

	return 0;
}

static void setup_trigger_names (void)
{
	char filename[PATH_MAX];
	char buf[MAX_NAME_SIZE];
	int s;
	int trigger;
	int ret;
	int updated[MAX_SENSORS] = {0};

	/* By default, use the name-dev convention that most drivers use */
	for (s=0; s<sensor_count; s++)
		snprintf(sensor[s].init_trigger_name, MAX_NAME_SIZE, "%s-dev%d", sensor[s].internal_name, sensor[s].dev_num);

	/* Now have a look to /sys/bus/iio/devices/triggerX entries */

	for (trigger=0; trigger<MAX_TRIGGERS; trigger++) {

		snprintf(filename, sizeof(filename), TRIGGER_FILE_PATH, trigger);

		ret = sysfs_read_str(filename, buf, sizeof(buf));

		if (ret < 0)
			break;

		/* Record initial and any-motion triggers names */
		update_sensor_matching_trigger_name(buf, updated, trigger);
	}


	/* If we don't have any other trigger exposed and quirk hrtimer is set setup the hrtimer name here  - and create it also */
	for (s=0; s<sensor_count && trigger<MAX_TRIGGERS; s++) {
		if ((sensor[s].quirks & QUIRK_HRTIMER) && !updated[s]) {
			create_hrtimer_trigger(s, trigger);
			trigger++;
		}
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
			if (sensor[s].hrtimer_trigger_name[0])
				ALOGI("Sensor %d (%s) hrtimer trigger: %s\n", s, sensor[s].friendly_name, sensor[s].hrtimer_trigger_name);
		}
}


static int catalog_index_from_sensor_type (int type)
{
	/* Return first matching catalog entry index for selected type */
	unsigned int i;

	for (i=0; i<catalog_size; i++)
		if (sensor_catalog[i].type == type)
			return i;

	return -1;
}


static void post_process_sensor_list (char poll_map[catalog_size], char trig_map[catalog_size], char event_map[catalog_size])
{
	int illuminance_cat_index = catalog_index_from_sensor_type(SENSOR_TYPE_INTERNAL_ILLUMINANCE);
	int intensity_cat_index	  = catalog_index_from_sensor_type(SENSOR_TYPE_INTERNAL_INTENSITY);
	int illuminance_found	  = poll_map[illuminance_cat_index] || trig_map[illuminance_cat_index] || event_map[illuminance_cat_index];

	/* If an illumimance sensor has been reported */
	if (illuminance_found) {
		/* Hide any intensity sensors we can have for the same iio device */
		poll_map [intensity_cat_index     ] = 0;
		trig_map [intensity_cat_index     ] = 0;
		event_map[intensity_cat_index     ] = 0;
		return;
	}
}


static void swap_sensors (int s1, int s2)
{
	struct sensor_t	temp_sensor_desc;
	sensor_info_t	temp_sensor;

	/* S1 -> temp */
	memcpy(&temp_sensor, &sensor[s1], sizeof(sensor_info_t));
	memcpy(&temp_sensor_desc, &sensor_desc[s1], sizeof(struct sensor_t));

	/* S2 -> S1 */
	memcpy(&sensor[s1], &sensor[s2], sizeof(sensor_info_t));
	memcpy(&sensor_desc[s1], &sensor_desc[s2], sizeof(struct sensor_t));

	/* temp -> S2 */
	memcpy(&sensor[s2], &temp_sensor, sizeof(sensor_info_t));
	memcpy(&sensor_desc[s2], &temp_sensor_desc,  sizeof(struct sensor_t));

	/* Fix-up sensor id mapping, which is stale */
	sensor_desc[s1].handle	= s1;
	sensor_desc[s2].handle	= s2;

	/* Fix up name and vendor buffer pointers, which are potentially stale pointers */
	sensor_desc[s1].name		= sensor_get_name(s1);
	sensor_desc[s1].vendor		= sensor_get_vendor(s1);
	sensor_desc[s2].name		= sensor_get_name(s2);
	sensor_desc[s2].vendor		= sensor_get_vendor(s2);
}


static void reorder_sensors (void)
{
	/* Some sensors may be marked as secondary - these need to be listed after other sensors of the same type */
	int s1, s2;

	for (s1=0; s1<sensor_count-1; s1++)
		if (sensor[s1].quirks & QUIRK_SECONDARY) {
			/* Search for subsequent sensors of same type */
			for (s2 = s1+1; s2<sensor_count; s2++)
				if (sensor[s2].type == sensor[s1].type && !(sensor[s2].quirks & QUIRK_SECONDARY)) {
					ALOGI("Sensor S%d has higher priority than S%d, swapping\n", s2, s1);
					swap_sensors(s1, s2);
					break;
				}
		}
}


void enumerate_sensors (void)
{
	/*
	 * Discover supported sensors and allocate control structures for them. Multiple sensors can potentially rely on a single iio device (each
	 * using their own channels). We can't have multiple sensors of the same type on the same device. In case of detection as both a poll-mode
	 * and trigger-based sensor, use the trigger usage mode.
	 */
	char poll_sensors[catalog_size];
	char trig_sensors[catalog_size];
	char event_sensors[catalog_size];
	int dev_num;
	unsigned int i;
	int trig_found;
	int s;

	for (dev_num=0; dev_num<MAX_DEVICES; dev_num++) {
		trig_found = 0;

		discover_sensors(dev_num, BASE_PATH, poll_sensors, check_poll_sensors);
		discover_sensors(dev_num, CHANNEL_PATH, trig_sensors, check_trig_sensors);
		discover_sensors(dev_num, EVENTS_PATH, event_sensors, check_event_sensors);

		/* Hide specific sensor types if appropriate */
		post_process_sensor_list(poll_sensors, trig_sensors, event_sensors);

		for (i=0; i<catalog_size; i++) {
			/* Try using events interface */
			if (event_sensors[i] && !add_sensor(dev_num, i, MODE_EVENT))
				continue;

			/* Then trigger */
			if (trig_sensors[i] && !add_sensor(dev_num, i, MODE_TRIGGER)) {
				trig_found = 1;
				continue;
			}

			/* Try polling otherwise */
			if (poll_sensors[i])
				add_sensor(dev_num, i, MODE_POLL);
		}

		if (trig_found)
			build_sensor_report_maps(dev_num);
	}

	/* Make sure secondary sensors appear after primary ones */
	reorder_sensors();

	ALOGI("Discovered %d sensors\n", sensor_count);

	/* Set up default - as well as custom - trigger names */
	setup_trigger_names();

	ALOGI("Discovered %d sensors\n", sensor_count);

	virtual_sensors_check();

	for (s=0; s<sensor_count; s++) {
		ALOGI("S%d: %s\n", s, sensor[s].friendly_name);
	}
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

