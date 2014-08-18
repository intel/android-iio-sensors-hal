/*
 * Copyright (C) 2014 Intel Corporation.
 */

#include <ctype.h>
#include <dirent.h>
#include <stdlib.h>
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

struct sensor_catalog_entry_t sensor_catalog[] = {
	DECLARE_SENSOR3("accel",      SENSOR_TYPE_ACCELEROMETER,  "x", "y", "z")
	DECLARE_SENSOR3("anglvel",    SENSOR_TYPE_GYROSCOPE,      "x", "y", "z")
	DECLARE_SENSOR3("magn",       SENSOR_TYPE_MAGNETIC_FIELD, "x", "y", "z")
	DECLARE_SENSOR1("intensity",  SENSOR_TYPE_LIGHT,          "both"       )
	DECLARE_SENSOR0("illuminance",SENSOR_TYPE_LIGHT                        )
	DECLARE_SENSOR3("incli",      SENSOR_TYPE_ORIENTATION,    "x", "y", "z")
	DECLARE_SENSOR4("rot",        SENSOR_TYPE_ROTATION_VECTOR,
					 "quat_x", "quat_y", "quat_z", "quat_w")
	DECLARE_SENSOR0("temp",	      SENSOR_TYPE_AMBIENT_TEMPERATURE	       )
	DECLARE_SENSOR0("proximity",  SENSOR_TYPE_PROXIMITY		       )
	DECLARE_SENSOR3("anglvel",      SENSOR_TYPE_GYROSCOPE_UNCALIBRATED, "x", "y", "z")
};

#define CATALOG_SIZE	ARRAY_SIZE(sensor_catalog)


/* We equate sensor handles to indices in these tables */

struct sensor_t      sensor_desc[MAX_SENSORS];	/* Android-level descriptors */
struct sensor_info_t sensor_info[MAX_SENSORS];	/* Internal descriptors      */
int sensor_count;				/* Detected sensors 	     */


static void setup_properties_from_pld(int s, int panel, int rotation,
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

	if (panel == 5) {
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
		sensor_info[s].order[0] = 1;
		sensor_info[s].order[1] = 0;
		sensor_info[s].order[2] = 2;
		sensor_info[s].quirks |= QUIRK_FIELD_ORDERING;
	}

	sensor_info[s].channel[0].opt_scale = x;
	sensor_info[s].channel[1].opt_scale = y;
	sensor_info[s].channel[2].opt_scale = z;
}

static void decode_placement_information (int dev_num, int num_channels, int s)
{
	/*
	 * See if we have optional "physical location of device" ACPI tags.
	 * We're only interested in panel and rotation specifiers.
	 */

	char sysfs_path[PATH_MAX];
	int panel;
	int rotation;

	sprintf(sysfs_path, BASE_PATH "../firmware_node/pld/panel", dev_num);

	if (sysfs_read_int(sysfs_path, &panel))
		return;	/* Attribute not found */

	sprintf(sysfs_path, BASE_PATH "../firmware_node/pld/rotation", dev_num);

	if (sysfs_read_int(sysfs_path, &rotation))
		return; /* Attribute not found */

	ALOGI("Found PLD for S%d: panel=%d, rotation=%d\n", s, panel, rotation);

	if (panel != 4 && panel != 5) { /* 4 = front ; 5 = back */
		ALOGW("Unhandled panel spec\n");
		return;
	}

	/* Only deal with 90° rotations for now */
	if (rotation < 0 || rotation > 7 || (rotation & 1)) {
		ALOGW("Unhandled rotation spec\n");
		return;
	}

	/* Map that to field ordering and scaling mechanisms */
	setup_properties_from_pld(s, panel, rotation, num_channels);
}


static void add_sensor (int dev_num, int catalog_index, int use_polling)
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

	sensor_info[s].dev_num		= dev_num;
	sensor_info[s].catalog_index	= catalog_index;

        num_channels = sensor_catalog[catalog_index].num_channels;

        if (use_polling)
                sensor_info[s].num_channels = 0;
        else
                sensor_info[s].num_channels = num_channels;

	prefix = sensor_catalog[catalog_index].tag;

	/*
	 * receiving the illumination sensor calibration inputs from
	 * the Android properties and setting it within sysfs
	 */
	if (sensor_catalog[catalog_index].type == SENSOR_TYPE_LIGHT) {
		retval = sensor_get_illumincalib(s);
                if (retval > 0) {
			sprintf(sysfs_path, ILLUMINATION_CALIBPATH, dev_num);
			sysfs_write_int(sysfs_path, retval);
                }
	}

	/* Read name attribute, if available */
	sprintf(sysfs_path, NAME_PATH, dev_num);
	sysfs_read_str(sysfs_path, sensor_info[s].internal_name, MAX_NAME_SIZE);

	/* See if we have general offsets and scale values for this sensor */

	sprintf(sysfs_path, SENSOR_OFFSET_PATH, dev_num, prefix);
	sysfs_read_float(sysfs_path, &sensor_info[s].offset);

	sprintf(sysfs_path, SENSOR_SCALE_PATH, dev_num, prefix);
	if (!sysfs_read_float(sysfs_path, &scale)) {
                sensor_info[s].scale = scale;
		ALOGI("Scale path:%s scale:%f dev_num:%d\n",
                                        sysfs_path, scale, dev_num);
	} else {
                sensor_info[s].scale = 1;

                /* Read channel specific scale if any*/
                for (c = 0; c < num_channels; c++)
                {
                        sprintf(sysfs_path, BASE_PATH "%s", dev_num,
                           sensor_catalog[catalog_index].channel[c].scale_path);

                        if (!sysfs_read_float(sysfs_path, &scale)) {
                                sensor_info[s].channel[c].scale = scale;
			        sensor_info[s].scale = 0;

			        ALOGI(  "Scale path:%s "
					"channel scale:%f dev_num:%d\n",
                                        sysfs_path, scale, dev_num);
                        }
                }
        }

        /* Set default scaling - if num_channels is zero, we have one channel */

	sensor_info[s].channel[0].opt_scale = 1;

	for (c = 1; c < num_channels; c++)
		sensor_info[s].channel[c].opt_scale = 1;

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
				sensor_info[s].channel[c].opt_scale = opt_scale;
		}
        } else
		if (!sensor_get_fl_prop(s, "opt_scale", &opt_scale))
			sensor_info[s].channel[0].opt_scale = opt_scale;

	/* Initialize Android-visible descriptor */
	sensor_desc[s].name		= sensor_get_name(s);
	sensor_desc[s].vendor		= sensor_get_vendor(s);
	sensor_desc[s].version		= sensor_get_version(s);
	sensor_desc[s].handle		= s;
	sensor_desc[s].type		= sensor_type;
	sensor_desc[s].maxRange		= sensor_get_max_range(s);
	sensor_desc[s].resolution	= sensor_get_resolution(s);
	sensor_desc[s].power		= sensor_get_power(s);
	sensor_desc[s].stringType = sensor_get_string_type(s);

	/* None of our supported sensors requires a special permission.
	*  If this will be the case we should implement a sensor_get_perm
	*/
	sensor_desc[s].requiredPermission = "";
	sensor_desc[s].flags = sensor_get_flags(s);
	sensor_desc[s].maxDelay = sensor_get_max_delay(s);

	if (sensor_info[s].internal_name[0] == '\0') {
		/*
		 * In case the kernel-mode driver doesn't expose a name for
		 * the iio device, use (null)-dev%d as the trigger name...
		 * This can be considered a kernel-mode iio driver bug.
		 */
		ALOGW("Using null trigger on sensor %d (dev %d)\n", s, dev_num);
		strcpy(sensor_info[s].internal_name, "(null)");
	}

	if (sensor_type == SENSOR_TYPE_GYROSCOPE ||
		sensor_type == SENSOR_TYPE_GYROSCOPE_UNCALIBRATED) {
		struct gyro_cal* calibration_data = calloc(1, sizeof(struct gyro_cal));
		sensor_info[s].cal_data = calibration_data;
	}

	if (sensor_type == SENSOR_TYPE_MAGNETIC_FIELD) {
		struct compass_cal* calibration_data = calloc(1, sizeof(struct compass_cal));
		sensor_info[s].cal_data = calibration_data;
	}

	/* Select one of the available sensor sample processing styles */
	select_transform(s);

	/* Initialize fields related to sysfs reads offloading */
	sensor_info[s].thread_data_fd[0]  = -1;
	sensor_info[s].thread_data_fd[1]  = -1;
	sensor_info[s].acquisition_thread = -1;

	/* Check if we have a special ordering property on this sensor */
	if (sensor_get_order(s, sensor_info[s].order))
		sensor_info[s].quirks |= QUIRK_FIELD_ORDERING;

	sensor_count++;
}


static void discover_poll_sensors (int dev_num, char map[CATALOG_SIZE])
{
	char base_dir[PATH_MAX];
	DIR *dir;
	struct dirent *d;
	unsigned int i;
        int c;

	memset(map, 0, CATALOG_SIZE);

	snprintf(base_dir, sizeof(base_dir), BASE_PATH, dev_num);

	dir = opendir(base_dir);
	if (!dir) {
		return;
	}

	/* Enumerate entries in this iio device's base folder */

	while ((d = readdir(dir))) {
		if (!strcmp(d->d_name, ".") || !strcmp(d->d_name, ".."))
			continue;

		/* If the name matches a catalog entry, flag it */
		for (i = 0; i<CATALOG_SIZE; i++) {
		/* This will be added separately later */
		if (sensor_catalog[i].type == SENSOR_TYPE_GYROSCOPE_UNCALIBRATED)
			continue;
		for (c=0; c<sensor_catalog[i].num_channels; c++)
			if (!strcmp(d->d_name,sensor_catalog[i].channel[c].raw_path) ||
				!strcmp(d->d_name, sensor_catalog[i].channel[c].input_path)) {
					map[i] = 1;
					break;
			}
		}
	}

	closedir(dir);
}


static void discover_trig_sensors (int dev_num, char map[CATALOG_SIZE])
{
	char scan_elem_dir[PATH_MAX];
	DIR *dir;
	struct dirent *d;
	unsigned int i;

	memset(map, 0, CATALOG_SIZE);

	/* Enumerate entries in this iio device's scan_elements folder */

	snprintf(scan_elem_dir, sizeof(scan_elem_dir), CHANNEL_PATH, dev_num);

	dir = opendir(scan_elem_dir);
	if (!dir) {
		return;
	}

	while ((d = readdir(dir))) {
		if (!strcmp(d->d_name, ".") || !strcmp(d->d_name, ".."))
			continue;

		/* Compare en entry to known ones and create matching sensors */

		for (i = 0; i<CATALOG_SIZE; i++) {
			if (sensor_catalog[i].type == SENSOR_TYPE_GYROSCOPE_UNCALIBRATED)
				continue;
			if (!strcmp(d->d_name,
					sensor_catalog[i].channel[0].en_path)) {
					map[i] = 1;
					break;
			}
		}
	}

	closedir(dir);
}


static void orientation_sensor_check(void)
{
	/*
	 * If we have accel + gyro + magn but no rotation vector sensor,
	 * SensorService replaces the HAL provided orientation sensor by the
	 * AOSP version... provided we report one. So initialize a virtual
	 * orientation sensor with zero values, which will get replaced. See:
	 * frameworks/native/services/sensorservice/SensorService.cpp, looking
	 * for SENSOR_TYPE_ROTATION_VECTOR; that code should presumably fall
	 * back to mUserSensorList.add instead of replaceAt, but accommodate it.
	 */

	int i;
	int has_acc = 0;
	int has_gyr = 0;
	int has_mag = 0;
	int has_rot = 0;
	int has_ori = 0;
	int catalog_size = CATALOG_SIZE;

	for (i=0; i<sensor_count; i++)
		switch (sensor_catalog[sensor_info[i].catalog_index].type) {
			case SENSOR_TYPE_ACCELEROMETER:
				has_acc = 1;
				break;
			case SENSOR_TYPE_GYROSCOPE:
				has_gyr = 1;
				break;
			case SENSOR_TYPE_MAGNETIC_FIELD:
				has_mag = 1;
				break;
			case SENSOR_TYPE_ORIENTATION:
				has_ori = 1;
				break;
			case SENSOR_TYPE_ROTATION_VECTOR:
				has_rot = 1;
				break;
		}

	if (has_acc && has_gyr && has_mag && !has_rot && !has_ori)
		for (i=0; i<catalog_size; i++)
			if (sensor_catalog[i].type == SENSOR_TYPE_ORIENTATION) {
				ALOGI("Adding placeholder orientation sensor");
				add_sensor(0, i, 1);
				break;
			}
}

static void uncalibrated_gyro_check (void)
{
	unsigned int has_gyr = 0;
	unsigned int dev_num;
	int i, c;
	unsigned int is_poll_sensor;

	int cal_idx = 0;
	int uncal_idx = 0;

	/* Checking to see if we have a gyroscope - we can only have uncal if we have the base sensor */
	for (i=0; i < sensor_count; i++)
		if(sensor_catalog[sensor_info[i].catalog_index].type == SENSOR_TYPE_GYROSCOPE)
		{
			has_gyr=1;
			dev_num = sensor_info[i].dev_num;
			is_poll_sensor = !sensor_info[i].num_channels;
			cal_idx = i;
			break;
		}

	/*
	 * If we have a gyro we can add the uncalibrated sensor of the same type and
	 * on the same dev_num. We will save indexes for easy finding and also save the
	 * channel specific information.
	 */
	if (has_gyr)
		for (i=0; i<CATALOG_SIZE; i++)
			if (sensor_catalog[i].type == SENSOR_TYPE_GYROSCOPE_UNCALIBRATED) {
				add_sensor(dev_num, i, is_poll_sensor);

				uncal_idx = sensor_count - 1; /* Just added uncalibrated sensor */

				/* Similar to build_sensor_report_maps */
				for (c = 0; c < sensor_info[uncal_idx].num_channels; c++)
				{
					memcpy( &(sensor_info[uncal_idx].channel[c].type_spec),
						&(sensor_info[cal_idx].channel[c].type_spec),
						sizeof(sensor_info[uncal_idx].channel[c].type_spec));
					sensor_info[uncal_idx].channel[c].type_info = sensor_info[cal_idx].channel[c].type_info;
					sensor_info[uncal_idx].channel[c].offset    = sensor_info[cal_idx].channel[c].offset;
					sensor_info[uncal_idx].channel[c].size      = sensor_info[cal_idx].channel[c].size;
				}
				strncpy(sensor_info[uncal_idx].trigger_name,
					sensor_info[cal_idx].trigger_name,
					MAX_NAME_SIZE);
				sensor_info[uncal_idx].pair_idx = cal_idx;
				sensor_info[cal_idx].pair_idx = uncal_idx;
				break;
			}
}

static int is_continuous (int s)
{
	/* Is sensor s of the continous trigger type kind? */

	int catalog_index = sensor_info[s].catalog_index;
	int sensor_type = sensor_catalog[catalog_index].type;

	switch (sensor_type) {
		case SENSOR_TYPE_ACCELEROMETER:
		case SENSOR_TYPE_MAGNETIC_FIELD:
		case SENSOR_TYPE_ORIENTATION:
		case SENSOR_TYPE_GYROSCOPE:
		case SENSOR_TYPE_PRESSURE:
		case SENSOR_TYPE_GRAVITY:
		case SENSOR_TYPE_LINEAR_ACCELERATION:
		case SENSOR_TYPE_ROTATION_VECTOR:
		case SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED:
		case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
		case SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR:
			return 1;

		default:
			return 0;
	}
}


static void propose_new_trigger (int s, char trigger_name[MAX_NAME_SIZE],
				 int sensor_name_len)
{
	/*
	 * A new trigger has been enumerated for this sensor. Check if it makes
	 * sense to use it over the currently selected one, and select it if it
	 * is so. The format is something like sensor_name-dev0.
	 */

	const char *suffix = trigger_name + sensor_name_len + 1;

	/* dev is the default, and lowest priority; no need to update */
	if (!memcmp(suffix, "dev", 3))
		return;

	/*
	 * Anything else is higher priority. However if we already found an
	 * any-motion trigger, don't select anything else.
	 */

	if (!memcmp(sensor_info[s].trigger_name + sensor_name_len + 1,
		    "any-motion-", 11))
		return;

	/*
	 * If we're switching to an any-motion trigger, force the sensor to
	 * automatic intermediate event generation mode, at least if it is of a
	 * continuously firing sensor type.
	 */

	if (!memcmp(suffix, "any-motion-", 11) && is_continuous(s))
		sensor_info[s].quirks |= QUIRK_TERSE_DRIVER;

	/* Update the trigger name to use for this sensor */
	strcpy(sensor_info[s].trigger_name, trigger_name);
}


static void update_sensor_matching_trigger_name (char name[MAX_NAME_SIZE])
{
	/*
	 * Check if we have a sensor matching the specified trigger name,
	 * which should then begin with the sensor name, and end with a number
	 * equal to the iio device number the sensor is associated to. If so,
	 * update the string we're going to write to trigger/current_trigger
	 * when enabling this sensor.
	 */

	int s;
	int dev_num;
	int len;
	char* cursor;
	int sensor_name_len;

	/*
	 * First determine the iio device number this trigger refers to. We
	 * expect the last few characters (typically one) of the trigger name
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
		if (sensor_info[s].dev_num == dev_num) {

			sensor_name_len = strlen(sensor_info[s].internal_name);

			if (!strncmp(name,
				     sensor_info[s].internal_name,
				     sensor_name_len))
				/* Switch to new trigger if appropriate */
				propose_new_trigger(s, name, sensor_name_len);
		}
}


static void setup_trigger_names (void)
{
	char filename[PATH_MAX];
	char buf[MAX_NAME_SIZE];
	int len;
	int s;
	int trigger;
	int ret;

	/* By default, use the name-dev convention that most drivers use */
	for (s=0; s<sensor_count; s++)
		snprintf(sensor_info[s].trigger_name, MAX_NAME_SIZE, "%s-dev%d",
			sensor_info[s].internal_name, sensor_info[s].dev_num);

	/* Now have a look to /sys/bus/iio/devices/triggerX entries */

	for (trigger=0; trigger<MAX_TRIGGERS; trigger++) {

		snprintf(filename, sizeof(filename), TRIGGER_FILE_PATH,trigger);

		ret = sysfs_read_str(filename, buf, sizeof(buf));

		if (ret < 0)
			break;

		update_sensor_matching_trigger_name(buf);
	}

	for (s=0; s<sensor_count; s++)
		if (sensor_info[s].num_channels) {
			ALOGI(	"Sensor %d (%s) using iio trigger %s\n", s,
				sensor_info[s].friendly_name,
				sensor_info[s].trigger_name);
		}
}


void enumerate_sensors (void)
{
	/*
	 * Discover supported sensors and allocate control structures for them.
	 * Multiple sensors can potentially rely on a single iio device (each
	 * using their own channels). We can't have multiple sensors of the same
	 * type on the same device. In case of detection as both a poll-mode
	 * and trigger-based sensor, use the trigger usage mode.
	 */
	char poll_sensors[CATALOG_SIZE];
	char trig_sensors[CATALOG_SIZE];
	int dev_num;
	unsigned int i;
	int trig_found;

	for (dev_num=0; dev_num<MAX_DEVICES; dev_num++) {
		trig_found = 0;

		discover_poll_sensors(dev_num, poll_sensors);
		discover_trig_sensors(dev_num, trig_sensors);

		for (i=0; i<CATALOG_SIZE; i++)
			if (trig_sensors[i]) {
				add_sensor(dev_num, i, 0);
				trig_found = 1;
			}
			else
				if (poll_sensors[i])
					add_sensor(dev_num, i, 1);

		if (trig_found) {
			build_sensor_report_maps(dev_num);
		}
	}

	ALOGI("Discovered %d sensors\n", sensor_count);

	/* Set up default - as well as custom - trigger names */
	setup_trigger_names();

	/* Make sure Android fall backs to its own orientation sensor */
	orientation_sensor_check();

	/* Create the uncalibrated counterpart to the compensated gyroscope;
	 * This is is a new sensor type in Android 4.4 */
	uncalibrated_gyro_check();
}


void delete_enumeration_data (void)
{

	int i;
	for (i = 0; i < sensor_count; i++)
	switch (sensor_catalog[sensor_info[i].catalog_index].type) {
		case SENSOR_TYPE_MAGNETIC_FIELD:
		case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
		case SENSOR_TYPE_GYROSCOPE:
			if (sensor_info[i].cal_data != NULL) {
				free(sensor_info[i].cal_data);
				sensor_info[i].cal_data = NULL;
				sensor_info[i].cal_level = 0;
			}
			break;
		default:
			break;
	}
	/* Reset sensor count */
	sensor_count = 0;
}


int get_sensors_list(	struct sensors_module_t* module,
			struct sensor_t const** list)
{
	*list = sensor_desc;
	return sensor_count;
}

