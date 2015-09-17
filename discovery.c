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

#include <stdlib.h>
#include <utils/Log.h>
#include <stdint.h>
#include <dirent.h>
#include <sys/types.h>
#include <hardware/sensors.h>

#include "common.h"

void discover_sensors(int dev_num, char *sysfs_base_path, char map[catalog_size],
		      void (*discover_sensor)(int, char*, char*))
{
	char sysfs_dir[PATH_MAX];
	DIR *dir;
	struct dirent *d;
	unsigned int i;

	memset(map, 0, catalog_size);

	snprintf(sysfs_dir, sizeof(sysfs_dir), sysfs_base_path, dev_num);

	dir = opendir(sysfs_dir);
	if (!dir) {
		return;
	}

	/* Enumerate entries in this iio device's base folder */

	while ((d = readdir(dir))) {
		if (!strncmp(d->d_name, ".", sizeof(".")) || !strncmp(d->d_name, "..", sizeof("..")))
			continue;

		/* If the name matches a catalog entry, flag it */
		for (i = 0; i < catalog_size; i++) {

			/* No discovery for virtual sensors */
			if (sensor_catalog[i].is_virtual)
				continue;
			discover_sensor(i, d->d_name, map);
		}
	}

	closedir(dir);
}

void check_event_sensors(int i, char *sysfs_file, char map[catalog_size])
{
	int j, k;

	for (j = 0; j < sensor_catalog[i].num_channels; j++)
		for (k = 0; k < sensor_catalog[i].channel[j].num_events; k++) {
			if (!strcmp(sysfs_file, sensor_catalog[i].channel[j].event[k].ev_en_path)) {
				map[i] = 1;
				return;
			}
		}
}

void check_poll_sensors (int i, char *sysfs_file, char map[catalog_size])
{
        int c;

	for (c = 0; c < sensor_catalog[i].num_channels; c++)
		if (!strcmp(sysfs_file, sensor_catalog[i].channel[c].raw_path) ||
		    !strcmp(sysfs_file, sensor_catalog[i].channel[c].input_path)) {
			map[i] = 1;
			break;
		}
}

void check_trig_sensors (int i, char *sysfs_file, char map[catalog_size])
{

	if (!strcmp(sysfs_file, sensor_catalog[i].channel[0].en_path)) {
		map[i] = 1;
		return;
	}
}
