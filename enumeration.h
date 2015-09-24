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

#ifndef __ENUMERATION_H__
#define __ENUMERATION_H__

#include "common.h"

int	get_sensors_list	(struct sensors_module_t* module,
				 struct sensor_t const** list);

void	enumerate_sensors	(void);
void	delete_enumeration_data (void);

/*
 * These are fine-grained type definitions that are used internally, in the sensor array, but mapped to an Android sensor type in the processing pipeline.
 * The sensor array uses these, not the desc array.
 */
#define SENSOR_TYPE_INTERNAL_ILLUMINANCE	-1	/* Global illuminance, in lux				 */
#define SENSOR_TYPE_INTERNAL_INTENSITY		-2	/* Global intensity, in sensor specific units		 */

#endif
