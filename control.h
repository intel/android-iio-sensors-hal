/*
 * Copyright (C) 2014-2015 Intel Corporation.
 */

#ifndef __CONTROL_H__
#define __CONTROL_H__

int	sensor_activate		(int handle, int enabled, int from_virtual);
int	sensor_set_delay	(int handle, int64_t ns);
int	sensor_poll 		(sensors_event_t* data, int count);
int	sensor_flush		(int handle);

int	allocate_control_data	(void);
void	delete_control_data	(void);

void	build_sensor_report_maps(int dev_num);

#endif
