/*
 * Copyright (C) 2015 Intel Corporation.
 */

#ifndef __ACTIVITY_EVENT_UTILS_H__
#define __ACTIVITY_EVENT_UTILS_H__

#include <hardware/activity_recognition.h>

#include "utils.h"

#define MAX_ACTIVITIES		6
#define MAX_EVENTS_PER_ACTIVITY	2

typedef unsigned bool;
#define true	1
#define false	0

/* For each activity in activity_recognition.h we can monitor 2 events at most :
 * ENTER and EXIT */
struct activity_event_info {
	struct activity_event	*event[MAX_EVENTS_PER_ACTIVITY];
	int			modifier;
	int			sensor_catalog_index;
	int			channel_index;
	int 			dev_num;
	int 			event_fd;
	int 			event_count;
	bool			monitored[MAX_EVENTS_PER_ACTIVITY];
};

struct control_event_data {
	uint8_t	enable;
	uint8_t	activity;
	uint8_t	event;
};

/**
 * Creates a control event identifier:
 *	[unused]      EVENT      ACTIVITY   ENABLE
 *	63 ... 24   23 ... 16    15 ... 8   7 ... 0
 * @enable:	Says if the <activity, event> pair needs to be enabled or disabled (0 or 1)
 * @activity:	What activity are we working on - index in the list returned by
 * 		get_supported_activities_list()
 * @event:	What type of event to asociate with the given activity (one of
 * 		the ACTIVITY_EVENT_* enum)
 */
static inline uint64_t get_control_code(uint8_t enable, uint8_t activity, uint8_t event)
{
	return ((uint64_t)enable << 56) |
		((uint64_t)activity << 48) |
		((uint64_t)event << 40);
}

/**
 * Parses the given control identifier and retrieves the control data.
 * @control_code:	the unified control data
 * @control_data:	extracted data from the control code
 */
static inline void get_control_data(uint64_t control_code,
				    struct control_event_data *control_data)
{
	control_data->enable	= (uint8_t)(control_code >> 56);
	control_data->activity	= (uint8_t)(control_code >> 48 & 0xFF);
	control_data->event	= (uint8_t)(control_code >> 40 & 0xFF);
}

#endif
