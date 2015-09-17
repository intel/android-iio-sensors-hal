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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <dlfcn.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>

#include <hardware/activity_recognition.h>
#include <hardware/sensors.h>
#include <utils/Log.h>

#include "activity_event_utils.h"

/* Return error codes */
#define ARGV_ERR	1
#define HAL_ACCESS_ERR	1
#define HAL_OPEN_ERR	2
#define HAL_SYMBOL_ERR	3

#define START_CMD		"start"
#define STOP_CMD		"stop"
#define REGISTER_CMD		"register_callback"
#define ENABLE_CMD		"enable"
#define DISABLE_CMD		"disable"
#define LIST_CMD		"list"
#define FLUSH_CMD		"flush"
#define MONITOR_START_CMD	"monitor_start"
#define MONITOR_STOP_CMD	"monitor_stop"

#define MAX_CMD_SIZE	512
#define MAX_ARGS	8

#define CMD_SEPARATOR	" "

#define ACK_FLAG		1
#define MONITOR_START_FLAG	2
#define MONITOR_STOP_FLAG	3

/* We define a default latency since current HAL does not use this latency */
#define DEFAULT_LATENCY	0

static struct activity_recognition_module *hmi;
static struct hw_device_t *device;

static FILE *client, *monitor_client;
static int monitor_connection = -1;

#ifdef ANDROID
#define ACTIVITY_SERVER_NAME	"/dev/socket/activity-server"
#else
#define ACTIVITY_SERVER_NAME	"/tmp/activity-server"
#endif

#define CLIENT_ERR(f, fmt...)					\
	 { if (f) { fprintf(f, fmt); fprintf(f, "\n"); } ALOGE(fmt); }

struct sockaddr_un server_addr = {
	.sun_family	= AF_UNIX,
	.sun_path	= ACTIVITY_SERVER_NAME,
};

/* Event types are indexed so that a type is positioned at an index represented
 * by its value in the ACTIVITY_TYPE_* enum in activity_recognition.h .
 */
static const char* event_types[] = {
	"FLUSH_COMPLETE",
	"ENTER",
	"EXIT",
};

static void dummy_activity_callback(const activity_recognition_callback_procs_t *procs __attribute((unused)),
				    const activity_event_t *events, int count)
{
	int i;

	/* Exit if nobody is monitoring */
	if (monitor_client == NULL)
		return;

	fprintf(monitor_client, "No. of activity events: %d\n", count);

	for (i = 0; i < count; i++)
		fprintf(monitor_client, "\t<activity [%d], event %s> occurred at %lld (ns)\n",
			events[i].activity,
			event_types[events[i].event_type],
			events[i].timestamp);
}

static activity_recognition_callback_procs_t dummy_callback_procs = {
	.activity_callback = dummy_activity_callback
};

/* We register a dummy callback that just prints the events reported from HAL. */
static void register_activity_callback(void)
{
	struct activity_recognition_device *activity_dev =
		(struct activity_recognition_device *) device;

	activity_dev->register_activity_callback(activity_dev, &dummy_callback_procs);
}

static int enable_activity_event(uint32_t handle, uint32_t event_type)
{
	struct activity_recognition_device *activity_dev =
		(struct activity_recognition_device *) device;

	return activity_dev->enable_activity_event(activity_dev, handle,
						   event_type, DEFAULT_LATENCY);
}

static int disable_activity_event(uint32_t handle, uint32_t event_type)
{
	struct activity_recognition_device *activity_dev =
		(struct activity_recognition_device*) device;

	return activity_dev->disable_activity_event(activity_dev, handle, event_type);
}

static int flush(void)
{
	struct activity_recognition_device *activity_dev =
		(struct activity_recognition_device *) device;

	return activity_dev->flush(activity_dev);
}

static int print_usage(void)
{
	fprintf(stderr, "Program usage:\n");
	fprintf(stderr, "\tactivity %s\n", START_CMD);
	fprintf(stderr, "\tactivity %s\n", STOP_CMD);
	fprintf(stderr, "\tactivity %s\n", REGISTER_CMD);
	fprintf(stderr, "\tactivity %s\n", LIST_CMD);
	fprintf(stderr, "\tactivity %s <activity_id> <event_type>\n", ENABLE_CMD);
	fprintf(stderr, "\tactivity %s <activity_id> <event_type>\n", DISABLE_CMD);
	fprintf(stderr, "\tactivity %s\n", FLUSH_CMD);
	fprintf(stderr, "\tactivity %s\n", MONITOR_START_CMD);
	fprintf(stderr, "\tactivity %s\n", MONITOR_STOP_CMD);
	fprintf(stderr, "\t* <activity_id> is the index of the activity as shown by running \"activity list\"\n");
	fprintf(stderr, "\t* <event_type> is one of the following:\n");
	fprintf(stderr, "\t\t%d => activity event ENTER\n", ACTIVITY_EVENT_ENTER);
	fprintf(stderr, "\t\t%d => activity event EXIT\n", ACTIVITY_EVENT_EXIT);
	fprintf(stderr, "\t\t%d => activity event FLUSH COMPLETE\n", ACTIVITY_EVENT_FLUSH_COMPLETE);

	return ARGV_ERR;
}

static int parse_cmd(char buffer[])
{
	char *tmp, *args[MAX_ARGS];
	int count;

	tmp = strtok(buffer, CMD_SEPARATOR);
	count = 0;
	while (tmp) {
		args[count++]	= tmp;
		tmp		= strtok(NULL, CMD_SEPARATOR);
	}

	if (!count) {
		CLIENT_ERR(client, "Invalid command %s", buffer);
		return -1;
	}

	if (strncmp(args[0], STOP_CMD, sizeof(STOP_CMD)) == 0) {
		if (count != 1) {
			CLIENT_ERR(client, "Too many arguments. Trimming command down to \
				   'activity %s'", STOP_CMD);
		} else {
			fprintf(client, "Stopping server\n");
			fflush(client);
		}

		unlink(ACTIVITY_SERVER_NAME);

		exit(EXIT_SUCCESS);
	}

	if (strncmp(args[0], LIST_CMD, sizeof(LIST_CMD)) == 0) {
		const char * const* activities;
		int size, i;

		if (count != 1)
			CLIENT_ERR(client, "Too many arguments. Trimming command down to \
				   'activity %s'", LIST_CMD);
		size = hmi->get_supported_activities_list(hmi, &activities);
		if (client) {
			fprintf(client, "Activities list:\n");
			for (i = 0; i < size; i++)
				fprintf(client, "\t[%d] %s\n", i + 1, activities[i]);
		}

		return 0;
	}

	if (strncmp(args[0], REGISTER_CMD, sizeof(REGISTER_CMD)) == 0) {
		if (count != 1)
			CLIENT_ERR(client, "Too many arguments. Trimming command down to \
				   'activity %s'", REGISTER_CMD);
		register_activity_callback();

		return 0;
	}

	if (strncmp(args[0], ENABLE_CMD, sizeof(ENABLE_CMD)) == 0) {
		if (count > 3)
			CLIENT_ERR(client, "Too many arguments. Trimming command down to \
				   'activity %s %s %s'", ENABLE_CMD, args[1], args[2])
		else if (count != 3) {
			CLIENT_ERR(client, "Insufficient arguments. Command should be \
				   'activity %s <activity_handle> <event_type>'", ENABLE_CMD);
			return -1;
		}

		return enable_activity_event(atoi(args[1]), atoi(args[2]));
	}

	if (strncmp(args[0], DISABLE_CMD, sizeof(DISABLE_CMD)) == 0) {
		if (count > 3)
			CLIENT_ERR(client, "Too many arguments. Trimming command down to \
				   'activity %s %s %s'", DISABLE_CMD, args[1], args[2])
		else if (count != 3) {
			CLIENT_ERR(client, "Insufficient arguments. Command should be \
				   'activity %s <activity_handle> <event_type>'", DISABLE_CMD);
			return -1;
		}

		return disable_activity_event(atoi(args[1]), atoi(args[2]));
	}

	if (strncmp(args[0], FLUSH_CMD, sizeof(FLUSH_CMD)) == 0) {
		if (count != 1)
			CLIENT_ERR(client, "Too many arguments. Trimming command down to \
				   'activity %s'", FLUSH_CMD);

		return flush();
	}

	if (strncmp(args[0], MONITOR_START_CMD, sizeof(MONITOR_START_CMD)) == 0) {
		if (count != 1)
			CLIENT_ERR(client, "Too many arguments. Trimming command \
				   down to 'activity %s'", MONITOR_START_CMD);
		return MONITOR_START_FLAG;
	}

	if (strncmp(args[0], MONITOR_STOP_CMD, sizeof(MONITOR_STOP_CMD)) == 0) {
		if (count != 1)
			CLIENT_ERR(client, "Too many arguments. Trimming command \
				   down to 'activity %s'", MONITOR_STOP_CMD);
		return MONITOR_STOP_FLAG;
	}

	CLIENT_ERR(client, "Invalid command %s", buffer);

	return -1;
}

static void stop_monitoring(int *flag)
{
	int current_flag = ACK_FLAG;

	/* Send ACK to client */
	write(monitor_connection, &current_flag, sizeof(current_flag));

	/* Cleanup */
	fclose(monitor_client);
	close(monitor_connection);
	monitor_client		= NULL;
	monitor_connection	= -1;

	*flag = ACK_FLAG;
}

static void start_monitoring(int conn, int *flag)
{
	/* Stop other started monitor if that's the case */
	if (monitor_client != NULL)
		stop_monitoring(flag);

	monitor_client		= client;
	monitor_connection	= conn;
	*flag			= MONITOR_START_FLAG;

}

static void start_server(void)
{
	int sock, conn, ret;
	int flag;

	/* Just to make sure we do not have more than one server instance */
	unlink(ACTIVITY_SERVER_NAME);

	sock = socket(AF_UNIX, SOCK_SEQPACKET, 0);
	if (sock == -1) {
		ALOGE("Error %s creating socket\n", strerror(errno));
		exit(1);
	}

	ret = bind(sock, (struct sockaddr *) &server_addr, sizeof(server_addr));
	if (ret == -1) {
		ALOGE("Error %s binding socket\n", strerror(errno));
		exit(1);
	}

	ret = listen(sock, 1);
	if (ret == -1)
		ALOGW("Error %s setting socket to listening state\n", strerror(errno));

	/* Accept commands and send them to HAL */
	while (1) {
		char buffer[526], cmsg_buffer[526];
		struct sockaddr_un from;
		struct iovec recv_buff = {
			.iov_base	= buffer,
			.iov_len	= sizeof(buffer),
		};
		struct msghdr msg = {
			.msg_name	= &from,
			.msg_namelen	= sizeof(from),
			.msg_iov	= &recv_buff,
			.msg_iovlen	= 1,
			.msg_control	= cmsg_buffer,
			.msg_controllen	= sizeof(cmsg_buffer),
		};
		struct cmsghdr *cmsg;
		bool close_now = true;

		conn = accept(sock, NULL, NULL);
		if (conn == -1) {
			ALOGE("Error %s accepting connection\n", strerror(errno));
			continue;
		}

		ret = recvmsg(conn, &msg, 0);
		if (ret == -1) {
			ALOGE("Error %s in receiving message, conn = %d\n", strerror(errno), conn);
			close(conn);

			continue;
		}

		if (!ret)
			continue;

		/* Check for shutdown from the peer */
		if (ret == 0)
			break;

		for (cmsg = CMSG_FIRSTHDR(&msg); cmsg != NULL; cmsg = CMSG_NXTHDR(&msg, cmsg)) {
			if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SCM_RIGHTS) {
				int *received_fd = (int *)CMSG_DATA(cmsg);
				client = fdopen(*received_fd, "w");
				break;
			}
		}

		ret = parse_cmd(buffer);
		if (ret < 0) {
			close(conn);
			continue;
		}
		if (ret == MONITOR_START_FLAG) {
			start_monitoring(conn, &flag);
			close_now = false;
		} else if (ret == MONITOR_STOP_FLAG)
			stop_monitoring(&flag);
		else
			flag = ACK_FLAG;

		/* Confirm succesfull dispatch */
		write(conn, &flag, sizeof(flag));

		if (close_now) {
			fclose(client);
			close(conn);
		}
	}
}

static const char *hal_paths[] ={
	"./activity.gmin.so",
	"/lib/activity.gmin.so",
	"/system/lib/hw/activity.gmin.so",
};

static int start_hal(void)
{
	void *hal;
	int i, ret, no_paths;
	const char *path = NULL;
	pid_t child;

	no_paths = sizeof(hal_paths)/sizeof(const char*);
	for (i = 0; i < no_paths; i++) {
		if (access(hal_paths[i], R_OK) == 0) {
			path = hal_paths[i];
			break;
		}
	}

	if (!path) {
		fprintf(stderr, "Unable to find HAL\n");
		exit(1);
	}

	hal = dlopen(path, RTLD_NOW);
	if (!hal) {
		fprintf(stderr, "Error \"%s\" opening activity HAL\n", dlerror());
		return HAL_OPEN_ERR;
	}

	hmi = dlsym(hal, HAL_MODULE_INFO_SYM_AS_STR);
	if (!hmi) {
		fprintf(stderr, "Error \"%s\" finding entry symbol\n", dlerror());
		return HAL_SYMBOL_ERR;
	}

	printf("Activity HAL loaded: name %s vendor %s version %d.%d id %s\n",
	       hmi->common.name, hmi->common.author,
	       hmi->common.version_major, hmi->common.version_minor,
	       hmi->common.id);

	/* Daemonize it */
	child = fork();
	if (child) {
		usleep(100);
		return 0;
	}

	if (setsid() == (pid_t)-1) {
		fprintf(stderr, "failed to send process to background\n");
		exit(1);
	}

	/* Close stdio */
	close(0); close(1); close(2);

	ALOGI("Proceeding to HAL initialization\n");

	ret = hmi->common.methods->open((struct hw_module_t *) hmi,
					ACTIVITY_RECOGNITION_HARDWARE_INTERFACE,
					&device);
	if (ret) {
		ALOGE("Error %d occurred at HAL module opening\n", ret);
		exit(1);
	}

	start_server();

	return 0;
}

static int send_cmd(int argc, char **argv)
{
	char cmd[MAX_CMD_SIZE];
	int i, sock, ret, flag;
	struct iovec rcv_buffer = {
		.iov_base	= cmd,
		.iov_len	= sizeof(cmd) + 1,
	};
	struct cmsg_fd {
		struct cmsghdr hdr;
		int fd;
	} cmsg_buff = {
		.hdr = {
			.cmsg_level	= SOL_SOCKET,
			.cmsg_type	= SCM_RIGHTS,
			.cmsg_len	= CMSG_LEN(sizeof(int)),
		},
		.fd = 1,
	};
	struct msghdr msg = {
		.msg_name	= NULL,
		.msg_namelen	= 0,
		.msg_iov	= &rcv_buffer,
		.msg_iovlen	= 1,
		.msg_control	= &cmsg_buff,
		.msg_controllen	= sizeof(cmsg_buff),
	};

	strcpy(cmd, argv[1]);
	for (i = 2; i < argc; i++) {
		strncat(cmd, CMD_SEPARATOR, sizeof(CMD_SEPARATOR));
		strcat(cmd, argv[i]);
	}

	sock = socket(AF_UNIX, SOCK_SEQPACKET, 0);
	if (sock == -1) {
		fprintf(stderr, "Error \"%s\" creating socket\n", strerror(errno));
		return errno;
	}

	ret = connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr));
	if (ret == -1) {
		fprintf(stderr, "Error \"%s\" connecting to server\n", strerror(errno));
		return errno;
	}

	ret = sendmsg(sock, &msg, 0);
	if (ret == -1) {
		fprintf(stderr, "Error \"%s\" sending message to server\n", strerror(errno));
		return errno;
	}

	ret = read(sock, &flag, sizeof(flag));
	if (ret == -1) {
		fprintf(stderr, "Error \"%s\" getting answer from server\n", strerror(errno));
		return errno;
	}

	/* Check for ACK or monitoring */
	if (flag == MONITOR_START_FLAG) {
		do {
			ret = read(sock, &flag, sizeof(flag));
		} while (ret > 0 && flag != ACK_FLAG);
	} else if (flag != ACK_FLAG)
		fprintf(stderr, "Error answer from HAL server: %d. Check logs for more details\n", flag);

	close(sock);

	return 0;
}

static int run_cmd(int argc, char **argv)
{
	if (strncmp(argv[1], START_CMD, sizeof(START_CMD)) == 0) {
		if (argc == 2)
			return start_hal();
		return print_usage();
	}

	/* Send user command to HAL server socket */
	return send_cmd(argc, argv);
}

int main(int argc, char **argv)
{
	if (argc < 2)
		return print_usage();

	return run_cmd(argc, argv);
}
