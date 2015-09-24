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
#include <sys/socket.h>
#include <sys/un.h>
#include <dlfcn.h>
#include <pthread.h>
#include <errno.h>
#include <signal.h>
#include <string.h>

#include <hardware/sensors.h>
#include <utils/Log.h>

int usage(void)
{
	fprintf(stderr, "sens start [sensors.gmin.so]\n");
	fprintf(stderr, "sens [activate | deactivate] sensor_id\n");
	fprintf(stderr, "sens set_delay sensor_id delay\n");
	fprintf(stderr, "sens poll\n");
	fprintf(stderr, "sens poll [duration] [number_of_events] \n");
	fprintf(stderr, "sens poll_stop\n");
	fprintf(stderr, "sens check_sample_rate [rate] \n");
	return 1;
}

static struct sensors_module_t *hmi;

static const char* types[] = {
	"metadata",
	"accelerometer",
	"magnetometer",
	"orientation",
	"gyroscope",
	"light",
	"pressure",
	"temperature",
	"proximity",
	"gravity",
	"linear acceleration",
	"rotation vector",
	"relative humitidy",
	"ambient temperature",
	"uncalibrated magnetometer",
	"game rotation vector",
	"uncalibrated gyrocope",
	"significant motion",
	"step detector",
	"step counter",
	"geomagnetic rotation vector",
};

static const char *type_str(int type)
{
	int type_count = sizeof(types)/sizeof(char *);

	if (type < 0 || type >= type_count)
		return "unknown";
	return types[type];
}


static struct sensors_module_t *hmi;
static struct hw_device_t *dev;
static FILE *client;
static pthread_mutex_t client_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
static int ready_to_close = 0;
static int number_of_events = 0;
static int non_param_poll = 1;
static int event_no = 0;
static int init_events = 0;
static int print_events = 1;
static long long timestamp = 0;
static long long event_init_poll_time = 0;
static long long poll_duration = 0;

static void print_event(struct sensors_event_t *e)
{
	FILE *f;

	pthread_mutex_lock(&client_mutex);
	if (!client) {
		pthread_mutex_unlock(&client_mutex);
		return;
	}
	f = client;

	fprintf(f, "event %d: version=%d sensor=%d type=%s timestamp=%lld\n",event_no,
		e->version, e->sensor, type_str(e->type), (long long)e->timestamp);
	if (poll_duration != 0)
		fprintf(f,"Time remaining:%lld \n",poll_duration - ((long long)e->timestamp
			- event_init_poll_time));
	switch (e->type) {
	case SENSOR_TYPE_META_DATA:
		break;
	case SENSOR_TYPE_ACCELEROMETER:
	case SENSOR_TYPE_LINEAR_ACCELERATION:
	case SENSOR_TYPE_GRAVITY:
		fprintf(f, "event: x=%10.2f y=%10.2f z=%10.2f status=%d\n",
			e->acceleration.x, e->acceleration.y, e->acceleration.z,
			e->acceleration.status);
		break;
	case SENSOR_TYPE_MAGNETIC_FIELD:
		fprintf(f, "event: x=%10.2f y=%10.2f z=%10.2f status=%d\n",
			e->magnetic.x, e->magnetic.y, e->magnetic.z,
			e->magnetic.status);
		break;
	case SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED:
		fprintf(f, "event: x=%10.2f y=%10.2f z=%10.2f bias_x=%10.2f bias_y=%10.2f bias_z=%10.2f \n",
			e->uncalibrated_magnetic.x_uncalib,
			e->uncalibrated_magnetic.y_uncalib,
			e->uncalibrated_magnetic.z_uncalib,
			e->uncalibrated_magnetic.x_bias,
			e->uncalibrated_magnetic.y_bias,
			e->uncalibrated_magnetic.z_bias);
		break;
	case SENSOR_TYPE_ORIENTATION:
		fprintf(f, "event: azimuth=%10.2f pitch=%10.2f roll=%10.2f status=%d\n",
			e->orientation.azimuth, e->orientation.pitch, e->orientation.roll,
			e->orientation.status);
		break;
	case SENSOR_TYPE_GYROSCOPE:
		fprintf(f, "event: x=%10.2f y=%10.2f z=%10.2f status=%d\n",
			e->gyro.x, e->gyro.y, e->gyro.z, e->gyro.status);
		break;
	case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
		fprintf(f, "event: x=%10.2f y=%10.2f z=%10.2f bias_x=%10.2f bias_y=%10.2f bias_z=%10.2f \n",
			e->uncalibrated_gyro.x_uncalib,
			e->uncalibrated_gyro.y_uncalib,
			e->uncalibrated_gyro.z_uncalib,
			e->uncalibrated_gyro.x_bias,
			e->uncalibrated_gyro.y_bias,
			e->uncalibrated_gyro.z_bias);
		break;
	case SENSOR_TYPE_LIGHT:
		fprintf(f, "event: light=%10.2f\n", e->light);
		break;
	case SENSOR_TYPE_PRESSURE:
		fprintf(f, "event: pressure=%10.2f\n", e->pressure);
		break;
	case SENSOR_TYPE_TEMPERATURE:
	case SENSOR_TYPE_AMBIENT_TEMPERATURE:
		fprintf(f, "event: temperature=%10.2f\n", e->temperature);
		break;
	case SENSOR_TYPE_PROXIMITY:
		fprintf(f, "event: distance=%10.2f\n", e->distance);
		break;
	case SENSOR_TYPE_ROTATION_VECTOR:
	case SENSOR_TYPE_GAME_ROTATION_VECTOR:
	case SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR:
		fprintf(f, "event: rot_x=%10.2f rot_y=%10.2f rot_z=%10.2f cos=%10.2f estimated_accuracy=%10.2f\n",
			e->data[0], e->data[1], e->data[2], e->data[3], e->data[4]);
		break;
	case SENSOR_TYPE_RELATIVE_HUMIDITY:
		fprintf(f, "event: humidity=%10.2f\n", e->relative_humidity);
		break;
	case SENSOR_TYPE_SIGNIFICANT_MOTION:
		fprintf(f, "event: significant_motion=%10.2f\n", e->data[0]);
		break;
	case SENSOR_TYPE_STEP_DETECTOR:
		fprintf(f, "event: step_detector=%10.2f\n", e->data[0]);
		break;
	case SENSOR_TYPE_STEP_COUNTER:
		fprintf(f, "event: step_counter=%llu\n",
			(unsigned long long)e->u64.step_counter);
		break;
	}
	fprintf(f, "\n");
	fflush(f);

	pthread_mutex_unlock(&client_mutex);
}

static void print_result(int result)
{
	FILE *f;
	pthread_mutex_lock(&client_mutex);
	if (!client) {
		pthread_mutex_unlock(&client_mutex);
		return;
	}
	f = client;
	fprintf(f, "Number of events: %d \n", event_no - init_events);
	fprintf(f, "Duration: %lld \n\n", (long long) timestamp - event_init_poll_time);
	if(!print_events){
		if(result)
			fprintf(f, "Test passed\n\n");
		else
			fprintf(f, "Test failed\n\n");
	}
	fflush(f);
	pthread_mutex_unlock(&client_mutex);

}

static void process_event(struct sensors_event_t *e)
{
	int is_poll_duration_over = 0;
	int is_event_number_reached = 0;

	if (event_init_poll_time == 0) {
		event_init_poll_time = (long long) e->timestamp;
		init_events = event_no;
	}
	is_poll_duration_over = (long long) e->timestamp - event_init_poll_time <= poll_duration ? 0 : 1;
	is_event_number_reached = (event_no - init_events) < number_of_events ? 0 : 1;

	if ((!is_poll_duration_over && !is_event_number_reached) || non_param_poll)
	{
		timestamp = e -> timestamp;
		event_no++;
		if(print_events)
			print_event(e);
	} else {
		ready_to_close = 1;
		print_result(is_event_number_reached);
		pthread_cond_signal(&cond);
	}
}

static void run_sensors_poll_v0(void)
{
	struct sensors_poll_device_t *poll_dev = (struct sensors_poll_device_t *)dev;

	while (1) {
		sensors_event_t events[256];
		int i, count;

		count = poll_dev->poll(poll_dev, events, sizeof(events)/sizeof(sensors_event_t));

		for(i = 0; i < count; i++)
			process_event(&events[i]);
	}
}

static void sig_pipe(int sig)
{
	client = NULL;
}

static void *run_sensors_thread(void *arg __attribute((unused)))
{

	signal(SIGPIPE, sig_pipe);

	switch (dev->version) {
	case SENSORS_DEVICE_API_VERSION_0_1:
	default:
		run_sensors_poll_v0();
		break;
	}

	return NULL;
}

void print_sensor(const struct sensor_t *s, FILE *f)
{
	if (!f)
		return;

	fprintf(f, "sensor%d: name=%s vendor=%s version=%d type=%s\n",
		s->handle, s->name, s->vendor, s->version, type_str(s->type));
	fprintf(f, "sensor%d: maxRange=%10.2f resolution=%10.2f power=%10.2f\n",
		s->handle, s->maxRange, s->resolution, s->power);
	fprintf(f, "sensor%d: minDelay=%d fifoReservedEventCount=%d fifoMaxEventCount=%d\n",
		s->handle, s->minDelay, s->fifoReservedEventCount,
		s->fifoMaxEventCount);

}

static int sensor_set_delay(int handle, int64_t delay)
{
	switch (dev->version) {
	default:
	case SENSORS_DEVICE_API_VERSION_0_1:
	{
		struct sensors_poll_device_t *poll_dev = (struct sensors_poll_device_t *)dev;

		return poll_dev->setDelay(poll_dev, handle, delay);
	}
	}
}


static int sensor_activate(int handle, int enable)
{
	switch (dev->version) {
	default:
	case SENSORS_DEVICE_API_VERSION_0_1:
	{
		struct sensors_poll_device_t *poll_dev = (struct sensors_poll_device_t *)dev;

		return poll_dev->activate(poll_dev, handle, enable);
	}
	}
}

#define CLIENT_ERR(f, fmt...)			\
	{ if (f) { fprintf(f, fmt); fprintf(f, "\n"); } ALOGE(fmt); }

static int dispatch_cmd(char *cmd, FILE *f)
{
	char *argv[16], *tmp;
	int argc = 0, handle;

	tmp = strtok(cmd, " ");
	while (tmp) {
		argv[argc++] = tmp;
		tmp = strtok(NULL, " ");
	}
	if (!argc)
		argv[argc++] = tmp;

	if (argc < 1) {
		CLIENT_ERR(f, "invalid cmd: %s", cmd);
		return -1;
	}

	if (!strcmp(argv[0], "ls")) {
		struct sensor_t const* list;
		int i, count = hmi->get_sensors_list(hmi, &list);

		for(i = 0; i < count; i++)
			print_sensor(&list[i], f);;

		return 0;
	} else if (!strcmp(argv[0], "activate")) {

		if (argc < 2) {
			CLIENT_ERR(f, "activate: no sensor handle");
			return -1;
		}

		handle = atoi(argv[1]);

		return sensor_activate(handle, 1);

	} else if (!strcmp(argv[0], "deactivate")) {

		if (argc < 2) {
			CLIENT_ERR(f, "activate: no sensor handle");
			return -1;
		}

		handle = atoi(argv[1]);

		return sensor_activate(handle, 0);

	} else if (!strcmp(argv[0], "set_delay")) {
		int64_t delay;

		if (argc < 3) {
			CLIENT_ERR(f, "setDelay: no sensor handle and/or delay");
			return -1;
		}

		handle=atoi(argv[1]);
		delay=atoll(argv[2]);

		return sensor_set_delay(handle, delay);

	} else if (!strcmp(argv[0], "poll")) {
		if (argc == 1) {
			non_param_poll = 1;
		} else if (argc == 3) {
			non_param_poll = 0;
			poll_duration = atoll(argv[1]);
			number_of_events = atoi(argv[2]);
			event_init_poll_time = 0;
			ready_to_close = 0;
		} else {
			CLIENT_ERR(f, "poll: no poll duration or number of events set");
			return -1;
		}
		print_events = 1;
		pthread_mutex_lock(&client_mutex);
		if (client)
			fclose(client);
		client = f;

		if (!non_param_poll) {
			pthread_cond_wait(&cond, &client_mutex);
			fclose(client);
			client = NULL;
		}

		pthread_mutex_unlock(&client_mutex);

		return 1;
	} else if (!strcmp(argv[0], "check_sample_rate")) {

		if (argc < 2) {
			CLIENT_ERR(f, "check_sample_rate: no events rate");
			return -1;
		}

		non_param_poll = 0;
		poll_duration = 1000000000;
		number_of_events = atoi(argv[1]);
		event_init_poll_time = 0;
		ready_to_close = 0;
		print_events = 0;

		pthread_mutex_lock(&client_mutex);
		if (client)
			fclose(client);
		client = f;
		pthread_cond_wait(&cond, &client_mutex);
		fclose(client);
		client = NULL;
		pthread_mutex_unlock(&client_mutex);
		return 1;
	} else if (!strcmp(argv[0], "poll_stop")) {
		pthread_mutex_lock(&client_mutex);
		if (client){
			fclose(client);
			client = NULL;
		}
		pthread_mutex_unlock(&client_mutex);

		return 1;
	} else if (!strcmp(argv[0], "stop")) {
		exit(1);
	} else {
		CLIENT_ERR(f, "invalid command: %s", cmd);
		return -1;
	}

}

#ifdef ANDROID
#define NAME_PREFIX "/dev/socket/"
#else
#define NAME_PREFIX "/tmp/"
#endif

#define SENS_SERVER_NAME NAME_PREFIX "sens-server"

struct sockaddr_un server_addr = {
	.sun_family = AF_UNIX,
	.sun_path = SENS_SERVER_NAME,
};

static int start_server(void)
{
	int sock = socket(AF_UNIX, SOCK_SEQPACKET, 0), conn;
	int err;

	unlink(SENS_SERVER_NAME);

	if (sock < 0) {
		ALOGE("failed to create socket: %s", strerror(errno));
		exit(1);
	}

	err = bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr));
	if (err) {
		ALOGE("failed to bind socket: %s", strerror(errno));
		exit(1);
	}

	listen(sock, 1);

	while (1) {
		char data_buff[1024], cmsg_buffer[1024];
		struct iovec recv_buff = {
			.iov_base = data_buff,
			.iov_len = sizeof(data_buff),
		};
		struct sockaddr_un from;
		struct msghdr msg = {
			.msg_name = &from,
			.msg_namelen = sizeof(from),
			.msg_iov = &recv_buff,
			.msg_iovlen = 1,
			.msg_control = cmsg_buffer,
			.msg_controllen = sizeof(cmsg_buffer),
		};
		FILE *f =NULL;
		struct cmsghdr *cmsg;

		conn = accept(sock, NULL, NULL);
		if (conn < 0) {
			ALOGE("failed to accept connection: %s", strerror(errno));
			continue;
		}

		err = recvmsg(conn, &msg, 0);
		if (err < 0) {
			ALOGE("error in recvmsg: %s", strerror(errno));
			close(conn);
			continue;
		}

		if (err == 0)
			continue;

		for (cmsg = CMSG_FIRSTHDR(&msg); cmsg != NULL;
		     cmsg = CMSG_NXTHDR(&msg,cmsg)) {
			if (cmsg->cmsg_level == SOL_SOCKET
			    && cmsg->cmsg_type == SCM_RIGHTS) {
				int *fd = (int *)CMSG_DATA(cmsg);
				f = fdopen(*fd, "w");
				break;
			}
		}

		if (data_buff[err - 1] != 0) {
			ALOGE("command is not NULL terminated\n");
			close(conn);
			continue;
		}

		err = dispatch_cmd(data_buff, f);
		if (err < 0) {
			ALOGE("error dispatching command: %d", err);
			close(conn);
			continue;
		}

		/* send ack */
		if (!err) {
			write(conn, data_buff, 1);
			fclose(f);
		}

		close(conn);
	}
}

static const char *hal_paths[] = {
	"/system/lib/hw/sensors.gmin.so",
	"sensors.gmin.so",
	"/lib/sensors.gmin.so",
};

static int start_hal(int argc, char **argv)
{
	void *hal;
	pid_t child;
	int err;
	pthread_t sensors_thread;
	const char *hal_path = NULL;

	if (argc == 2) {
		unsigned i;

		for(i = 0; i < sizeof(hal_paths)/sizeof(const char*); i++) {
			if (!access(hal_paths[i], R_OK)) {
				hal_path = hal_paths[i];
				break;
			}
		}

		if (!hal_path) {
			fprintf(stderr, "unable to find HAL\n");
			exit(1);
		}
	} else
		hal_path = argv[2];

	hal = dlopen(hal_path, RTLD_NOW);
	if (!hal) {
		fprintf(stderr, "unable to load HAL %s: %s\n", hal_path,
			dlerror());
		return 2;
	}

	hmi = dlsym(hal, HAL_MODULE_INFO_SYM_AS_STR);
	if (!hmi) {
		fprintf(stderr, "unable to find %s entry point in HAL\n",
			HAL_MODULE_INFO_SYM_AS_STR);
		return 3;
	}

	printf("HAL loaded: name %s vendor %s version %d.%d id %s\n",
	       hmi->common.name, hmi->common.author,
	       hmi->common.version_major, hmi->common.version_minor,
	       hmi->common.id);

	child = fork();
	if (child) {
		usleep(100);
		return 0;
	}

	if (setsid() == (pid_t)-1) {
		fprintf(stderr, "failed to send process to background\n");
		exit(1);
	}

	close(0); close(1); close(2);

	ALOGI("Initializing HAL");

	err = hmi->common.methods->open((struct hw_module_t *)hmi,
					SENSORS_HARDWARE_POLL, &dev);

	if (err) {
		ALOGE("failed to initialize HAL: %d\n", err);
		exit(1);
	}

	if (pthread_create(&sensors_thread, NULL, run_sensors_thread, NULL)) {
		ALOGE("failed to create sensor thread");
		exit(1);
	}

	return start_server();
}

int main(int argc, char **argv)
{
	char cmd[1024];
	int sock, i;
	struct iovec buff = {
		.iov_base = cmd,
	};
	struct cmsg_fd {
		struct cmsghdr hdr;
		int fd;
	}  cmsg_buff = {
		.hdr = {
			.cmsg_level = SOL_SOCKET,
			.cmsg_type = SCM_RIGHTS,
			.cmsg_len = CMSG_LEN(sizeof(int)),
		},
		.fd = 1,
	};
	struct msghdr msg = {
		.msg_name = NULL,
		.msg_namelen = 0,
		.msg_iov = &buff,
		.msg_iovlen = 1,
		.msg_control = &cmsg_buff,
		.msg_controllen = sizeof(cmsg_buff),
	};


	if (argc < 2) {
		usage();
		return 1;
	}

	if (!strcmp(argv[1], "start")) {
		if (argc < 2)
			return usage();

		return start_hal(argc, argv);
	}

	if (strlen(argv[1]) >= sizeof(cmd))
		return usage();
	strncpy(cmd, argv[1], sizeof(cmd) - 1);
	strncat(cmd, " ", sizeof(cmd) - strlen(cmd) - 1);
	for(i = 2; i < argc; i++) {
		strncat(cmd, argv[i], sizeof(cmd) - strlen(cmd) - 1);
		strncat(cmd, " ", sizeof(cmd) - strlen(cmd) - 1);
	}

	sock = socket(AF_UNIX, SOCK_SEQPACKET, 0);
	if (!sock) {
		fprintf(stderr, "failed to create socket: %s\n", strerror(errno));
		return 3;
	}

	if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
		fprintf(stderr, "failed to connect to server: %s\n", strerror(errno));
		return 5;
	}

	buff.iov_len = strlen(cmd) + 1;
	if (sendmsg(sock, &msg, 0) < 0) {
		fprintf(stderr, "failed sending command to server: %s\n", strerror(errno));
		return 6;
	}

	buff.iov_len = sizeof(cmd);
	if (read(sock, cmd, 1) < 0) {
		fprintf(stderr, "failed getting ack from server: %s\n", strerror(errno));
		return 7;
	}

	close(sock);

	return 0;
}
