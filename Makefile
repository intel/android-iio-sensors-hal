USE_IIO_SENSOR_HAL := true
USE_IIO_ACTIVITY_RECOGNITION_HAL := true
include Android.mk

LIBHARDWARE?=../../../../hardware/libhardware/
CFLAGS=-DLOG_TAG=\"sens\" -I$(LIBHARDWARE)include/ -I./linux -fPIC -Wall
LDFLAGS=-ldl -lpthread -lm -lrt

all: sensors.gmin.so sens activity_recognition.gmin.so activity

linux_src = linux/log.o

sens: sens.o $(linux_src)
	cc -o $@ $^ $(LDFLAGS)

activity: activity.o $(linux_src)
	cc -o $@ $^ $(LDFLAGS)

sensors.gmin.so: $(patsubst %.c,%.o,$(src_files) $(linux_src))
	cc -o $@ $^ $(LDFLAGS) -shared

activity_recognition.gmin.so: $(patsubst %.c,%.o,$(activity_src_files) $(linux_src))
	cc -o $@ $^ $(LDFLAGS) -shared

clean:
	-rm $(patsubst %.c,%.o,$(src_files) $(activity_src_files) $(linux_src) sens.c activity.c) sens sensors.gmin.so activity activity_recognition.gmin.so 2>/dev/null
