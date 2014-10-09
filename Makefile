USE_INTEL_SENSOR_HAL := true
include Android.mk

LIBHARDWARE?=../../../../hardware/libhardware/
CFLAGS=-DLOG_TAG=\"sens\" -I$(LIBHARDWARE)include/ -I./linux -fPIC -Wall
LDFLAGS=-ldl -lpthread -lm -lrt

all: sensors.gmin.so sens

linux_src = linux/log.o

sens: sens.o $(linux_src)
	cc -o $@ $^ $(LDFLAGS)

sensors.gmin.so: $(patsubst %.c,%.o,$(src_files) $(linux_src))
	cc -o $@ $^ $(LDFLAGS) -shared

clean:
	-rm $(patsubst %.c,%.o,$(src_files) $(linux_src) sens.c) sens sensors.gmin.so 2>/dev/null
