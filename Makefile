LIBHARDWARE?=../../../../hardware/libhardware/
CFLAGS=-DLOG_TAG=\"sens\" -I$(LIBHARDWARE)include/ -I./linux -fPIC
LDFLAGS=-ldl -lpthread -lm -lrt

all: sensors-hal.so sens

linux_src = linux/log.o

sens: sens.o $(linux_src)
	cc -o $@ $^ $(LDFLAGS)

hal_src = entry.c enumeration.c control.c description.c utils.c transform.c compass-calibration.c matrix-ops.c \

sensors-hal.so: $(patsubst %.c,%.o,$(hal_src) $(linux_src))
	cc -o $@ $^ $(LDFLAGS) -shared

clean:
	-rm $(patsubst %.c,%.o,$(hal_src) $(linux_src) sens.c) sens sensors-hal.so 2>/dev/null
