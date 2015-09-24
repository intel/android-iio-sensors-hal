# Copyright (c) 2015 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
