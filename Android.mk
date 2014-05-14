#
# Copyright (C) 2014 Intel Corporation.
#

# IIO sensors HAL module implementation, compiled as hw/iio-sensors-hal.so

LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

src_path := .
src_files := $(src_path)/entry.c \
	     $(src_path)/enumeration.c \
	     $(src_path)/control.c \
	     $(src_path)/description.c \
	     $(src_path)/utils.c \
	     $(src_path)/transform.c \
	     $(src_path)/compass-calibration.c \
	     $(src_path)/matrix-ops.c \

LOCAL_C_INCLUDES += $(LOCAL_PATH) vendor/intel/hardware/iio-sensors
LOCAL_MODULE := sensors.gmin
LOCAL_MODULE_OWNER := intel
LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw
LOCAL_MODULE_TAGS := optional
LOCAL_CFLAGS := -DLOG_TAG=\"Sensors\" -fvisibility=hidden
LOCAL_LDFLAGS := -Wl,--gc-sections
LOCAL_SHARED_LIBRARIES := liblog libcutils libdl
LOCAL_PRELINK_MODULE := false
LOCAL_SRC_FILES := $(src_files)

include $(BUILD_SHARED_LIBRARY)

