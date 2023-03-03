iio sensors HAL documentation
_____________________________


PURPOSE OF THE IIO SENSORS HAL

This library links the Android sensors framework to the set of Linux sensors
drivers that expose a iio interface.

These layers are mostly documented here:
https://source.android.com/devices/sensors/hal-interface.html
 [basic tour of the Android sensors HAL interface]

http://source.android.com/devices/halref/sensors_8h_source.html
 [Android sensor details]

https://www.kernel.org/doc/Documentation/ABI/testing/sysfs-bus-iio
 [overview of the iio interface]


DESIGN GOALS

The iio sensors HAL is designed to drive a complete collection of sensors,
whose types and properties are discovered dynamically. It should be reusable
without modification across a variety of boards, avoiding creating custom
HALs over and over, and allowing quick sensors enabling on new hardware
platforms. It's meant to be small, simple, and have minimal CPU and memory
overhead.


FUNCTION

The HAL discovers the set of available sensors at startup, reports them to
Android, and performs basic operations on them:

- enable and disable sensors
- set the rate at which sensors should report events to Android
- await for samples and return them to Android in the format it expects

This is primarily done by reading and writing sysfs files located under
/sys/bus/iio, as well as interacting with /dev/iio:deviceX character devices.


SUPPORTED SENSOR TYPES

As of march 2015 the following sensor types are supported:

- accelerometer
- gyroscope
- magnetometer
- ambient light sensor
- temperature sensor
- proximity sensor
- step detector
- step counter


ENUMERATION

Basic enumeration happens by scanning /sys/bus/iio/devices. We search each of
the device subfolders for sysfs entries that we can associate to known sensor
types. Each of the iio devices can possibly support several sensors.
Of particular interest is the scan_elements subfolder, which we use to
determine if a specific sensor will be used in trigger mode (interrupt driven)
or in polled mode (sampling happens in response to sysfs reads).


EVENTS and POLLING

The preferred way to retrieve data from a sensor is though the associated iio
character device. That implies that the sensor can report events whenever new
samples become available. The iio sensor HAL opens a fd on each of the
/dev/iio:deviceX files associated to enabled sensors, and add these fds to a
fd set monitored by a single poll call, which blocks the HAL's main sensor data
polling function. From there we read "device reports" that can possibly hold
data for several sensors, and split that in "sensor reports" that we translate
into the format Android expects.

Another mode of operation is polling mode. It is engaged if no scan_elements
folder is found, or if no iio channels are detected for the sensor. In that
case we start a dedicated data acquisition thread for the sensor, which will
periodically read sysfs entries (either _raw or _input) to get sensor data.
This data is then transmitted to the main poll thread through Unix pipes,
whose reader end fds get added to the monitored fd set.


TRIGGERS

Triggers are the iio way of selecting when a iio driver should alert userspace
that there is new data available. At least one "data ready" trigger needs to be
exposed. These are expected to fire periodically, at a programmed sampling
rate, whenever the sensor driver acquires a new sample from hardware.

Another type of trigger we support is motion based ; if a motion trigger is
selected, the driver will avoid sending duplicate samples to userspace. In
practice, the sensors HAL only gets data when there the device position
changes. The iio sensor HAL has an internal "repeat last sample" logic for
sensor types from which Android expects to get data periodically, unbeknown
that layers below are only sending data on motion. We selected to engage this
mode only for low frequencies (~25 Hz or less) as certain games are sensitive
to the thresholding effects that motion triggers yield.


FDs and CHANNELS

The iio sensors HAL always open a fd on the iio device associated to an
enabled sensor. The assumption there is that the hardware should be powered
off unless a fd is open on the associated iio device. That is done even for
polled sensors.

If the iio device supports several sensors types we can handle, all recognized
channels get enabled at startup, and remain so. Although the iio interface
supports dynamic enabling and disabling of individual channels, doing so
changes the size of the "device reports" we read from the iio device fds, and
creates complicated synchronization issues in the data decoding code path of
the HAL, that we preferred to avoid for the time being.


TIMESTAMPS

Android associates timestamps to samples. These timestamps are expressed as
the time elapsed since the beginning of the boot sequence.

If possible, we read the iio timestamps from the timestamp channel, alongside
sample data ; these are closely correlated to the actual data acquisition time,
as they come from the driver, and possibly from hardware.


ORIENTATION MAPPING

The sensors HAL is able to interpret optional 'panel' and 'rotation' specifiers
from ACPI DSDT entries for sensors. See:
http://www.spinics.net/lists/linux-acpi/msg51540.html

It is possible to supersede these values using the .panel and .rotation
properties (both need to be specified, and they are read only once at boot
time).


UNIT CONVERSION

IIO and Android occasionally disagree on the units to use.
That is the case for:
- magnetic field strength: Tesla vs Gauss
- proximity

The HAL performs appropriate mappings.


OPTIONAL PROPERTIES

We support a variety of properties. Some convey user visible information which
is passed Android. Properties are expressed by sensor type, such as:
ro.iio.accel.name = "Intel Accelerometer"

On certain boards we may have several sensors of the same type. It's then
possible to specialize the name using the iio sysfs 'name' contents:
ro.iio.temp.bmg160.name = "BMG160 Thermometer".

If several properties match, the most specific form has higher priority.

All properties are optional. As of March 2015 the following properties are
supported:

.name         : user visible sensor name, passed to Android
.vendor       : user visible sensor manufacturer name, passed to Android
.resolution   : sensor measurement resolution, in Android units, passed to
                Android
.power        : sensor estimated power draw, in mA, presumably at 3.7V
.transform    : used to switch to the units used by early ISH drivers;
                deprecated
.max_freq     : specifies a cap for the sensor sampling rate
.min_freq     : specify a floor for the sensor sampling rate
.cal_steps    : specify the maximum attempted calibration level for the
                magnetometer
.illumincalib : specify a gain for certain ALS drivers ; passed through sysfs
.order        : allows reordering channels ; used internally ; deprecated
.quirks       : allows specifying various processing options ; see QUIRKS
.panel        : allows expressing/superseding the _PLD panel indicator
                (4=front, 5=back)
.rotation     : allows expressing/superseding the _PLD rotation indicator
                (x 45° clockwise)
.scale        : scaling/sensitivity hint for the driver, stored through sysfs
.opt_scale    : optional scaling applied at a late stage to channel values;
                deprecated
.filter       : allows selecting one of the available filters, and its strength


QUIRKS

One of the properties we use allows influencing how a specific sensor is used.
It's the 'quirks' property, and allows the HAL to compensate for hardware or
driver idiosyncrasies. Several quirks can be specified using commas to separate
them.

Available quirks are:

noisy         : engage default filter for the sensor type to smooth out noise
terse         : auto-repeat events as if the trigger was a motion trigger,
                even though it's not advertised as such
continuous    : disable use of motion trigger even if the sensor supports it
init-rate     : set sampling rate at 10 Hz after enabling that sensor
biased        : the sensor has unusually high bias ; engage high bias detection
                and compensation routines
spotty        : the sensor may have gaps in its events sequence; adjust
                timestamps accordingly
no-poll       : specifically disable the iio polling (sysfs) way of getting
                data from this driver, even if it's seemingly available
no-trig       : specifically disable the iio trigger way of getting data from
                this driver, even if it's seemingly available
no-event      : specifically disable the iio event way of getting data from
                this driver, even if it's seemingly available


FILTERING

Some sensor types are inherently low precision and provide data that is
statistically noisy. If the noisy quirk is specified, we apply a predetermined
filtering strategy depending on the sensor type, to smooth out the noise in
samples before they are passed to Android. That can add latency in the sensor
output. It's also possible to individually set a sensor filter selection and
strength through properties.

ro.iio.anglvel.filter = average

or

ro.iio.anglvel.filter = average, 10


CALIBRATION

Calibration is a different concept from filtering. It has a different meaning
depending on the sensor type.


UNCALIBRATED SENSORS

Android 4.4 (KitKat) introduced the uncalibrated gyroscope and uncalibrated
magnetometer sensor types. They are virtual sensors that decouple the sensor
data from the correction that is applied to it by the HAL, so upper layers can
choose to alter or ignore the correction that got applied at the HAL level.


VIRTUAL SENSORS

The HAL can expose logical sensors, such as the uncalibrated gyroscope, in
addition to the set of iio sensors. These are built on top of base sensors.
The Android framework can add its own virtual sensors too. Those are typically
composite (fusion) sensors, relying on several base sensors for their work.
The current Android code for this, as of Android 5.0, sets the gyroscope at
200 Hz, the magnetometer at 50 Hz, and the accelerometer at the target
frequency for the virtual sensor.


SAMPLING RATE

Arbitration levels and iio device collocation, virtual sensors, per sensor
rate, Android level arbitration, published rates.


TRANSFORMS

The transform property got used to support early iio drivers for the Intel
Sensor Hub on Haswell machines. It should no longer be used and support for
it may be deleted in the future.


SOURCE TREE

The most central source files are:

common.h       : definitions shared among all files
entry.c        : iio sensors HAL entry points
enumeration.c  : sensor enumeration routines
control.c      : enabling, disabling, sampling rate control


THREADS

The sensors HAL code runs in the context of the calling threads (Android Sensor
Service threads, from the Service Manager process). It spawns one additional
thread per polling sensor in use though. This thread communicates its data to
the single polling thread through a pipe, whose fd is added to the set of fds
the polling thread waits on.


BATCHING


DRIVER DESIDERATA

- one iio device per sensor
- interrupt driven
- no jitter
- sampling frequency per sensor
- timestamp channel
- fast stabilization time on enabling


MISC

- ueventd.rc file access rights
- iio-sensors-hal.so (IRDA autodetect) vs sensors.gmin.so (GMIN)
- .conf files, persistency
- code writing convention


HISTORICAL PERSPECTIVE
- Star Peak on XPS 12, Harris Beach, T100
- GMIN MRD 7, Anchor 8
- IRDA ECS, ECS 2, CHIPHD, Malata
- SoFIA
- ISH


AUTO-DETECTION

Multi-device targets (coho/cohol) rely on the hardware auto-detection daemon
(hald) to set properties for the enumerated sensors. hald listens for uevents
that get sent by the kernel during system startup, and matches them against a
list of known sensor parts. This list is built from the set of HAL record files
located under /system/etc/hald/hrec.d. Whenever a match is found, the
properties defined in the sensor's record file are set. Additional actions,
such as installing permission files, are possible. For targets that don't rely
on autodetection, sensor properties are set in system init scripts.

The following commands may be useful (run as root):

halctl -l                      : lists detected devices
halctl -g sensors              : identifies the selected sensors HAL library
getprop | grep iio             : list sensor properties
pm list features | grep sensor : list sensor features, as defined through
                                 permission files
logcat | grep -i sensor        : get sensor traces


TIPS AND TRICKS

The iio sensors HAL .so file is stored on a read-only partition, under
/system/lib/hw. Quick testing of code changes can be done using the following
commands:

mmm
adb root
adb remount
adb pull
adb push
adb reboot
adb shell

ALOGV traces are compiled out ; you may want to redefine ALOGV in common.h in
order to get them.
