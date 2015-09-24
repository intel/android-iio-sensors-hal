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

#include <stdarg.h>
#include <syslog.h>

static int log;

int __android_log_print(int prio, const char *tag, const char *fmt, ...)
{
	va_list args;
	int loglevel;

	if (!log) {
		openlog("android", LOG_CONS, LOG_DAEMON);
		log = 1;
	}

	switch (prio) {
	default:
	case 0:
	case 1:
		loglevel = LOG_INFO;
		break;
	case 2:
	case 3:
		loglevel = LOG_DEBUG;
		break;
	case 4:
		loglevel = LOG_NOTICE;
		break;
	case 5:
		loglevel = LOG_WARNING;
		break;
	case 6:
		loglevel = LOG_ERR;
		break;
	case 7:
		loglevel = LOG_EMERG;
		break;
	}

	va_start(args, fmt);
	vsyslog(loglevel, fmt, args);
	va_end(args);

	return 0;
}
