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

#ifndef __TRANSFORM_H__
#define __TRANSFORM_H__

#include "common.h"

#define CONVERT_GAUSS_TO_MICROTESLA(x)	((x) * 100)
#define CONVERT_MICROTESLA_TO_GAUSS(x)	((x) / 100)

void	select_transform	(int s);
float	acquire_immediate_float_value	(int s, int c);
uint64_t acquire_immediate_uint64_value	(int s, int c);

#endif



