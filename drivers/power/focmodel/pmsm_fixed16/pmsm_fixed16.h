/****************************************************************************
 * drivers/power/focmodel/fixed16_model/focmodel_fixed16.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __DRIVERS_POWER_FOCMODEL_FOCMODEL_FIXED16_H
#define __DRIVERS_POWER_FOCMODEL_FOCMODEL_FIXED16_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/power/focmodel/focmodel_typespec.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* FOC model driver fixed16 specific logic */

extern struct focmodel_typespec_s g_focmodel_typespec_b16;

#endif /* __DRIVERS_POWER_FOCMODEL_FOCMODEL_FIXED16_H */
