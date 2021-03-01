/****************************************************************************
 * include/nuttx/power/foc/foc_types.h
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

#ifndef __INCLUDE_NUTTX_DRIVERS_POWER_FOC_FOC_TYPES_H
#define __INCLUDE_NUTTX_DRIVERS_POWER_FOC_FOC_TYPES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_POWER_FOC_USE_FLOAT
#  include <dsp.h>
#endif
#ifdef CONFIG_POWER_FOC_USE_FIXED16
#  include <dspb16.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* For now only 3-phase devices are supported */

#if CONFIG_POWER_FOC_PHASES != 3
#  error Only 3-phase supported for now
#endif

/* Supported FOC float32 devices */

#ifndef CONFIG_POWER_FOC_FLOAT_INST
#  define CONFIG_POWER_FOC_FLOAT_INST   (0)
#endif

/* Supported FOC fixed16 devices */

#ifndef CONFIG_POWER_FOC_FIXED16_INST
#  define CONFIG_POWER_FOC_FIXED16_INST (0)
#endif

/* All supported FOC devices */

#define CONFIG_POWER_FOC_INST (CONFIG_POWER_FOC_FLOAT_INST +   \
                               CONFIG_POWER_FOC_FIXED16_INST)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* FOC number type identifiers */

enum foc_number_type_e
{
  FOC_NUMBER_TYPE_INVALID = 0,
#ifdef CONFIG_POWER_FOC_USE_FLOAT
  FOC_NUMBER_TYPE_FLOAT   = 1,   /* float */
#endif
#ifdef CONFIG_POWER_FOC_USE_FIXED16
  FOC_NUMBER_TYPE_FIXED16 = 2,   /* b16_t */
#endif
};

/* FOC number */

union foc_number_u
{
#ifdef CONFIG_POWER_FOC_USE_FLOAT
  float f32;
#endif
#ifdef CONFIG_POWER_FOC_USE_FIXED16
  b16_t b16;
#endif
};
typedef union foc_number_u foc_number_t;

/* FOC dq-frame */

union foc_dqframe_u
{
#ifdef CONFIG_POWER_FOC_USE_FLOAT
  dq_frame_f32_t f32;
#endif
#ifdef CONFIG_POWER_FOC_USE_FIXED16
  dq_frame_b16_t b16;
#endif
};
typedef union foc_dqframe_u foc_dqframe_t;

/* FOC ab-frame */

union foc_abframe_u
{
#ifdef CONFIG_POWER_FOC_USE_FLOAT
  ab_frame_f32_t f32;
#endif
#ifdef CONFIG_POWER_FOC_USE_FIXED16
  ab_frame_b16_t b16;
#endif
};
typedef union foc_abframe_u foc_abframe_t;

/* FOC abc-frame */

union foc_abcframe_u
{
#ifdef CONFIG_POWER_FOC_USE_FLOAT
  abc_frame_f32_t f32;
#endif
#ifdef CONFIG_POWER_FOC_USE_FIXED16
  abc_frame_b16_t b16;
#endif
};
typedef union foc_abcframe_u foc_abcframe_t;

/* FOC phase angle */

union foc_angle_u
{
#ifdef CONFIG_POWER_FOC_USE_FLOAT
  struct phase_angle_f32_s f32;
#endif
#ifdef CONFIG_POWER_FOC_USE_FIXED16
  struct phase_angle_b16_s b16;
#endif
};
typedef union foc_angle_u foc_angle_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __INCLUDE_NUTTX_DRIVERS_POWER_FOC_FOC_TYPES_H */
