/****************************************************************************
 * drivers/power/foc/float/foc_float.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>

#include "foc_float.h"

/****************************************************************************
 * External Definitions
 ****************************************************************************/

#ifdef CONFIG_POWER_FOC_CONTROL_PI
extern struct foc_typespec_control_s    g_foc_control_pi_f32;
#endif
#ifdef CONFIG_POWER_FOC_MODULATION_SVM3
extern struct foc_typespec_modulation_s g_foc_modulation_svm3_f32;
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int foc_f32_configure(FAR struct foc_dev_s *dev,
                             FAR struct foc_cfgcmn_s *cfg);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Instnace counter */

static uint8_t g_foc_f32_inst_now = 0;

/* FOC driver b16 specific logic */

static struct foc_typespec_s
  g_foc_typespec_f32[CONFIG_POWER_FOC_FLOAT_INST];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: foc_f32_configure
 ****************************************************************************/

static int foc_f32_configure(FAR struct foc_dev_s *dev,
                             FAR struct foc_cfgcmn_s *cfg)
{
  FAR struct foc_typespec_modulation_s *m   = NULL;
  FAR struct foc_typespec_control_s    *c   = NULL;
  int                                   ret = OK;

  /* Get modulation */

  switch (cfg->modulation)
    {
#ifdef CONFIG_POWER_FOC_MODULATION_SVM3
      case FOC_CFG_MODULATION_SVM3:
        {
          m = &g_foc_modulation_svm3_f32;
          break;
        };
#endif

      default:
        {
          pwrerr("ERROR: unsupported f32 modulation %d\n", cfg->modulation);
          ret = -EINVAL;
          goto errout;
        }
    }

  /* Get controller */

  switch (cfg->controller)
    {
#ifdef CONFIG_POWER_FOC_CONTROL_PI
      case FOC_CFG_CONTROLLER_PI:
        {
          c = &g_foc_control_pi_f32;
          break;
        };
#endif

      default:
        {
          pwrerr("ERROR: unsupported f32 controller %d\n", cfg->controller);
          ret = -EINVAL;
          goto errout;
        }
    }

  /* Connect controller */

  dev->typespec->control = c;

  /* Connect modulation */

  dev->typespec->modulation = m;

errout:
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: foc_typespec_f32_init
 ****************************************************************************/

int foc_typespec_f32_init(FAR struct foc_dev_s *dev)
{
  int ret = OK;

  /* Check if the device instance is supported by the driver.
   * Don't increase type-specific instance counter just yet.
   */

  if (g_foc_f32_inst_now > CONFIG_POWER_FOC_FLOAT_INST)
    {
      pwrerr("ERROR: unsupported f32 instance %d > %d\n",
             g_foc_f32_inst_now, CONFIG_POWER_FOC_FLOAT_INST);
      set_errno(ENODEV);
      ret = ERROR;
      goto errout;
    }

  /* Store type-specific instance counter */

  dev->info.typeno = g_foc_f32_inst_now;

  /* Connect data with device */

  dev->typespec = &g_foc_typespec_f32[dev->info.typeno];

  /* Connect configuration handler */

  dev->typespec->configure = foc_f32_configure;

  /* Increase counter */

  g_foc_f32_inst_now += 1;

errout:
  return ret;
}
