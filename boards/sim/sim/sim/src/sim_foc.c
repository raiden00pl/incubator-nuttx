/****************************************************************************
 * configs/sim/sim/sim/src/sim_foc.c
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

#include <stdio.h>
#include <errno.h>

#include <arch/board/board.h>

#include <nuttx/power/foc/foc_lower.h>

#ifdef CONFIG_POWER_FOCMODEL
#  include <nuttx/power/focmodel/focmodel_lower.h>
#endif

#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_foc_setup
 *
 * Description:
 *   Initialize the FOC device.
 *
 *   This function should be call by board_app_initialize().
 *
 * Returned Value:
 *   0 on success, a negated errno value on failure
 *
 ****************************************************************************/

int sim_foc_setup(void)
{
#ifdef CONFIG_POWER_FOC_USE_FLOAT
  FAR struct foc_dev_s      *focf32[CONFIG_POWER_FOC_FLOAT_INST];
#endif
#ifdef CONFIG_POWER_FOC_USE_FIXED16
  FAR struct foc_dev_s      *focb16[CONFIG_POWER_FOC_FIXED16_INST];
#endif
#if defined(CONFIG_POWER_FOC_USE_FIXED16) && defined(CONFIG_POWER_FOCMODEL)
  FAR struct focmodel_dev_s *modelb16[CONFIG_POWER_FOC_FIXED16_INST];
#endif
#if defined(CONFIG_POWER_FOC_USE_FLOAT) && defined(CONFIG_POWER_FOCMODEL)
  FAR struct focmodel_dev_s *modelf32[CONFIG_POWER_FOC_FLOAT_INST];
#endif
  static bool                initialized = false;
  int                        ret         = OK;
  int                        i           = 0;
  int                        j           = 0;
  char                       devpath[20];

  printf("sim_foc_setup\n");

  /* Initialize only once */

  if (!initialized)
    {
#ifdef CONFIG_POWER_FOC_USE_FLOAT
      /* Register float devices */

      for (i = 0; i < CONFIG_POWER_FOC_FLOAT_INST; i += 1)
        {
          /* Initialize arch specific FOC lower-half */

          focf32[i] = sim_foc_initialize(j);
          if (focf32[i] == NULL)
            {
              ret = -errno;
              printf("ERROR: failed to initialize sim FOC%d: %d\n", j, ret);
              goto errout;
            }

          DEBUGASSERT(focf32[i]->lower);

          /* Get devpath for FOC */

          sprintf(devpath, "/dev/foc%d", j);

          /* Register FOC float device */

          ret = focf32_register(devpath, focf32[i]);
          if (ret < 0)
            {
              printf("ERROR: failed to register "
                     "float FOC device %s: %d\n",
                     devpath, ret);
              goto errout;
            }

#ifdef CONFIG_POWER_FOCMODEL
          /* Initialize arch specific FOC lower-half */

          modelf32[i] = sim_focmodel_initialize(j);
          if (modelf32[i] == NULL)
            {
              ret = -errno;
              printf("ERROR: failed to initialize sim FOCMODEL%d: %d\n",
                     j, ret);
              goto errout;
            }

          /* Get devpath for FOC model */

          sprintf(devpath, "/dev/focmodel%d", j);

          DEBUGASSERT(focf32[i]);

          /* Register FOC model */

          ret = focf32_model_register(devpath, modelf32[i]);
          if (ret < 0)
            {
              printf("ERROR: failed to register "
                     "float FOC model %s: %d\n",
                     devpath, ret);
              goto errout;
            }
#endif

          j += 1;
        }
#endif

#ifdef CONFIG_POWER_FOC_USE_FIXED16
      /* Register fixed16 devices */

      for (i = 0 ; i < CONFIG_POWER_FOC_FIXED16_INST; i += 1)
        {
          /* Initialize arch specific FOC lower-half */

          focb16[i] = sim_foc_initialize(j);
          if (focb16[i] == NULL)
            {
              ret = -errno;
              printf("ERROR: failed to initialize sim FOC%d: %d\n", i, ret);
              goto errout;
            }

          DEBUGASSERT(focb16[i]->lower);

          /* Get devpath for FOC */

          sprintf(devpath, "/dev/foc%d", j);

          /* Register FOC fixed16 device */

          ret = focb16_register(devpath, focb16[i]);
          if (ret < 0)
            {
              printf("ERROR: failed to register "
                     "fixed16 FOC device %s: %d\n",
                     devpath, ret);
              goto errout;
            }

#ifdef CONFIG_POWER_FOCMODEL
          /* Initialize arch specifci FOC model control logic */

          modelb16[i] = sim_focmodel_initialize(j);
          if (modelb16[i] == NULL)
            {
              ret = -errno;
              printf("ERROR: failed to initialize sim FOCMODEL%d: %d\n",
                     j, ret);
              goto errout;
            }

          /* Get devpath for FOC model */

          sprintf(devpath, "/dev/focmodel%d", j);

          DEBUGASSERT(focb16[i]);

          /* Register FOC model */

          ret = focb16_model_register(devpath, modelb16[i]);
          if (ret < 0)
            {
              printf("ERROR: failed to register "
                     "fixed16 FOC model %s: %d\n",
                     devpath, ret);
              goto errout;
            }
#endif

          j += 1;
        }
#endif

      initialized = true;
    }

errout:
  return ret;
}
