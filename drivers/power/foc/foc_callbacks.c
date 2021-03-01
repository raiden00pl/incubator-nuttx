/****************************************************************************
 * drivers/power/foc/foc_callback.c
 * Common FOC device callbacks
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

#include <assert.h>
#include <sched.h>

#include <nuttx/semaphore.h>

#include <nuttx/power/foc/foc_lower.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int foc_run(FAR struct foc_dev_s *dev, FAR foc_number_t *current);
static int foc_work(FAR struct foc_dev_s *dev, FAR foc_number_t *current);
static int foc_notifier(FAR struct foc_dev_s *dev);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* FOC callbacks from the lower-half implementation to this driver */

struct foc_callbacks_s g_foc_callbacks =
{
  .foc_work = foc_work,
  .notifier = foc_notifier,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: foc_run
 *
 * Description:
 *   Single step of the control algorithm
 *
 ****************************************************************************/

static int foc_run(FAR struct foc_dev_s *dev, FAR foc_number_t *current)
{
  foc_number_t  duty[CONFIG_POWER_FOC_PHASES];
  foc_abframe_t v_ab_mod;
  int           ret = OK;

  DEBUGASSERT(dev);
  DEBUGASSERT(current);

  /* Do nothing if control mode not specified yet.
   * This also protects against initial state when the controller is
   * started but input data has not yet been provided.
   */

  if (dev->mode <= FOC_CONTROL_MODE_INIT)
    {
      /* Set mode fault flag */

      dev->state.fault.s.fault |= FOC_FAULT_MODE;

      ret = ERROR;
      goto errout;
    }

  /* Correct current samples according to modulation state */

  FOC_TS_MOD_CURRENT(dev, current);

  /* Feed controller with phase currents */

  FOC_TS_CTRL_CURR_SET(dev, current);

  /* Call controller */

  switch (dev->mode)
    {
      /* IDLE */

      case FOC_CONTROL_MODE_IDLE:
        {
          /* Do nothing and set duty to zeros */

          ret = OK;
          goto errout;
        }

      /* FOC current mode - control DQ-current */

      case FOC_CONTROL_MODE_CURRENT:
        {
          /* Input:
           *    - reference i_dq
           *    - compensation v_dq
           *
           *  Output:
           *    - modulation v_ab
           */

          FOC_TS_CTRL_CURRENT(dev,
                              &dev->params.dq_ref,
                              &dev->params.vdq_comp,
                              &v_ab_mod);
          break;
        }

      /* FOC voltage mode - control DQ-voltage */

      case FOC_CONTROL_MODE_VOLTAGE:
        {
          /* Input:
           *    - reference v_dq
           *
           *  Output:
           *    - modulation v_ab
           */

          FOC_TS_CTRL_VOLTAGE(dev,
                              &dev->params.dq_ref,
                              &v_ab_mod);

          break;
        }

      /* Otherwise - we should not be here */

      default:
        {
          DEBUGASSERT(0);
          ret = ERROR;

          dev->state.fault.s.fault |= FOC_FAULT_MODE;

          goto errout;
        }
    }

  /* Duty cycle modulation */

  FOC_TS_MOD_RUN(dev, &v_ab_mod, duty);

  /* Set duty cycle */

  FOC_OPS_DUTY(dev, duty);

  return ret;

errout:

  /* Set duty to zeros */

  memset(duty, 0, sizeof(foc_number_t) * CONFIG_POWER_FOC_PHASES);

  /* Set duty cycle */

  FOC_OPS_DUTY(dev, duty);

  return ret;
}

/****************************************************************************
 * Name: foc_work
 *
 * Description:
 *   The FOC work.
 *
 *   We require that the lower-half logic provides 3-phase currents.
 *
 ****************************************************************************/

static int foc_work(FAR struct foc_dev_s *dev, FAR foc_number_t *current)
{
  int ret = OK;

  DEBUGASSERT(dev);
  DEBUGASSERT(current);

#ifdef CONFIG_POWER_FOC_TRACE
  FOC_OPS_TRACE(dev, FOC_TRACE_WORKER, true);
#endif

  /* Run control algorithm */

  ret = foc_run(dev, current);
  if (ret < 0)
    {
      goto errout;
    }

  /* Store FOC feedback data */

  FOC_TS_CTRL_FB(dev, &dev->state.fb);

errout:

#ifdef CONFIG_POWER_FOC_TRACE
  FOC_OPS_TRACE(dev, FOC_TRACE_WORKER, false);
#endif

  return ret;
}

/****************************************************************************
 * Name: foc_notifier
 *
 * Description:
 *   Notify the user-space
 *
 ****************************************************************************/

static int foc_notifier(FAR struct foc_dev_s *dev)
{
  int ret  = OK;
  int sval = 0;

  DEBUGASSERT(dev != NULL);

#ifdef CONFIG_POWER_FOC_TRACE
  FOC_OPS_TRACE(dev, FOC_TRACE_NOTIFIER, true);
#endif

  /* Disable pre-emption until all of the waiting threads have been
   * restarted. This is necessary to assure that the sval behaves as
   * expected in the following while loop
   */

  sched_lock();

  /* Check if the previous cycle was handled */

  ret = nxsem_get_value(&dev->statesem, &sval);
  if (ret != OK)
    {
      ret = -EINVAL;
    }
  else
    {
      if (sval < -dev->ocount)
        {
          /* This is a critical fault */

          DEBUGASSERT(0);

          /* Set timeout fault if not in debug mode */

          dev->state.fault.s.fault |= FOC_FAULT_TIMEOUT;

          /* Reset semaphore */

          nxsem_reset(&dev->statesem, 0);
        }
      else
        {
          /* Loop until all of the waiting threads have been restarted. */

          while (sval < 0)
            {
              /* Post semaphore */

              nxsem_post(&dev->statesem);

              /* Increment the semaphore count (as was done by the
               * above post).
               */

              sval += 1;
            }
        }
    }

#ifdef CONFIG_POWER_FOC_TRACE
  FOC_OPS_TRACE(dev, FOC_TRACE_NOTIFIER, false);
#endif

  /* Now we can let the restarted threads run */

  sched_unlock();

  return ret;
}
