/****************************************************************************
 * drivers/power/foc/foc_dev.c
 * Upper-half FOC controller logic
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
#include <fcntl.h>
#include <debug.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/power/power_ioctl.h>
#include <nuttx/power/foc/foc_lower.h>

#ifdef CONFIG_POWER_FOC_USE_FLOAT
#  include "float/foc_float.h"
#endif
#ifdef CONFIG_POWER_FOC_USE_FIXED16
#  include "fixed16/foc_fixed16.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(CONFIG_POWER_FOC_USE_FLOAT) && \
    !defined(CONFIG_POWER_FOC_USE_FIXED16)
#  error
#endif

#ifndef CONFIG_LIBDSP
#  error "CONFIG_LIBDSP is requered"
#endif

/****************************************************************************
 * External Definitions
 ****************************************************************************/

extern struct foc_callbacks_s g_foc_callbacks;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int foc_open(FAR struct file *filep);
static int foc_close(FAR struct file *filep);
static int foc_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

static int foc_lower_ops_assert(FAR struct foc_lower_ops_s *ops);
static int foc_typespec_assert(FAR struct foc_typespec_s *ts);

static int foc_setup(FAR struct foc_dev_s *dev);
static int foc_shutdown(FAR struct foc_dev_s *dev);
static int foc_stop(FAR struct foc_dev_s *dev);
static int foc_start(FAR struct foc_dev_s *dev);
static int foc_cfg_set(FAR struct foc_dev_s *dev, FAR struct foc_cfg_s *cfg);
static int foc_cfg_get(FAR struct foc_dev_s *dev, FAR struct foc_cfg_s *cfg);
static int foc_state_get(FAR struct foc_dev_s *dev,
                         FAR struct foc_state_s *state);
static int foc_params_set(FAR struct foc_dev_s *dev,
                          FAR struct foc_params_s *params);
static int foc_params_get(FAR struct foc_dev_s *dev,
                          FAR struct foc_params_s *params);
static int foc_fault_clean(FAR struct foc_dev_s *dev);
static int foc_mode_set(FAR struct foc_dev_s *dev, FAR int *mode);
static int foc_info_get(FAR struct foc_dev_s *dev,
                        FAR struct foc_info_s *info);

static int foc_register(FAR const char *path, FAR struct foc_dev_s *dev,
                        int ftype);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Device counter */

static uint8_t g_devno_cntr = 0;

/* File operations */

static const struct file_operations g_foc_fops =
{
  foc_open,                     /* open */
  foc_close,                    /* close */
  NULL,                         /* read */
  NULL,                         /* write */
  NULL,                         /* seek */
  foc_ioctl,                    /* ioctl */
  NULL                          /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL                        /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: foc_open
 *
 * Description:
 *   This function is called whenever the foc device is opened.
 *
 ****************************************************************************/

static int foc_open(FAR struct file *filep)
{
  FAR struct inode     *inode = filep->f_inode;
  FAR struct foc_dev_s *dev   = inode->i_private;
  uint8_t               tmp   = 0;
  int                   ret   = OK;
  irqstate_t            flags;

  /* Non-blocking operations not supported */

  if (filep->f_oflags & O_NONBLOCK)
    {
      ret = -EPERM;
      goto errout;
    }

  /* If the port is the middle of closing, wait until the close is finished */

  ret = nxsem_wait(&dev->closesem);
  if (ret >= 0)
    {
      /* Increment the count of references to the device.  If this the first
       * time that the driver has been opened for this device, then
       * initialize the device.
       */

      tmp = dev->ocount + 1;
      if (tmp == 0)
        {
          /* More than 255 opens; uint8_t overflows to zero */

          ret = -EMFILE;
        }
      else
        {
          /* Check if this is the first time that the driver has been opened
           */

          if (tmp == 1)
            {
              /* Yes.. perform one time driver setup */

              flags = enter_critical_section();

              ret = foc_setup(dev);
              if (ret == OK)
                {
                  /* Save the new open count on success */

                  dev->ocount = tmp;
                }

              leave_critical_section(flags);
            }
          else
            {
              /* Save the incremented open count */

              dev->ocount = tmp;
            }
        }

      nxsem_post(&dev->closesem);
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: foc_close
 *
 * Description:
 *   This routine is called when the foc device is closed.
 *
 ****************************************************************************/

static int foc_close(FAR struct file *filep)
{
  FAR struct inode     *inode = filep->f_inode;
  FAR struct foc_dev_s *dev   = inode->i_private;
  int                   ret   = 0;
  irqstate_t            flags;

  ret = nxsem_wait(&dev->closesem);
  if (ret >= 0)
    {
      /* Decrement the references to the driver. If the reference count will
       * decrement to 0, then uninitialize the driver.
       */

      if (dev->ocount > 1)
        {
          dev->ocount--;
          nxsem_post(&dev->closesem);
        }
      else
        {
          /* There are no more references to the port */

          dev->ocount = 0;

          /* Shutdown the device */

          flags = enter_critical_section();
          ret = foc_shutdown(dev);
          leave_critical_section(flags);

          nxsem_post(&dev->closesem);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: foc_ioctl
 *
 * Description:
 *   Supported IOCTLs:
 *
 *   PWRIOC_START:        Start the FOC device,
 *                        arg: none
 *
 *   PWRIOC_STOP:         Stop the FOC device,
 *                        arg: none
 *
 *   PWRIOC_SET_MODE:     Set the FOC device operation mode,
 *                        arg: int pointer
 *
 *   PWRIOC_GET_STATE:    Get the FOC device state,
 *                        arg: struct foc_state_s pointer
 *                        This is a blocking operation that is used to
 *                        synchronize the user space application with
 *                        a FOC worker.
 *
 *   PWRIOC_CLEAN_FAULT:  Clean the FOC device fault state,
 *                        arg: none
 *
 *   PWRIOC_SET_PARAMS:   Set the FOC device operation parameters,
 *                        arg: struct foc_params_s pointer
 *
 *   PWRIOC_GET_PARAMS:   Get the FOC device operation parameters,
 *                        arg: struct foc_params_s pointer
 *
 *   PWRIOC_SET_CONFIG:   Set the FOC device configuration,
 *                        arg: struct foc_cfg_s pointer
 *
 *   PWRIOC_GET_CONFIG:   Get the FOC device configuration,
 *                        arg: struct foc_cfg_s pointer
 *
 *   PWRIOC_GET_INFO:     Get the FOC device info,
 *                        arg: struct foc_info_s pointer
 *
 ****************************************************************************/

static int foc_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct foc_dev_s       *dev   = inode->i_private;
  int                         ret   = 0;
  irqstate_t                  flags;

  flags = enter_critical_section();

  switch (cmd)
    {
      /* Start the FOC device */

      case PWRIOC_START:
        {
          ret = foc_start(dev);
          if (ret != OK)
            {
              pwrerr("ERROR: PWRIOC_START failed %d\n", ret);
            }

          break;
        }

      /* Stop the FOC device */

      case PWRIOC_STOP:
        {
          ret = foc_stop(dev);
          if (ret != OK)
            {
              pwrerr("ERROR: PWRIOC_STOP failed %d\n", ret);
            }

          break;
        }

      /* Set the FOC operation mode */

      case PWRIOC_SET_MODE:
        {
          FAR int *mode = (FAR int *)arg;

          DEBUGASSERT(mode != NULL);

          ret = foc_mode_set(dev, mode);
          if (ret != OK)
            {
              pwrerr("ERROR: PWRIOC_SET_MODE failed %d\n", ret);
            }

          break;
        }

      /* Get device state */

      case PWRIOC_GET_STATE:
        {
          FAR struct foc_state_s *state = (FAR struct foc_state_s *)arg;

          DEBUGASSERT(state != NULL);

          ret = foc_state_get(dev, state);
          if (ret != OK)
            {
              pwrerr("ERROR: PWRIOC_GET_STATE failed %d\n", ret);
            }

          break;
        }

      /* Clean fault state */

      case PWRIOC_CLEAN_FAULT:
        {
          DEBUGASSERT(arg == 0);

          ret = foc_fault_clean(dev);
          if (ret != OK)
            {
              pwrerr("ERROR: PWRIOC_CLEAN_FAULT failed %d\n", ret);
            }

          break;
        }

      /* Set device parameters */

      case PWRIOC_SET_PARAMS:
        {
          FAR struct foc_params_s *params = (FAR struct foc_params_s *)arg;

          DEBUGASSERT(params != NULL);

          ret = foc_params_set(dev, params);
          if (ret != OK)
            {
              pwrerr("ERROR: PWRIOC_SET_PARAMS failed %d\n", ret);
            }

          break;
        }

      /* Get device parameters */

      case PWRIOC_GET_PARAMS:
        {
          FAR struct foc_params_s *params = (FAR struct foc_params_s *)arg;

          DEBUGASSERT(params != NULL);

          ret = foc_params_get(dev, params);
          if (ret != OK)
            {
              pwrerr("ERROR: PWRIOC_GET_PARAMS failed %d\n", ret);
            }

          break;
        }

      /* Set the device configuration */

      case PWRIOC_SET_CONFIG:
        {
          FAR struct foc_cfg_s *cfg = (FAR struct foc_cfg_s *)arg;

          DEBUGASSERT(cfg != NULL);

          ret = foc_cfg_set(dev, cfg);
          if (ret != OK)
            {
              pwrerr("ERROR: PWRIOC_SET_CONFIG failed %d\n", ret);
            }

          break;
        }

      /* Get the FOC device configuration */

      case PWRIOC_GET_CONFIG:
        {
          FAR struct foc_cfg_s *cfg = (FAR struct foc_cfg_s *)arg;

          DEBUGASSERT(cfg != NULL);

          ret = foc_cfg_get(dev, cfg);
          if (ret != OK)
            {
              pwrerr("ERROR: PWRIOC_GET_CONFIG failed %d\n", ret);
            }

          break;
        }

      /* Get the FOC device info */

      case PWRIOC_GET_INFO:
        {
          FAR struct foc_info_s *info = (struct foc_info_s *)arg;

          DEBUGASSERT(info != NULL);

          ret = foc_info_get(dev, info);
          if (ret != OK)
            {
              pwrerr("ERROR: PWRIOC_GET_INFO failed %d\n", ret);
            }

          break;
        }

      /* Not supported */

      default:
        {
          pwrinfo("Forwarding unrecognized cmd: %d arg: %ld\n", cmd, arg);

          /* Call lower-half logic */

          ret = FOC_OPS_IOCTL(dev, cmd, arg);

          break;
        }
    }

  leave_critical_section(flags);

  return ret;
}

/****************************************************************************
 * Name: foc_lower_ops_assert
 *
 * Description:
 *   Assert the lower-half FOC operations
 *
 ****************************************************************************/

static int foc_lower_ops_assert(FAR struct foc_lower_ops_s *ops)
{
  DEBUGASSERT(ops->configure);
  DEBUGASSERT(ops->setup);
  DEBUGASSERT(ops->shutdown);
  DEBUGASSERT(ops->start);
  DEBUGASSERT(ops->ioctl);
  DEBUGASSERT(ops->bind);
  DEBUGASSERT(ops->fault_clean);
#ifdef CONFIG_POWER_FOC_HAVE_CORDIC
  DEBUGASSERT(ops->angle_update);
  DEBUGASSERT(ops->dq_saturate);
#endif
#ifdef CONFIG_POWER_FOC_TRACE
  DEBUGASSERT(ops->trace);
#endif

  UNUSED(ops);

  return OK;
}

/****************************************************************************
 * Name: foc_typespec_assert
 *
 * Description:
 *   Assert the type-specific implementation
 *
 ****************************************************************************/

static int foc_typespec_assert(FAR struct foc_typespec_s *ts)
{
  DEBUGASSERT(ts);
  DEBUGASSERT(ts->control);
  DEBUGASSERT(ts->control->init);
  DEBUGASSERT(ts->control->start);
  DEBUGASSERT(ts->control->cfg_set);
  DEBUGASSERT(ts->control->cfg_get);
  DEBUGASSERT(ts->control->curr_set);
  DEBUGASSERT(ts->control->params_set);
  DEBUGASSERT(ts->control->voltage_run);
  DEBUGASSERT(ts->control->current_run);
  DEBUGASSERT(ts->control->fb_get);
  DEBUGASSERT(ts->modulation);
  DEBUGASSERT(ts->modulation->init);
  DEBUGASSERT(ts->modulation->vbase_get);
  DEBUGASSERT(ts->modulation->current);
  DEBUGASSERT(ts->modulation->run);

  UNUSED(ts);

  return OK;
}

/****************************************************************************
 * Name: foc_lower_bind
 *
 * Description:
 *   Bind the upper-half with the lower-half FOC logic
 *
 ****************************************************************************/

static int foc_lower_bind(FAR struct foc_dev_s *dev)
{
  DEBUGASSERT(dev);
  DEBUGASSERT(g_foc_callbacks.foc_work);
  DEBUGASSERT(g_foc_callbacks.notifier);

  return FOC_OPS_BIND(dev, &g_foc_callbacks);
}

/****************************************************************************
 * Name: foc_setup
 *
 * Description:
 *   Setup the FOC device
 *
 ****************************************************************************/

static int foc_setup(FAR struct foc_dev_s *dev)
{
  int ret = OK;

  DEBUGASSERT(dev);

  pwrinfo("FOC SETUP\n");

  /* Reset device data */

  dev->mode = 0;

  memset(&dev->cfgcmn, 0, sizeof(struct foc_cfgcmn_s));
  memset(&dev->state, 0, sizeof(struct foc_state_s));
  memset(&dev->params, 0, sizeof(struct foc_params_s));

  /* Bind the upper-half with the lower-half FOC logic */

  ret = foc_lower_bind(dev);
  if (ret < 0)
    {
      pwrerr("ERROR: foc_lower_bind failed %d\n", ret);
      set_errno(EINVAL);
      goto errout;
    }

  /* Call lower-half setup */

  ret = FOC_OPS_SETUP(dev);
  if (ret < 0)
    {
      pwrerr("FOC_OPS_SETUP failed %d\n", ret);
      goto errout;
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: foc_shutdown
 *
 * Description:
 *   Shutdown the FOC device
 *
 ****************************************************************************/

int foc_shutdown(FAR struct foc_dev_s *dev)
{
  int ret = OK;

  DEBUGASSERT(dev);

  pwrinfo("FOC SHUTDOWN\n");

  /* Call the lower-half shutdown */

  ret = FOC_OPS_SHUTDOWN(dev);

  return ret;
}

/****************************************************************************
 * Name: foc_start
 *
 * Description:
 *   Start the FOC device
 *
 ****************************************************************************/

static int foc_start(FAR struct foc_dev_s *dev)
{
  int ret = OK;

  DEBUGASSERT(dev);

  pwrinfo("FOC START\n");

  /* Start the modulation block */

  ret = FOC_TS_MOD_START(dev);
  if (ret < 0)
    {
      pwrerr("ERROR: FOC_TS_MOD_START failed %d\n", ret);
      goto errout;
    }

  /* Start the controller block */

  ret = FOC_TS_CTRL_START(dev);
  if (ret < 0)
    {
      pwrerr("ERROR: FOC_TS_CTRL_START failed %d\n", ret);
      goto errout;
    }

  /* Reset the notifier semaphore */

  ret = nxsem_reset(&dev->statesem, 0);
  if (ret < 0)
    {
      pwrerr("ERROR: nxsem_reset failed %d\n", ret);
      goto errout;
    }

  /* Start the FOC */

  ret = FOC_OPS_START(dev, true);
  if (ret < 0)
    {
      pwrerr("ERROR: FOC_OPS_START failed %d !\n", ret);
      goto errout;
    }

  /* Set start flag */

  dev->state.start = true;

errout:
  return ret;
}

/****************************************************************************
 * Name: foc_stop
 *
 * Description:
 *   Stop the FOC device
 *
 ****************************************************************************/

static int foc_stop(FAR struct foc_dev_s *dev)
{
  foc_number_t d_zero[CONFIG_POWER_FOC_PHASES];
  int          ret = OK;

  DEBUGASSERT(dev);

  pwrinfo("FOC STOP\n");

  /* Zero duty cycle */

  memset(&d_zero, 0, CONFIG_POWER_FOC_PHASES * sizeof(foc_number_t));

  /* Reset duty cycle */

  ret = FOC_OPS_DUTY(dev, d_zero);
  if (ret < 0)
    {
      pwrerr("ERROR: FOC_OPS_DUTY failed %d\n", ret);
    }

  /* Stop the FOC */

  ret = FOC_OPS_START(dev, false);
  if (ret < 0)
    {
      pwrerr("ERROR: FOC_OPS_START failed %d\n", ret);
    }

  /* Reset device data */

  dev->mode = 0;
  memset(&dev->state, 0, sizeof(struct foc_state_s));
  memset(&dev->params, 0, sizeof(struct foc_params_s));

  return ret;
}

/****************************************************************************
 * Name: foc_cfg_set
 *
 * Description:
 *   Set the FOC device configuration
 *
 ****************************************************************************/

static int foc_cfg_set(FAR struct foc_dev_s *dev, FAR struct foc_cfg_s *cfg)
{
  int ret = OK;

  DEBUGASSERT(dev);
  DEBUGASSERT(cfg);

  if (cfg->type == FOC_CFG_TYPE_COMMON)
    {
      /* Copy common configuration */

      memcpy(&dev->cfgcmn, cfg->data, sizeof(struct foc_cfgcmn_s));

      DEBUGASSERT(dev->cfgcmn.pwm_freq > 0);
      DEBUGASSERT(dev->cfgcmn.work_freq > 0);
      DEBUGASSERT(dev->cfgcmn.notifier_freq > 0);
      DEBUGASSERT(dev->cfgcmn.modulation > 0);
      DEBUGASSERT(dev->cfgcmn.controller > 0);

      pwrinfo("FOC %" PRIu8 " PWM=%" PRIu32 " work=%" PRIu32
              " notifier=%" PRIu32 "\n",
              dev->info.devno,
              dev->cfgcmn.pwm_freq,
              dev->cfgcmn.work_freq,
              dev->cfgcmn.notifier_freq);
      pwrinfo("      modualtion = %d controller = %d\n",
              dev->cfgcmn.modulation,
              dev->cfgcmn.controller);

      /* Call arch configuration */

      ret = FOC_OPS_CONFIGURE(dev, &dev->cfgcmn);
      if (ret < 0)
        {
          pwrerr("FOC_OPS_CONFIGURE failed %d\n", ret);
          goto errout;
        }

      /* Configure type-specific logic */

      ret = FOC_TS_CONFIGURE(dev, &dev->cfgcmn);
      if (ret < 0)
        {
          pwrerr("ERROR: FOC_TS_CONFIGURE failed %d\n", ret);
          goto errout;
        }

      /* Assert the type-specific interface */

      ret = foc_typespec_assert(dev->typespec);
      if (ret < 0)
        {
          goto errout;
        }

      /* Initialize modulation block */

      ret = FOC_TS_MOD_INIT(dev);
      if (ret < 0)
        {
          pwrerr("ERROR: FOC_TS_MOD_INIT failed %d\n", ret);
          goto errout;
        }

      /* Initialize controller block */

      ret = FOC_TS_CTRL_INIT(dev);
      if (ret < 0)
        {
          pwrerr("ERROR: FOC_TS_CTRL_INIT failed %d\n", ret);
          goto errout;
        }
    }
  else if (cfg->type == FOC_CFG_TYPE_CONTROL)
    {
      /* Type-specific logic must be configured */

      if (dev->typespec == NULL)
        {
          pwrerr("ERROR: typespec == NULL\n");
          ret = -EPERM;
          goto errout;
        }

      /* Controller configuration */

      ret = FOC_TS_CTRL_CFG_SET(dev, cfg->data);
      if (ret < 0)
        {
          pwrerr("ERROR: FOC_TS_CTRL_CFG_SET failed %d\n", ret);
          goto errout;
        }
    }
  else
    {
      pwrerr("ERROR: unknown FOC configuration type %d\n", cfg->type);
      ret = -EINVAL;
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: foc_cfg_get
 *
 * Description:
 *   Get the FOC device configuration
 *
 ****************************************************************************/

static int foc_cfg_get(FAR struct foc_dev_s *dev, FAR struct foc_cfg_s *cfg)
{
  int ret = OK;

  DEBUGASSERT(dev);
  DEBUGASSERT(cfg);

  if (cfg->type == FOC_CFG_TYPE_COMMON)
    {
      /* Get common configuration */

      memcpy(cfg->data, &dev->cfgcmn, sizeof(struct foc_cfgcmn_s));
    }
  else if (cfg->type == FOC_CFG_TYPE_CONTROL)
    {
      /* Type-specific logic must be configured */

      if (dev->typespec == NULL)
        {
          pwrerr("ERROR: typespec == NULL\n");
          ret = -EPERM;
          goto errout;
        }

      /* Get controller configuration */

      ret = FOC_TS_CTRL_CFG_GET(dev, cfg->data);
      if (ret < 0)
        {
          pwrerr("ERROR: FOC_TS_CTRL_CFG_GET failed %d\n", ret);
          goto errout;
        }
    }
  else
    {
      pwrerr("ERROR: unknown FOC configuration type %d\n", cfg->type);
      ret = -EINVAL;
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: foc_state_get
 *
 * Description:
 *   Get the FOC device state
 *
 ****************************************************************************/

static int foc_state_get(FAR struct foc_dev_s *dev,
                         FAR struct foc_state_s *state)
{
  int ret = OK;

  DEBUGASSERT(dev);
  DEBUGASSERT(state);

  /* Signal trace */

#ifdef CONFIG_POWER_FOC_TRACE
  FOC_OPS_TRACE(dev, FOC_TRACE_STATE, true);
#endif

  /* Wait for notification if blocking */

  ret = nxsem_wait_uninterruptible(&dev->statesem);
  if (ret < 0)
    {
      goto errout;
    }

#ifdef CONFIG_POWER_FOC_TRACE
  FOC_OPS_TRACE(dev, FOC_TRACE_STATE, false);
#endif

  /* Copy state */

  memcpy(state, &dev->state, sizeof(struct foc_state_s));

errout:
  return ret;
}

/****************************************************************************
 * Name: foc_fault_clean
 *
 * Description:
 *   Clean the FOC device fault state
 *
 ****************************************************************************/

static int foc_fault_clean(FAR struct foc_dev_s *dev)
{
  int ret = OK;

  /* Call lower-half logic */

  ret = FOC_OPS_FAULT_CLEAN(dev);
  if (ret < 0)
    {
      pwrerr("ERROR: FOC_OPS_FAULT_CLEAN failed %d\n", ret);
      goto errout;
    }

  /* Clean all faults */

  dev->state.fault.s.fault     = FOC_FAULT_NONE;
  dev->state.fault.s.fault_ext = FOC_FAULT_EXT_NONE;

errout:
  return ret;
}

/****************************************************************************
 * Name: foc_params_set
 *
 * Description:
 *   Set the FOC device parameters
 *
 ****************************************************************************/

static int foc_params_set(FAR struct foc_dev_s *dev,
                          FAR struct foc_params_s *params)
{
  foc_number_t vbase;

  DEBUGASSERT(dev);
  DEBUGASSERT(params);

#ifdef CONFIG_POWER_FOC_TRACE
  FOC_OPS_TRACE(dev, FOC_TRACE_PARAMS, true);
#endif

  /* Copy parameters */

  memcpy(&dev->params, params, sizeof(struct foc_params_s));

  /* Get the FOC modulation base voltage */

  FOC_TS_MOD_VBASE(dev, &dev->params.vbus, &vbase);

  /* Update base voltage and phase angle */

  FOC_TS_CTRL_PARAMS_SET(dev, &vbase, &dev->params.angle);

#ifdef CONFIG_POWER_FOC_TRACE
  FOC_OPS_TRACE(dev, FOC_TRACE_PARAMS, false);
#endif

  return OK;
}

/****************************************************************************
 * Name: foc_params_get
 *
 * Description:
 *   Get the FOC device parameters
 *
 ****************************************************************************/

static int foc_params_get(FAR struct foc_dev_s *dev,
                          FAR struct foc_params_s *params)
{
  DEBUGASSERT(dev);
  DEBUGASSERT(params);

  /* Get stored parameters */

  memcpy(params, &dev->params, sizeof(struct foc_params_s));

  return OK;
}

/****************************************************************************
 * Name: foc_mode_set
 *
 * Description:
 *   Set the FOC device controller mode
 *
 ****************************************************************************/

static int foc_mode_set(FAR struct foc_dev_s *dev, FAR int *mode)
{
  DEBUGASSERT(dev);
  DEBUGASSERT(mode);

  /* Copy mode */

  dev->mode = *mode;

  return OK;
}

/****************************************************************************
 * Name: foc_info_get
 *
 * Description:
 *   Get the FOC device info
 *
 ****************************************************************************/

static int foc_info_get(FAR struct foc_dev_s *dev,
                        FAR struct foc_info_s *info)
{
  /* Copy data from device */

  memcpy(info, &dev->info, sizeof(struct foc_info_s));

  return OK;
}

/****************************************************************************
 * Name: foc_register
 *
 * Description:
 *   Register the FOC character device as 'path'
 *
 * Input Parameters:
 *   path  - The full path to the driver to register
 *   dev   - An instance of the FOC device
 *   ftype - The FOC device type
 *
 ****************************************************************************/

static int foc_register(FAR const char *path, FAR struct foc_dev_s *dev,
                        int ftype)
{
  int ret = OK;

  DEBUGASSERT(path != NULL);
  DEBUGASSERT(dev != NULL);

  /* Lower-half must be initialized */

  DEBUGASSERT(dev->lower);
  DEBUGASSERT(dev->lower->ops);
  DEBUGASSERT(dev->lower->data);

  /* Check if the device instance is supported by the driver */

  if (dev->info.devno > CONFIG_POWER_FOC_INST)
    {
      pwrerr("ERROR: unsupported foc devno %d\n\n", dev->info.devno);
      set_errno(EINVAL);
      ret = ERROR;
      goto errout;
    }

  /* Reset counter */

  dev->ocount = 0;

  /* Get type-specific data */

  switch (ftype)
    {
#ifdef CONFIG_POWER_FOC_USE_FLOAT
      case FOC_NUMBER_TYPE_FLOAT:
        {
          /* Controller type identifier */

          dev->info.ftype = FOC_NUMBER_TYPE_FLOAT;

          /* Initialize f32 logic */

          ret = foc_typespec_f32_init(dev);
          if (ret < 0)
            {
              pwrerr("ERROR: foc_typespec_f32_init failed %d\n", ret);
              goto errout;
            }

          break;
        }
#endif

#ifdef CONFIG_POWER_FOC_USE_FIXED16
      case FOC_NUMBER_TYPE_FIXED16:
        {
          /* Controller type identifier */

          dev->info.ftype = FOC_NUMBER_TYPE_FIXED16;

          /* Initialize b16 logic */

          ret = foc_typespec_b16_init(dev);
          if (ret < 0)
            {
              pwrerr("ERROR: foc_typespec_b16_init failed %d\n", ret);
              goto errout;
            }

          break;
        }
#endif

      default:
        {
          ASSERT(0);
          break;
        }
    }

  DEBUGASSERT(dev->typespec);
  DEBUGASSERT(dev->typespec->configure);

  /* Store device number */

  dev->info.devno = g_devno_cntr;

  /* Assert the lower-half interface */

  ret = foc_lower_ops_assert(dev->lower->ops);
  if (ret < 0)
    {
      goto errout;
    }

  /* Initialize semaphores */

  nxsem_init(&dev->closesem, 0, 1);
  nxsem_init(&dev->statesem, 0, 0);
  nxsem_set_protocol(&dev->statesem, SEM_PRIO_NONE);

  /* Register the FOC character driver */

  ret = register_driver(path, &g_foc_fops, 0444, dev);
  if (ret < 0)
    {
      nxsem_destroy(&dev->closesem);
      set_errno(ret);
      ret = ERROR;
      goto errout;
    }

  /* Increase device counter */

  g_devno_cntr += 1;

errout:
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_POWER_FOC_USE_FLOAT
/****************************************************************************
 * Name: focf32_register
 *
 * Description:
 *   Register the FOC (float) character device as 'path'
 *
 * Input Parameters:
 *   path  - The full path to the driver to register
 *   dev   - An instance of the FOC device
 *
 ****************************************************************************/

int focf32_register(FAR const char *path, FAR struct foc_dev_s *dev)
{
  return foc_register(path, dev, FOC_NUMBER_TYPE_FLOAT);
}
#endif

#ifdef CONFIG_POWER_FOC_USE_FIXED16
/****************************************************************************
 * Name: focb16_register
 *
 * Description:
 *   Register the FOC (fixed16) character device as 'path'
 *
 * Input Parameters:
 *   path  - The full path to the driver to register
 *   dev   - An instance of the FOC device
 *
 ****************************************************************************/

int focb16_register(FAR const char *path, FAR struct foc_dev_s *dev)
{
  return foc_register(path, dev, FOC_NUMBER_TYPE_FIXED16);
}
#endif
