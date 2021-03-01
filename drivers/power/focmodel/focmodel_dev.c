/****************************************************************************
 * drivers/power/focmodel/focmodel_dev.c
 * Upper-half FOC model logic
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
#include <nuttx/power/focmodel/focmodel_lower.h>

#ifdef CONFIG_POWER_FOCMODEL_PMSM_FLOAT
#  include "pmsm_float/pmsm_float.h"
#endif
#ifdef CONFIG_POWER_FOCMODEL_PMSM_FIXED16
#  include "pmsm_fixed16/pmsm_fixed16.h"
#endif

/****************************************************************************
 * External Definitions
 ****************************************************************************/

extern struct focmodel_callbacks_s g_focmodel_callbacks;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int focmodel_open(FAR struct file *filep);
static int focmodel_close(FAR struct file *filep);
static int focmodel_ioctl(FAR struct file *filep, int cmd,
                          unsigned long arg);

static int focmodel_lower_ops_assert(FAR struct focmodel_lower_ops_s *ops);
static int focmodel_typespec_assert(FAR struct focmodel_typespec_s *ts);

static int focmodel_setup(FAR struct focmodel_dev_s *dev);
static int focmodel_shutdown(FAR struct focmodel_dev_s *dev);
static int focmodel_lower_bind(FAR struct focmodel_dev_s *dev);

static int focmodel_cfg_set(FAR struct focmodel_dev_s *dev,
                            FAR struct focmodel_cfg_s *cfg);
static int focmodel_params_set(FAR struct focmodel_dev_s *dev,
                               FAR struct focmodel_params_s *params);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Device counter */

static uint8_t g_devno_cntr = 0;

/* File operations */

static const struct file_operations g_focmodel_fops =
{
  focmodel_open,                /* open */
  focmodel_close,               /* close */
  NULL,                         /* read */
  NULL,                         /* write */
  NULL,                         /* seek */
  focmodel_ioctl,               /* ioctl */
  NULL                          /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL                        /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: focmodel_open
 *
 * Description:
 *   This function is called whenever the foc device is opened.
 *
 ****************************************************************************/

static int focmodel_open(FAR struct file *filep)
{
  FAR struct inode          *inode = filep->f_inode;
  FAR struct focmodel_dev_s *dev   = inode->i_private;
  uint8_t                    tmp   = 0;
  int                        ret   = OK;
  irqstate_t                 flags;

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
          /* Check if this is the first time that the driver has been opened.
           */

          if (tmp == 1)
            {
              /* Yes.. perform one time driver initialization. */

              flags = enter_critical_section();

              ret = focmodel_setup(dev);
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

  return ret;
}

/****************************************************************************
 * Name: focmodel_close
 *
 * Description:
 *   This routine is called when the foc device is closed.
 *
 ****************************************************************************/

static int focmodel_close(FAR struct file *filep)
{
  FAR struct inode          *inode = filep->f_inode;
  FAR struct focmodel_dev_s *dev   = inode->i_private;
  int                        ret   = 0;
  irqstate_t                 flags;

  ret = nxsem_wait(&dev->closesem);
  if (ret >= 0)
    {
      /* Decrement the references to the driver.  If the reference count will
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

          /* Free the IRQ and disable the FOC model device */

          flags = enter_critical_section();
          ret = focmodel_shutdown(dev);
          leave_critical_section(flags);

          nxsem_post(&dev->closesem);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: focmodel_ioctl
 *
 * Description:
 *
 ****************************************************************************/

static int focmodel_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode          *inode = filep->f_inode;
  FAR struct focmodel_dev_s *dev   = inode->i_private;
  int                        ret   = OK;
  irqstate_t                 flags;

  flags = enter_critical_section();

  switch (cmd)
    {
        /* Get the FOC driver configuration */

      case PWRIOC_SET_CONFIG:
        {
          FAR struct focmodel_cfg_s *cfg =
            (FAR struct focmodel_cfg_s *)arg;

          DEBUGASSERT(cfg != NULL);

          ret = focmodel_cfg_set(dev, cfg);
          if (ret != OK)
            {
              pwrerr("ERROR: PWRIOC_SET_CONFIG failed %d\n", ret);
            }

          break;
        }

        /* Set FOC parameters */

      case PWRIOC_SET_PARAMS:
        {
          FAR struct focmodel_params_s *params =
            (FAR struct focmodel_params_s *)arg;

          DEBUGASSERT(params != NULL);

          ret = focmodel_params_set(dev, params);
          if (ret != OK)
            {
              pwrerr("ERROR: PWRIOC_SET_PARAMS failed %d\n", ret);
            }

          break;
        }

      default:
        {
          pwrinfo("Unknown FOC model cmd: %d arg: %ld\n", cmd, arg);
          ret = ERROR;
          break;
        }
    }

  leave_critical_section(flags);

  return ret;
}

/****************************************************************************
 * Name: focmodel_lower_ops_assert
 *
 * Description:
 *   Assert the lower-half FOC model ops
 *
 ****************************************************************************/

static int focmodel_lower_ops_assert(FAR struct focmodel_lower_ops_s *ops)
{
  DEBUGASSERT(ops->bind);

  UNUSED(ops);

  return OK;
}

/****************************************************************************
 * Name: focmodel_typespec_assert
 *
 * Description:
 *   Assert the type-specific implementation
 *
 ****************************************************************************/

static int focmodel_typespec_assert(FAR struct focmodel_typespec_s *ts)
{
  DEBUGASSERT(ts);
  DEBUGASSERT(ts->init);
  DEBUGASSERT(ts->start);
  DEBUGASSERT(ts->vab);
  DEBUGASSERT(ts->current);
  DEBUGASSERT(ts->duty);
  DEBUGASSERT(ts->freq);

  UNUSED(ts);

  return OK;
}

/****************************************************************************
 * Name: focmodel_setup
 *
 * Description:
 *   Setup the FOC model driver
 *
 ****************************************************************************/

static int focmodel_setup(FAR struct focmodel_dev_s *dev)
{
  int ret = OK;

  DEBUGASSERT(dev);

  pwrinfo("FOCMODEL SETUP\n");

  /* Reset some data */

  dev->cfg_set    = false;
  dev->params_set = false;

  memset(&dev->cfg, 0, sizeof(struct focmodel_cfg_s));
  memset(&dev->params, 0, sizeof(struct focmodel_params_s));

  return ret;
}

/****************************************************************************
 * Name: focmodel_shutdown
 *
 * Description:
 *   Shutdown the FOC model driver
 *
 ****************************************************************************/

static int focmodel_shutdown(FAR struct focmodel_dev_s *dev)
{
  DEBUGASSERT(dev);

  pwrinfo("FOCMODEL SHUTDOWN\n");

  return OK;
}

/****************************************************************************
 * Name: focmodel_lower_bind
 *
 * Description:
 *   Bind the upper-half with the lower half FOC model driver
 *
 ****************************************************************************/

static int focmodel_lower_bind(FAR struct focmodel_dev_s *dev)
{
  DEBUGASSERT(dev);
  DEBUGASSERT(g_focmodel_callbacks.start);
  DEBUGASSERT(g_focmodel_callbacks.current);
  DEBUGASSERT(g_focmodel_callbacks.duty);
  DEBUGASSERT(g_focmodel_callbacks.freq);

  return FOCMODEL_OPS_BIND(dev, &g_focmodel_callbacks);
}

/****************************************************************************
 * Name: focmodel_cfg_set
 *
 * Description:
 *   Set the model configuration
 *
 ****************************************************************************/

static int focmodel_cfg_set(FAR struct focmodel_dev_s *dev,
                            FAR struct focmodel_cfg_s *cfg)
{
  DEBUGASSERT(dev);
  DEBUGASSERT(cfg);

  /* Copy parameters */

  memcpy(&dev->cfg, cfg, sizeof(struct focmodel_cfg_s));

  dev->cfg_set = true;

  return OK;
}

/****************************************************************************
 * Name: focmodel_params_set
 *
 * Description:
 *   Set the model parameters
 *
 ****************************************************************************/

static int focmodel_params_set(FAR struct focmodel_dev_s *dev,
                               FAR struct focmodel_params_s *params)
{
  DEBUGASSERT(dev);
  DEBUGASSERT(params);

  /* Copy parameters */

  memcpy(&dev->params, params, sizeof(struct focmodel_params_s));

  dev->params_set = true;

  return OK;
}

/****************************************************************************
 * Name: foc_model_register
 ****************************************************************************/

int foc_model_register(FAR const char *path, FAR struct focmodel_dev_s *dev,
                       int ftype)
{
  int ret = OK;

  DEBUGASSERT(path != NULL);
  DEBUGASSERT(dev != NULL);

  /* Lower-half must be initialized */

  DEBUGASSERT(dev->lower);
  DEBUGASSERT(dev->lower->ops);
  DEBUGASSERT(dev->lower->data);

  /* Check if the driver instance is supported by the driver */

  if (dev->devno > CONFIG_POWER_FOC_INST)
    {
      pwrerr("ERROR: unsupported foc devno %d\n\n", dev->devno);
      set_errno(EINVAL);
      ret = ERROR;
      goto errout;
    }

  /* Initialize FOC type-specific data */

  dev->ocount = 0;

  /* For now only float is supported */

  switch (ftype)
    {
#ifdef CONFIG_POWER_FOC_USE_FLOAT
      case FOC_NUMBER_TYPE_FLOAT:
        {
          dev->ftype = FOC_NUMBER_TYPE_FLOAT;

          /* Connect type-specific logic */

          dev->typespec = &g_focmodel_typespec_f32;

          break;
        }
#endif

#ifdef CONFIG_POWER_FOC_USE_FIXED16
      case FOC_NUMBER_TYPE_FIXED16:
        {
          dev->ftype = FOC_NUMBER_TYPE_FIXED16;

          /* Connect type-specific logic */

          dev->typespec = &g_focmodel_typespec_b16;

          break;
        }
#endif

      default:
        {
          ASSERT(0);
          break;
        }
    }

  /* Store device number */

  dev->devno = g_devno_cntr;

  /* Bind the upper-half with the lower half FOC model driver */

  ret = focmodel_lower_bind(dev);
  if (ret < 0)
    {
      pwrerr("ERROR: failed to bind FOC model callbacks %d\n", ret);
      set_errno(EINVAL);
      goto errout;
    }

  /* Assert the lower-half interface */

  ret = focmodel_lower_ops_assert(dev->lower->ops);
  if (ret < 0)
    {
      goto errout;
    }

  /* Assert the type-specific interface */

  ret = focmodel_typespec_assert(dev->typespec);
  if (ret < 0)
    {
      goto errout;
    }

  /* Initialize semaphores */

  nxsem_init(&dev->closesem, 0, 1);

  /* Initialize FOC model typespec */

  ret = FOCMODEL_TS_INIT(dev);
  if (ret < 0)
    {
      pwrerr("ERROR: failed to initialize FOC model typespec %d\n", ret);
      set_errno(ret);
      ret = ERROR;
      goto errout;
    }

  /* Register the FOC character driver */

  ret = register_driver(path, &g_focmodel_fops, 0444, dev);
  if (ret < 0)
    {
      nxsem_destroy(&dev->closesem);
      set_errno(ret);
      ret = ERROR;
      goto errout;
    }

errout:
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_POWER_FOC_USE_FLOAT
/****************************************************************************
 * Name: focf32_model_register
 ****************************************************************************/

int focf32_model_register(FAR const char *path,
                          FAR struct focmodel_dev_s *dev)
{
  return foc_model_register(path, dev, FOC_NUMBER_TYPE_FLOAT);
}
#endif

#ifdef CONFIG_POWER_FOC_USE_FIXED16
/****************************************************************************
 * Name: focb16_model_register
 ****************************************************************************/

int focb16_model_register(FAR const char *path,
                          FAR struct focmodel_dev_s *dev)
{
  return foc_model_register(path, dev, FOC_NUMBER_TYPE_FIXED16);
}
#endif
