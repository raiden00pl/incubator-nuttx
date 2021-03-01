/****************************************************************************
 * drivers/power/focmodel/focmodel_callback.c
 * Common FOC model callbacks
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
#include <debug.h>

#include <nuttx/power/focmodel/focmodel_lower.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int focmodel_start(FAR struct focmodel_dev_s *dev);
static void focmodel_vab(FAR struct focmodel_dev_s *dev,
                         FAR foc_abframe_t *v_ab);
static void focmodel_current(FAR struct focmodel_dev_s *dev,
                             FAR foc_number_t *current);
static void focmodel_duty(FAR struct focmodel_dev_s *dev,
                          FAR foc_number_t *duty,
                          FAR foc_number_t *vbus);
static void focmodel_freq(FAR struct focmodel_dev_s *dev, uint32_t freq);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* FOC callbacks from the arch-specific logic to this driver */

struct focmodel_callbacks_s g_focmodel_callbacks =
{
  .start   = focmodel_start,
  .vab     = focmodel_vab,
  .current = focmodel_current,
  .duty    = focmodel_duty,
  .freq    = focmodel_freq,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: focmodel_start
 ****************************************************************************/

static int focmodel_start(FAR struct focmodel_dev_s *dev)
{
  int ret = OK;

  if (dev->params_set == false)
    {
      pwrerr("ERROR: FOCMODEL params_set == false\n");
      ret = ERROR;
      goto errout;
    }

  if (dev->cfg_set == false)
    {
      pwrerr("ERROR: FOCMODEL cfg_set == false\n");
      ret = ERROR;
      goto errout;
    }

  ret = FOCMODEL_TS_START(dev);

errout:
  return ret;
}

/****************************************************************************
 * Name: focmodel_vab
 ****************************************************************************/

static void focmodel_vab(FAR struct focmodel_dev_s *dev,
                         FAR foc_abframe_t *v_ab)
{
  FOCMODEL_TS_VAB(dev, v_ab);
}

/****************************************************************************
 * Name: focmodel_current
 ****************************************************************************/

static void focmodel_current(FAR struct focmodel_dev_s *dev,
                             FAR foc_number_t *current)
{
  FOCMODEL_TS_CURRENT(dev, current);
}

/****************************************************************************
 * Name: focmodel_duty
 ****************************************************************************/

static void focmodel_duty(FAR struct focmodel_dev_s *dev,
                          FAR foc_number_t *duty,
                          FAR foc_number_t *vbus)
{
  FOCMODEL_TS_DUTY(dev, duty, vbus);
}

/****************************************************************************
 * Name: focmodel_freq
 ****************************************************************************/

static void focmodel_freq(FAR struct focmodel_dev_s *dev, uint32_t freq)
{
  FOCMODEL_TS_FREQ(dev, freq);
}
