/****************************************************************************
 * drivers/power/focmodel/fixed16_model/foc_fixed16.c
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

#include "pmsm_fixed16.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if CONFIG_POWER_FOC_PHASES != 3
#  error
#endif

/****************************************************************************
 * Private Data Types
 ****************************************************************************/

/* Model data */

struct pmsm_model_data_b16_s
{
  struct pmsm_model_b16_s model;
  b16_t                   per;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Model data */

static struct pmsm_model_data_b16_s
  g_foc_model_b16[CONFIG_POWER_FOC_FIXED16_INST];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int focmodel_init_b16(FAR struct focmodel_dev_s *dev);
static int focmodel_start_b16(FAR struct focmodel_dev_s *dev);
static void focmodel_vab_b16(FAR struct focmodel_dev_s *dev,
                             FAR foc_abframe_t *v_ab);
static void focmodel_current_b16(FAR struct focmodel_dev_s *dev,
                                 FAR foc_number_t *current);
static void focmodel_duty_b16(FAR struct focmodel_dev_s *dev,
                              FAR foc_number_t *duty,
                              FAR foc_number_t *vbus);
static void focmodel_freq_b16(FAR struct focmodel_dev_s *dev, uint32_t freq);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* FOC model fixed16 interface */

struct focmodel_typespec_s g_focmodel_typespec_b16 =
{
  .init    = focmodel_init_b16,
  .start   = focmodel_start_b16,
  .vab     = focmodel_vab_b16,
  .current = focmodel_current_b16,
  .duty    = focmodel_duty_b16,
  .freq    = focmodel_freq_b16,
};

/* Device counter */

static uint8_t g_typeno_cntr = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: focmodel_init_b16
 ****************************************************************************/

static int focmodel_init_b16(FAR struct focmodel_dev_s *dev)
{
  int ret = OK;

  /* Check if the driver instance is supported by the driver */

  if (dev->typeno > CONFIG_POWER_FOC_FIXED16_INST)
    {
      pwrerr("ERROR: unsupported foc model fixed16 typeno %d\n\n",
             dev->typeno);
      set_errno(EINVAL);
      ret = ERROR;
      goto errout;
    }

  /* Connect FOC model */

  dev->model = (FAR void *)&g_foc_model_b16[dev->typeno];

  /* Store device number */

  dev->typeno = g_typeno_cntr;
  g_typeno_cntr += 1;

errout:
  return ret;
}

/****************************************************************************
 * Name: focmodel_start_b16
 ****************************************************************************/

static int focmodel_start_b16(FAR struct focmodel_dev_s *dev)
{
  FAR struct pmsm_model_data_b16_s *model = dev->model;
  struct pmsm_phy_params_b16_s     phy;
  int                              ret   = OK;

  DEBUGASSERT(dev);
  DEBUGASSERT(model);

  /* Reset data */

  memset(model, 0, sizeof(struct pmsm_model_b16_s));

  /* Initialize model */

  DEBUGASSERT(dev->cfg_set == true);

  pmsm_phy_params_init_b16(&phy,
                           dev->cfg.poles,
                           dev->cfg.res.b16,
                           dev->cfg.ind.b16,
                           dev->cfg.interia.b16,
                           dev->cfg.fluxlink.b16,
                           dev->cfg.ind_d.b16,
                           dev->cfg.ind_q.b16);

  DEBUGASSERT(model->per != 0);

  ret = pmsm_model_initialize_b16(&model->model, &phy, model->per);

  return ret;
}

/****************************************************************************
 * Name: focmodel_vab_b16
 ****************************************************************************/

static void focmodel_vab_b16(FAR struct focmodel_dev_s *dev,
                             FAR foc_abframe_t *v_ab)
{
  FAR struct pmsm_model_data_b16_s *model = dev->model;

  DEBUGASSERT(dev);
  DEBUGASSERT(model);
  DEBUGASSERT(v_ab);

  /* Update FOC motor electrical model */

  pmsm_model_elec_b16(&model->model, &v_ab->b16);

  /* Update FOC motor mechanical model */

  DEBUGASSERT(dev->params_set == true);

  pmsm_model_mech_b16(&model->model, dev->params.load.b16);
}

/****************************************************************************
 * Name: focmodel_current_b16
 ****************************************************************************/

static void focmodel_current_b16(FAR struct focmodel_dev_s *dev,
                                 FAR foc_number_t *current)
{
  FAR struct pmsm_model_data_b16_s *model = dev->model;

  DEBUGASSERT(dev);
  DEBUGASSERT(model);
  DEBUGASSERT(current);

  /* Copy motor currents */

  current[0].b16 = model->model.state.i_abc.a;
  current[1].b16 = model->model.state.i_abc.b;
  current[2].b16 = model->model.state.i_abc.c;
}

/****************************************************************************
 * Name: focmodel_duty_b16
 ****************************************************************************/

static void focmodel_duty_b16(FAR struct focmodel_dev_s *dev,
                              FAR foc_number_t *duty,
                              FAR foc_number_t *vbus)
{
  /* TODO: get v_ab from duty cycle and vbus */
}

/****************************************************************************
 * Name: focmodel_freq_b16
 ****************************************************************************/

static void focmodel_freq_b16(FAR struct focmodel_dev_s *dev, uint32_t freq)
{
  FAR struct pmsm_model_data_b16_s *model = dev->model;

  model->per = b16divi(b16ONE, freq);
}
