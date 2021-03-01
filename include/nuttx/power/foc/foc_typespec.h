/****************************************************************************
 * include/nuttx/power/foc/foc_typespec.h
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

#ifndef __INCLUDE_NUTTX_DRIVERS_POWER_FOC_FOC_TYPESPEC_H
#define __INCLUDE_NUTTX_DRIVERS_POWER_FOC_FOC_TYPESPEC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/power/foc/foc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Only for kernel side */

#ifndef __KERNEL__
#  error
#endif

/* Helper macros */

#define FOC_TS_CTRL_INIT(d)             (d)->typespec->control->init(d)
#define FOC_TS_CTRL_CFG_SET(d, c)       (d)->typespec->control->cfg_set(d, c)
#define FOC_TS_CTRL_CFG_GET(d, c)       (d)->typespec->control->cfg_get(d, c)
#define FOC_TS_CTRL_START(d)            (d)->typespec->control->start(d)
#define FOC_TS_CTRL_CURR_SET(d, i)      (d)->typespec->control->curr_set(d, i)
#define FOC_TS_CTRL_VOLTAGE(d, x, y)    (d)->typespec->control->voltage_run(d, x, y)
#define FOC_TS_CTRL_CURRENT(d, x, y, z) (d)->typespec->control->current_run(d, x, y, z)
#define FOC_TS_CTRL_FB(d, f)            (d)->typespec->control->fb_get(d, f)
#define FOC_TS_CTRL_PARAMS_SET(d, v, a) (d)->typespec->control->params_set(d, v, a)

#define FOC_TS_MOD_INIT(d)              (d)->typespec->modulation->init(d)
#define FOC_TS_MOD_START(d)             (d)->typespec->modulation->start(d)
#define FOC_TS_MOD_VBASE(d, v, b)       (d)->typespec->modulation->vbase_get(d, v, b)
#define FOC_TS_MOD_CURRENT(d, c)        (d)->typespec->modulation->current(d, c)
#define FOC_TS_MOD_RUN(d, m, x)         (d)->typespec->modulation->run(d, m, x)

#define FOC_TS_CONFIGURE(d, c)          (d)->typespec->configure(d, c)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* FOC controller interface */

struct foc_typespec_control_s
{
  /* Initialize the controller */

  CODE int  (*init)(FAR struct foc_dev_s *dev);

  /* Start the controller */

  CODE int  (*start)(FAR struct foc_dev_s *dev);

  /* Configure the controller */

  CODE int  (*cfg_set)(FAR struct foc_dev_s *dev, FAR void *data);

  /* Get controller configuration */

  CODE int  (*cfg_get)(FAR struct foc_dev_s *dev, FAR void *data);

  /* Feed the controller with currents */

  CODE void (*curr_set)(FAR struct foc_dev_s *dev,
                        FAR foc_number_t *curr);

  /* Feed the controller with VBASE and phase angle */

  CODE void (*params_set)(FAR struct foc_dev_s *dev,
                          FAR foc_number_t *vbase,
                          FAR foc_number_t *angle);

  /* Run voltage controller */

  CODE void (*voltage_run)(FAR struct foc_dev_s *dev,
                           FAR foc_dqframe_t *vdq_ref,
                           FAR foc_abframe_t *v_ab_mod);

  /* Run current controller */

  CODE void (*current_run)(FAR struct foc_dev_s *dev,
                           FAR foc_dqframe_t *idq_ref,
                           FAR foc_dqframe_t *vdq_comp,
                           FAR foc_abframe_t *v_ab_mod);

  /* Get feedback data */

  CODE void (*fb_get)(FAR struct foc_dev_s *dev,
                      FAR struct foc_fb_s *fb);
};

/* FOC modulation interface */

struct foc_typespec_modulation_s
{
  /* Initialize the modulation block */

  CODE int  (*init)(FAR struct foc_dev_s *dev);

  /* Start the modulation block */

  CODE int  (*start)(FAR struct foc_dev_s *dev);

  /* Get the base voltage for a given modulation scheme */

  CODE void (*vbase_get)(FAR struct foc_dev_s *dev,
                         FAR foc_number_t *vbus,
                         FAR foc_number_t *vbase);

  /* Modulation specific currents correction */

  CODE void (*current)(FAR struct foc_dev_s *dev,
                       FAR foc_number_t *curr);

  /* Modulation */

  CODE void (*run)(FAR struct foc_dev_s *dev,
                   FAR foc_abframe_t *v_ab_mod,
                   FAR foc_number_t *duty);
};

/* FOC type-specific implementation */

struct foc_typespec_s
{
  /* Configure type-specyfic logic */

  CODE int (*configure)(FAR struct foc_dev_s *dev,
                        FAR struct foc_cfgcmn_s *cfg);

  /* Controller interface */

  FAR struct foc_typespec_control_s *control;

  /* Modulation interface */

  FAR struct foc_typespec_modulation_s *modulation;
};

#endif /* __INCLUDE_NUTTX_DRIVERS_POWER_FOC_FOC_TYPESPEC_H */
