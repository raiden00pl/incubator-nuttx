/****************************************************************************
 * drivers/power/foc/float/foc_picontrol_f32.c
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

#include <nuttx/power/foc/foc_lower.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if CONFIG_POWER_FOC_PHASES != 3
#  error
#endif

/****************************************************************************
 * Private Data Types
 ****************************************************************************/

/* FOC PI float controller data */

struct foc_picontrol_f32_s
{
  bool                  cfg_set;    /* Controller configured flag */
  foc_number_t          vbase_last; /* Last VBASE sample */
  foc_angle_t           angle;      /* Phase angle */
  struct foc_ctrl_cfg_s cfg;        /* Controller configuration */
  struct foc_data_f32_s data;       /* Controller private data */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int foc_control_init_f32(FAR struct foc_dev_s *dev);
static int foc_control_start_f32(FAR struct foc_dev_s *dev);
static int foc_control_cfg_set_f32(FAR struct foc_dev_s *dev,
                                   FAR void *data);
static int foc_control_cfg_get_f32(FAR struct foc_dev_s *dev,
                                   FAR void *data);
static void foc_control_current_set_f32(FAR struct foc_dev_s *dev,
                                        FAR foc_number_t *curr);
static void foc_control_params_set_f32(FAR struct foc_dev_s *dev,
                                       FAR foc_number_t *vbase,
                                       FAR foc_number_t *angle);
static void foc_control_voltage_f32(FAR struct foc_dev_s *dev,
                                    FAR foc_dqframe_t *vdq_ref,
                                    FAR foc_abframe_t *v_ab_mod);
static void foc_control_current_f32(FAR struct foc_dev_s *dev,
                                    FAR foc_dqframe_t *idq_ref,
                                    FAR foc_dqframe_t *vdq_comp,
                                    FAR foc_abframe_t *v_ab_mod);
static void foc_control_fb_get_f32(FAR struct foc_dev_s *dev,
                                   FAR struct foc_fb_s *fb);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* FOC controller data */

static struct foc_picontrol_f32_s
  g_foc_picontrol_f32[CONFIG_POWER_FOC_FLOAT_INST];

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* FOC control float interface */

struct foc_typespec_control_s g_foc_control_pi_f32 =
{
  .init        = foc_control_init_f32,
  .start       = foc_control_start_f32,
  .cfg_set     = foc_control_cfg_set_f32,
  .cfg_get     = foc_control_cfg_get_f32,
  .curr_set    = foc_control_current_set_f32,
  .params_set  = foc_control_params_set_f32,
  .voltage_run = foc_control_voltage_f32,
  .current_run = foc_control_current_f32,
  .fb_get      = foc_control_fb_get_f32,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: foc_control_init_f32
 *
 * Description:
 *   Initialize the FOC controller
 *
 * Parameters:
 *   dev (in) - FOC device
 *
 ****************************************************************************/

static int foc_control_init_f32(FAR struct foc_dev_s *dev)
{
  FAR struct foc_picontrol_f32_s *foc = NULL;

  DEBUGASSERT(dev);

  /* Connect controller data */

  dev->control = (FAR void *)&g_foc_picontrol_f32[dev->info.typeno];

  foc = (struct foc_picontrol_f32_s *)dev->control;
  DEBUGASSERT(foc);

  /* Reset configuration flag */

  foc->cfg_set = false;

  return OK;
}

/****************************************************************************
 * Name: foc_control_start_f32
 *
 * Description:
 *   Start the FOC controller
 *
 * Parameters:
 *   dev (in) - FOC device
 *
 ****************************************************************************/

static int foc_control_start_f32(FAR struct foc_dev_s *dev)
{
  FAR struct foc_picontrol_f32_s *foc = dev->control;
  struct foc_initdata_f32_s       init;
  int                             ret = OK;

  DEBUGASSERT(dev);
  DEBUGASSERT(foc);

  /* Must be configured */

  if (foc->cfg_set == false)
    {
      pwrerr("ERROR: picontrol_f32 %d not configured!\n", dev->info.typeno);
      ret = -EACCES;
      goto errout;
    }

  /* Reset controler data */

  memset(&foc->data, 0, sizeof(struct foc_data_f32_s));
  memset(&foc->vbase_last, 0, sizeof(foc_number_t));
  memset(&foc->angle, 0, sizeof(foc_angle_t));

  /* Get FOC init data */

  init.id_kp = foc->cfg.id_kp.f32;
  init.id_ki = foc->cfg.id_ki.f32;
  init.iq_kp = foc->cfg.iq_kp.f32;
  init.iq_ki = foc->cfg.iq_ki.f32;

  /* Initialize FOC controller data */

  foc_init(&foc->data, &init);

errout:
  return ret;
}

/****************************************************************************
 * Name: foc_control_cfg_set_f32
 *
 * Description:
 *   Configure the FOC controller
 *
 * Parameters:
 *   dev  (in) - FOC device
 *   data (in) - controller configuration data
 *
 ****************************************************************************/

static int foc_control_cfg_set_f32(FAR struct foc_dev_s *dev,
                                   FAR void *data)
{
  FAR struct foc_picontrol_f32_s *foc = dev->control;

  DEBUGASSERT(dev);
  DEBUGASSERT(data);
  DEBUGASSERT(foc);

  /* Copy data */

  memcpy(&foc->cfg, data, sizeof(struct foc_ctrl_cfg_s));

  /* Set configuration flag */

  foc->cfg_set = true;

  return OK;
}

/****************************************************************************
 * Name: foc_control_cfg_get_f32
 *
 * Description:
 *   Get configuration
 *
 * Parameters:
 *   dev  (in)  - FOC device
 *   data (out) - controller configuration data
 *
 ****************************************************************************/

static int foc_control_cfg_get_f32(FAR struct foc_dev_s *dev,
                                   FAR void *data)
{
  FAR struct foc_picontrol_f32_s *foc = dev->control;

  DEBUGASSERT(dev);
  DEBUGASSERT(data);
  DEBUGASSERT(foc);

  /* Copy data */

  memcpy(data, &foc->cfg, sizeof(struct foc_ctrl_cfg_s));

  return OK;
}

/****************************************************************************
 * Name: foc_control_current_set_f32
 *
 * Description:
 *   Feed the controller with phase currents
 *
 * Parameters:
 *   dev  (in) - FOC device
 *   curr (in) - phase currents
 *
 ****************************************************************************/

static void foc_control_current_set_f32(FAR struct foc_dev_s *dev,
                                        FAR foc_number_t *curr)
{
  FAR struct foc_picontrol_f32_s *foc = dev->control;
  foc_abcframe_t                  i_abc;

  DEBUGASSERT(dev);
  DEBUGASSERT(curr);
  DEBUGASSERT(foc);

  /* Get current in abc frame */

  i_abc.f32.a = curr[0].f32;
  i_abc.f32.b = curr[1].f32;
  i_abc.f32.c = curr[2].f32;

  foc_iabc_update(&foc->data, &i_abc.f32);
}

/****************************************************************************
 * Name: foc_control_params_set_f32
 *
 * Description:
 *   Update VBASE and angle
 *
 * Parameters:
 *   dev   (in) - FOC device
 *   vbase (in) - base voltage
 *   angle (in) - phase angle
 *
 ****************************************************************************/

static void foc_control_params_set_f32(FAR struct foc_dev_s *dev,
                                       FAR foc_number_t *vbase,
                                       FAR foc_number_t *angle)
{
  FAR struct foc_picontrol_f32_s *foc = dev->control;

  DEBUGASSERT(dev);
  DEBUGASSERT(vbase);
  DEBUGASSERT(angle);
  DEBUGASSERT(foc);

  /* Update base voltage only if changed */

  if (foc->vbase_last.f32 != vbase->f32)
    {
      /* Update FOC base voltage */

      foc_vbase_update(&foc->data, vbase->f32);

      /* Update last FOC base voltage  */

      foc->vbase_last.f32 = vbase->f32;
    }

#ifndef CONFIG_POWER_FOC_HAVE_CORDIC
  phase_angle_update(&foc->angle.f32, angle->f32);
#else
  FOC_OPS_ANGLE_UPDATE(dev, &foc->angle, angle);
#endif

  /* Feed the controller with phase angle */

  foc_angle_update(&foc->data, &foc->angle.f32);
}

/****************************************************************************
 * Name: foc_control_voltage_f32
 *
 * Description:
 *  Handle the FOC voltage control
 *
 * Parameters:
 *   dev      (in)  - FOC device
 *   vdq_ref  (in)  - voltage dq reference frame
 *   v_ab_mod (out) - output modulation voltage
 *
 ****************************************************************************/

static void foc_control_voltage_f32(FAR struct foc_dev_s *dev,
                                    FAR foc_dqframe_t *vdq_ref,
                                    FAR foc_abframe_t *v_ab_mod)
{
  FAR struct foc_picontrol_f32_s *foc     = dev->control;
  float                           mag_max = 0;

  DEBUGASSERT(dev);
  DEBUGASSERT(vdq_ref);
  DEBUGASSERT(v_ab_mod);
  DEBUGASSERT(foc);

  /* Get maximum possible voltage DQ vetor magnitude */

  foc_vdq_mag_max_get(&foc->data, &mag_max);

  /* Saturate voltage DQ vector */

#ifndef CONFIG_POWER_FOC_HAVE_CORDIC
  dq_saturate(&vdq_ref->f32, mag_max);
#else
  FOC_OPS_DQ_SATURATE(dev, &vdq_ref->f32, mag_max);
#endif

  /* Call FOC voltage controller */

  foc_voltage_control(&foc->data, &vdq_ref->f32);

  /* Get output v_ab_mod frame */

  foc_vabmod_get(&foc->data, &v_ab_mod->f32);
}

/****************************************************************************
 * Name: foc_control_current_f32
 *
 * Description:
 *  Handle the FOC current control
 *
 * Parameters:
 *   dev      (in)  - FOC device
 *   idq_ref  (in)  - current dq reference frame
 *   vdq_comp (in)  - voltage dq compensation frame
 *   v_ab_mod (out) - output modulation voltage
 *
 ****************************************************************************/

static void foc_control_current_f32(FAR struct foc_dev_s *dev,
                                    FAR foc_dqframe_t *idq_ref,
                                    FAR foc_dqframe_t *vdq_comp,
                                    FAR foc_abframe_t *v_ab_mod)
{
  FAR struct foc_picontrol_f32_s *foc = dev->control;
  dq_frame_f32_t                  v_dq_ref;

  DEBUGASSERT(dev);
  DEBUGASSERT(idq_ref);
  DEBUGASSERT(vdq_comp);
  DEBUGASSERT(v_ab_mod);
  DEBUGASSERT(foc);

  v_dq_ref.d = 0.0f;
  v_dq_ref.q = 0.0f;

  /* Call FOC current controller */

  foc_current_control(&foc->data, &idq_ref->f32, &vdq_comp->f32, &v_dq_ref);

  /* Call FOC voltage control */

  foc_voltage_control(&foc->data, &v_dq_ref);

  /* Get output v_ab_mod frame */

  foc_vabmod_get(&foc->data, &v_ab_mod->f32);
}

/****************************************************************************
 * Name: foc_control_fb_get_f32
 *
 * Description:
 *   Get the FOC feedback data
 *
 * Parameters:
 *   dev (in)  - FOC device
 *   fb  (out) - FOC feedback data
 *
 ****************************************************************************/

static void foc_control_fb_get_f32(FAR struct foc_dev_s *dev,
                                   FAR struct foc_fb_s *fb)
{
  FAR struct foc_picontrol_f32_s *foc = dev->control;

  DEBUGASSERT(dev);
  DEBUGASSERT(fb);
  DEBUGASSERT(foc);

  /* Copy DQ voltage */

  fb->vdq.f32.q = foc->data.v_dq.q;
  fb->vdq.f32.d = foc->data.v_dq.d;

  /* Copy DQ current */

  fb->idq.f32.q = foc->data.i_dq.q;
  fb->idq.f32.d = foc->data.i_dq.d;

  /* Copy alpha-beta current */

  fb->iab.f32.a = foc->data.i_ab.a;
  fb->iab.f32.b = foc->data.i_ab.b;

  /* Copy alpha-beta voltage */

  fb->vab.f32.a = foc->data.v_ab.a;
  fb->vab.f32.b = foc->data.v_ab.b;

  /* Copy phase current */

  fb->curr[0].f32 = foc->data.i_abc.a;
  fb->curr[1].f32 = foc->data.i_abc.b;
  fb->curr[2].f32 = foc->data.i_abc.c;

  /* Copy phase voltage */

  fb->volt[0].f32 = foc->data.v_abc.a;
  fb->volt[1].f32 = foc->data.v_abc.b;
  fb->volt[2].f32 = foc->data.v_abc.c;
}
