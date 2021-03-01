/****************************************************************************
 * drivers/power/foc/fixed16/foc_svm_b16.c
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

#include <nuttx/power/foc/foc_lower.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if CONFIG_POWER_FOC_PHASES != 3
#  error
#endif

/* Enable current samples correction if 3-shunts */

#if CONFIG_POWER_FOC_SHUNTS == 3
#  define FOC_CORRECT_CURRENT_SAMPLES 1
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int foc_modulation_init_b16(FAR struct foc_dev_s *dev);
static int foc_modulation_start_b16(FAR struct foc_dev_s *dev);
static void foc_modulation_vbase_get_b16(FAR struct foc_dev_s *dev,
                                         FAR foc_number_t *vbus,
                                         FAR foc_number_t *vbase);
static void foc_modulation_current_b16(FAR struct foc_dev_s *dev,
                                       FAR foc_number_t *curr);
static void foc_modulation_b16(FAR struct foc_dev_s *dev,
                               FAR foc_abframe_t *v_ab_mod,
                               FAR foc_number_t *duty);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* FOC 3 phase SVM data */

static struct svm3_state_b16_s
  g_foc_svm3_b16[CONFIG_POWER_FOC_FIXED16_INST];

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* FOC modulation fixed16 interface */

struct foc_typespec_modulation_s g_foc_modulation_svm3_b16 =
{
  .init      = foc_modulation_init_b16,
  .start     = foc_modulation_start_b16,
  .vbase_get = foc_modulation_vbase_get_b16,
  .current   = foc_modulation_current_b16,
  .run       = foc_modulation_b16
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: foc_modulation_init_b16
 *
 * Description:
 *   Initialize the SVM3 modulation
 *
 * Parameters:
 *   dev (in) - FOC device
 *
 ****************************************************************************/

static int foc_modulation_init_b16(FAR struct foc_dev_s *dev)
{
  DEBUGASSERT(dev);

  /* Connect SVM3 modulation data */

  dev->modulation = (FAR void *)&g_foc_svm3_b16[dev->info.typeno];
  DEBUGASSERT(dev->modulation);

  return OK;
}

/****************************************************************************
 * Name: foc_modulation_start_b16
 *
 * Description:
 *   Start the SVM3 modulation
 *
 * Parameters:
 *   dev (in) - FOC device
 *
 ****************************************************************************/

static int foc_modulation_start_b16(FAR struct foc_dev_s *dev)
{
  FAR struct svm3_state_b16_s *svm = dev->modulation;

  DEBUGASSERT(dev);
  DEBUGASSERT(svm);

  /* Reset SVM3 data */

  memset(svm, 0, sizeof(struct svm3_state_b16_s));

  /* Initialize SVM3 data */

  svm3_init_b16(svm);

  return OK;
}

/****************************************************************************
 * Name: foc_modulation_vbase_get_b16
 *
 * Description:
 *   Get the modulation base voltage
 *
 * Parameters:
 *   dev   (in)  - FOC device
 *   vbus  (in)  - bus voltage
 *   vbase (out) - base voltage
 *
 ****************************************************************************/

static void foc_modulation_vbase_get_b16(FAR struct foc_dev_s *dev,
                                         FAR foc_number_t *vbus,
                                         FAR foc_number_t *vbase)
{
  b16_t pwm_max = 0;

  DEBUGASSERT(dev);
  DEBUGASSERT(vbus);
  DEBUGASSERT(vbase);

  /* Get max PWM duty cycle */

  pwm_max = dev->lower->hw_cfg.pwm_max.b16;
  DEBUGASSERT(pwm_max > 0);
  DEBUGASSERT(pwm_max < b16ONE);

  /* Get base voltage */

  vbase->b16 = b16mulb16(SVM3_BASE_VOLTAGE_GET_B16(vbus->b16), pwm_max);
}

/****************************************************************************
 * Name: foc_modulation_current_b16
 *
 * Description:
 *   Correct current samples accrding to the SVM3 modulation state
 *
 * Parameters:
 *   dev     (in)  - FOC device
 *   current (in)  - phase currents
 *
 ****************************************************************************/

static void foc_modulation_current_b16(FAR struct foc_dev_s *dev,
                                       FAR foc_number_t *curr)
{
  DEBUGASSERT(dev);
  DEBUGASSERT(curr);

#ifdef FOC_CORRECT_CURRENT_SAMPLES
  FAR struct svm3_state_b16_s *svm = dev->modulation;
  foc_abcframe_t               i_abc;

  DEBUGASSERT(svm);

  /* Get current abc frame */

  i_abc.b16.a = curr[0].b16;
  i_abc.b16.b = curr[1].b16;
  i_abc.b16.c = curr[2].b16;

  /* Correct ADC current samples */

  svm3_current_correct_b16(svm,
                           &i_abc.b16.a,
                           &i_abc.b16.b,
                           &i_abc.b16.c);
#endif
}

/****************************************************************************
 * Name: foc_modulation_b16
 *
 * Description:
 *   Handle the SVM3 modulation
 *
 * Parameters:
 *   dev      (in)  - FOC device
 *   v_ab_mod (in)  - modulation alpha-beta voltage normalized to 0.0-1.0
 *   duty     (out) - PWM duty cycle
 *
 ****************************************************************************/

static void foc_modulation_b16(FAR struct foc_dev_s *dev,
                               FAR foc_abframe_t *v_ab_mod,
                               FAR foc_number_t *duty)
{
  FAR struct svm3_state_b16_s *svm = dev->modulation;

  DEBUGASSERT(dev);
  DEBUGASSERT(v_ab_mod);
  DEBUGASSERT(duty);
  DEBUGASSERT(svm);

  /* Call 3-phase space vector modulation */

  svm3_b16(svm, &v_ab_mod->b16);

  /* Copy duty cycle */

  duty[0].b16 = svm->d_u;
  duty[1].b16 = svm->d_v;
  duty[2].b16 = svm->d_w;

  /* Saturate duty cycle */

  f_saturate_b16(&duty[0].b16, 0.0f, dev->lower->hw_cfg.pwm_max.b16);
  f_saturate_b16(&duty[1].b16, 0.0f, dev->lower->hw_cfg.pwm_max.b16);
  f_saturate_b16(&duty[2].b16, 0.0f, dev->lower->hw_cfg.pwm_max.b16);
}
