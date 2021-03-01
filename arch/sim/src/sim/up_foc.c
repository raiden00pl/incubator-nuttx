/****************************************************************************
 * arch/sim/src/sim/up_foc.h
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
#include <stdlib.h>
#include <strings.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/power/foc/foc_lower.h>

#ifdef CONFIG_POWER_FOCMODEL
#  include <nuttx/power/focmodel/focmodel_lower.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_POWER_FOC
#  error Upper-half FOC driver must be enabled
#endif

/* Board HW configuration */

#define SIM_FOC_HW_PWM_NS      (500)
#define SIM_FOC_HW_PWM_MAX     (0.95f)

/* Helper macros ************************************************************/

#define SIM_FOC_DATA_FROM_DEV_GET(d)                                     \
  ((FAR struct sim_foc_data_s *)(d)->lower->data)

#ifdef CONFIG_POWER_FOCMODEL
#  define SIM_FOCMODEL_DATA_FROM_DEV_GET(d)                              \
  ((FAR struct sim_focmodel_data_s *)(d)->lower->data)
#  define SIM_FOCMODEL_DATA(d) g_sim_focmodel_data[d->info.devno]
#  define SIM_FOCMODEL_DEV(d) g_focmodel_dev[d->info.devno]
#  define SIM_FOCMODEL_CB_START(d)                                       \
  SIM_FOCMODEL_DATA(d).cb->start(&SIM_FOCMODEL_DEV(d))
#  define SIM_FOCMODEL_CB_FREQ(d, f)                                     \
  SIM_FOCMODEL_DATA(d).cb->freq(&SIM_FOCMODEL_DEV(d), f)
#  define SIM_FOCMODEL_CB_CURR(d, c, v)                                  \
  SIM_FOCMODEL_DATA(d).cb->current(&SIM_FOCMODEL_DEV(d), c, v)
#  define SIM_FOCMODEL_CB_DUTY(d, x, y)                                  \
  SIM_FOCMODEL_DATA(d).cb->duty(&SIM_FOCMODEL_DEV(d), x, y)
#endif  /* CONFIG_POWER_FOCMODEL */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* SIM FOC board data */

struct sim_foc_board_s
{
  uint32_t reserved;
};

/* SIM FOC specific data */

struct sim_foc_data_s
{
  /* Upper-half FOC controller callbacks */

  FAR const struct foc_callbacks_s *cb;

  /* SIM FOC board data */

  FAR struct sim_foc_board_s *board;

  /* Phase currents */

  foc_number_t current[CONFIG_POWER_FOC_PHASES];

  /* FOC worker loop helpers */

  bool     worker_state;
  uint8_t  notifier_div;
  uint32_t worker_cntr;
  uint32_t pwm_freq;
  uint32_t work_freq;
  uint32_t work_div;
};

#ifdef CONFIG_POWER_FOCMODEL
/* SIM FOC model specific data */

struct sim_focmodel_data_s
{
  /* Upper-half FOC model callbacks */

  FAR const struct focmodel_callbacks_s *cb;
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Ops */

static int sim_foc_configure(FAR struct foc_dev_s *dev,
                             FAR struct foc_cfgcmn_s *cfg);
static int sim_foc_setup(FAR struct foc_dev_s *dev);
static int sim_foc_shutdown(FAR struct foc_dev_s *dev);
static int sim_foc_start(FAR struct foc_dev_s *dev, bool state);
static int sim_foc_pwm_duty_set(FAR struct foc_dev_s *dev,
                                FAR foc_number_t *duty);
static int sim_foc_ioctl(FAR struct foc_dev_s *dev, int cmd,
                         unsigned long arg);
static int sim_foc_bind(FAR struct foc_dev_s *dev,
                        FAR struct foc_callbacks_s *cb);
static int sim_foc_fault_clean(FAR struct foc_dev_s *dev);
#ifdef CONFIG_POWER_FOC_HAVE_CORDIC
static void sim_foc_angle_update(FAR struct foc_dev_s *dev,
                                 FAR foc_angle_t *angle_out,
                                 FAR foc_number_t *angle_in);
static void sim_foc_dq_saturate(FAR struct foc_dev_s *dev,
                                FAR foc_dqframe_t *dq,
                                FAR foc_number_t *max);
#endif
#ifdef CONFIG_POWER_FOC_TRACE
static void sim_foc_trace(FAR struct foc_dev_s *dev, int type, bool state);
#endif

/* Handlers */

static int sim_foc_worker_handler(FAR struct foc_dev_s *dev);

/* Helpers */

static void sim_foc_hw_config_get(FAR struct foc_dev_s *dev);
static int sim_foc_work_cfg(FAR struct foc_dev_s *dev, uint32_t freq);
static int sim_foc_notifier_cfg(FAR struct foc_dev_s *dev, uint32_t freq);
static int sim_foc_pwm_setup(FAR struct foc_dev_s *dev, uint32_t freq);
static int sim_foc_pwm_start(FAR struct foc_dev_s *dev, bool state);
static int sim_foc_adc_setup(FAR struct foc_dev_s *dev);
static int sim_foc_adc_start(FAR struct foc_dev_s *dev, bool state);
static void sim_foc_work(FAR struct foc_dev_s *dev);

/* Model ops */

#ifdef CONFIG_POWER_FOCMODEL
static int sim_focmodel_bind(FAR struct focmodel_dev_s *dev,
                             FAR struct focmodel_callbacks_s *cb);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* SIM FOC specific data */

static struct sim_foc_data_s  g_sim_foc_data[CONFIG_POWER_FOC_INST];
static struct sim_foc_board_s g_sim_foc_board[CONFIG_POWER_FOC_INST];

/* SIM specific FOC ops */

static struct foc_lower_ops_s g_sim_foc_ops =
{
  .configure      = sim_foc_configure,
  .setup          = sim_foc_setup,
  .shutdown       = sim_foc_shutdown,
  .start          = sim_foc_start,
  .pwm_duty_set   = sim_foc_pwm_duty_set,
  .ioctl          = sim_foc_ioctl,
  .bind           = sim_foc_bind,
  .fault_clean    = sim_foc_fault_clean,
#ifdef CONFIG_POWER_FOC_HAVE_CORDIC
  .angle_update   = sim_foc_angle_update,
  .dq_saturate    = sim_foc_dq_saturate
#endif
#ifdef CONFIG_POWER_FOC_TRACE
  .trace          = sim_foc_trace
#endif
};

/* FOC lower-half */

static struct foc_lower_s g_sim_foc_lower[CONFIG_POWER_FOC_INST];

/* FOC upper-half device data */

static struct foc_dev_s g_foc_dev[CONFIG_POWER_FOC_INST];

#ifdef CONFIG_POWER_FOCMODEL
/* FOC model ops */

static struct focmodel_lower_ops_s g_sim_focmodel_ops =
{
  .bind = sim_focmodel_bind,
};

/* FOC model specific data */

static struct sim_focmodel_data_s g_sim_focmodel_data[CONFIG_POWER_FOC_INST];

/* FOC model lower-half */

static struct focmodel_lower_s g_sim_focmodel_lower[CONFIG_POWER_FOC_INST];

/* FOC model upper-half device data */

static struct focmodel_dev_s g_focmodel_dev[CONFIG_POWER_FOC_INST];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_foc_pwm_setup
 *
 * Description:
 *   Setup PWM for FOC controller
 *
 ****************************************************************************/

static int sim_foc_pwm_setup(FAR struct foc_dev_s *dev, uint32_t freq)
{
  FAR struct sim_foc_data_s *sim = SIM_FOC_DATA_FROM_DEV_GET(dev);

  DEBUGASSERT(dev);
  DEBUGASSERT(sim);

  pwrinfo("[PWM_SETUP] devno=%d freq=%d\n", dev->info.devno, freq);

  DEBUGASSERT(freq > 0);

  /* Store frequency */

  sim->pwm_freq = freq;

#ifdef CONFIG_POWER_FOCMODEL
  /* Configure model frequency */

  SIM_FOCMODEL_CB_FREQ(dev, freq);
#endif

  return OK;
}

/****************************************************************************
 * Name: sim_foc_start
 *
 * Description:
 *   Start/stop PWM for the FOC controller
 *
 ****************************************************************************/

static int sim_foc_start(FAR struct foc_dev_s *dev, bool state)
{
  FAR struct sim_foc_data_s *sim = SIM_FOC_DATA_FROM_DEV_GET(dev);
  irqstate_t                 flags;
  int                        ret = OK;

  pwrinfo("[FOC_START] devno=%d state=%d\n", dev->info.devno, state);

  ret = sim_foc_pwm_start(dev, state);
  if (ret < 0)
    {
      pwrerr("ERROR: sim_foc_pwm_start failed %d\n", ret);
      goto errout;
    }

  ret = sim_foc_adc_start(dev, state);
  if (ret < 0)
    {
      pwrerr("ERROR: sim_foc_adc_start failed %d\n", ret);
      goto errout;
    }

  /* Store FOC worker state */

  flags = enter_critical_section();
  sim->worker_state = state;
  leave_critical_section(flags);

#ifdef CONFIG_POWER_FOCMODEL
  if (state == true)
    {
      DEBUGASSERT(sim->cb->foc_work);
      DEBUGASSERT(sim->cb->notifier);

      /* Start model */

      ret = SIM_FOCMODEL_CB_START(dev);
      if (ret < 0)
        {
          pwrerr("ERROR: SIM_FOCMODEL_CB_START failed %d\n", ret);
          goto errout;
        }
    }
#endif

errout:
  return ret;
}

/****************************************************************************
 * Name: sim_foc_pwm_start
 *
 * Description:
 *   Start/stop PWM for the FOC controller
 *
 ****************************************************************************/

static int sim_foc_pwm_start(FAR struct foc_dev_s *dev, bool state)
{
  DEBUGASSERT(dev);

  pwrinfo("[PWM_START] devno=%d state=%d\n", dev->info.devno, state);

  return OK;
}

/****************************************************************************
 * Name: sim_foc_adc_setup
 *
 * Description:
 *   Setup ADC for the FOC controller
 *
 ****************************************************************************/

static int sim_foc_adc_setup(FAR struct foc_dev_s *dev)
{
  DEBUGASSERT(dev);

  pwrinfo("[ADC_SETUP] devno=%d\n", dev->info.devno);

  return OK;
}

/****************************************************************************
 * Name: sim_foc_adc_start
 *
 * Description:
 *   Start/stop ADC conversion for the FOC controller
 *
 ****************************************************************************/

static int sim_foc_adc_start(FAR struct foc_dev_s *dev, bool state)
{
  DEBUGASSERT(dev);

  pwrinfo("[ADC_START] devno=%d state=%d\n", dev->info.devno, state);

  return OK;
}

/****************************************************************************
 * Name: sim_foc_work_cfg
 *
 * Description:
 *   Configure FOC work
 *
 ****************************************************************************/

static int sim_foc_work_cfg(FAR struct foc_dev_s *dev, uint32_t freq)
{
  FAR struct sim_foc_data_s *sim = SIM_FOC_DATA_FROM_DEV_GET(dev);
  int                        ret = OK;

  DEBUGASSERT(dev);
  DEBUGASSERT(sim);

  pwrinfo("[WORK_CFG] devno=%d freq=%d\n", dev->info.devno, freq);

  DEBUGASSERT(freq > 0);

  if (sim->pwm_freq % freq != 0)
    {
      ret = -EINVAL;
      goto errout;
    }

  /* Call FOC worker on every update */

  sim->work_freq = freq;
  sim->work_div  = 1;

errout:
  return ret;
}

/****************************************************************************
 * Name: sim_foc_notifier_cfg
 *
 * Description:
 *   Configure FOC notifier
 *
 ****************************************************************************/

static int sim_foc_notifier_cfg(FAR struct foc_dev_s *dev, uint32_t freq)
{
  FAR struct sim_foc_data_s *sim = SIM_FOC_DATA_FROM_DEV_GET(dev);
  int                        ret = OK;

  DEBUGASSERT(dev);
  DEBUGASSERT(sim);

  pwrinfo("[NOTIFIER_CFG] devno=%d freq=%d\n", dev->info.devno, freq);

  DEBUGASSERT(freq > 0);

  if (sim->pwm_freq % freq != 0)
    {
      ret = -EINVAL;
      goto errout;
    }

  /* Set divider */

  sim->notifier_div = (sim->pwm_freq / freq);

errout:
  return ret;
}

/****************************************************************************
 * Name: sim_foc_configure
 *
 * Description:
 *   Arch-specific FOC controller configuration
 *
 ****************************************************************************/

static int sim_foc_configure(FAR struct foc_dev_s *dev,
                             FAR struct foc_cfgcmn_s *cfg)
{
  FAR struct sim_foc_data_s *sim = SIM_FOC_DATA_FROM_DEV_GET(dev);
  int                        ret = OK;

  DEBUGASSERT(dev);
  DEBUGASSERT(sim);

  DEBUGASSERT(cfg->pwm_freq > 0);
  DEBUGASSERT(cfg->work_freq > 0);
  DEBUGASSERT(cfg->notifier_freq > 0);

  pwrinfo("[FOC_SETUP] devno=%d\n", dev->info.devno);

  /* Configure ADC */

  ret = sim_foc_adc_setup(dev);
  if (ret < 0)
    {
      pwrerr("ERROR: sim_foc_adc_setup failed %d\n", ret);
      goto errout;
    }

  /* Configure PWM */

  ret = sim_foc_pwm_setup(dev, cfg->pwm_freq);
  if (ret < 0)
    {
      pwrerr("ERROR: sim_foc_pwm_setup failed %d\n", ret);
      goto errout;
    }

  /* Configure worker */

  ret = sim_foc_work_cfg(dev, cfg->work_freq);
  if (ret < 0)
    {
      pwrerr("ERROR: sim_foc_work_cfg failed %d\n", ret);
      goto errout;
    }

  /* Configure notifier */

  ret = sim_foc_notifier_cfg(dev, cfg->notifier_freq);
  if (ret < 0)
    {
      pwrerr("ERROR: sim_foc_notifier_cfg failed %d\n", ret);
      goto errout;
    }

  /* Get HW configuration */

  sim_foc_hw_config_get(dev);

  /* Reset some data */

  sim->worker_cntr = 0;

errout:
  return ret;
}

/****************************************************************************
 * Name: sim_foc_setup
 *
 * Description:
 *   Arch-specific FOC controller setup
 *
 ****************************************************************************/

static int sim_foc_setup(FAR struct foc_dev_s *dev)
{
  DEBUGASSERT(dev);

  pwrinfo("[FOC_SETUP] devno=%d\n", dev->info.devno);

  return OK;
}

/****************************************************************************
 * Name: sim_foc_shutdown
 *
 * Description:
 *   Arch-specific FOC controller shutdown
 *
 ****************************************************************************/

static int sim_foc_shutdown(FAR struct foc_dev_s *dev)
{
  DEBUGASSERT(dev);

  pwrinfo("[FOC_SHUTDOWN] devno=%d\n", dev->info.devno);

  return OK;
}

/****************************************************************************
 * Name: sim_foc_ioctl
 *
 * Description:
 *   Arch-specific FOC controller ioctl
 *
 ****************************************************************************/

static int sim_foc_ioctl(FAR struct foc_dev_s *dev, int cmd,
                         unsigned long arg)
{
  int ret = OK;

  DEBUGASSERT(dev);

  pwrinfo("[FOC_IOCTL] devno=%d cmd=%d\n", dev->info.devno, cmd);

  switch (cmd)
    {
      default:
        {
          ret = -1;
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: sim_foc_work
 *
 * Description:
 *   Perform FOC work
 *
 ****************************************************************************/

static void sim_foc_work(FAR struct foc_dev_s *dev)
{
  FAR struct sim_foc_data_s *sim = SIM_FOC_DATA_FROM_DEV_GET(dev);
  irqstate_t                 flags;

  DEBUGASSERT(dev);
  DEBUGASSERT(sim);

  flags = enter_critical_section();

  /* Call FOC worker handler */

  if (sim->worker_cntr % sim->work_div == 0)
    {
#ifdef CONFIG_POWER_FOCMODEL
      /* Get currents from model */

      SIM_FOCMODEL_CB_CURR(dev, sim->current, &dev->state.fb.vab);
#endif

      /* Call FOC work */

      sim->cb->foc_work(dev, sim->current);
    }

  if (sim->worker_cntr % sim->notifier_div == 0)
    {
      /* Notify user-space */

      sim->cb->notifier(dev);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: sim_foc_worker_handler
 *
 * Description:
 *   Handle ADC conversion and do controller work.
 *
 ****************************************************************************/

static int sim_foc_worker_handler(FAR struct foc_dev_s *dev)
{
  FAR struct sim_foc_data_s *sim = SIM_FOC_DATA_FROM_DEV_GET(dev);

  DEBUGASSERT(dev);
  DEBUGASSERT(sim);

  pwrinfo("[FOC_WORKER_HANDLER] devno=%d cntr=%d\n",
          dev->info.devno, sim->worker_cntr);

  /* Run FOC work */

  sim_foc_work(dev);

  /* Increase counter */

  sim->worker_cntr += 1;

  return OK;
}

/****************************************************************************
 * Name: sim_duty_set
 *
 * Description:
 *   Set 3-phase PWM duty cycle
 *
 ****************************************************************************/

static int sim_foc_pwm_duty_set(FAR struct foc_dev_s *dev,
                                FAR foc_number_t *duty)
{
  int i = 0;

  DEBUGASSERT(dev);
  DEBUGASSERT(duty);

#ifdef CONFIG_POWER_FOCMODEL
  /* Feed the model with the new duty cycle and bus voltage */

  SIM_FOCMODEL_CB_DUTY(dev, duty, &dev->params.vbus);
#endif

  switch (dev->info.ftype)
    {
#ifdef CONFIG_POWER_FOC_USE_FLOAT
      case FOC_NUMBER_TYPE_FLOAT:
        {
          for (i = 0; i < CONFIG_POWER_FOC_PHASES; i += 1)
            {
              DEBUGASSERT(duty[i].f32 >= 0.0f);
            }

          pwrinfo("[PWM_DUTY_SET] devno=%d duty= ", dev->info.devno);

          for (i = 0; i < CONFIG_POWER_FOC_PHASES; i += 1)
            {
              pwrinfo("%.03f ", duty[i].f32);
            }

          pwrinfo("\n");

          break;
        }
#endif

#ifdef CONFIG_POWER_FOC_USE_FIXED16
      case FOC_NUMBER_TYPE_FIXED16:
        {
          for (i = 0; i < CONFIG_POWER_FOC_PHASES; i += 1)
            {
              DEBUGASSERT(duty[i].b16 >= 0);
            }

          pwrinfo("[PWM_DUTY_SET] devno=%d duty= ", dev->info.devno);

          for (i = 0; i < CONFIG_POWER_FOC_PHASES; i += 1)
            {
              pwrinfo("%.03f ", b16tof(duty[i].b16));
            }

          pwrinfo("\n");

          break;
        }
#endif

      default:
        {
          ASSERT(0);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: sim_foc_hw_config_get
 *
 * Description:
 *   Get HW configuration for FOC controller
 *
 ****************************************************************************/

static void sim_foc_hw_config_get(FAR struct foc_dev_s *dev)
{
  FAR struct foc_lower_s *lower = dev->lower;

  DEBUGASSERT(dev);
  DEBUGASSERT(lower);
  DEBUGASSERT(dev->info.ftype);

  switch (dev->info.ftype)
    {
#ifdef CONFIG_POWER_FOC_USE_FLOAT
      case FOC_NUMBER_TYPE_FLOAT:
        {
          lower->hw_cfg.pwm_dt_ns   = SIM_FOC_HW_PWM_NS;
          lower->hw_cfg.pwm_max.f32 = SIM_FOC_HW_PWM_MAX;

          break;
        }
#endif

#ifdef CONFIG_POWER_FOC_USE_FIXED16
      case FOC_NUMBER_TYPE_FIXED16:
        {
          lower->hw_cfg.pwm_dt_ns   = SIM_FOC_HW_PWM_NS;
          lower->hw_cfg.pwm_max.b16 = ftob16(SIM_FOC_HW_PWM_MAX);

          break;
        }
#endif

      default:
        {
          ASSERT(0);
          break;
        }
    }
}

/****************************************************************************
 * Name: sim_foc_bind
 *
 * Description:
 *   Bind lower-half FOC controller with upper-half FOC logic
 *
 ****************************************************************************/

static int sim_foc_bind(FAR struct foc_dev_s *dev,
                        FAR struct foc_callbacks_s *cb)
{
  FAR struct sim_foc_data_s *sim = SIM_FOC_DATA_FROM_DEV_GET(dev);
  int                        ret = OK;

  DEBUGASSERT(dev);
  DEBUGASSERT(cb);
  DEBUGASSERT(sim);

  pwrinfo("[FOC_BIND] devno=%d ftype=%d\n",
          dev->info.devno, dev->info.ftype);

  /* Do we support given FOC instance? */

  if (dev->info.devno > CONFIG_POWER_FOC_INST)
    {
      pwrerr("ERROR: unsupported SIM FOC instance %d\n",
             dev->info.devno);
      ret = -EINVAL;
      goto errout;
    }

  /* Bind upper-half FOC controller callbacks */

  sim->cb = cb;

errout:
  return ret;
}

/****************************************************************************
 * Name: sim_foc_fault_clean
 *
 * Description:
 *   Arch-specific fault clean
 *
 ****************************************************************************/

static int sim_foc_fault_clean(FAR struct foc_dev_s *dev)
{
  DEBUGASSERT(dev);

  pwrinfo("[FAULT_CLEAN] devno=%d\n", dev->info.devno);

  return OK;
}

#ifdef CONFIG_POWER_FOC_TRACE
/****************************************************************************
 * Name: sim_foc_trace
 *
 * Description:
 *   SIM FOC trace
 *
 ****************************************************************************/

static void sim_foc_trace(FAR struct foc_dev_s *dev, int type, bool state)
{
  DEBUGASSERT(dev);

  pwrinfo("[FOC_TRACE] devno=%d type=%d state=%d\n",
          dev->devno, type, state);
}
#endif  /* CONFIG_POWER_FOC_TRACE */

#ifdef CONFIG_POWER_FOC_HAVE_CORDIC
/****************************************************************************
 * Name: sim_foc_angle_update
 *
 * Description:
 *   Arch-specific angle update
 *
 ****************************************************************************/

static void sim_foc_angle_update(FAR struct foc_dev_s *dev,
                                FAR foc_angle_t *angle_out,
                                FAR foc_number_t *angle_in)
{
  DEBUGASSERT(dev);
  DEBUGASSERT(angle_out);
  DEBUGASSERT(angle_in);

  switch (dev->info.ftype)
    {
#ifdef CONFIG_POWER_FOC_USE_FLOAT
      case FOC_NUMBER_TYPE_FLOAT:
        {
          phase_angle_update(&angle_out->f32, angle_in->f32);
          break;
        }
#endif

#ifdef CONFIG_POWER_FOC_USE_FIXED16
      case FOC_NUMBER_TYPE_FIXED16:
        {
          ASSERT(0);
          break;
        }
#endif

      default:
        {
          ASSERT(0);
        }
    }
}

/****************************************************************************
 * Name: sim_foc_dq_saturate
 *
 * Description:
 *   Arch-specific dq vector saturation
 *
 ****************************************************************************/

static int sim_foc_dq_saturate(FAR struct foc_dev_s *dev,
                               FAR foc_dqframe_t *dq,
                               FAR foc_number_t *max)
{
  DEBUGASSERT(dev);
  DEBUGASSERT(dq);
  DEBUGASSERT(max);

  switch (dev->info.ftype)
    {
#ifdef CONFIG_POWER_FOC_USE_FLOAT
      case FOC_NUMBER_TYPE_FLOAT:
        {
          dq_saturate(&dq->f32, max->f32);
          break;
        }
#endif

#ifdef CONFIG_POWER_FOC_USE_FIXED16
      case FOC_NUMBER_TYPE_FIXED16:
        {
          ASSERT(0);
          break;
        }
#endif

      default:
        {
          ASSERT(0);
        }
    }
}
#endif  /* CONFIG_POWER_FOC_HAVE_CORDIC */

#ifdef CONFIG_POWER_FOCMODEL
/****************************************************************************
 * Name: sim_focmodel_bind
 *
 * Description:
 *   Bind lower-half FOC model with upper-half logic
 *
 ****************************************************************************/

static int sim_focmodel_bind(FAR struct focmodel_dev_s *dev,
                             FAR struct focmodel_callbacks_s *cb)
{
  FAR struct sim_focmodel_data_s *sim = SIM_FOCMODEL_DATA_FROM_DEV_GET(dev);
  int                             ret = OK;

  DEBUGASSERT(dev);
  DEBUGASSERT(cb);
  DEBUGASSERT(sim);

  pwrinfo("[FOCMODEL_BIND] devno=%d ftype=%d\n",
          dev->devno, dev->ftype);

  /* Do we support given FOC instance? */

  if (dev->devno > CONFIG_POWER_FOC_INST)
    {
      pwrerr("ERROR: unsupported SIM FOC instance %d\n", dev->devno);
      ret = -EINVAL;
      goto errout;
    }

  /* Bind upper-half FOC controller callbacks */

  sim->cb = cb;

errout:
  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_foc_initialize
 *
 * Description:
 *   Initialize the FOC controller lower-half support.
 *
 ****************************************************************************/

FAR struct foc_dev_s *sim_foc_initialize(int inst)
{
  FAR struct foc_lower_s *foc_lower = NULL;
  FAR struct foc_dev_s   *dev       = NULL;

  pwrinfo("[FOC_INITIALIZE] inst=%d\n", inst);

  /* Reset data */

  memset(&g_sim_foc_data[inst], 0, sizeof(struct sim_foc_data_s));

  /* Get SIM FOC arch-specific driver */

  foc_lower = &g_sim_foc_lower[inst];

  /* Connect ops, data and dev with arch-specific FOC driver */

  foc_lower->data  = &g_sim_foc_data[inst];
  foc_lower->ops   = &g_sim_foc_ops;

  /* Connect board data */

  g_sim_foc_data[inst].board = &g_sim_foc_board[inst];

  /* Get FOC device */

  dev = &g_foc_dev[inst];

  /* Connect lower half FOC with FOC devic */

  dev->lower = (FAR void *)foc_lower;

  /* Return lower-half driver instance */

  return dev;
}

#ifdef CONFIG_POWER_FOCMODEL
/****************************************************************************
 * Name: sim_focmodel_initialize
 *
 * Description:
 *   Initialize the FOC model
 *
 ****************************************************************************/

FAR struct focmodel_dev_s *sim_focmodel_initialize(int inst)
{
  FAR struct focmodel_lower_s *lower = NULL;
  FAR struct focmodel_dev_s   *dev   = NULL;

  pwrinfo("[FOCMODEL_INITIALIZE] inst=%d\n", inst);

  /* Reset data */

  memset(&g_sim_focmodel_data[inst], 0, sizeof(struct sim_focmodel_data_s));

  /* Get SIM FOC arch-specific driver */

  lower = (FAR void *)&g_sim_focmodel_lower[inst];

  /* Connect ops and data  with arch-specific FOC driver */

  lower->ops  = &g_sim_focmodel_ops;
  lower->data = &g_sim_focmodel_data[inst];

  /* Get FOC device */

  dev = (FAR void *)&g_focmodel_dev[inst];

  /* Connect lower half FOC with FOC devic */

  dev->lower = (FAR void *)lower;

  /* Return lower-half driver instance */

  return dev;
}
#endif

/****************************************************************************
 * Name: sim_foc_update
 *
 * Description:
 *   Called periodically from the IDLE loop to simulate FOC driver interrupts
 *
 ****************************************************************************/

void sim_foc_update(void)
{
  FAR struct foc_dev_s      *dev  = NULL;
  FAR struct sim_foc_data_s *sim  = NULL;
  static uint32_t            cntr = 0;
  int                        i    = 0;
  irqstate_t                 flags;

  /* Increase local counter */

  cntr += 1;

  flags = enter_critical_section();

  /* Update all FOC instances */

  for (i = 0; i < CONFIG_POWER_FOC_INST; i += 1)
    {
      /* Get FOC device */

      dev = &g_foc_dev[i];

      /* Get SIM data */

      sim = SIM_FOC_DATA_FROM_DEV_GET(dev);

      if (sim->worker_state == true)
        {
          sim_foc_worker_handler(dev);
        }
    }

  leave_critical_section(flags);

  return;
}
