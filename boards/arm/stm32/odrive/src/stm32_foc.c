/****************************************************************************
 * boards/arm/stm32/odrive/src/stm32_foc.c
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

#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>

#include <nuttx/signal.h>
#include <nuttx/analog/adc.h>
#include <nuttx/motor/foc/drv8301.h>

#include "stm32_foc.h"
#include "stm32_gpio.h"
#ifdef CONFIG_ADC
#  include "stm32_adc.h"
#endif

#include "odrive.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_ODRIVE_HW_VOLTAGE_56
#  error Tested only for ODrive 56V version
#endif

#ifndef CONFIG_ODRIVE_HW_36
#  error Tested only for ODrive 3.6 version
#endif

/* Supported FOC instances */

#ifdef CONFIG_ODRIVE_FOC_FOC0
#  define ODRIVE_FOC_FOC0 1
#else
#  define ODRIVE_FOC_FOC0 0
#endif

#ifdef CONFIG_ODRIVE_FOC_FOC1
#  define ODRIVE_FOC_FOC1 1
#else
#  define ODRIVE_FOC_FOC1 0
#endif

#define ODRIVE_FOC_INST (ODRIVE_FOC_FOC0 + ODRIVE_FOC_FOC1)

#ifdef CONFIG_ODRIVE_FOC_FOC0
#  define ODRIVE32_FOC0_DEVPATH "/dev/foc0"
#  define ODRIVE32_FOC0_INST    (0)
#endif

#ifdef CONFIG_ODRIVE_FOC_FOC1
#  define ODRIVE32_FOC1_DEVPATH "/dev/foc1"
#  define ODRIVE32_FOC1_INST     (1)
#endif

/* Must match upper-half configuration */

#if ODRIVE_FOC_INST != CONFIG_MOTOR_FOC_INST
#  error Invalid configuration
#endif

/* Only 2-shunt configuration supported by board */

#if CONFIG_MOTOR_FOC_SHUNTS != 2
#  error For now ony 2-shunts configuration is supported
#endif

/* Configuration specific for DRV8301:
 *   1. PWM channels must have positive polarity
 *   2. PWM complementary channels must have positive polarity
 */

#ifndef CONFIG_STM32_FOC_HAS_PWM_COMPLEMENTARY
#  error
#endif

#ifdef CONFIG_ODRIVE_FOC_FOC0

#  if CONFIG_STM32_TIM1_CH1POL != 0
#    error
#  endif
#  if CONFIG_STM32_TIM1_CH2POL != 0
#    error
#  endif
#  if CONFIG_STM32_TIM1_CH3POL != 0
#    error
#  endif
#  if CONFIG_STM32_TIM1_CH1NPOL != 0
#    error
#  endif
#  if CONFIG_STM32_TIM1_CH2NPOL != 0
#    error
#  endif
#  if CONFIG_STM32_TIM1_CH3NPOL != 0
#    error
#  endif

/* FOC0 uses ADC2 */

#  ifndef CONFIG_STM32_FOC_FOC0_ADC2
#    error
#  endif

#  if CONFIG_STM32_ADC2_RESOLUTION != 0
#    error
#  endif

#endif  /* CONFIG_ODRIVE_FOC_FOC0 */

#ifdef CONFIG_ODRIVE_FOC_FOC1

#  if CONFIG_STM32_TIM8_CH1POL != 0
#    error
#  endif
#  if CONFIG_STM32_TIM8_CH2POL != 0
#    error
#  endif
#  if CONFIG_STM32_TIM8_CH3POL != 0
#    error
#  endif
#  if CONFIG_STM32_TIM8_CH1NPOL != 0
#    error
#  endif
#  if CONFIG_STM32_TIM8_CH2NPOL != 0
#    error
#  endif
#  if CONFIG_STM32_TIM8_CH3NPOL != 0
#    error
#  endif

/* FOC1 uses ADC3 */

#  ifndef CONFIG_STM32_FOC_FOC1_ADC3
#    error
#  endif

#  if CONFIG_STM32_ADC3_RESOLUTION != 0
#    error
#  endif

#endif  /* CONFIG_ODRIVE_FOC_FOC1 */

/* Aux ADC needs DMA enabled */

#ifdef CONFIG_ADC
#  ifndef CONFIG_STM32_ADC1_DMA
#    error
#  endif
#  ifndef CONFIG_STM32_ADC1_SCAN
#    error
#  endif
#endif

/* TODO: */

#define PWM_DEADTIME    (50)
#define PWM_DEADTIME_NS (320)

/* Only gain=10 supported for now */

#if CONFIG_ODRIVE_FOC_CURRENT_GAIN != 80
#  error not supported for now
#endif

/* Board parameters:
 *   Current shunt resistance                    = 0.0005
 *   Current sense gain                          = (10/20/40/80)
 *   Vbus min                                    = 12V
 *   Vbus max                                    = 24V or 56V
 *   Iout max                                    = 40A (no cooling for MOSFETs)
 *   IPHASE_RATIO                                = 1/(R_shunt*gain)
 *   ADC_REF_VOLTAGE                             = 3.3
 *   ADC_VAL_MAX                                 = 4095
 *   ADC_TO_VOLT = ADC_REF_VOLTAGE / ADC_VAL_MAX
 *   IPHASE_ADC = IPHASE_RATIO * ADC_TO_VOLT     = 0.02014 (gain=80)
 *   VBUS_RATIO = 1/VBUS_gain                    = 11 or 19
 */

/* Center-aligned PWM duty cycle limits */

#define MAX_DUTY_FLOAT ftob16(0.95f)

/* ADC configuration */

#define CURRENT_SAMPLE_TIME    ADC_SMPR_15
#define VBUS_SAMPLE_TIME       ADC_SMPR_15
#define TEMP_SAMPLE_TIME       ADC_SMPR_15

#define ODRIVE_ADC_AUX  (1)
#define ODRIVE_ADC_FOC0 (2)
#define ODRIVE_ADC_FOC1 (3)

#ifdef CONFIG_ODRIVE_FOC_VBUS
#  define ODRIVE_FOC_VBUS 1
#else
#  define ODRIVE_FOC_VBUS 0
#endif
#ifdef CONFIG_ODRIVE_FOC_TEMP
#  define ODRIVE_FOC_TEMP 3
#else
#  define ODRIVE_FOC_TEMP 0
#endif

#ifdef CONFIG_ADC
#  define ODRIVE_ADC_AUX_DEVPATH "/dev/adc0"
#  define ODRIVE_ADC_AUX_NCHAN   (ODRIVE_FOC_VBUS + ODRIVE_FOC_TEMP)
#endif

#define ADC1_INJECTED  (0)
#define ADC1_REGULAR   (0)
#define ADC1_NCHANNELS (ADC1_INJECTED + ADC1_REGULAR)

#define ADC2_INJECTED  (CONFIG_MOTOR_FOC_SHUNTS)
#define ADC2_REGULAR   (0)
#define ADC2_NCHANNELS (ADC2_INJECTED + ADC2_REGULAR)

#define ADC3_INJECTED  (CONFIG_MOTOR_FOC_SHUNTS)
#define ADC3_REGULAR   (0)
#define ADC3_NCHANNELS (ADC3_INJECTED + ADC3_REGULAR)

#if ADC1_INJECTED != CONFIG_STM32_ADC1_INJECTED_CHAN
#  error
#endif

#if ADC2_INJECTED != CONFIG_STM32_ADC2_INJECTED_CHAN
#  error
#endif

#if ADC3_INJECTED != CONFIG_STM32_ADC3_INJECTED_CHAN
#  error
#endif

/* DRV8301 configuration */

#ifndef CONFIG_STM32_SPI3
#  error
#endif

#define DRV8301_0_SPI (3)
#define DRV8301_1_SPI (3)

#define DRV8301_FREQUENCY (500000)

/* Helper macros ************************************************************/

#define FOC_DRV8301_FROM_DEV_GET(d) (g_drv8301_list[d->devno])

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* DRV8301 device */

struct foc_drv8301_priv_s
{
  bool                  initialized; /* Initialized flag */
  uint8_t               devno;       /* Device instance */
  uint8_t               spino;       /* SPI interface */
  FAR struct spi_dev_s *spi;         /* SPI device reference */
  FAR struct foc_dev_s *dev;         /* Reference to FOC device */
};

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

static int board_foc_setup(FAR struct foc_dev_s *dev);
static int board_foc_shutdown(FAR struct foc_dev_s *dev);
static int board_foc_calibration(FAR struct foc_dev_s *dev, bool state);
static int board_foc_fault_clear(FAR struct foc_dev_s *dev);
static int board_foc_pwm_start(FAR struct foc_dev_s *dev, bool state);
static int board_foc_current_get(FAR struct foc_dev_s *dev,
                                 FAR int16_t *curr_raw,
                                 FAR foc_current_t *curr);
#ifdef CONFIG_MOTOR_FOC_TRACE
static int board_foc_trace_init(FAR struct foc_dev_s *dev);
static void board_foc_trace(FAR struct foc_dev_s *dev, int type, bool state);
#endif

static int stm32_foc_drv8301_init(FAR struct foc_drv8301_priv_s *priv);
static int stm32_foc_drv8301_deinit(FAR struct foc_drv8301_priv_s *priv);
static int stm32_foc_drv8301_calibration(FAR struct foc_drv8301_priv_s *priv,
                                         bool state);
static int stm32_foc_drv8301_fault_isr(int irq, FAR void *context,
                                       FAR void *arg);

#ifdef CONFIG_ODRIVE_FOC_FOC0
static int stm32_foc0_setup(void);
#endif
#ifdef CONFIG_ODRIVE_FOC_FOC1
static int stm32_foc1_setup(void);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Board specific ops */

static struct stm32_foc_board_ops_s g_stm32_foc_board_ops =
{
  .setup         = board_foc_setup,
  .shutdown      = board_foc_shutdown,
  .calibration   = board_foc_calibration,
  .fault_clear   = board_foc_fault_clear,
  .pwm_start     = board_foc_pwm_start,
  .current_get   = board_foc_current_get,
#ifdef CONFIG_MOTOR_FOC_TRACE
  .trace_init    = board_foc_trace_init,
  .trace         = board_foc_trace
#endif
};

#ifdef CONFIG_ODRIVE_FOC_FOC0
/* DRV8301 device 0 */

struct foc_drv8301_priv_s g_drv8301_foc0 =
{
  .devno = 0,
  .spino = DRV8301_0_SPI,
  .spi   = NULL,
};
#endif

#ifdef CONFIG_ODRIVE_FOC_FOC1
/* DRV8301 device 1 */

struct foc_drv8301_priv_s g_drv8301_foc1 =
{
  .devno = 1,
  .spino = DRV8301_1_SPI,
  .spi   = NULL,
};
#endif

/* Board specific ADC configuration
 *
 * AUX (only VBUS used):
 *    VBUS     - ADC1 - ADC1_IN6  (PA6)
 *    M0_TEMP  - ADC1 - ADC1_IN15 (PC5)
 *    M1_TEMP  - ADC1 - ADC1_IN4  (PA4)
 *    AUX_TEMP - ADC1 - ADC1_IN5  (PA5)
 *
 * FOC device 0:
 *   Phase 1 - ADC2 - ADC2_IN10   (PC0)
 *   Phase 2 - ADC2 - ADC2_IN11   (PC1)
 *
 * FOC device 1:
 *   Phase 1 - ADC3 - ADC3_IN13   (PC3)
 *   Phase 2 - ADC3 - ADC3_IN12   (PC2)
 *
 */

#ifdef CONFIG_ADC

/* AUX ADC configuration */

static uint8_t g_adc_aux_chan[] =
{
#ifdef CONFIG_ODRIVE_FOC_VBUS
  6,
#endif
#ifdef ODRIVE_ADC_TEMP
  15,
  4,
  5
#endif
};

static uint32_t g_adc_aux_pins[] =
{
#ifdef CONFIG_ODRIVE_FOC_VBUS
  GPIO_ADC1_IN6,
#endif
#ifdef ODRIVE_ADC_TEMP
  GPIO_ADC1_IN15,
  GPIO_ADC1_IN4,
  GPIO_ADC1_IN5
#endif
};

static adc_channel_t g_adc_aux_stime[] =
{
#ifdef CONFIG_ODRIVE_FOC_VBUS
  {
    .channel     = 6,
    .sample_time = VBUS_SAMPLE_TIME
  },
#endif
#ifdef ODRIVE_ADC_TEMP
  {
    .channel     = 15,
    .sample_time = TEMP_SAMPLE_TIME
  },
  {
    .channel     = 4,
    .sample_time = TEMP_SAMPLE_TIME
  },
  {
    .channel     = 5,
    .sample_time = TEMP_SAMPLE_TIME
  }
#endif
};
#endif

#ifdef CONFIG_ODRIVE_FOC_FOC0
/* Board specific ADC configuration for FOC device 0 */

static uint8_t g_adc_foc0_chan[] =
{
  10,
  11
};

static uint32_t g_adc_foc0_pins[] =
{
  GPIO_ADC2_IN10,
  GPIO_ADC2_IN11,
};

static adc_channel_t g_adc_foc0_stime[] =
{
  {
    .channel     = 10,
    .sample_time = CURRENT_SAMPLE_TIME
  },
  {
    .channel     = 11,
    .sample_time = CURRENT_SAMPLE_TIME
  }
};

static struct stm32_foc_adc_s g_adc_foc0_cfg =
{
  .chan  = g_adc_foc0_chan,
  .pins  = g_adc_foc0_pins,
  .stime = g_adc_foc0_stime,
  .nchan = ADC2_NCHANNELS,
  .regch = ADC2_REGULAR,
  .intf  = ODRIVE_ADC_FOC0
};
#endif

#ifdef CONFIG_ODRIVE_FOC_FOC1
/* Board specific ADC configuration for FOC device 1 */

static uint8_t g_adc_foc1_chan[] =
{
  13,
  12
};

static uint32_t g_adc_foc1_pins[] =
{
  GPIO_ADC3_IN13,
  GPIO_ADC3_IN12,
};

static adc_channel_t g_adc_foc1_stime[] =
{
  {
    .channel     = 13,
    .sample_time = CURRENT_SAMPLE_TIME
  },
  {
    .channel     = 12,
    .sample_time = CURRENT_SAMPLE_TIME
  }
};

static struct stm32_foc_adc_s g_adc_foc1_cfg =
{
  .chan  = g_adc_foc1_chan,
  .pins  = g_adc_foc1_pins,
  .stime = g_adc_foc1_stime,
  .nchan = ADC3_NCHANNELS,
  .regch = ADC3_REGULAR,
  .intf  = ODRIVE_ADC_FOC1
};
#endif

#ifdef CONFIG_ODRIVE_FOC_FOC0
/* Board specific data - FOC 0 */

static struct stm32_foc_board_data_s g_stm32_foc0_board_data =
{
  .adc_cfg   = &g_adc_foc0_cfg,
  .duty_max  = (MAX_DUTY_FLOAT),
  .pwm_dt    = (PWM_DEADTIME),
  .pwm_dt_ns = (PWM_DEADTIME_NS)
};

/* Board specific configuration */

static struct stm32_foc_board_s g_stm32_foc0_board =
{
  .data = &g_stm32_foc0_board_data,
  .ops  = &g_stm32_foc_board_ops,
};
#endif

#ifdef CONFIG_ODRIVE_FOC_FOC1
/* Board specific data - FOC 1 */

static struct stm32_foc_board_data_s g_stm32_foc1_board_data =
{
  .adc_cfg   = &g_adc_foc1_cfg,
  .duty_max  = (MAX_DUTY_FLOAT),
  .pwm_dt    = (PWM_DEADTIME),
  .pwm_dt_ns = (PWM_DEADTIME_NS)
};

/* Board specific configuration */

static struct stm32_foc_board_s g_stm32_foc1_board =
{
  .data = &g_stm32_foc1_board_data,
  .ops  = &g_stm32_foc_board_ops,
};
#endif

/* Global data */

static sem_t   g_drv8301_sem;
static uint8_t g_drv8301_cntr = 0;

/* Global pointer to the upper FOC driver */

#ifdef CONFIG_ODRIVE_FOC_FOC0
static FAR struct foc_dev_s *g_foc0_dev = NULL;
#endif
#ifdef CONFIG_ODRIVE_FOC_FOC1
static FAR struct foc_dev_s *g_foc1_dev = NULL;
#endif

/* DRV8301 devices list */

static FAR struct foc_drv8301_priv_s *g_drv8301_list[] =
{
#ifdef CONFIG_ODRIVE_FOC_FOC0
  &g_drv8301_foc0,
#endif
#ifdef CONFIG_ODRIVE_FOC_FOC1
  &g_drv8301_foc1
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_foc_setup
 ****************************************************************************/

static int board_foc_setup(FAR struct foc_dev_s *dev)
{
  FAR struct foc_drv8301_priv_s *drv = FOC_DRV8301_FROM_DEV_GET(dev);
  int                            ret = OK;

  DEBUGASSERT(dev);
  DEBUGASSERT(drv);

  /* Initialize drv8301 */

  ret = stm32_foc_drv8301_init(drv);
  if (ret < 0)
    {
      mtrerr("Failed to initialize DRV8301: %d\n", ret);
      goto errout;
    }

  /* Connect FOC device */

  drv->dev = dev;

errout:
  return ret;
}

/****************************************************************************
 * Name: board_foc_shutdown
 ****************************************************************************/

static int board_foc_shutdown(FAR struct foc_dev_s *dev)
{
  FAR struct foc_drv8301_priv_s *drv = FOC_DRV8301_FROM_DEV_GET(dev);
  int                            ret = OK;

  DEBUGASSERT(dev);
  DEBUGASSERT(drv);

  /* Deinitialize drv8301 */

  ret = stm32_foc_drv8301_deinit(drv);
  if (ret < 0)
    {
      mtrerr("Failed to de-initialize DRV8301: %d\n", ret);
      goto errout;
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: board_foc_calibration
 ****************************************************************************/

static int board_foc_calibration(FAR struct foc_dev_s *dev, bool state)
{
  FAR struct foc_drv8301_priv_s *drv = FOC_DRV8301_FROM_DEV_GET(dev);
  int                            ret = OK;

  DEBUGASSERT(dev);
  DEBUGASSERT(drv);

  /* Calibration mode */

  ret = stm32_foc_drv8301_calibration(drv, state);
  if (ret < 0)
    {
      mtrerr("stm32_foc_drv8301_calibration failed: %d\n", ret);
      goto errout;
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: board_foc_fault_clear
 ****************************************************************************/

static int board_foc_fault_clear(FAR struct foc_dev_s *dev)
{
  DEBUGASSERT(dev);

  UNUSED(dev);

  /* TODO: clear DRV8301 faults */

  return OK;
}

/****************************************************************************
 * Name: board_foc_pwm_start
 ****************************************************************************/

static int board_foc_pwm_start(FAR struct foc_dev_s *dev, bool state)
{
  DEBUGASSERT(dev);

  return OK;
}

/****************************************************************************
 * Name: board_foc_current_get
 ****************************************************************************/

static int board_foc_current_get(FAR struct foc_dev_s *dev,
                                 FAR int16_t *curr_raw,
                                 FAR foc_current_t *curr)
{
  DEBUGASSERT(dev);
  DEBUGASSERT(curr_raw);
  DEBUGASSERT(curr);

  /* Get currents */

  curr[1] = curr_raw[0];
  curr[2] = curr_raw[1];

  /* From Kirchhoff's current law: ia = -(ib + ic) */

  curr[0] = -(curr[1] + curr[2]);

  return OK;
}

#ifdef CONFIG_MOTOR_FOC_TRACE
/****************************************************************************
 * Name: board_foc_trace_init
 ****************************************************************************/

static int board_foc_trace_init(FAR struct foc_dev_s *dev)
{
  DEBUGASSERT(dev);

  UNUSED(dev);

  /* Configure debug GPIO */

  stm32_configgpio(GPIO_FOC_DEBUG0);
  stm32_configgpio(GPIO_FOC_DEBUG1);
  stm32_configgpio(GPIO_FOC_DEBUG2);
  stm32_configgpio(GPIO_FOC_DEBUG3);

  return OK;
}

/****************************************************************************
 * Name: board_foc_trace
 ****************************************************************************/

static void board_foc_trace(FAR struct foc_dev_s *dev, int type, bool state)
{
  DEBUGASSERT(dev);

  UNUSED(dev);

  switch (type)
    {
      case FOC_TRACE_NONE:
        {
          break;
        }

      case FOC_TRACE_PARAMS:
        {
          stm32_gpiowrite(GPIO_FOC_DEBUG0, state);

          break;
        }

      case FOC_TRACE_STATE:
        {
          stm32_gpiowrite(GPIO_FOC_DEBUG1, state);

          break;
        }

      case FOC_TRACE_NOTIFIER:
        {
          stm32_gpiowrite(GPIO_FOC_DEBUG2, state);

          break;
        }

      case FOC_TRACE_LOWER:
        {
          stm32_gpiowrite(GPIO_FOC_DEBUG3, state);

          break;
        }

      default:
        {
          mtrerr("board_foc_trace type=%d not supported!\n", type);
          DEBUGASSERT(0);
        }
    }
}
#endif

/****************************************************************************
 * Name: stm32_foc_drv8301_lock
 ****************************************************************************/

static void stm32_foc_drv8301_lock(FAR struct foc_drv8301_priv_s *priv)
{
  SPI_LOCK(priv->spi, 1);
  SPI_SETBITS(priv->spi, 16);
  SPI_SETMODE(priv->spi, SPIDEV_MODE1);
  SPI_SETFREQUENCY(priv->spi, DRV8301_FREQUENCY);
}

/****************************************************************************
 * Name: stm32_foc_drv8301_unlock
 ****************************************************************************/

static void stm32_foc_drv8301_unlock(FAR struct foc_drv8301_priv_s *priv)
{
  SPI_LOCK(priv->spi, 0);
}

/****************************************************************************
 * Name: stm32_foc_drv8301_read
 ****************************************************************************/

static void stm32_foc_drv8301_read(FAR struct foc_drv8301_priv_s *priv,
                                   uint8_t addr, uint16_t *data)
{
  uint16_t regval = 0;

  stm32_foc_drv8301_lock(priv);
  SPI_SELECT(priv->spi, SPIDEV_MOTOR(priv->devno), true);

  /* Read command */

  regval |= (1 << 15);
  regval |= ((addr & 0x0f) << 11);

  /* Send command */

  SPI_SEND(priv->spi, regval);

  /* Toggle CS pin, otherwise read doesn't work */

  SPI_SELECT(priv->spi, SPIDEV_MOTOR(priv->devno), false);
  SPI_SELECT(priv->spi, SPIDEV_MOTOR(priv->devno), true);

  /* Read output */

  regval = 0;
  SPI_RECVBLOCK(priv->spi, &regval, 1);

  /* Retrun data */

  *data = (regval & 0x7ff);

  SPI_SELECT(priv->spi, SPIDEV_MOTOR(priv->devno), false);
  stm32_foc_drv8301_unlock(priv);
}

/****************************************************************************
 * Name: stm32_foc_drv8301_write
 ****************************************************************************/

static void stm32_foc_drv8301_write(FAR struct foc_drv8301_priv_s *priv,
                                    uint8_t addr, uint16_t data)
{
  uint16_t regval = 0;

  stm32_foc_drv8301_lock(priv);
  SPI_SELECT(priv->spi, SPIDEV_MOTOR(priv->devno), true);

  /* Write command */

  regval |= (0 << 15);
  regval |= ((addr & 0x0f) << 11);
  regval |= (0x7ff & data);

  /* Send data */

  SPI_SEND(priv->spi, regval);

  SPI_SELECT(priv->spi, SPIDEV_MOTOR(priv->devno), false);
  stm32_foc_drv8301_unlock(priv);
}

/****************************************************************************
 * Name: stm32_foc_drv8301_init
 ****************************************************************************/

static int stm32_foc_drv8301_init(FAR struct foc_drv8301_priv_s *priv)
{
  uint16_t status1 = 0;
  uint16_t status2 = 0;
  uint16_t ctrl1   = 0;
  uint16_t ctrl2   = 0;
  int      ret     = OK;

  /* Initialize only once */

  if (priv->initialized == true)
    {
      goto errout;
    }

  /* Get SPI device */

  priv->spi = stm32_spibus_initialize(priv->spino);
  if (priv->spi == NULL)
    {
      ret = -errno;
      goto errout;
    }

  /* Lock global data and common initialization */

  ret = nxsem_wait(&g_drv8301_sem);
  if (ret < 0)
    {
      goto errout;
    }

  /* Common EN_GATE and nFAULT pins */

  if (g_drv8301_cntr == 0)
    {
      /* Configure EN_GATE */

      stm32_configgpio(GPIO_DRV8301_ENGATE);

      /* Reset chip */

      nxsig_usleep(25000);
      stm32_gpiowrite(GPIO_DRV8301_ENGATE, true);
      nxsig_usleep(25000);
      stm32_gpiowrite(GPIO_DRV8301_ENGATE, false);
      nxsig_usleep(25000);
      stm32_gpiowrite(GPIO_DRV8301_ENGATE, true);
      nxsig_usleep(25000);

      /* Attach nFAULT interrupt
       * TODO: not tested
       */

      stm32_gpiosetevent(GPIO_DRV8301_NFAULT, false, true, false,
                         stm32_foc_drv8301_fault_isr, (void *)&priv);
    }

  /* Increase counter */

  g_drv8301_cntr += 1;

  /* Unlock global data */

  ret = nxsem_post(&g_drv8301_sem);
  if (ret < 0)
    {
      goto errout;
    }

  /* Get status registers */

  stm32_foc_drv8301_read(priv, DRV8301_REG_STAT1, &status1);
  stm32_foc_drv8301_read(priv, DRV8301_REG_STAT2, &status2);

  /* Default CTRL1 */

  ctrl1 = 0;

  /* Write CTRL1 */

  stm32_foc_drv8301_write(priv, DRV8301_REG_CTRL1, ctrl1);

  /* Default CTRL2 */

  ctrl2 = 0;
#if CONFIG_ODRIVE_FOC_CURRENT_GAIN == 10
  ctrl2 |= DRV8301_CTRL2_GAIN_10;
#elif CONFIG_ODRIVE_FOC_CURRENT_GAIN == 20
  ctrl2 |= DRV8301_CTRL2_GAIN_20;
#elif CONFIG_ODRIVE_FOC_CURRENT_GAIN == 40
  ctrl2 |= DRV8301_CTRL2_GAIN_40;
#elif CONFIG_ODRIVE_FOC_CURRENT_GAIN == 80
  ctrl2 |= DRV8301_CTRL2_GAIN_80;
#else
#  error Unsupported current gain
#endif

  /* Write CTRL2 */

  stm32_foc_drv8301_write(priv, DRV8301_REG_CTRL2, ctrl2);

  /* Set flag */

  priv->initialized = true;

errout:
  return ret;
}

/****************************************************************************
 * Name: stm32_foc_drv8301_deinit
 ****************************************************************************/

static int stm32_foc_drv8301_deinit(FAR struct foc_drv8301_priv_s *priv)
{
  int ret = OK;

  /* Lock global data and common deinitialization */

  ret = nxsem_wait(&g_drv8301_sem);
  if (ret < 0)
    {
      goto errout;
    }

  /* Decrease counter */

  g_drv8301_cntr -= 1;

  /* Only if last instance deinit */

  if (g_drv8301_cntr == 0)
    {
      /* Disable chip */

      stm32_gpiowrite(GPIO_DRV8301_ENGATE, false);

      /* Disable nFAULT interrupt */

      stm32_gpiosetevent(GPIO_DRV8301_NFAULT, false, true, false,
                         NULL, NULL);
    }

  /* Unlock global data */

  ret = nxsem_post(&g_drv8301_sem);
  if (ret < 0)
    {
      goto errout;
    }

  /* Reset flag */

  priv->initialized = false;

errout:
  return ret;
}

/****************************************************************************
 * Name: stm32_foc_drv8301_calibration
 ****************************************************************************/

static int stm32_foc_drv8301_calibration(FAR struct foc_drv8301_priv_s *priv,
                                         bool state)
{
  uint16_t regval = 0;

  stm32_foc_drv8301_read(priv, DRV8301_REG_CTRL2, &regval);

  if (state == true)
    {
      regval |= DRV8301_CTRL2_DCCALCH1;
      regval |= DRV8301_CTRL2_DCCALCH2;
    }
  else
    {
      regval &= ~DRV8301_CTRL2_DCCALCH1;
      regval &= ~DRV8301_CTRL2_DCCALCH2;
    }

  stm32_foc_drv8301_write(priv, DRV8301_REG_CTRL2, regval);

  return OK;
}

/****************************************************************************
 * Name: stm32_foc_drv8301_fault_isr
 ****************************************************************************/

static int stm32_foc_drv8301_fault_isr(int irq, FAR void *context,
                                       FAR void *arg)
{
  FAR struct foc_drv8301_priv_s *priv    = NULL;
  irqstate_t                     flags;
  uint16_t                       status1 = 0;
  uint16_t                       status2 = 0;

  priv = (FAR struct foc_drv8301_priv_s *)arg;
  DEBUGASSERT(priv != NULL);

  flags = enter_critical_section();

  stm32_foc_drv8301_read(priv, DRV8301_REG_STAT1, &status1);
  stm32_foc_drv8301_read(priv, DRV8301_REG_STAT2, &status2);

  /* Set fault state */

  priv->dev->state.fault |= FOC_FAULT_BOARD;

  leave_critical_section(flags);

  return OK;
}

#ifdef CONFIG_ODRIVE_FOC_FOC0
/****************************************************************************
 * Name: stm32_foc0_setup
 ****************************************************************************/

static int stm32_foc0_setup(void)
{
  FAR struct foc_dev_s *foc = NULL;
  int                   ret = OK;

  /* Initialize only once */

  if (g_foc0_dev == NULL)
    {
      /* Initialize arch specific FOC 0 lower-half */

      foc = stm32_foc_initialize(ODRIVE32_FOC0_INST,
                                 &g_stm32_foc0_board);
      if (foc == NULL)
        {
          ret = -errno;
          mtrerr("Failed to initialize STM32 FOC: %d\n", ret);
          goto errout;
        }

      DEBUGASSERT(foc->lower);

      /* Register FOC device */

      ret = foc_register(ODRIVE32_FOC0_DEVPATH, foc);
      if (ret < 0)
        {
          mtrerr("Failed to register FOC device: %d\n", ret);
          goto errout;
        }

      /* Store pointer to driver */

      g_foc0_dev = foc;
    }

errout:
  return ret;
}
#endif

#ifdef CONFIG_ODRIVE_FOC_FOC1
/****************************************************************************
 * Name: stm32_foc1_setup
 ****************************************************************************/

static int stm32_foc1_setup(void)
{
  FAR struct foc_dev_s *foc = NULL;
  int                   ret = OK;

  /* Initialize only once */

  if (g_foc1_dev == NULL)
    {
      /* Initialize arch specific FOC lower-half */

      foc = stm32_foc_initialize(ODRIVE32_FOC1_INST,
                                 &g_stm32_foc1_board);
      if (foc == NULL)
        {
          ret = -errno;
          mtrerr("Failed to initialize STM32 FOC: %d\n", ret);
          goto errout;
        }

      DEBUGASSERT(foc->lower);

      /* Register FOC device */

      ret = foc_register(ODRIVE32_FOC1_DEVPATH, foc);
      if (ret < 0)
        {
          mtrerr("Failed to register FOC device: %d\n", ret);
          goto errout;
        }

      /* Store pointer to driver */

      g_foc1_dev = foc;
    }

errout:
  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_foc_setup
 *
 * Description:
 *   Setup FOC devices
 *
 *   This function should be call by board_app_initialize().
 *
 * Returned Value:
 *   0 on success, a negated errno value on failure
 *
 ****************************************************************************/

int stm32_foc_setup(void)
{
  int ret = OK;

  /* Initialize global semaphore */

  nxsem_init(&g_drv8301_sem, 0, 1);

#ifdef CONFIG_ODRIVE_FOC_FOC0
  ret = stm32_foc0_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_foc0_setup failed: %d\n", ret);
      goto errout;
    }
#endif

#ifdef CONFIG_ODRIVE_FOC_FOC1
  ret = stm32_foc1_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_foc1_setup failed: %d\n", ret);
      goto errout;
    }
#endif

errout:
  return ret;
}

#ifdef CONFIG_ADC
/****************************************************************************
 * Name: stm32_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

int stm32_adc_setup(void)
{
  FAR struct adc_dev_s       *adc       = NULL;
  FAR struct stm32_adc_dev_s *stm32_adc = NULL;
  struct adc_sample_time_s    stime;
  int                         i         = 0;
  int                         ret       = OK;

  /* Configure pins */

  for (i = 0; i < ODRIVE_ADC_AUX_NCHAN; i += 1)
    {
      stm32_configgpio(g_adc_aux_pins[i]);
    }

  /* Initialize ADC */

  adc = stm32_adcinitialize(ODRIVE_ADC_AUX, g_adc_aux_chan,
                            ODRIVE_ADC_AUX_NCHAN);
  if (adc == NULL)
    {
      aerr("ERROR: Failed to get ADC interface %d\n", ODRIVE_ADC_AUX);
      ret = -ENODEV;
      goto errout;
    }

  /* Regsiter ADC */

  ret = adc_register(ODRIVE_ADC_AUX_DEVPATH, adc);
  if (ret < 0)
    {
      aerr("ERROR: adc_register %s failed: %d\n",
           ODRIVE_ADC_AUX_DEVPATH, ret);
      goto errout;
    }

  /* Get lower-half ADC */

  stm32_adc = (FAR struct stm32_adc_dev_s *)adc->ad_priv;
  DEBUGASSERT(stm32_adc);

  /* Configure ADC sample time */

  memset(&stime, 0, sizeof(struct adc_sample_time_s));

  stime.channels_nbr = ODRIVE_ADC_AUX_NCHAN;
  stime.channel      = g_adc_aux_stime;

  STM32_ADC_SAMPLETIME_SET(stm32_adc, &stime);
  STM32_ADC_SAMPLETIME_WRITE(stm32_adc);

  ret = OK;

errout:
  return ret;
}
#endif
