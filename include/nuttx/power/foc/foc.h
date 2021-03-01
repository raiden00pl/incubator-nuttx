/****************************************************************************
 * include/nuttx/power/foc/foc.h
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

#ifndef __INCLUDE_NUTTX_DRIVERS_POWER_FOC_FOC_H
#define __INCLUDE_NUTTX_DRIVERS_POWER_FOC_FOC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <semaphore.h>

#include <nuttx/power/foc/foc_types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* FOC device fault code */

enum foc_fault_e
{
  FOC_FAULT_NONE             = (0),      /* No fault */
  FOC_FAULT_MODE             = (1 << 0), /* Invalid FOC mode */
  FOC_FAULT_INTERNAL         = (1 << 1), /* Internal driver fault */
  FOC_FAULT_TIMEOUT          = (1 << 2), /* Timeout fault */
  FOC_FAULT_ARCH             = (1 << 3), /* Arch-specific fault */
  FOC_FAULT_BOARD            = (1 << 4), /* Board-specific fault */
  FOC_FAULT_EXT              = (1 << 5), /* External fault */
  FOC_FAULT_RES2             = (1 << 6), /* Reserved 2 */
  FOC_FAULT_RES3             = (1 << 7), /* Reserved 3 */
};

/* FOC device fault code - external */

enum foc_fault_ext_e
{
  FOC_FAULT_EXT_NONE         = (0),      /* No external fault */
  FOC_FAULT_EXT_OVERCURRENT  = (1 << 0), /* Overcurrent fault */
  FOC_FAULT_EXT_OVERVOLTAGE  = (1 << 1), /* Overvoltage fault */
  FOC_FAULT_EXT_UNDERVOLTAGE = (1 << 2), /* Undervoltage fault */
  FOC_FAULT_EXT_OVERTEMP     = (1 << 3), /* Overtemperature fault */
  FOC_FAULT_EXT_GATEDRV      = (1 << 4), /* Gate driver fault */
  FOC_FAULT_EXT_RES1         = (1 << 5), /* Reserved 1 */
  FOC_FAULT_EXT_RES2         = (1 << 6), /* Reserved 2 */
  FOC_FAULT_EXT_RES3         = (1 << 7), /* Reserved 3 */
};

/* FOC device control mode */

enum foc_control_mode_e
{
  FOC_CONTROL_MODE_INIT      = 0, /* Initial state */
  FOC_CONTROL_MODE_IDLE      = 1, /* IDLE mode */
  FOC_CONTROL_MODE_VOLTAGE   = 2, /* Voltage control mode */
  FOC_CONTROL_MODE_CURRENT   = 3, /* Current control mode */
};

/* FOC device configuration type */

enum foc_cfg_type_e
{
  FOC_CFG_TYPE_INVAL         = 0, /* Reserved */
  FOC_CFG_TYPE_COMMON        = 1, /* Common configuration */
  FOC_CFG_TYPE_CONTROL       = 2  /* Controller configuration */
};

/* FOC device modulation type */

enum foc_cfg_modulation_e
{
  FOC_CFG_MODULATION_INVAL,
#ifdef CONFIG_POWER_FOC_MODULATION_SVM3
  FOC_CFG_MODULATION_SVM3,      /* 3-phase space vector modulation */
#endif
};

/* FOC device controller type */

enum foc_cfg_controller_e
{
  FOC_CFG_CONTROLLER_INVAL,
#ifdef CONFIG_POWER_FOC_CONTROL_PI
  FOC_CFG_CONTROLLER_PI,        /* Classic FOC PI current controller */
#endif
};

/* FOC device fault data */

struct foc_fault_s
{
  uint8_t fault;                /* Device fault state */
  uint8_t fault_ext;            /* External fault state */
};

/* FOC device fault union */

union foc_fault_u
{
  uint16_t           val;
  struct foc_fault_s s;
};

/* FOC device configuration data */

struct foc_cfgcmn_s
{
  uint8_t        modulation;          /* Modualtion type */
  uint8_t        controller;          /* Controller type */
  uint32_t       pwm_freq;            /* FOC PWM frequency */
  uint32_t       work_freq;           /* FOC work frequency */
  uint32_t       notifier_freq;       /* FOC notifier frequency */
};

#ifdef  CONFIG_POWER_FOC_CONTROL_PI
/* FOC PI controller configuration data */

struct foc_ctrl_cfg_s
{
  /* FOC d (torque) current PI controller consntants */

  foc_number_t   id_kp;
  foc_number_t   id_ki;

  /* FOC q (flux) current PI controller consntants */

  foc_number_t   iq_kp;
  foc_number_t   iq_ki;
};
#endif  /* CONFIG_POWER_FOC_CONTROL_PI */

/* FOC device configuration */

struct foc_cfg_s
{
  int            type;        /* FOC configuration type */
  FAR void      *data;        /* FOC configuration data */
};

/* FOC device feedback data */

struct foc_fb_s
{
  foc_number_t  curr[CONFIG_POWER_FOC_PHASES]; /* Phase current feedback */
  foc_number_t  volt[CONFIG_POWER_FOC_PHASES]; /* Phase voltage feedback */
  foc_dqframe_t idq;                           /* DQ current feedback */
  foc_dqframe_t vdq;                           /* DQ voltage feedback */
  foc_abframe_t iab;                           /* Alpha-beta current feedback */
  foc_abframe_t vab;                           /* Alpha-beta voltage feedback */
};

/* Output data from the FOC device */

struct foc_state_s
{
  bool              start;       /* FOC start flag */
  union foc_fault_u fault;       /* Fault state */
  struct foc_fb_s   fb;          /* FOC controller feedback */
};

/* Input data to the FOC device */

struct foc_params_s
{
  foc_dqframe_t     dq_ref;      /* DQ reference */
  foc_dqframe_t     vdq_comp;    /* DQ voltage compensation */
  foc_number_t      angle;       /* Phase angle */
  foc_number_t      vbus;        /* Bus voltage */
};

/* FOC driver info */

struct foc_info_s
{
  uint8_t           devno;       /* FOC device instance number */
  uint8_t           typeno;      /* FOC type-specific device number */
  uint8_t           ftype;       /* FOC number type */
};

/* FOC device upper-half */

struct foc_lower_s;
struct foc_typespec_s;
struct foc_dev_s
{
  /* Fields managed by common upper-half FOC logic **************************/

  uint8_t                    ocount;     /* The number of times the device
                                          * has been opened
                                          */
  sem_t                      closesem;   /* Locks out new opens while close
                                          * is in progress
                                          */
  sem_t                      statesem;   /* Notifier semaphore */

  /* Fields provided by lower-half foc logic ********************************/

  FAR struct foc_lower_s    *lower;      /* Reference to the FOC lower-half */

  /* Fields provided by type-specific implementation ************************/

  FAR struct foc_typespec_s *typespec;  /* FOC type-specific implementation */

  /* FOC device specific data ***********************************************/

  struct foc_info_s          info;       /* Device info */
  struct foc_cfgcmn_s        cfgcmn;     /* FOC common configuration */
  FAR void                  *modulation; /* Modulation data */
  FAR void                  *control;    /* Controller data */

  /* FOC device input/output data *******************************************/

  int                        mode;       /* FOC device control mode */
  struct foc_state_s         state;      /* FOC device state */
  struct foc_params_s        params;     /* FOC device input data */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#ifdef CONFIG_POWER_FOC_USE_FLOAT
/****************************************************************************
 * Name: focf32_register
 ****************************************************************************/

int focf32_register(FAR const char *path, FAR struct foc_dev_s *dev);
#endif

#ifdef CONFIG_POWER_FOC_USE_FIXED16
/****************************************************************************
 * Name: focb16_register
 ****************************************************************************/

int focb16_register(FAR const char *path, FAR struct foc_dev_s *dev);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_DRIVERS_POWER_FOC_FOC_H */
