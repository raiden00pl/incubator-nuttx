/****************************************************************************
 * include/nuttx/power/focmodel/focmodel.h
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

#ifndef __INCLUDE_NUTTX_DRIVERS_POWER_FOCMODEL_FOCMODEL_H
#define __INCLUDE_NUTTX_DRIVERS_POWER_FOCMODEL_FOCMODEL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <semaphore.h>

#include <nuttx/power/foc/foc_types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* FOC model configuration */

struct focmodel_cfg_s
{
  uint8_t      poles;            /* Motor poles */
  foc_number_t res;              /* Motor resistance */
  foc_number_t ind;              /* Motor inducutance */
  foc_number_t interia;          /* Motor interia */
  foc_number_t fluxlink;         /* Flux link */
  foc_number_t ind_d;            /* L_d */
  foc_number_t ind_q;            /* L_q */
};

/* Input parameters to the FOC model device */

struct focmodel_params_s
{
  foc_number_t load;             /* Motor external load */
};

/* FOC model device */

struct focmodel_callbacks_s;
struct focmodel_dev_s
{
  /* Fields managed by common upper half FOC model logic ********************/

  uint8_t                      devno;    /* Device instance number */
  uint8_t                      typeno;   /* Device type instance number */
  uint8_t                      ocount;   /* The number of times the device
                                          * has been opened
                                          */
  uint8_t                      ftype;    /* FOC driver float type */
  sem_t                        closesem; /* Locks out new opens while close
                                          * is in progress
                                          */

  /* Fields provided by lower half foc logic ********************************/

  FAR struct focmodel_lower_s *lower;    /* Reference to the FOC lower-half */

  /* Fields provided by type specific implementation ************************/

  struct focmodel_typespec_s *typespec; /* FOC type-specific implementation */

  /* FOC model driver specific data *****************************************/

  FAR void                    *model;      /* FOC model data */
  bool                         params_set; /* Params set flag */
  bool                         cfg_set;    /* Config set flag */
  struct focmodel_cfg_s        cfg;        /* FOC model configuration */
  struct focmodel_params_s     params;     /* FOC model parameters */
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
 * Name: focf32_model_register
 ****************************************************************************/

int focf32_model_register(FAR const char *path,
                          FAR struct focmodel_dev_s *dev);
#endif

#ifdef CONFIG_POWER_FOC_USE_FIXED16
/****************************************************************************
 * Name: focb16_model_register
 ****************************************************************************/

int focb16_model_register(FAR const char *path,
                          FAR struct focmodel_dev_s *dev);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_DRIVERS_POWER_FOCMODEL_FOCMODEL_H */
