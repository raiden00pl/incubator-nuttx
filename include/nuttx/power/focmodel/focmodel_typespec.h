/****************************************************************************
 * include/nuttx/power/focmodel/focmodel_typespec.h
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

#ifndef __INCLUDE_NUTTX_DRIVERS_POWER_FOCMODEL_FOCMODEL_TYPESPEC_H
#define __INCLUDE_NUTTX_DRIVERS_POWER_FOCMODEL_FOCMODEL_TYPESPEC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/power/focmodel/focmodel.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Only for kernel side */

#ifndef __KERNEL__
#  error
#endif

/* Helper macros */

#define FOCMODEL_TS_INIT(d)          (d)->typespec->init(d)
#define FOCMODEL_TS_START(d)         (d)->typespec->start(d)
#define FOCMODEL_TS_VAB(d, v)        (d)->typespec->vab(d, v)
#define FOCMODEL_TS_CURRENT(d, c)    (d)->typespec->current(d, c)
#define FOCMODEL_TS_DUTY(d, c, v)    (d)->typespec->duty(d, c, v)
#define FOCMODEL_TS_FREQ(d, f)       (d)->typespec->freq(d, f)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* FOC type specific interface */

struct focmodel_typespec_s
{
  /* Initialzie the FOC model */

  CODE int  (*init)(FAR struct focmodel_dev_s *dev);

  /* Start the FOC model */

  CODE int  (*start)(FAR struct focmodel_dev_s *dev);

  /* Feed model with alpha-veta voltage */

  CODE void (*vab)(FAR struct focmodel_dev_s *dev,
                   FAR foc_abframe_t *v_ab);

  /* Get model phase currents */

  CODE void (*current)(FAR struct focmodel_dev_s *dev,
                       FAR foc_number_t *current);

  /* Feed model with PWM duty cycle and VBUS */

  CODE void (*duty)(FAR struct focmodel_dev_s *dev,
                    FAR foc_number_t *duty,
                    FAR foc_number_t *vbus);

  /* Configure sampling frequency */

  CODE void (*freq)(FAR struct focmodel_dev_s *dev,
                    uint32_t freq);
};

#endif /* __INCLUDE_NUTTX_DRIVERS_POWER_FOCMODEL_FOCMODEL_TYPESPEC_H */
