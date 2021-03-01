/****************************************************************************
 * include/nuttx/power/focmodel/focmodel_lower.h
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

#ifndef __INCLUDE_NUTTX_DRIVERS_POWER_FOCMODEL_FOCMODEL_LOWER_H
#define __INCLUDE_NUTTX_DRIVERS_POWER_FOCMODEL_FOCMODEL_LOWER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/power/focmodel/focmodel.h>
#include <nuttx/power/focmodel/focmodel_typespec.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Only for kernel side */

#ifndef __KERNEL__
#  error
#endif

/* Helper macros */

#define FOCMODEL_OPS_BIND(d, c) (d)->lower->ops->bind(d, c)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* FOC model callbacks */

struct focmodel_dev_s;
struct focmodel_callbacks_s
{
  /* Get model phase currents */

  FAR void (*current)(FAR struct focmodel_dev_s *dev,
                      FAR foc_number_t *current);

  /* Feed model with alpha-beta voltage
   * TODO: use the bus voltage and PWM duty cycle to get v_ab frame
   */

  FAR void (*vab)(FAR struct focmodel_dev_s *dev,
                  FAR foc_abframe_t *v_ab);

  /* Update duty cycle and VBUS */

  FAR void (*duty)(FAR struct focmodel_dev_s *dev,
                   FAR foc_number_t *duty,
                   FAR foc_number_t *vbus);

  /* Configure sampling frequency */

  CODE void (*freq)(FAR struct focmodel_dev_s *dev, uint32_t freq);

  /* Start */

  CODE int (*start)(FAR struct focmodel_dev_s *dev);
};

/* Arch-specific FOC model ops */

struct focmodel_lower_ops_s
{
  /* Bind the upper-half driver with the lower half arch-specific driver */

  CODE int (*bind)(FAR struct focmodel_dev_s *dev,
                   FAR struct focmodel_callbacks_s *cb);
};

/* Lower-half FOC model data */

struct focmodel_lower_s
{
  FAR struct focmodel_lower_ops_s *ops;    /* The FOC model lower-half operations */
  FAR void                        *data;   /* The FOC model lower-half data */
};

#endif /* __INCLUDE_NUTTX_DRIVERS_POWER_FOCMODEL_FOCMODEL_LOWER_H */
