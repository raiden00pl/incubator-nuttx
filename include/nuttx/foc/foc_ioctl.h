/****************************************************************************
 * include/nuttx/foc/foc_ioctl.h
 * NuttX Foc-Related IOCTLs definitions
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

#ifndef __INCLUDE_NUTTX_FOC_FOC_IOCTL_H
#define __INCLUDE_NUTTX_FOC_FOC_IOCTL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* All foc-related IOCTL commands must be defined in this header file
 * in order to assure that every IOCTL command is unique and will not be
 * aliased.
 */

#define FOCIOC_START          _FOCIOC(1)
#define FOCIOC_STOP           _FOCIOC(2)
#define FOCIOC_GET_STATE      _FOCIOC(3)
#define FOCIOC_CLEAN_FAULT    _FOCIOC(4)
#define FOCIOC_SET_PARAMS     _FOCIOC(5)
#define FOCIOC_GET_PARAMS     _FOCIOC(6)
#define FOCIOC_SET_CONFIG     _FOCIOC(7)
#define FOCIOC_GET_CONFIG     _FOCIOC(8)
#define FOCIOC_GET_INFO       _FOCIOC(9)

#endif /* __INCLUDE_NUTTX_FOC_FOC_IOCTL_H */
