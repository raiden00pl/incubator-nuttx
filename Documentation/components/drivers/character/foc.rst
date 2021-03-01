====================
FOC Driver Interface
====================

FOC Driver
===========

********************************************************************************

Field Oriented Control (FOC) is a common technique to control either synchronous
or asynchronous alternating current machines. The main goal of FOC is to control
direct current (Id) and quadrature current (Iq) in powered device.

There is a two possible way to implement this type of device.
In the first, implement the entire control logic on the kernel space and provide
an interface to update the controller settings with a user space.

The second option is to implement the controller logic on the application level,
which simplifies the kernel device.

This device uses a second approach. It must ensure the following functionality
for the user space:

#. update PWM duty cycles
#. return ADC current samples
#. synchronise user-space with PWM events

The Nuttx FOC driver is split into two parts:

#. An "upper half", generic driver that provides the common FOC interface to application level code,
#. A "lower half", platform-specific driver that implemets the low-level logic to implement the FOC functionality

Files supporitng FOC can be found in the following locations:

-  ``include/nuttx/power/foc/foc.h``. "Upper-half" FOC interface available for user-space.
-  ``include/nuttx/power/foc/foc_lower.h``. "Lower-half" FOC interface.
-  ``drivers/power/foc/foc_dev.c``. The generic "upper half" FOC driver

The majority of the functionality available to the application is implemented in driver ioctl calls. Supported ioctl commands:

- ``PWRIOC_START`` - Start the FOC device, arg: none
- ``PWRIOC_STOP`` - Stop the FOC device, arg: none
- ``PWRIOC_GET_STATE`` - Get the FOC device state, arg: ``struct foc_state_s`` pointer.
  This is a blocking operation that is used to synchronize the user space application with ADC samples
- ``PWRIOC_CLEAN_FAULT`` - Clean the FOC device fault state, arg: none
- ``PWRIOC_SET_PARAMS`` - Set the FOC device operation parameters, arg: ``struct foc_params_s`` pointer
- ``PWRIOC_GET_PARAMS`` - Get the FOC device operation parameters, arg: ``struct foc_params_s`` pointer
- ``PWRIOC_SET_CONFIG`` - Set the FOC device configuration, arg: ``struct foc_cfg_s`` pointer
- ``PWRIOC_GET_CONFIG`` - Get the FOC device configuration, arg: ``struct foc_cfg_s`` pointer
- ``PWRIOC_GET_INFO`` -  Get the FOC device info, arg: ``struct foc_info_s`` pointer
