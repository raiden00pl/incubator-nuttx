README
======

The B-G431B-ESC Discovery kit board is based on the STM32G431CB microcontroller,
the L6387 driver and STL180N6F7 power MOSFETs.

UART/USART PINS
---------------

USART2 is accessible through J3 pads and ST LINK Virtual Console:
  USART2_TX - PB3
  USART2_RX - PB4

Configuration Sub-directories
-------------------------

  nsh:
  ---
    Configures the NuttShell (nsh) located at apps/examples/nsh.  The
    Configuration enables the serial interfaces on USART2.

  foc_f32 and foc_b16:
  -------------------

    Pin configuration for FOC examples (TIM1 configuration):

    Board Function   Chip Function      Chip Pin Number
    -------------   ----------------   -----------------
     Phase U high    TIM1_CH1           PA8
     Phase U low     TIM1_CH1N          PC13
     Phase V high    TIM1_CH2           PA9
     Phase V low     TIM1_CH2N          PA12
     Phase W high    TIM1_CH3           PA10
     Phase W low     TIM1_CH3N          PB15

     Current U +     OPAMP1_VINP        PA1
     Current U -     OPAMP1_VINM        PA3
     Current V +     (OPAMP2_VINP)[1]   PA7
     Current V -     (OPAMP2_VINM)[1]   PA5
     Current W +     OPAMP3_VINP        PB0
     Current W -     OPAMP3_VINM        PB2
     Temperature     ADC1_IN            PB14
     VBUS            ADC1_IN            PA0
     POT             ADC1_IN            PB12

     ENCO_A/HALL_H1                     PB6
     ENCO_B/HALL_H2                     PB7
     ENCO_Z/HALL_H3                     PB8

     CAN_RX                             PA11
     CAN_TX                             PB9
     CAN_SHDN                           PC11


   [1] - Not used.
         OPAMP2_VOUT can be connected only for ADC2 which can't be
         properly handled by STM32 FOC lower-half driver.
         
