/****************************************************************************
 * boards/arm/stm32/odrive/include/board.h
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

#ifndef __BOARDS_ARM_STM32_ODRIVE_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32_ODRIVE_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif
#include "stm32_rcc.h"
#include "stm32.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* HSI - 16 MHz RC factory-trimmed
 * LSI - 32 KHz RC (30-60KHz, uncalibrated)
 * HSE - On-board crystal frequency is 8MHz
 * LSE - 32.768 kHz
 */

#define STM32_BOARD_XTAL        8000000ul

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768

/* Main PLL Configuration.
 *
 * PLL source is HSE
 * PLL_VCO = (STM32_HSE_FREQUENCY / PLLM) * PLLN
 *         = (25,000,000 / 12) * 360
 *         = 240,000,000
 * SYSCLK  = PLL_VCO / PLLP
 *         = 240,000,000 / 2 = 120,000,000
 * USB OTG FS, SDIO and RNG Clock
 *         =  PLL_VCO / PLLQ
 *         = 240,000,000 / 5 = 48,000,000
 *         = 48,000,000
 */

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(12)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(360)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_2
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(5)

#define STM32_SYSCLK_FREQUENCY  120000000ul

/* AHB clock (HCLK) is SYSCLK (120MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK  /* HCLK  = SYSCLK / 1 */
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY
#define STM32_BOARD_HCLK        STM32_HCLK_FREQUENCY  /* same as above, to satisfy compiler */

/* APB1 clock (PCLK1) is HCLK/4 (30MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd4     /* PCLK1 = HCLK / 4 */
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/4)

/* Timers driven from APB1 will be twice PCLK1 (60Mhz) */

#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM12_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM13_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM14_CLKIN  (2*STM32_PCLK1_FREQUENCY)

/* APB2 clock (PCLK2) is HCLK/2 (60MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLKd2     /* PCLK2 = HCLK / 2 */
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* Timers driven from APB2 will be twice PCLK2 (120Mhz) */

#define STM32_APB2_TIM1_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM9_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM10_CLKIN  (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM11_CLKIN  (2*STM32_PCLK2_FREQUENCY)

/* Timer Frequencies, if APBx is set to 1, frequency is same to APBx
 * otherwise frequency is 2xAPBx.
 * Note: TIM1,8 are on APB2, others on APB1
 */

#define BOARD_TIM1_FREQUENCY    (2*STM32_PCLK2_FREQUENCY)
#define BOARD_TIM2_FREQUENCY    (2*STM32_PCLK1_FREQUENCY)
#define BOARD_TIM3_FREQUENCY    (2*STM32_PCLK1_FREQUENCY)
#define BOARD_TIM4_FREQUENCY    (2*STM32_PCLK1_FREQUENCY)
#define BOARD_TIM5_FREQUENCY    (2*STM32_PCLK1_FREQUENCY)
#define BOARD_TIM6_FREQUENCY    (2*STM32_PCLK1_FREQUENCY)
#define BOARD_TIM7_FREQUENCY    (2*STM32_PCLK1_FREQUENCY)
#define BOARD_TIM8_FREQUENCY    (2*STM32_PCLK2_FREQUENCY)

/* DMA Channel/Stream Selections ********************************************/

/* ADC 1 */

#define ADC1_DMA_CHAN           DMAMAP_ADC1_1

/* Alternate function pin selections ****************************************/

/* USART2:
 *  USART2_TX - PA2 - GPIO_3
 *  USART2_RX - PA3 - GPIO_4
 */

#define GPIO_USART2_RX    GPIO_USART2_RX_1
#define GPIO_USART2_TX    GPIO_USART2_TX_1

/* CAN:
 *   CAN_R - PB8
 *   CAN_T - PB9
 */

#define GPIO_CAN1_RX      GPIO_CAN1_RX_2
#define GPIO_CAN1_TX      GPIO_CAN1_TX_2

/* SPI3 - connected to DRV8301
 *   SPI3_SCK  - PC10
 *   SPI3_MISO - PC11
 *   SPI3_MOSI - PC12
 */

#define GPIO_SPI3_SCK  GPIO_SPI3_SCK_2
#define GPIO_SPI3_MISO GPIO_SPI3_MISO_2
#define GPIO_SPI3_MOSI GPIO_SPI3_MOSI_2

/* Dual FOC configuration */

/* TIM1 configuration *******************************************************/

#define GPIO_TIM1_CH1OUT   GPIO_TIM1_CH1OUT_1 /* TIM1 CH1  - PA8  - U high */
#define GPIO_TIM1_CH1NOUT  GPIO_TIM1_CH1N_2   /* TIM1 CH1N - PB13 - U low */
#define GPIO_TIM1_CH2OUT   GPIO_TIM1_CH2OUT_1 /* TIM1 CH2  - PA9  - V high */
#define GPIO_TIM1_CH2NOUT  GPIO_TIM1_CH2N_2   /* TIM1 CH2N - PB14 - V low */
#define GPIO_TIM1_CH3OUT   GPIO_TIM1_CH3OUT_1 /* TIM1 CH3  - PA10 - W high */
#define GPIO_TIM1_CH3NOUT  GPIO_TIM1_CH3N_2   /* TIM1 CH3N - PB15 - W low */
#define GPIO_TIM1_CH4OUT   0                  /* not used as output */

/* TIM8 configuration *******************************************************/

#define GPIO_TIM8_CH1OUT   GPIO_TIM8_CH1OUT_1 /* TIM8 CH1  - PC6  - U high */
#define GPIO_TIM8_CH1NOUT  GPIO_TIM8_CH1N_2   /* TIM8 CH1N - PA7 - U low */
#define GPIO_TIM8_CH2OUT   GPIO_TIM8_CH2OUT_1 /* TIM8 CH2  - PC7  - V high */
#define GPIO_TIM8_CH2NOUT  GPIO_TIM8_CH2N_1   /* TIM8 CH2N - PB0 - V low */
#define GPIO_TIM8_CH3OUT   GPIO_TIM8_CH3OUT_1 /* TIM8 CH3  - PC8 - W high */
#define GPIO_TIM8_CH3NOUT  GPIO_TIM8_CH3N_1   /* TIM8 CH3N - PB1 - W low */
#define GPIO_TIM8_CH4OUT   0                  /* not used as output */

/* Debug pin */

#  define GPIO_FOC_DEBUG0  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                            GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN0)
#  define GPIO_FOC_DEBUG1  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                            GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN1)
#  define GPIO_FOC_DEBUG2  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                            GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN4)
#  define GPIO_FOC_DEBUG3  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                            GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN2)
#  define GPIO_FOC_DEBUG4  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                            GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN15)

#endif /* __BOARDS_ARM_STM32_ODRIVE_INCLUDE_BOARD_H */
