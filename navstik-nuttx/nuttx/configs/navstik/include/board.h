/************************************************************************************
 * configs/navstik/include/board.h
 * include/arch/board/board.h
 *
 *   Copyright (C) 2013 Navstik Development Team. Based on PX4 port.
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_BOARD_BOARD_H
#define __ARCH_BOARD_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/
/* The Navstik uses a 25MHz crystal connected to the HSE.
 *
 * This is the "standard" configuration as set up by arch/arm/src/stm32f40xx_rcc.c:
 *   System Clock source           : PLL (HSE)
 *   SYSCLK(Hz)                    : 168000000    Determined by PLL configuration
 *   HCLK(Hz)                      : 168000000    (STM32_RCC_CFGR_HPRE)
 *   AHB Prescaler                 : 1            (STM32_RCC_CFGR_HPRE)
 *   APB1 Prescaler                : 4            (STM32_RCC_CFGR_PPRE1)
 *   APB2 Prescaler                : 2            (STM32_RCC_CFGR_PPRE2)
 *   HSE Frequency(Hz)             : 25000000     (STM32_BOARD_XTAL)
 *   PLLM                          : 25           (STM32_PLLCFG_PLLM)
 *   PLLN                          : 336          (STM32_PLLCFG_PLLN)
 *   PLLP                          : 2            (STM32_PLLCFG_PLLP)
 *   PLLQ                          : 7            (STM32_PLLCFG_PPQ)
 *   Main regulator output voltage : Scale1 mode  Needed for high speed SYSCLK
 *   Flash Latency(WS)             : 5
 *   Prefetch Buffer               : OFF
 *   Instruction cache             : ON
 *   Data cache                    : ON
 *   Require 48MHz for USB OTG FS, : Enabled
 *   SDIO and RNG clock
 */

/* HSI - 16 MHz RC factory-trimmed
 * LSI - 32 KHz RC
 * HSE - On-board crystal frequency is 25MHz
 * LSE - not installed
 */

#define STM32_BOARD_XTAL        25000000ul

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
//#define STM32_LSE_FREQUENCY     0

/* Main PLL Configuration.
 *
 * PLL source is HSE
 * PLL_VCO = (STM32_HSE_FREQUENCY / PLLM) * PLLN
 *         = (25,000,000 / 25) * 336
 *         = 336,000,000
 * SYSCLK  = PLL_VCO / PLLP
 *         = 336,000,000 / 2 = 168,000,000
 * USB OTG FS, SDIO and RNG Clock
 *         =  PLL_VCO / PLLQ
 *         = 48,000,000
 */

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(25)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(336)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_2
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(7)

#define STM32_SYSCLK_FREQUENCY  168000000ul

/* AHB clock (HCLK) is SYSCLK (168MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK  /* HCLK  = SYSCLK / 1 */
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY
#define STM32_BOARD_HCLK        STM32_HCLK_FREQUENCY  /* same as above, to satisfy compiler */

/* APB1 clock (PCLK1) is HCLK/4 (42MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd4     /* PCLK1 = HCLK / 4 */
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/4)

/* Timers driven from APB1 will be twice PCLK1 */

#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM12_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM13_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM14_CLKIN  (2*STM32_PCLK1_FREQUENCY)

/* APB2 clock (PCLK2) is HCLK/2 (84MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLKd2     /* PCLK2 = HCLK / 2 */
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* Timers driven from APB2 will be twice PCLK2 */

#define STM32_APB2_TIM1_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM9_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM10_CLKIN  (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM11_CLKIN  (2*STM32_PCLK2_FREQUENCY)

/* Timer Frequencies, if APBx is set to 1, frequency is same to APBx
 * otherwise frequency is 2xAPBx. 
 * Note: TIM1,8 are on APB2, others on APB1
 */

#define STM32_TIM18_FREQUENCY   (2*STM32_PCLK2_FREQUENCY)
#define STM32_TIM27_FREQUENCY   (2*STM32_PCLK1_FREQUENCY)

/* SDIO dividers.  Note that slower clocking is required when DMA is disabled 
 * in order to avoid RX overrun/TX underrun errors due to delayed responses
 * to service FIFOs in interrupt driven mode.  These values have not been
 * tuned!!!
 *
 * HCLK=72MHz, SDIOCLK=72MHz, SDIO_CK=HCLK/(178+2)=400 KHz
 */
  
#define SDIO_INIT_CLKDIV        (178 << SDIO_CLKCR_CLKDIV_SHIFT)

/* DMA ON:  HCLK=72 MHz, SDIOCLK=72MHz, SDIO_CK=HCLK/(2+2)=18 MHz
 * DMA OFF: HCLK=72 MHz, SDIOCLK=72MHz, SDIO_CK=HCLK/(3+2)=14.4 MHz
 */

#ifdef CONFIG_SDIO_DMA
#  define SDIO_MMCXFR_CLKDIV    (2 << SDIO_CLKCR_CLKDIV_SHIFT) 
#else
#  define SDIO_MMCXFR_CLKDIV    (3 << SDIO_CLKCR_CLKDIV_SHIFT) 
#endif

/* DMA ON:  HCLK=72 MHz, SDIOCLK=72MHz, SDIO_CK=HCLK/(1+2)=24 MHz
 * DMA OFF: HCLK=72 MHz, SDIOCLK=72MHz, SDIO_CK=HCLK/(3+2)=14.4 MHz
 */

#ifdef CONFIG_SDIO_DMA
#  define SDIO_SDXFR_CLKDIV     (1 << SDIO_CLKCR_CLKDIV_SHIFT)
#else
#  define SDIO_SDXFR_CLKDIV     (3 << SDIO_CLKCR_CLKDIV_SHIFT)
#endif

/* High-resolution timer
 */
#ifdef CONFIG_HRT_TIMER
# define HRT_TIMER		4	/* use timer4 for the HRT */
# define HRT_TIMER_CHANNEL	1	/* use capture/compare channel */
#endif

/* LED definitions ******************************************************************/
/* Navstik has two LEDs that we will encode as: */

#define LED_STARTED       0  /* LED? */
#define LED_INIRQ         1  /* LED? + LED? */
#define LED_HEAPALLOCATE  2  /* LED? */
#define LED_IRQSENABLED   3  /* LED? + LED? */
#define LED_STACKCREATED  4  /* LED? */
#define LED_SIGNAL        5  /* LED? + LED? */
#define LED_ASSERTION     6  /* LED? + LED? + LED? */
#define LED_PANIC         7  /* N/C  + N/C  + N/C + LED? */

/* Alternate function pin selections ************************************************/

/*
 * UARTs.
 *
 * Note that UART5 has no optional pinout.
 */
#define GPIO_USART1_RX	GPIO_USART1_RX_2
#define GPIO_USART1_TX	GPIO_USART1_TX_2

#define GPIO_USART2_RX	GPIO_USART2_RX_1
#define GPIO_USART2_TX	GPIO_USART2_TX_1
//#define GPIO_USART2_RTS	GPIO_USART2_RTS_1
//#define GPIO_USART2_CTS	GPIO_USART2_CTS_1

#ifdef NASVSTIK_CONFIG_SPEKTRUM
#define GPIO_USART6_RX	GPIO_USART6_RX_1
#define GPIO_USART6_TX	GPIO_USART6_TX_1
#define DMAMAP_USART6_RX DMAMAP_USART6_RX_2
#endif

/* UART DMA configuration for USART1 */
#define DMAMAP_USART1_RX DMAMAP_USART1_RX_2

/*
 * PWM
 *
 * The Navstik has six PWM outputs
 * The mapping are as follows :
 * PB5	Servo 1 Timer 3 Channel 2 (AF2)
 * PA10	Servo 2 Timer 1 channel 3 (AF1)
 * PC8	Servo 3 Timer 8 Channel 3 (AF3)
 * PB11	Servo 4 Timer 2 Channel 4 (AF1)
 * PB1	Servo 5 Timer 3 Channel 4 (AF2)
 * PB0	Servo 6 Timer 3 Channel 3 (AF2)
 *
 */

#define GPIO_TIM3_CH2OUT	GPIO_TIM3_CH2OUT_2
#define GPIO_TIM1_CH3OUT	GPIO_TIM1_CH3OUT_1		
#define GPIO_TIM8_CH3OUT	GPIO_TIM8_CH3OUT_1
#define GPIO_TIM2_CH4OUT	GPIO_TIM2_CH4OUT_2
#define GPIO_TIM3_CH4OUT	GPIO_TIM3_CH4OUT_1
#define GPIO_TIM3_CH3OUT	GPIO_TIM3_CH3OUT_1

#ifdef NAVSTIK_CONFIG_PWMIN

/* PWM Input 
 * RC1 PA7	Timer 14 Channel 1 (AF9)
 * RC2 PA6	Timer 3 Channel 1 (AF2)
 * RC3 PB14 Timer 12 Channel 1 (AF9)
 * RC4 PB15	Timer 12 Channel 2 (AF9)
 * RC5 PC6	Timer 8 Channel 1 (AF3)
 * RC6 PC7	Timer 8 Channel 2 (AF3)
 */

#define GPIO_TIM14_CH1IN	GPIO_TIM14_CH1IN_1
#define GPIO_TIM3_CH1IN		GPIO_TIM3_CH1IN_1
#define GPIO_TIM12_CH1IN	GPIO_TIM12_CH1IN_2
#define GPIO_TIM12_CH2IN	GPIO_TIM12_CH2IN_1
#define GPIO_TIM8_CH1IN		GPIO_TIM8_CH1IN_1
#define GPIO_TIM8_CH2IN		GPIO_TIM8_CH2IN_1

#endif	// NAVSTIK_CONFIG_PWMIN

/*
 *  PPM
 *  PPM input is handled by the HRT timer.
 */
#if defined(CONFIG_HRT_TIMER) && defined (CONFIG_HRT_PPM)
# define HRT_PPM_CHANNEL	3	/* use capture/compare channel 3 */
# define GPIO_PPM_IN		(GPIO_ALT|GPIO_AF9|GPIO_SPEED_50MHz|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN7)
#endif



/*
 * I2C
 *
 * The optional _GPIO configurations allow the I2C driver to manually
 * reset the bus to clear stuck slaves.  They match the pin configuration,
 * but are normally-high GPIOs.
 */
#define GPIO_I2C1_SCL		GPIO_I2C1_SCL_2
#define GPIO_I2C1_SDA		GPIO_I2C1_SDA_2
#define GPIO_I2C1_SCL_GPIO	(GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN8)
#define GPIO_I2C1_SDA_GPIO	(GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN9)

#define GPIO_I2C3_SCL		GPIO_I2C3_SCL_1
#define GPIO_I2C3_SDA		GPIO_I2C3_SDA_1
#define GPIO_I2C3_SCL_GPIO	(GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN8)
#define GPIO_I2C3_SDA_GPIO	(GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN9)

/*
 * I2C busses
 */
#define NAVSTIK_I2C_BUS_MPU		1
//#define NAVSTIK_I2C_BUS_EXT		2
#define NAVSTIK_I2C_BUS_SENSORS		3

/*
 * Devices on the onboard bus.
 *
 * Note that these are unshifted addresses.
 */
#define NAVSTIK_I2C_OBDEV_HMC5883	0x1e	// Magnetometer
#define NAVSTIK_I2C_OBDEV_BMP180	0x77	// Static pressure sensor
#define NAVSTIK_I2C_OBDEV_MS4515	0x28	// Differential pressure sensor (optional component)
#define NAVSTIK_I2C_OBDEV_MPU60x0	0x69	// Accelerometer + Gyro

/* Configure interrupt GPIOs of peripherals on I2C bus */
#define NAVSTIK_I2C_OBDEV_MPU60x0_INT	(GPIO_INPUT|GPIO_PORTC|GPIO_PIN0)
#define NAVSTIK_I2C_OBDEV_HMC5883_INT	(GPIO_INPUT|GPIO_PORTC|GPIO_PIN14)
#define NAVSTIK_I2C_OBDEV_MS4515_INT	(GPIO_INPUT|GPIO_PORTC|GPIO_PIN13)

/*
 * SPI
 */
#define GPIO_SPI2_MISO	GPIO_SPI2_MISO_2
#define GPIO_SPI2_MOSI	GPIO_SPI2_MOSI_2
#define GPIO_SPI2_SCK	GPIO_SPI2_SCK_1

/*
 * Use these in place of the spi_dev_e enumeration to
 * select a specific SPI device on SPI2
 */
#define NAVSTIK_SPIDEV_FLASH	1
#define NAVSTIK_SPIDEV_SDCARD	2

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/
/************************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the intitialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

EXTERN void stm32_boardinitialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __ARCH_BOARD_BOARD_H */
