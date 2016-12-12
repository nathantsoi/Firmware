/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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
 ****************************************************************************/

/**
 * @file board_config.h
 *
 * PX4-REVOLUTION internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include <stm32.h>
#include <arch/board/board.h>

#define UDID_START		0x1FFF7A10

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/
/* Configuration ************************************************************************************/

/* Revolution GPIOs **********************************************************************************/
/* LEDs */

#define GPIO_LED_RED       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                            GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN0)
#define GPIO_LED_GREEN       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                              GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN1)

#define GPIO_DRDY_MPU6000  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN4)

/*
 * I2C buses
 * I2C_1: PB8 (SCL) and PB9 (SDA), Onboard HMC5883L & Pressure Sensor
 * I2C_2: PB10 (SCL) and PB11 (SDA), Flex Port 1
 */
#define PX4_I2C_BUS_ONBOARD	1
#define PX4_I2C_BUS_EXPANSION	2

#define PX4_I2C_BUS_ONBOARD_HZ      400000
#define PX4_I2C_BUS_EXPANSION_HZ      400000

#define PX4_I2C_BUS_MTD	PX4_I2C_BUS_EXPANSION



/* Devices on the onboard bus.
 *
 * Note that these are unshifted addresses.
 */
#define PX4_I2C_OBDEV_MPU9250	0x1E //HMC5883L, default address 0x1E

/* USB OTG FS
 *
 * PC5  OTG_FS_VBUS VBUS sensing
 */
#define GPIO_OTGFS_VBUS		(GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_OPENDRAIN|GPIO_PORTC|GPIO_PIN5)

/*
 * TODO: ADC channels
 *
 * PC1: ADC123_11 current
 * PC2: ADC123_12 voltage
 */
#define ADC_CHANNELS 0

// ADC defines to be used in sensors.cpp to read from a particular channel
// Crazyflie 2 performs battery sensing via the NRF module
#define ADC_BATTERY_VOLTAGE_CHANNEL	((uint8_t)(-1))
#define ADC_BATTERY_CURRENT_CHANNEL	((uint8_t)(-1))
#define ADC_AIRSPEED_VOLTAGE_CHANNEL	((uint8_t)(-1))

/*
 * TODO: Buzzer
 * Tone alarm output : These are only applicable when the buzzer deck is attached
 */
//#define TONE_ALARM_TIMER	5	/* timer 5 */
//#define TONE_ALARM_CHANNEL 3	/* channel 3 */
//#define GPIO_TONE_ALARM_IDLE	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN2)
//#define GPIO_TONE_ALARM		(GPIO_ALT|GPIO_AF2|GPIO_SPEED_2MHz|GPIO_PUSHPULL|GPIO_PORTA|GPIO_PIN2)
//#define GPIO_TONE_ALARM_NEG (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN3)

/* PWM
*
* Four PWM motor outputs are configured.
*
* Pins:
*
* CH1 : PB0 : TIM3_CH3
* CH2 : PB1 : TIM3_CH4
* CH3 : PA3 : TIM2_CH4
* CH4 : PA2 : TIM2_CH3
* CH5 : PA1 : TIM5_CH2
* CH6 : PA0 : TIM5_CH1
*/

#define GPIO_TIM3_CH3OUT GPIO_TIM3_CH3OUT_1
#define GPIO_TIM3_CH4OUT GPIO_TIM3_CH4OUT_1
#define GPIO_TIM2_CH4OUT GPIO_TIM2_CH4OUT_1
#define GPIO_TIM2_CH3OUT GPIO_TIM2_CH3OUT_1
#define GPIO_TIM5_CH2OUT GPIO_TIM5_CH2OUT_1
#define GPIO_TIM5_CH1OUT GPIO_TIM5_CH1OUT_1

#define DIRECT_PWM_OUTPUT_CHANNELS	6

#define GPIO_TIM3_CH3IN GPIO_TIM3_CH3IN_1
#define GPIO_TIM3_CH4IN GPIO_TIM3_CH4IN_1
#define GPIO_TIM2_CH4IN GPIO_TIM2_CH4IN_1
#define GPIO_TIM2_CH3IN GPIO_TIM2_CH3IN_1
#define GPIO_TIM5_CH2IN GPIO_TIM5_CH2IN_1
#define GPIO_TIM5_CH1IN GPIO_TIM5_CH1IN_1

/* This board overrides the defaults by providing
 * PX4_PWM_ALTERNATE_RANGES and a replacement set of
 * constants
 */


/* PWM directly wired to transistor. Duty cycle directly corresponds to power
 * So we need to override the defaults
 */

//#define PX4_PWM_ALTERNATE_RANGES
//#define PWM_LOWEST_MIN 0
//#define PWM_MOTOR_OFF	0
//#define PWM_DEFAULT_MIN 0
//#define PWM_HIGHEST_MIN 0
//#define PWM_HIGHEST_MAX 255
//#define PWM_DEFAULT_MAX 255
//#define PWM_LOWEST_MAX 255


/* High-resolution timer */
#define HRT_TIMER		8	/* use timer8 for the HRT */
#define HRT_TIMER_CHANNEL	1	/* use capture/compare channel */



#define BOARD_HAS_PWM	DIRECT_PWM_OUTPUT_CHANNELS

#define BOARD_FMU_GPIO_TAB { {0, 0, 0}, }

#define BOARD_NAME "REVOLUTION"

__BEGIN_DECLS

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************************************
 * Name: board_spi_reset board_peripheral_reset
 *
 * Description:
 *   Called to reset SPI and the perferal bus
 *
 ****************************************************************************************************/

#define board_spi_reset(ms)
#define board_peripheral_reset(ms)

/****************************************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called to configure USB IO.
 *
 ****************************************************************************************************/

extern void stm32_usbinitialize(void);

/****************************************************************************
 * Name: nsh_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization for NSH.
 *
 *   CONFIG_NSH_ARCHINIT=y :
 *     Called from the NSH library
 *
 *   CONFIG_BOARD_INITIALIZE=y, CONFIG_NSH_LIBRARY=y, &&
 *   CONFIG_NSH_ARCHINIT=n :
 *     Called from board_initialize().
 *
 ****************************************************************************/

#ifdef CONFIG_NSH_LIBRARY
int nsh_archinitialize(void);
#endif

/****************************************************************************
 * Name: board_i2c_initialize
 *
 * Description:
 *   Called to set I2C bus frequncies.
 *
 ****************************************************************************/

int board_i2c_initialize(void);

#include "../common/board_common.h"

#endif /* __ASSEMBLY__ */

__END_DECLS
