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
 * omnibusf4sd internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/
/* Configuration ************************************************************************************/

/* omnibusf4sd GPIOs ***********************************************************************************/
/* LEDs */
// power - green
// LED1 - PB5 - blue
#define GPIO_LED1       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN5)
#define GPIO_LED_GREEN  GPIO_LED1

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that can be used by the Px4 Firmware in the adc driver
 */
// TODO: ADCs, eg. pixracer
//#define ADC_CHANNELS (1 << 2) | (1 << 3) | (1 << 4) | (1 << 10) | (1 << 11) | (1 << 12) | (1 << 13) | (1 << 14)

// Placeholder
#define ADC_BATTERY_VOLTAGE_CHANNEL	((uint8_t)(-1))
#define ADC_BATTERY_CURRENT_CHANNEL	((uint8_t)(-1))
#define ADC_AIRSPEED_VOLTAGE_CHANNEL	((uint8_t)(-1))

// TODO: ADCs
//#define ADC_BATTERY_VOLTAGE_CHANNEL  2
//#define ADC_BATTERY_CURRENT_CHANNEL  3
//#define ADC_5V_RAIL_SENSE            4
//#define ADC_RC_RSSI_CHANNEL          11

/* Define Battery 1 Voltage Divider and A per V
 */

// TODO:
//#define BOARD_BATTERY1_V_DIV         (13.653333333f)
//#define BOARD_BATTERY1_A_PER_V       (36.367515152f)

/* User GPIOs
 *
 * GPIO0-5 are the PWM servo outputs.
 */
#define GPIO_GPIO0_INPUT             (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN0)
#define GPIO_GPIO1_INPUT             (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN1)
#define GPIO_GPIO2_INPUT             (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN3)
#define GPIO_GPIO3_INPUT             (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN2)
#define GPIO_GPIO4_INPUT             (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN1)
#define GPIO_GPIO5_INPUT             (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN8)

#define GPIO_GPIO0_OUTPUT            (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN0)
#define GPIO_GPIO1_OUTPUT            (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN1)
#define GPIO_GPIO2_OUTPUT            (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN3)
#define GPIO_GPIO3_OUTPUT            (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN2)
#define GPIO_GPIO4_OUTPUT            (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN1)
#define GPIO_GPIO5_OUTPUT            (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN8)

/*----------------------------------------------------------*/
/*         OMNIBUSF4SD SPI chip selects and DRDY            */
/*----------------------------------------------------------*/

/* SPI chip selects */
/*
 * Define the Chip Selects for SPI1
 *
 * MPU6000: PA4
 *
 */
#define GPIO_SPI_CS_MEMS           (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN4)

/*
 * Define the Chip Selects for SPI2
 *
 * SD Card: PB12
 *
 */
#define GPIO_SPI_CS_SDCARD         (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN12)

/*
 * Define the Chip Selects for SPI3
 *
 * BMP280: PB3
 * ABT7456: PA15
 *
 */

#define GPIO_SPI3_CS_BARO          (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN3)
#define GPIO_SPI3_CS_OSD           (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN15)

/*
 *  Define the ability to shut off off the sensor signals
 *  by changing the signals to inputs
 */

#define _PIN_OFF(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz))

/* SPI 1 bus off */
#define GPIO_SPI1_SCK_OFF            _PIN_OFF(GPIO_SPI1_SCK)
#define GPIO_SPI1_MISO_OFF           _PIN_OFF(GPIO_SPI1_MISO)
#define GPIO_SPI1_MOSI_OFF           _PIN_OFF(GPIO_SPI1_MOSI)
/* SPI 1 CS's  off */
#define GPIO_SPI1_CS_MEMS_OFF         _PIN_OFF(GPIO_SPI_CS_MEMS)

/* SPI 2 bus off */
#define GPIO_SPI2_SCK_OFF            _PIN_OFF(GPIO_SPI2_SCK)
#define GPIO_SPI2_MISO_OFF           _PIN_OFF(GPIO_SPI2_MISO)
#define GPIO_SPI2_MOSI_OFF           _PIN_OFF(GPIO_SPI2_MOSI)
/* SPI 2 CS's  off */
#define GPIO_SPI2_CS_SDCARD_OFF         _PIN_OFF(GPIO_SPI_CS_SDCARD)

/* SPI 3 bus off */
#define GPIO_SPI3_SCK_OFF            _PIN_OFF(GPIO_SPI3_SCK)
#define GPIO_SPI3_MISO_OFF           _PIN_OFF(GPIO_SPI3_MISO)
#define GPIO_SPI3_MOSI_OFF           _PIN_OFF(GPIO_SPI3_MOSI)
/* SPI 3 CS's  off */
#define GPIO_SPI3_CS_BARO_OFF        _PIN_OFF(GPIO_SPI3_CS_BARO)

// One device per bus
#define PX4_SPI_BUS_SENSORS         1
#define PX4_SPIDEV_MPU              1
#define PX4_SPIDEV_BARO_BUS         3
#define PX4_SPIDEV_BARO             1

/* USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing
 */
#define GPIO_OTGFS_VBUS		(GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_OPENDRAIN|GPIO_PORTC|GPIO_PIN5)

/*----------------------------------------------------------*/
/*        End OMNIBUSF4SD SPI chip selects and DRDY         */
/*----------------------------------------------------------*/

#define PX4_SPI_BUS_BARO         3

#define PX4_I2C_BUS_EXPANSION	2
//#define PX4_I2C_BUS_ONBOARD	2


/* PWM
 *
 * 6 PWM outputs are configured.
 *
 * Alternatively CH3/CH4 could be assigned to UART6_TX/RX
 *
 * Pins:
 *
 * INPUTS:
 *  CH1 : PB8 : TIM10_CH1
 *  CH2 : PB9 : TIM4_CH4
 *  CH3 : PC6 : TIM8_CH1 // OR UART6_TX
 *  CH4 : PC7 : TIM8_CH2 // OR UART6_RX
 *  CH5 : PC8 : TIM8_CH3
 *  CH6 : PC9 : TIM8_CH4
 *
 * From Betaflight:
 *  DEF_TIM(TIM10, CH1, PB8,  TIM_USE_PWM | TIM_USE_PPM,   TIMER_OUTPUT_NONE,     0), // PPM
 *
 *  DEF_TIM(TIM4,  CH4, PB9,  TIM_USE_PWM,                 TIMER_OUTPUT_NONE,     0), // S2_IN
 *  DEF_TIM(TIM8,  CH1, PC6,  TIM_USE_PWM,                 TIMER_OUTPUT_NONE,     0), // S3_IN, UART6_TX
 *  DEF_TIM(TIM8,  CH2, PC7,  TIM_USE_PWM,                 TIMER_OUTPUT_NONE,     0), // S4_IN, UART6_RX
 *  DEF_TIM(TIM8,  CH3, PC8,  TIM_USE_PWM,                 TIMER_OUTPUT_NONE,     0), // S5_IN
 *  DEF_TIM(TIM8,  CH4, PC9,  TIM_USE_PWM,                 TIMER_OUTPUT_NONE,     0), // S6_IN
 *
 *  DEF_TIM(TIM3,  CH3, PB0,  TIM_USE_MOTOR,               TIMER_OUTPUT_STANDARD, 0), // S1_OUT D1_ST7
 *  DEF_TIM(TIM3,  CH4, PB1,  TIM_USE_MOTOR,               TIMER_OUTPUT_STANDARD, 0), // S2_OUT D1_ST2
 *  DEF_TIM(TIM2,  CH4, PA3,  TIM_USE_MOTOR,               TIMER_OUTPUT_STANDARD, 1), // S3_OUT D1_ST6
 *  DEF_TIM(TIM2,  CH3, PA2,  TIM_USE_MOTOR,               TIMER_OUTPUT_STANDARD, 0), // S4_OUT D1_ST1
 *  DEF_TIM(TIM5,  CH2, PA1,  TIM_USE_MOTOR,               TIMER_OUTPUT_STANDARD, 0), // S5_OUT
 *  DEF_TIM(TIM1,  CH1, PA8,  TIM_USE_MOTOR,               TIMER_OUTPUT_STANDARD, 0), // S6_OUT
 *
 *  DEF_TIM(TIM4,  CH1, PB6,  TIM_USE_LED,                 TIMER_OUTPUT_STANDARD, 0), // LED strip for F4 V2 / F4-Pro-0X and later (RCD_CS for F4)
 *  DEF_TIM(TIM1,  CH2, PA9,  TIM_USE_NONE,                TIMER_OUTPUT_NONE,     0), // UART1_TX
 *  DEF_TIM(TIM1,  CH3, PA10, TIM_USE_NONE,                TIMER_OUTPUT_NONE,     0), // UART1_RX
 */

// Re-defined from xxx_pinmap.h with the GPIO_OUTPUT_CLEAR
#define GPIO_TIM3_CH3OUT        (GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTB|GPIO_PIN0) //PB0 S1_OUT D1_ST7
#define GPIO_TIM3_CH4OUT        (GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTB|GPIO_PIN1) //PB1 S2_OUT D1_ST2
#define GPIO_TIM2_CH4OUT        (GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTA|GPIO_PIN3) //PA3 S3_OUT D1_ST6
#define GPIO_TIM2_CH3OUT        (GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTA|GPIO_PIN2) //PA2 S4_OUT D1_ST1
#define GPIO_TIM5_CH2OUT        (GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTA|GPIO_PIN1) //PA1 S5_OUT
#define GPIO_TIM1_CH1OUT        (GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTA|GPIO_PIN8) //PA8 S6_OUT
#define DIRECT_PWM_OUTPUT_CHANNELS      6

#define GPIO_TIM3_CH3IN         GPIO_TIM3_CH3IN_1
#define GPIO_TIM3_CH4IN         GPIO_TIM3_CH4IN_1
#define GPIO_TIM2_CH4IN         GPIO_TIM2_CH4IN_1
#define GPIO_TIM2_CH3IN         GPIO_TIM2_CH3IN_1
#define GPIO_TIM5_CH2IN         GPIO_TIM5_CH2IN_1
#define GPIO_TIM1_CH1IN         GPIO_TIM1_CH1IN_1
#define DIRECT_INPUT_TIMER_CHANNELS  6

// Has pwm outputs
#define BOARD_HAS_PWM    DIRECT_PWM_OUTPUT_CHANNELS

/* High-resolution timer */
#define HRT_TIMER		8	/* use timer8 for the HRT */
#define HRT_TIMER_CHANNEL	3	/* use capture/compare channel */

#define HRT_PPM_CHANNEL              1
#define GPIO_PPM_IN                  (GPIO_ALT|GPIO_AF2|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN8)

#define RC_SERIAL_PORT               "/dev/ttyS0"

/*
 * One RC_IN
 *
 * GPIO PPM_IN on PB8 T4CH3
 * SPEKTRUM_RX (it's TX or RX in Bind) on PA10 UART1
 * The FMU can drive GPIO PPM_IN as an output
 */
// TODO?
//#define GPIO_PPM_IN_AS_OUT            (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN6)
//#define SPEKTRUM_RX_AS_GPIO_OUTPUT()  px4_arch_configgpio(GPIO_PPM_IN_AS_OUT)
//#define SPEKTRUM_RX_AS_UART()         px4_arch_configgpio(GPIO_USART1_RX)
//#define SPEKTRUM_OUT(_one_true)       px4_arch_gpiowrite(GPIO_PPM_IN_AS_OUT, (_one_true))

#define BOARD_NAME "PX4_OMNIBUSF4SD"

#define BOARD_HAS_PWM    DIRECT_PWM_OUTPUT_CHANNELS

#define BOARD_FMU_GPIO_TAB { \
		{GPIO_GPIO0_INPUT,       GPIO_GPIO0_OUTPUT,       0}, \
		{GPIO_GPIO1_INPUT,       GPIO_GPIO1_OUTPUT,       0}, \
		{GPIO_GPIO2_INPUT,       GPIO_GPIO2_OUTPUT,       0}, \
		{GPIO_GPIO3_INPUT,       GPIO_GPIO3_OUTPUT,       0}, \
		{GPIO_GPIO4_INPUT,       GPIO_GPIO4_OUTPUT,       0}, \
		{GPIO_GPIO5_INPUT,       GPIO_GPIO5_OUTPUT,       0}, }

/*
 * PX4FMUv4 GPIO numbers.
 *
 * There are no alternate functions on this board.
 */
#define GPIO_SERVO_1                 (1<<0)  /**< servo 1 output */
#define GPIO_SERVO_2                 (1<<1)  /**< servo 2 output */
#define GPIO_SERVO_3                 (1<<2)  /**< servo 3 output */
#define GPIO_SERVO_4                 (1<<3)  /**< servo 4 output */
#define GPIO_SERVO_5                 (1<<4)  /**< servo 5 output */
#define GPIO_SERVO_6                 (1<<5)  /**< servo 6 output */

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE    5120

#define BOARD_HAS_ON_RESET 1

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
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PX4FMU board.
 *
 *   mask - is bus selection
 *   1 - 1 << 0
 *   2 - 1 << 1
 *
 ****************************************************************************************************/

extern void stm32_spiinitialize(void);
void board_spi_reset(int ms);


/****************************************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called to configure USB IO.
 *
 ****************************************************************************************************/

extern void stm32_usbinitialize(void);

extern void board_peripheral_reset(int ms);

#include "../common/board_common.h"

#endif /* __ASSEMBLY__ */

__END_DECLS
