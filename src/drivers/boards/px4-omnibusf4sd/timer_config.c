/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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

/*
 * @file px4fmu_timer_config.c
 *
 * Configuration data for the stm32 pwm_servo, input capture and pwm input driver.
 *
 * Note that these arrays must always be fully-sized.
 */

#include <stdint.h>

#include <stm32.h>
#include <stm32_gpio.h>
#include <stm32_tim.h>

#include <drivers/drv_pwm_output.h>
#include <drivers/stm32/drv_io_timer.h>

#include "board_config.h"

__EXPORT const io_timers_t io_timers[MAX_IO_TIMERS] = {
	{
		.base = STM32_TIM1_BASE,
		.clock_register = STM32_RCC_APB2ENR,
		.clock_bit = RCC_APB2ENR_TIM1EN,
		.clock_freq = STM32_APB2_TIM1_CLKIN,
		.first_channel_index = 0,
		.last_channel_index = 0,
		.handler = io_timer_handler0,
		.vectorno =  STM32_IRQ_TIM1CC
	},
	{
		.base = STM32_TIM2_BASE,
		.clock_register = STM32_RCC_APB1ENR,
		.clock_bit = RCC_APB1ENR_TIM2EN,
		.clock_freq = STM32_APB1_TIM2_CLKIN,
		.first_channel_index = 1,
		.last_channel_index = 2,
		.handler = io_timer_handler1,
		.vectorno =  STM32_IRQ_TIM2
	},
	{
		.base = STM32_TIM3_BASE,
		.clock_register = STM32_RCC_APB1ENR,
		.clock_bit = RCC_APB1ENR_TIM3EN,
		.clock_freq = STM32_APB1_TIM3_CLKIN,
		.first_channel_index = 3,
		.last_channel_index = 4,
		.handler = io_timer_handler2,
		.vectorno =  STM32_IRQ_TIM3
	},
	{
		.base = STM32_TIM5_BASE,
		.clock_register = STM32_RCC_APB1ENR,
		.clock_bit = RCC_APB1ENR_TIM5EN,
		.clock_freq = STM32_APB1_TIM5_CLKIN,
		.first_channel_index = 5,
		.last_channel_index = 5,
		.handler = io_timer_handler3,
		.vectorno =  STM32_IRQ_TIM5
	}
};

//#define GPIO_TIM1_CH1OUT        (GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTA|GPIO_PIN8) //PA8 S6_OUT
//#define GPIO_TIM2_CH3OUT        (GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTA|GPIO_PIN2) //PA2 S4_OUT D1_ST1
//#define GPIO_TIM2_CH4OUT        (GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTA|GPIO_PIN3) //PA3 S3_OUT D1_ST6
//#define GPIO_TIM3_CH3OUT        (GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTB|GPIO_PIN0) //PB0 S1_OUT D1_ST7
//#define GPIO_TIM3_CH4OUT        (GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTB|GPIO_PIN1) //PB1 S2_OUT D1_ST2
//#define GPIO_TIM5_CH2OUT        (GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTA|GPIO_PIN1) //PA1 S5_OUT

__EXPORT const timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
  //PA8 S6_OUT
	{
		.gpio_out = GPIO_TIM1_CH1OUT,
		.gpio_in  = GPIO_TIM1_CH1IN,
		.timer_index = 0,
		.timer_channel = 1,
		.ccr_offset = STM32_GTIM_CCR1_OFFSET,
		.masks  = GTIM_SR_CC1IF | GTIM_SR_CC1OF
	},
  //PA2 S4_OUT D1_ST1
	{
		.gpio_out = GPIO_TIM2_CH3OUT,
		.gpio_in = GPIO_TIM2_CH3IN,
		.timer_index = 1,
		.timer_channel = 3,
		.ccr_offset = STM32_GTIM_CCR3_OFFSET,
		.masks  = GTIM_SR_CC3IF | GTIM_SR_CC3OF
	},
  //PA3 S3_OUT D1_ST6
	{
		.gpio_out = GPIO_TIM2_CH4OUT,
		.gpio_in = GPIO_TIM2_CH4IN,
		.timer_index = 1,
		.timer_channel = 4,
		.ccr_offset = STM32_GTIM_CCR4_OFFSET,
		.masks  = GTIM_SR_CC4IF | GTIM_SR_CC4OF
	},
  //PB0 S1_OUT D1_ST7
	{
		.gpio_out = GPIO_TIM3_CH3OUT,
		.gpio_in = GPIO_TIM3_CH3IN,
		.timer_index = 2,
		.timer_channel = 3,
		.ccr_offset = STM32_GTIM_CCR3_OFFSET,
		.masks  = GTIM_SR_CC3IF | GTIM_SR_CC3OF
	},
  //PB1 S2_OUT D1_ST2
	{
		.gpio_out = GPIO_TIM3_CH4OUT,
		.gpio_in = GPIO_TIM3_CH4IN,
		.timer_index = 2,
		.timer_channel = 4,
		.ccr_offset = STM32_GTIM_CCR4_OFFSET,
		.masks  = GTIM_SR_CC4IF | GTIM_SR_CC4OF
	},
  //PA1 S5_OUT
	{
		.gpio_out = GPIO_TIM5_CH2OUT,
		.gpio_in = GPIO_TIM5_CH2IN,
		.timer_index = 3,
		.timer_channel = 2,
		.ccr_offset = STM32_GTIM_CCR2_OFFSET,
		.masks  = GTIM_SR_CC2IF | GTIM_SR_CC2OF
	}
};
