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
 * @file drv_pwm_servo.c
 *
 * Servo driver supporting PWM servos connected to STM32 timer blocks.
 *
 * Works with any of the 'generic' or 'advanced' STM32 timers that
 * have output pins, does not require an interrupt.
 */

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <sys/types.h>
#include <stdbool.h>

#include <assert.h>
#include <debug.h>
#include <time.h>
#include <queue.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>

#include <arch/board/board.h>
#include <drivers/drv_pwm_output.h>

#include "drv_pwm_servo.h"

#include <chip.h>
#include <up_internal.h>
#include <up_arch.h>

#include <stm32_internal.h>
#include <stm32_gpio.h>
#include <stm32_tim.h>

#define REG(_tmr, _reg)	(*(volatile uint32_t *)(pwm_timers[_tmr].base + _reg))

#define rCR1(_tmr)    	REG(_tmr, STM32_GTIM_CR1_OFFSET)
#define rCR2(_tmr)    	REG(_tmr, STM32_GTIM_CR2_OFFSET)
#define rSMCR(_tmr)   	REG(_tmr, STM32_GTIM_SMCR_OFFSET)
#define rDIER(_tmr)   	REG(_tmr, STM32_GTIM_DIER_OFFSET)
#define rSR(_tmr)     	REG(_tmr, STM32_GTIM_SR_OFFSET)
#define rEGR(_tmr)    	REG(_tmr, STM32_GTIM_EGR_OFFSET)
#define rCCMR1(_tmr)  	REG(_tmr, STM32_GTIM_CCMR1_OFFSET)
#define rCCMR2(_tmr)  	REG(_tmr, STM32_GTIM_CCMR2_OFFSET)
#define rCCER(_tmr)   	REG(_tmr, STM32_GTIM_CCER_OFFSET)
#define rCNT(_tmr)    	REG(_tmr, STM32_GTIM_CNT_OFFSET)
#define rPSC(_tmr)    	REG(_tmr, STM32_GTIM_PSC_OFFSET)
#define rARR(_tmr)    	REG(_tmr, STM32_GTIM_ARR_OFFSET)
#define rCCR1(_tmr)   	REG(_tmr, STM32_GTIM_CCR1_OFFSET)
#define rCCR2(_tmr)   	REG(_tmr, STM32_GTIM_CCR2_OFFSET)
#define rCCR3(_tmr)   	REG(_tmr, STM32_GTIM_CCR3_OFFSET)
#define rCCR4(_tmr)   	REG(_tmr, STM32_GTIM_CCR4_OFFSET)
#define rDCR(_tmr)    	REG(_tmr, STM32_GTIM_DCR_OFFSET)
#define rDMAR(_tmr)   	REG(_tmr, STM32_GTIM_DMAR_OFFSET)

#define ArCR1(_tmr)    	REG(_tmr, STM32_ATIM_CR1_OFFSET)
#define ArCR2(_tmr)    	REG(_tmr, STM32_ATIM_CR2_OFFSET)
#define ArSMCR(_tmr)   	REG(_tmr, STM32_ATIM_SMCR_OFFSET)
#define ArDIER(_tmr)   	REG(_tmr, STM32_ATIM_DIER_OFFSET)
#define ArSR(_tmr)     	REG(_tmr, STM32_ATIM_SR_OFFSET)
#define ArEGR(_tmr)    	REG(_tmr, STM32_ATIM_EGR_OFFSET)
#define ArCCMR1(_tmr)  	REG(_tmr, STM32_ATIM_CCMR1_OFFSET)
#define ArCCMR2(_tmr)  	REG(_tmr, STM32_ATIM_CCMR2_OFFSET)
#define ArCCER(_tmr)   	REG(_tmr, STM32_ATIM_CCER_OFFSET)
#define ArCNT(_tmr)    	REG(_tmr, STM32_ATIM_CNT_OFFSET)
#define ArPSC(_tmr)    	REG(_tmr, STM32_ATIM_PSC_OFFSET)
#define ArARR(_tmr)    	REG(_tmr, STM32_ATIM_ARR_OFFSET)
#define ArCCR1(_tmr)   	REG(_tmr, STM32_ATIM_CCR1_OFFSET)
#define ArCCR2(_tmr)   	REG(_tmr, STM32_ATIM_CCR2_OFFSET)
#define ArCCR3(_tmr)   	REG(_tmr, STM32_ATIM_CCR3_OFFSET)
#define ArCCR4(_tmr)   	REG(_tmr, STM32_ATIM_CCR4_OFFSET)
#define ArBDTR(_tmr)	REG(_tmr, STM32_ATIM_BDTR_OFFSET)
#define ArDCR(_tmr)    	REG(_tmr, STM32_ATIM_DCR_OFFSET)
#define ArDMAR(_tmr)   	REG(_tmr, STM32_ATIM_DMAR_OFFSET)

static void		pwm_timer_init(unsigned timer);
static void		pwm_timer_set_rate(unsigned timer, unsigned rate);
static void		pwm_channel_init(unsigned channel);

static void
pwm_timer_init(unsigned timer)
{	
	
	/* enable the timer clock before we try to talk to it */
	modifyreg32(pwm_timers[timer].clock_register, 0, pwm_timers[timer].clock_bit);
	
	
	if((timer == 1)||(timer==2) )
	{	/* disable and configure the timer */
	rCR1(timer) |= 0;
	rCR2(timer) |= 0;
	rSMCR(timer) |= 0;
	rDIER(timer) |= 0;
	rCCER(timer) |= 0;
	rCCMR1(timer) |= 0;
	rCCMR2(timer) |= 0;
	rCCER(timer) |= 0;
	rDCR(timer) |= 0;
	/* configure the timer to free-run at 1MHz */
	rPSC(timer) = (pwm_timers[timer].clock_freq / 1000000) - 1;
	}
	else 
	{
	ArCR1(timer) |= 0;
	ArCR2(timer) |= 0;
	ArSMCR(timer) |= 0;
	ArDIER(timer) |= 0;
	ArCCER(timer) |= 0;
	ArCCMR1(timer) |= 0;
	ArCCMR2(timer) |= 0;
	ArCCER(timer) |= 0;
	ArDCR(timer) |= 0;
	/* configure the timer to free-run at 1MHz */
	ArPSC(timer) |= (pwm_timers[timer].clock_freq / 1000000) - 1;
	}
	/* default to updating at 50Hz */
	pwm_timer_set_rate(timer, 50);

	/* note that the timer is left disabled - arming is performed separately */
}

static void
pwm_timer_set_rate(unsigned timer, unsigned rate)
{	if ((timer == 1)||(timer==2))
	{/* configure the timer to update at the desired rate */
	rARR(timer) = 1000000 / rate;
//	rARR(timer) = 0xffff;
	/* generate an update event; reloads the counter and all registers */
	rEGR(timer) |= GTIM_EGR_UG;
	}
	else
	{
	ArARR(timer) = 1000000 / rate;
//	rARR(timer) = 0xffff;
	ArEGR(timer) |= ATIM_EGR_UG;
	}
}

static void
pwm_channel_init(unsigned channel)
{
	unsigned timer = pwm_channels[channel].timer_index;

	/* configure the GPIO first */
	stm32_configgpio(pwm_channels[channel].gpio);
	
	if ((timer == 1)||(timer==2))
	{
	/* configure the channel */
	switch (pwm_channels[channel].timer_channel) {
	case 1:
		rCCMR1(timer) |= (GTIM_CCMR_MODE_PWM1 << GTIM_CCMR1_OC1M_SHIFT) | GTIM_CCMR1_OC1PE;
		rCCR1(timer) = pwm_channels[channel].default_value;
		rCCER(timer) |= GTIM_CCER_CC1E;
		break;

	case 2:
		rCCMR1(timer) |= (GTIM_CCMR_MODE_PWM1 << GTIM_CCMR1_OC2M_SHIFT) | GTIM_CCMR1_OC2PE;
		rCCR2(timer) = pwm_channels[channel].default_value;
		rCCER(timer) |= GTIM_CCER_CC2E;
		break;

	case 3:
		rCCMR2(timer) |= (GTIM_CCMR_MODE_PWM1 << GTIM_CCMR2_OC3M_SHIFT) | GTIM_CCMR2_OC3PE;
		rCCR3(timer) = pwm_channels[channel].default_value;
		rCCER(timer) |= GTIM_CCER_CC3E;
		break;

	case 4:
		rCCMR2(timer) |= (GTIM_CCMR_MODE_PWM1 << GTIM_CCMR2_OC4M_SHIFT) | GTIM_CCMR2_OC4PE;
		rCCR4(timer) = pwm_channels[channel].default_value;
		rCCER(timer) |= GTIM_CCER_CC4E;
		break;
	}
	}
	else
	{
	/* configure the channel */
	switch (pwm_channels[channel].timer_channel) {
	case 1:
		ArCCMR1(timer) |= (ATIM_CCMR_MODE_PWM1 << ATIM_CCMR1_OC1M_SHIFT) | ATIM_CCMR1_OC1PE;
		ArCCR1(timer) = pwm_channels[channel].default_value;
		ArCCER(timer) |= ATIM_CCER_CC1E;
		break;

	case 2:
		ArCCMR1(timer) |= (ATIM_CCMR_MODE_PWM1 << ATIM_CCMR1_OC2M_SHIFT) | ATIM_CCMR1_OC2PE;
		ArCCR2(timer) = pwm_channels[channel].default_value;
		ArCCER(timer) |= ATIM_CCER_CC2E;
		break;

	case 3:
		ArCCMR2(timer) |= (ATIM_CCMR_MODE_PWM1 << ATIM_CCMR2_OC3M_SHIFT) | ATIM_CCMR2_OC3PE;
		ArCCR3(timer) = pwm_channels[channel].default_value;
		ArCCER(timer) |= ATIM_CCER_CC3E;
		break;

	case 4:
		ArCCMR2(timer) |= (ATIM_CCMR_MODE_PWM1 << ATIM_CCMR2_OC4M_SHIFT) | ATIM_CCMR2_OC4PE;
		ArCCR4(timer) = pwm_channels[channel].default_value;
		ArCCER(timer) |= ATIM_CCER_CC4E;
		break;
	}
	
	}
}

int
up_pwm_servo_set(unsigned channel, servo_position_t value)
{
	if (channel >= PWM_SERVO_MAX_CHANNELS)
		return -1;

	unsigned timer = pwm_channels[channel].timer_index;

	/* test timer for validity */
	if ((pwm_timers[timer].base == 0) ||
	    (pwm_channels[channel].gpio == 0))
		return -1;

	/* configure the channel */
	if (value > 0)
		value--;
	
	if ((timer == 1)||(timer==2))
	{
	switch (pwm_channels[channel].timer_channel) {
	case 1:
		rCCR1(timer) = value;
		break;

	case 2:
		rCCR2(timer) = value;
		break;

	case 3:
		rCCR3(timer) = value;
		break;

	case 4:
		rCCR4(timer) = value;
		break;

	default:
		return -1;
	}
	}
	else
	{
	switch (pwm_channels[channel].timer_channel) {
	case 1:
		ArCCR1(timer) = value;
		break;

	case 2:
		ArCCR2(timer) = value;
		break;

	case 3:
		ArCCR3(timer) = value;
		break;

	case 4:
		ArCCR4(timer) = value;
		break;

	default:
		return -1;
	}
	}


	return 0;
}

servo_position_t
up_pwm_servo_get(unsigned channel)
{
	if (channel >= PWM_SERVO_MAX_CHANNELS)
		return 0;

	unsigned timer = pwm_channels[channel].timer_index;
	servo_position_t value = 0;

	/* test timer for validity */
	if ((pwm_timers[timer].base == 0) ||
	    (pwm_channels[channel].timer_channel == 0))
		return 0;
	if ((timer == 1)||(timer==2))
	{
	/* configure the channel */
	switch (pwm_channels[channel].timer_channel) {
	case 1:
		value = rCCR1(timer);
		break;

	case 2:
		value = rCCR2(timer);
		break;

	case 3:
		value = rCCR3(timer);
		break;

	case 4:
		value = rCCR4(timer);
		break;
	}
	}
	else
	{
	/* configure the channel */
	switch (pwm_channels[channel].timer_channel) {
	case 1:
		value = ArCCR1(timer);
		break;

	case 2:
		value = ArCCR2(timer);
		break;

	case 3:
		value = ArCCR3(timer);
		break;

	case 4:
		value = ArCCR4(timer);
		break;
	}
	}

	return value + 1;
}

int
up_pwm_servo_init(uint32_t channel_mask)
{
	/* do basic timer initialisation first */
	for (unsigned i = 0; i < PWM_SERVO_MAX_TIMERS; i++) {
		if (pwm_timers[i].base != 0)
			pwm_timer_init(i);
	}

	/* now init channels */
	for (unsigned i = 0; i < PWM_SERVO_MAX_CHANNELS; i++) {
		/* don't do init for disabled channels; this leaves the pin configs alone */
		if (((1 << i) & channel_mask) && (pwm_channels[i].timer_channel != 0))
			pwm_channel_init(i);
	}

	return OK;
}

void
up_pwm_servo_deinit(void)
{
	/* disable the timers */
	up_pwm_servo_arm(false);
}

int
up_pwm_servo_set_rate_group_update(unsigned group, unsigned rate)
{
	/* limit update rate to 1..10000Hz; somewhat arbitrary but safe */
	if (rate < 1)
		return -ERANGE;
	if (rate > 10000)
		return -ERANGE;

	if ((group >= PWM_SERVO_MAX_TIMERS) || (pwm_timers[group].base == 0))
		return ERROR;

	pwm_timer_set_rate(group, rate);

	return OK;
}

int
up_pwm_servo_set_rate(unsigned rate)
{
	for (unsigned i = 0; i < PWM_SERVO_MAX_TIMERS; i++)
		up_pwm_servo_set_rate_group_update(i, rate);
}

uint32_t
up_pwm_servo_get_rate_group(unsigned group)
{
	unsigned channels = 0;

	for (unsigned i = 0; i < PWM_SERVO_MAX_CHANNELS; i++) {
		if ((pwm_channels[i].gpio != 0) && (pwm_channels[i].timer_index == group))
			channels |= (1 << i);
	}
	return channels;
}

void
up_pwm_servo_arm(bool armed)
{
	/* iterate timers and arm/disarm appropriately */
	for (unsigned i = 0; i < PWM_SERVO_MAX_TIMERS; i++) {
		if (pwm_timers[i].base != 0) {
			if (armed) {
				if ((i == 1)||(i==2))
				{
				/* force an update to preload all registers */
				rEGR(i) |= GTIM_EGR_UG;				//c
				/* arm requires the timer be enabled */
				rCR1(i) |= GTIM_CR1_CEN | GTIM_CR1_ARPE;
				}
				else
				{
				/* force an update to preload all registers */
				ArEGR(i) |= ATIM_EGR_UG;			//c
				/* arm requires the timer be enabled */
				
				ArCR1(i) |= ATIM_CR1_CEN | ATIM_CR1_ARPE;
				ArBDTR(i) &= ~(ATIM_BDTR_OSSI | ATIM_BDTR_OSSR);// specific to tim1 and tim8
      				ArBDTR(i) |= ATIM_BDTR_MOE;
				
				}

			} else {
				// XXX This leads to FMU PWM being still active
				// but uncontrollable. Just disable the timer
				// and risk a runt.
				///* on disarm, just stop auto-reload so we don't generate runts */
				if ((i == 1)||(i==2))
				{
				rCR1(i) &= ~(GTIM_CR1_ARPE|GTIM_CR1_CEN);
				}
			//	rCR1(i) = 0;
				else
				{
				ArCR1(i) &= ~(ATIM_CR1_ARPE|ATIM_CR1_CEN);
				}
			}
		}
	}
}
