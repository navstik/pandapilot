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
 * @file input.c
 
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

#include <chip.h>
#include <up_internal.h>
#include <up_arch.h>

#include <stm32_internal.h>
#include <stm32_gpio.h>
#include <stm32_tim.h>
#include "input.h" 

/* PWM Input 
 * RC1 PA7	Timer 14 Channel 1 (AF9)
 * RC2 PA6	Timer 3 Channel 1 (AF2)
 * RC3 PB14 	Timer 12 Channel 1 (AF9)
 * RC4 PB15	Timer 12 Channel 2 (AF9)
 * RC5 PC7	Timer 8 Channel 2 (AF3)
 * RC6 PC6	Timer 8 Channel 1 (AF3)
 
 */
 

void configgpio(void);
static int set_timer(unsigned timer);
void attach_isr(void);
static int tim_isr14(void);
static int tim_isr12(void);
static int tim_isr3(void);
static int tim_isr8(void);
//static int input_pwm(uint32_t status,uint32_t count1);
void enable_irq(void);

uint32_t status;
uint16_t rc1;
uint16_t rc1_last;
uint16_t rc2;
uint16_t rc2_last;
uint16_t rc3;
uint16_t rc3_last;
uint16_t rc4;
uint16_t rc4_last;
uint16_t rc5;
uint16_t rc5_last;
uint16_t rc6;
uint16_t rc6_last;

#define RC_MAX_CHANNELS	6
__EXPORT volatile uint16_t rc_buffer[RC_MAX_CHANNELS];

#define MAX_PULSEWIDTH 2000

#define RC1  GPIO_TIM14_CH1IN
#define RC2  GPIO_TIM3_CH1IN		
#define RC3  GPIO_TIM12_CH1IN
#define RC4  GPIO_TIM12_CH2IN	
#define RC5  GPIO_TIM8_CH1IN		
#define RC6  GPIO_TIM8_CH2IN		

#define REG(_tmr, _reg)	(*(volatile uint32_t *)(input_timers[_tmr].base + _reg))

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

__EXPORT int  pwm_input_main(int argc, char *argv[]);

int pwm_input_main(int argc, char *argv[])
{
	configgpio();
	
	attach_isr();
	
	set_timer(0);//timer14
	set_timer(1);//timer12
	set_timer(2);//timer3
	set_timer(3);//timer8
	
	enable_irq();
	
}

static int set_timer(unsigned timer)
{
	/* enable the timer clock before we try to talk to it */
	modifyreg32(input_timers[timer].clock_register, 0, input_timers[timer].clock_bit);
	
	if(timer == 3) //timer 8
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
	//ArPSC(timer) = (input_timers[timer].clock_freq / 1000000) - 1;
	if (timer == 2)
		ArBDTR(timer) |= 0;
	/* run the full span of the counter */
	ArARR(timer) |= 0xffff;
	}

	else
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
	rPSC(timer) |= (input_timers[timer].clock_freq / 1000000) - 1;
	rARR(timer) |= 0xffff;
	} 
	
	switch (timer)
	{
	case 0: //timer 14 - channel 1
	
	rCCMR1(timer) |= ((GTIM_CCMR_CCS_CCIN1<<GTIM_CCMR1_CC1S_SHIFT)|(GTIM_CCMR_ICF_FCKINT8<<GTIM_CCMR1_IC1F_SHIFT));
	rCCMR2(timer) |= 0;
	rCCER(timer) |= (GTIM_CCER_CC1E|GTIM_CCER_CC1P|GTIM_CCER_CC1NP);
	rDIER(timer) |= GTIM_DIER_CC1IE;
	break;
	
	case 1: //timer 12 -channel -1 &2
	
	rCCMR1(timer) |= ((GTIM_CCMR_CCS_CCIN1<<GTIM_CCMR1_CC1S_SHIFT)|(GTIM_CCMR_ICF_FCKINT8<<GTIM_CCMR1_IC1F_SHIFT)|(GTIM_CCMR_CCS_CCIN1<<GTIM_CCMR1_CC2S_SHIFT)|(GTIM_CCMR_ICF_FCKINT8<<GTIM_CCMR1_IC1F_SHIFT));
	rCCMR2(timer) |= 0;
	rCCER(timer) |= (GTIM_CCER_CC1E|GTIM_CCER_CC2E|GTIM_CCER_CC1P|GTIM_CCER_CC1NP|GTIM_CCER_CC2P|GTIM_CCER_CC2NP);
	rDIER(timer) |= (GTIM_DIER_CC1IE|GTIM_DIER_CC2IE);
	break;
	
	case 2://timer 3 - channel 1
	
	rCCMR1(timer) |= ((GTIM_CCMR_CCS_CCIN1<<GTIM_CCMR1_CC1S_SHIFT)|(GTIM_CCMR_ICF_FCKINT8<<GTIM_CCMR1_IC1F_SHIFT));
	rCCMR2(timer) |= 0;
	rCCER(timer) |= (GTIM_CCER_CC1E|GTIM_CCER_CC1P|GTIM_CCER_CC1NP);
	rDIER(timer) |= GTIM_DIER_CC1IE;
	break;
	
	case 3://timer 8 - channel 1&2
	
	ArCCMR1(timer) |= ((ATIM_CCMR_CCS_CCIN1<<ATIM_CCMR1_CC1S_SHIFT)|(ATIM_CCMR_ICF_FCKINT8<<ATIM_CCMR1_IC1F_SHIFT)|(ATIM_CCMR_CCS_CCIN1<<ATIM_CCMR1_CC2S_SHIFT)|(ATIM_CCMR_ICF_FCKINT8<<ATIM_CCMR1_IC2F_SHIFT));
	ArCCMR2(timer) |= 0;
	ArCCER(timer) |= (ATIM_CCER_CC1E|ATIM_CCER_CC2E|ATIM_CCER_CC1P|ATIM_CCER_CC1NP|ATIM_CCER_CC2P|ATIM_CCER_CC2NP);
	ArDIER(timer) |= (ATIM_DIER_CC1IE|ATIM_DIER_CC2IE);
	break;
	
	}
	
	if(timer == 3)
	{
	/* generate an update event; reloads the counter, all registers */
	ArEGR(timer) |= ATIM_EGR_UG ;
	/* enable the timer */
	ArCR1(timer) |= GTIM_CR1_CEN;
	}
	else
	{
	/* generate an update event; reloads the counter, all registers */
	rEGR(timer) |= GTIM_EGR_UG ;
	/* enable the timer */
	rCR1(timer) |= GTIM_CR1_CEN;
	}
}
		


void configgpio(void)
{
 	stm32_configgpio(RC1);
 	stm32_configgpio(RC2);
 	stm32_configgpio(RC3);
 	stm32_configgpio(RC4);
 	stm32_configgpio(RC5);
 	stm32_configgpio(RC6);
 	
 return ;
}

void attach_isr(void)
{
	irq_attach(input_timers[0].vector, tim_isr14);
	irq_attach(input_timers[1].vector, tim_isr14);
	irq_attach(input_timers[2].vector, tim_isr14);
	irq_attach(input_timers[3].vector, tim_isr14);
	
	return;
} 

static int tim_isr14(void)
{
status = rSR(0);
 //ack the interrupts we just read 
rSR(0) = ~status;	
	if (status & (GTIM_SR_CC1IF | GTIM_SR_CC1OF)) 
	{
	uint16_t count1 = rCCR1(0);
  	//printf("Captured on RC1  %u\n", count1);
  	
  	/* if we missed an edge, we have to give up */
/*	if (status & (GTIM_SR_CC1OF))
	{ printf("\nmissed an edge on RC1");
		return;
	}	

	/* how long since the last edge? */
	rc1 = count1 - rc1_last;
	rc1_last = count1;
	
	if (rc1 <= MAX_PULSEWIDTH)
		rc_buffer[0]=rc1;
	//printf("RC1: %u\n", rc1);
	
  	}

status = rSR(1);
 //ack the interrupts we just read 
rSR(1) = ~status;
	
	if (status & (GTIM_SR_CC1IF | GTIM_SR_CC1OF)) 
	{
	uint16_t count1 = rCCR1(1);
  	//printf("Captured on RC3  %u\n", count1);
  	
  	/* if we missed an edge, we have to give up */
/*	if (status & (GTIM_SR_CC1OF))
	{ printf("\nmissed an edge on RC3");
		return;
	}	

	/* how long since the last edge? */
	rc3 = count1 - rc3_last;
	rc3_last = count1;
	
	if (rc3 <= MAX_PULSEWIDTH)
	rc_buffer[2]=rc3;
	
	//printf("RC3: %u\n", rc3);
	
  	}
  	if (status & (GTIM_SR_CC2IF | GTIM_SR_CC2OF)) 
	{
	uint16_t count1 = rCCR2(1);
  	//printf("Captured on RC4 %u\n", count1);
  	
  	/* if we missed an edge, we have to give up */
/*	if (status & (GTIM_SR_CC2OF))
	{ printf("\nmissed an edge on RC4");
		return;
	}	

	/* how long since the last edge? */
	rc4 = count1 - rc4_last;
	rc4_last = count1;
	
	if (rc4 <= MAX_PULSEWIDTH)
	rc_buffer[3]=rc4;
	
	//printf("RC4: %u\n", rc4);
	
  	}

status = rSR(2);
 //ack the interrupts we just read 
rSR(2) = ~status;
	
	if (status & (GTIM_SR_CC1IF | GTIM_SR_CC1OF)) 
	{
	uint16_t count1 = rCCR1(2);
  	//printf("Captured on RC2  %u\n", count1);
  	/* if we missed an edge, we have to give up */
/*	if (status & (GTIM_SR_CC1OF))
	{ printf("\nmissed an edge on RC2");
		return;
	}	

	/* how long since the last edge? */
	rc2 = count1 - rc2_last;
	rc2_last = count1;
	
	if (rc2 <= MAX_PULSEWIDTH)
	rc_buffer[1]=rc2;
	
	//printf("RC2: %u\n", rc2);
	
  	}

status = rSR(3);
 //ack the interrupts we just read 
rSR(3) = ~status;
	
	if (status & (ATIM_SR_CC1IF | ATIM_SR_CC1OF)) 
	{
	uint16_t count1 = rCCR1(3);
  	//printf("Captured on RC6  %u\n", count1);
  	/* if we missed an edge, we have to give up */
/*	if (status & (ATIM_SR_CC1OF))
	{ printf("\nmissed an edge on RC6");
		return;
	}	*/

	/* how long since the last edge? */
	rc6 = count1 - rc6_last;
	rc6_last = count1;
	
	if (rc6 <= MAX_PULSEWIDTH)
	rc_buffer[5]=rc6;
	
	//printf("RC6: %u\n", rc6);
	
  	}
  	
  	if (status & (ATIM_SR_CC2IF | ATIM_SR_CC2OF)) 
	{
	uint16_t count1 = rCCR2(3);
  	//printf("Captured on RC5  %u\n", count1);
  	
  	/* if we missed an edge, we have to give up */
/*	if (status & (ATIM_SR_CC2OF))
	{ printf("\nmissed an edge on RC5");
		return;
	}	

	/* how long since the last edge? */
	rc5 = count1 - rc5_last;
	rc5_last = count1;
	
	if (rc5 <= MAX_PULSEWIDTH)
	rc_buffer[4]=rc5;
	
	//printf("RC5: %u\n", rc5);
	
  	}
  	
  	return;
  	
}

void enable_irq(void)
{
	up_enable_irq(input_timers[0].vector);
	up_enable_irq(input_timers[1].vector);
	up_enable_irq(input_timers[2].vector);
	up_enable_irq(input_timers[3].vector);
	
	return;		
}
