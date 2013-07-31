#include <stdint.h>

#include <arch/board/board.h>

#include <stm32_internal.h>
#include <stm32_gpio.h>
#include <stm32_tim.h>

/* PWM Input 
 * RC1 PA7	Timer 14 Channel 1 (AF9)
 * RC2 PA6	Timer 3 Channel 1 (AF2)
 * RC3 PB14 	Timer 12 Channel 1 (AF9)
 * RC4 PB15	Timer 12 Channel 2 (AF9)
 * RC5 PC7	Timer 8 Channel 2 (AF3)
 * RC6 PC6	Timer 8 Channel 1 (AF3)
 
 */
 
#define PWM_INPUT_MAX_TIMERS 4
struct pwm_input_timer {
	uint32_t	base;
	uint32_t	clock_register;
	uint32_t	clock_bit;
	uint32_t	vector;
	uint32_t	clock_freq;
	
};

__EXPORT const struct pwm_input_timer input_timers[PWM_INPUT_MAX_TIMERS] = {
	{	/* Timer 14 */
		.base		= STM32_TIM14_BASE,
 		.clock_register	= STM32_RCC_APB1ENR,
 		.clock_bit	= RCC_APB1ENR_TIM14EN,
 		.vector		= STM32_IRQ_TIM14,
 		.clock_freq	= STM32_APB1_TIM14_CLKIN
		
	},
	{	/* Timer 12 */
		.base		= STM32_TIM12_BASE,
 		.clock_register	= STM32_RCC_APB1ENR,
 		.clock_bit	= RCC_APB1ENR_TIM12EN,
 		.vector		= STM32_IRQ_TIM12,
 		.clock_freq	= STM32_APB1_TIM12_CLKIN
	},
	{	/* Timer 3 */
		.base 		= STM32_TIM3_BASE,
		.clock_register = STM32_RCC_APB1ENR,
		.clock_bit 	= RCC_APB1ENR_TIM3EN,
		.vector		= STM32_IRQ_TIM3,
		.clock_freq 	= STM32_APB1_TIM3_CLKIN
	},
	{	/* Timer 8 */
		.base 		= STM32_TIM8_BASE,
		.clock_register = STM32_RCC_APB2ENR,
		.clock_bit 	= RCC_APB2ENR_TIM8EN,
		.vector		= STM32_IRQ_TIM8CC,
		.clock_freq 	= STM32_APB2_TIM8_CLKIN
	}
};
