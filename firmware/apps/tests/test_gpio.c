/*
 * @file input1.c
 
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

#define  TELE_TX_OP 	(GPIO_OUTPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN12)
#define  TELE_RX_OP 	(GPIO_OUTPUT|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN2)

#define  TELE_TX_IP 	(GPIO_INPUT|GPIO_FLOAT|GPIO_PORTC|GPIO_PIN12)
#define  TELE_RX_IP 	(GPIO_INPUT|GPIO_FLOAT|GPIO_PORTD|GPIO_PIN2)

#define  SPEK_TX_OP 	(GPIO_OUTPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN6)
#define  SPEK_RX_OP 	(GPIO_OUTPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN7)

#define  SPEK_TX_IP 	(GPIO_INPUT|GPIO_FLOAT|GPIO_PORTC|GPIO_PIN6)
#define  SPEK_RX_IP 	(GPIO_INPUT|GPIO_FLOAT|GPIO_PORTC|GPIO_PIN7)


__EXPORT int  test_gpio(int argc, char *argv[]);

int test_gpio(int argc, char *argv[])
{
	int flag = 0 ;
	
	stm32_configgpio(SPEK_TX_OP);
	stm32_configgpio(SPEK_RX_OP);

	stm32_gpiowrite(SPEK_TX_OP,1);	
	stm32_gpiowrite(SPEK_RX_OP,1);	

	stm32_configgpio(TELE_TX_OP);
	stm32_configgpio(TELE_RX_OP);
	
	if (stm32_gpioread(TELE_TX_OP) == 0)
		{
		printf ("\nEither TELE_TX or SPEK_RX is not working") ;
		flag++ ;
		}
	else
		{
		stm32_gpiowrite(SPEK_RX_OP,0);
		if (stm32_gpioread(TELE_TX_OP) == 1)
			{			
			printf ("\nEither TELE_TX or SPEK_RX is not working") ;
			flag++ ;
			}
		else
			printf ("\nTELE_TX and SPEK_RX Tested OK !!") ;
		}


	if (stm32_gpioread(TELE_RX_OP) == 0)
		{
		printf ("\nEither TELE_RX or SPEK_TX is not working") ;
		flag++ ;
		}
	else
		{
		stm32_gpiowrite(SPEK_TX_OP,0);
		if (stm32_gpioread(TELE_RX_OP) == 1)
			{
			printf ("\nEither TELE_RX or SPEK_TX is not working") ;
			flag++ ;
			}
		else
			printf ("\nTELE_RX and SPEK_TX Tested OK !!") ;
		}

	stm32_configgpio(TELE_TX_IP);
	stm32_configgpio(TELE_RX_IP);

	stm32_configgpio(SPEK_TX_OP);
	stm32_configgpio(SPEK_RX_OP);


printf ("\n\nUART TELE and UART SPEK Testing Complete.\n\n");

	if (flag != 0)
		return -1 ;	

return 0 ;
}
