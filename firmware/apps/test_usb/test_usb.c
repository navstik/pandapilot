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

#define  USB_HS_ID	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN12)
#define  USB_HS_VBUS 	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN13)

__EXPORT int  test_usb_main(int argc, char *argv[]);

int test_usb_main(int argc, char *argv[])
{
	int flag = 0 ;
	
	stm32_configgpio(USB_HS_ID);
	stm32_configgpio(USB_HS_VBUS);

	stm32_gpiowrite(USB_HS_ID,1);	
	
	if (stm32_gpioread(USB_HS_VBUS) == 0)
		printf ("\nEither USB_HS_ID(Pin 1) or USB_HS_VBUS(Pin 4) is not working") ;
	else
		{
		stm32_gpiowrite(USB_HS_ID,0);
		if (stm32_gpioread(USB_HS_VBUS) == 1)
			printf ("\nEither USB_HS_ID(Pin 1) or USB_HS_VBUS(Pin 4) is not working") ;
		else
			flag++ ;
		}

	if (flag >= 1)
		printf ("\nUSB Pins working fine.\n") ;


printf ("\n\nUSB Testing Complete.\n\n");

usleep(1000000) ;
}

