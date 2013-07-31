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

//#define  GREEN LED 	(GPIO_OUTPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN4)
//#define  AMBER LED 	(GPIO_OUTPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN5)

__EXPORT int  test_led_main(int argc, char *argv[]);

int test_led_main(int argc, char *argv[])
{
	int i ;
	
	up_ledoff(0) ;
	up_ledoff(1) ;

	printf ("\nBOTH LEDs OFF !!\n");
	usleep (2000000) ;

	printf ("\nGREEN LED ON !!\n");
	up_ledon(0) ;
	usleep (2000000) ;

	printf ("\nGREEN LED OFF !!\n");
	up_ledoff(0) ;
	usleep (2000000) ;
			
	printf ("\nGREEN LED BLINKING !!\n");
	for (i=0;i<10;i++) 
		{
		if (i%2 == 0)
			{
			up_ledon(0) ;
			usleep (500000) ;
			}
		else
			{
			up_ledoff(0) ;
			usleep (500000) ;
			}
		} 
	up_ledon(0) ;
	
	printf ("\nAMBER LED ON !!\n");
	up_ledon(1) ;
	usleep (2000000) ;

	printf ("\nAMBER LED OFF !!\n");
	up_ledoff(1) ;
	usleep (2000000) ;
			
 	printf ("\nAMBER LED BLINKING !!\n");
	for (i=0;i<10;i++) 
		{
		if (i%2 == 0)
			{
			up_ledon(1) ;
			usleep (500000) ;
			}
		else
			{
			up_ledoff(1) ;
			usleep (500000) ;
			}
		}

printf ("\nLED Testing Complete.\n\n");

}

