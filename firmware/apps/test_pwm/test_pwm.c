////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////
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

#include <drivers/stm32/drv_pwm_servo.h>
#include <chip.h>
#include <up_internal.h>
#include <up_arch.h>

#include <stm32_internal.h>
#include <stm32_gpio.h>
#include <stm32_tim.h>
#include "test_input.h" 

__EXPORT int  test_pwm_main(int argc, char *argv[]);
extern volatile uint16_t rc_buffer[6] ;

int	test_pwm_main(int argc, char *argv[])
{

	uint16_t value[6] = {1000,1200,1500,1700,1800,2000};
	int flag_input[6] = {0,0,0,0,0,0};
	int flag_output[6] = {0,0,0,0,0,0};
	int p=0,q=0,i,j ;

	for (i=0;i<6;i++)
		{
		rc_buffer[i] = 0 ;
		}

	pwm_output_main() ;
	usleep(1000000) ;
	pwm_input_main() ;	

	usleep (2000000) ;

	for (i=0;i<6;i++)
        {
	if (rc_buffer[i] == 0)
		usleep(1000000) ;

        if (rc_buffer[i] > (value[i] - 5) && rc_buffer[i] < (value[i] + 5))
            {
            flag_input[i] = 1 ;
            flag_output[i] = 1 ;
  	    printf("\nPWM pair %d working correctly.\n",i+1) ;        
 //           correct_pair[p] = i ;
   //         p++ ;
            }
            
        else
            {
            printf("\nPWM pair %d may not be functioning correctly\n",i+1) ;        
     //       error_pair[q] = i ;
       //     q++ ;
            }
        }
}
