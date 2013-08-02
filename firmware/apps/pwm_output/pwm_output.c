    /// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *  Authors:    Doug Weibel, Jose Julio, Jordi Munoz, Jason Short, Andrew Tridgell, Randy Mackay, Pat Hickey, John Arne Birkeland, Olivier Adler, Amilcar Lucas, Gregory Fletcher
 *  Thanks to:  Chris Anderson, Michael Oborne, Paul Mather, Bill Premerlani, James Cohen, JB from rotorFX, Automatik, Fefenin, Peter Meister, Remzibi, Yury Smirnov, Sandro Benigno, Max Levine, Roberto Navoni, Lorenz Meier, Yury MonZon
 *  Please contribute your ideas!
 * 
 *
 *  This firmware is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 */

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


/* PWM
 *
 * The Navstik has six PWM outputs
 * The mapping are as follows :
 * PB5	Servo 1 Timer 3 Channel 2 (AF2)
 * PA10	Servo 2 Timer 1 channel 3 (AF1)
 * PC8	Servo 3 Timer 8 Channel 3 (AF3)
 * PB11	Servo 4 Timer 2 Channel 4 (AF1)
 * PB1	Servo 5 Timer 3 Channel 4 (AF2)
 * PB0	Servo 6 Timer 3 Channel 3 (AF2)
 *
 */


__EXPORT int  pwm_output_main(int argc, char *argv[]);

int	pwm_output_main(int argc, char *argv[])
{
   uint16_t value[6] = {1000,1200,1500,1700,1800,2000};
   int t=0,n=0;
   n = up_pwm_servo_init(0x3F) ; // initialising pwm   
   
   up_pwm_servo_arm(1);//arming servos
 
  n = up_pwm_servo_set_rate(100); // setting update rate
   
   if (n==-ERANGE)
   printf("Rate not set \n");
   
  for (unsigned i = 0; i < PWM_SERVO_MAX_CHANNELS; i++) 
      {
            
            n = up_pwm_servo_set(i,value[i]);
            
            if (n==-1)
            printf("Servo Not Set for RC%d \n",i+1);
            
            value[i] = up_pwm_servo_get(i);
      }
           
}
