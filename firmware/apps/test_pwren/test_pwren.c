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

#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <poll.h>

#include <arch/board/board.h>

#include <chip.h>
#include <up_internal.h>
#include <up_arch.h>

#include <stm32_internal.h>
#include <stm32_gpio.h>
#include <stm32_tim.h>

#define GPS_PWR_EN		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN4)
#define SENSOR_PWR_EN		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN15)


__EXPORT int  test_pwren_main(int argc, char *argv[]);

int test_pwren_main(int argc, char *argv[])
{
	printf ("\nSensor power disabled. You should now see sensor data capture errors.\n\n");
	usleep(3000000) ;	
	stm32_gpiowrite(SENSOR_PWR_EN,0);
	usleep(100000) ;
	stm32_gpiowrite(SENSOR_PWR_EN,1);
	usleep(300000) ;
	printf ("\nSensor power enabled. Data capture errors should now stop.\n\n");
	usleep(4000000) ;

  int fd; /* File descriptor for the port */
  char c;
 
  /*GPS is connected to the controller via UART2 and the port for uart 2 is /dev/ttyS1 */
  fd = open("/dev/ttyS1", O_RDWR | O_NOCTTY | O_NDELAY);/*Opening the port */
 
  /* Try to set baud rate */
  struct termios uart_config;
 
  tcgetattr(fd, &uart_config); // get what is the current baud rate
 
   /* Setting baud rate of UART to 9600 for communicating with GPS at its default baudrate of 9600 */
   cfsetispeed(&uart_config, B9600);
   cfsetospeed(&uart_config, B9600);
 
  tcsetattr(fd, TCSANOW, &uart_config); //confirming that the baud rate is set
 
   int i=0,n=0;
 
 struct pollfd fds;
	fds.fd = fd;
	fds.events = POLLIN;
 
printf ("\nPrinting GPS data...\n") ;
usleep(2000000) ;

while (i<1000)
{   
	if (i == 500)
		{
		printf ("\nGPS Power Disabled. You should see no data for next 5 seconds.\n") ;
		stm32_gpiowrite(GPS_PWR_EN,0);
		usleep(5000000);
		stm32_gpiowrite(GPS_PWR_EN,1);	
		printf ("\nGPS Power Enabled. You should see the GPS data now.\n") ;
		usleep(2000000) ;			
		}
if (poll(&fds, 1, 1000)>0)
    {
     n=read(fd,&c,1); // reading and printing the characters coming from GPS to ensure the connection with gps
     if (n<0)
     printf("read failed");
 
     printf("%c ",c); // printing the characters on console
   }
i++;
}


close(fd);

printf ("\n\nGPS POWER ENABLE and SENSOR POWER ENABLE Testing Complete.\n\n");

}
	
