#include <stdio.h>

__EXPORT int  test_navstik_main(int argc, char *argv[]);


int	test_navstik_main(int argc, char *argv[])
{
	printf ("\nArming the Navstik Board for Testing. Make sure all the connections are properly made.\n") ;
	usleep (5000000) ;

 	printf ("\nBeginning Accelerometer Testing...\n");
	usleep (1000000) ;	
	test_accelerometer_main() ;
	usleep (2000000) ;

 	printf ("\nBeginning Gyrometer Testing...\n");
	usleep (1000000) ;
	test_gyrometer_main() ;
	usleep (2000000) ;

 	printf ("\nBeginning Magnetometer Testing...\n");
	usleep (1000000) ;
	test_magnetometer_main() ;
	usleep (2000000) ;

 	printf ("\nBeginning Barometer Testing...\n");
	usleep (1000000) ;
	test_barometer_main() ;
	usleep (1000000) ;

 	printf ("\nBeginning GPS Testing...\n");
	usleep (1000000) ;
	gpsuart_main() ;
	printf ("If you could see the GPS data printed on the screen, GPS is working fine.\n") ;
	usleep (3000000) ;

 	printf ("\nBeginning LED Testing...\n");
	usleep (1000000) ;
	test_led_main() ;
	usleep (2000000) ;

	printf ("\nBeginning UART Testing...\n");
	usleep (1000000) ;
	test_gpio_main() ;
	usleep (2000000) ;

	printf ("\nBeginning USB Testing...\n");
	usleep (1000000) ;
	test_usb_main() ;
	usleep (2000000) ;

	printf ("\nBeginning Battery Testing...\n");
	usleep (1000000) ;
	read_battery_voltage_main() ;
	usleep (2000000) ;

	printf ("\nBeginning Power Enable Pins Testing...\n");
	usleep (1000000) ;
	test_pwren_main() ;
	usleep (2000000) ;

	printf ("\nBeginning PWM Input-Output Testing...\n") ;
	usleep (1000000) ;
	test_pwm_main() ;
	usleep (2000000) ;

	printf ("\nNavstik Testing Complete.\n") ;

}
