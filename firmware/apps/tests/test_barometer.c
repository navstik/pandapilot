#include <nuttx/config.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
 
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>


__EXPORT int test_barometer(int argc, char *argv[]);
 
int test_barometer(int argc, char *argv[])
{	
	double press_acc=0 ;
	float x,press_low,press_high ;
	int flag=0,count_low=0,count_high=0,end_count=0;
//	printf("BAROMETER READINGS!\n");

	/* subscribe to sensor_combined topic */
	int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
 	orb_set_interval(sensor_sub_fd, 100);
	/* one could wait for multiple topics with this technique, just using one here */
	struct pollfd fds[] = {
		{ .fd = sensor_sub_fd,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};
 
	int error_counter = 0;
 
	while (true) {
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = poll(fds, 1, 1000);
 
		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			printf("[barometer] Got no data within a second\n");
		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				printf("[barometer] ERROR return value from poll(): %d\n"
					, poll_ret);
			}
			error_counter++;
		} else {
 
			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct sensor_combined_s raw;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
			//	printf("\nPresure:%.4f\tAltitude:%.4f\tTemperature:%.4f",(double)raw.baro_pres_mbar,(double)raw.baro_alt_meter,(double)raw.baro_temp_celcius);
				x=raw.baro_pres_mbar * 100.0 ;

			}
			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
		}
		
		if (count_low == 0)
			{
			printf ("\nKeep the board at the lowest possible height.\n\n") ;
			usleep (5000000) ;
			}

		if ((count_low == 20) && (flag == 0))
			{
			press_low = press_acc / 20.0 ;
			printf ("\nAverage Pressure : %.4f Pascals",press_low) ;
			count_low++ ;			
			press_acc = 0 ;
			flag = 1 ;
			continue ;	
			}
		else if (flag == 0)
			{
			press_acc += x ;
			printf ("\nPressure : %.4f Pascals",x) ;
			count_low++ ;
			continue ;
			}

		if (count_high == 0)
			{
			printf ("\n\n Now keep the board at the maximum possible height.\n\n") ;
			usleep (5000000) ;
			count_high++ ;
			continue ;
			}
	
		if ((count_high == 21) && (flag == 1))
			{
			press_high = press_acc / 20.0 ;
			printf ("\nAverage Pressure : %.4f Pascals",press_high) ;
			count_high++ ;			
			press_acc = 0 ;
			flag = 1 ;	
			}
		else if (flag == 1)
			{
			press_acc += x ;
			printf ("\nPressure : %.4f Pascals",x) ;
			count_high++ ;
			continue ;
			}

		if ((press_low - press_high) > 100)
			{
			printf("\n\nPressure difference may be too high :%.4f Pascals !!\n",(press_low - press_high));
			break;
			}


		if ((press_low - press_high) > 10)
			{
			printf("\n\nBarometer tested succesfully with pressure difference :%.4f Pascals !!\n",(press_low - press_high));
			break;
			}

		else 
			{
			printf("\n\nPressure Difference below 10 Pascals : %.4f Pascals !!\n",(press_low - press_high));
			return -1 ;
			}

		if (end_count > 1000)
		{
			printf("\nBarometer testing failed.\n");			
			return -1 ;
		}			
		

	end_count++ ;
	}
 
	return 0;
}

