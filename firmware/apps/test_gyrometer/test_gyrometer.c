#include <nuttx/config.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
 
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>


__EXPORT int test_gyrometer_main(int argc, char *argv[]);
 
int test_gyrometer_main(int argc, char *argv[])
{	
	float x=0,y=0,z=0;
//	int flag_x=0,flag_y=0,flag_z=0,FLAG=0;
	float angle_rad_x = 0, angle_deg_x = 0 ;
	float angle_rad_y = 0, angle_deg_y = 0 ;
	float angle_rad_z = 0, angle_deg_z = 0 ;
	int FLAG = 0, count = 0 ;
	int flag_x_n=0,flag_y_n=0,flag_z_n=0,flag_x_p=0,flag_y_p=0,flag_z_p=0,end_count=0;

	printf("\nRotate the board along all the 3 axes. The gyro will acquire a lock on all the axes when you move it to +90 degrees and -90 degrees along the axes.\n");
	usleep(3000000) ;
 
	/* subscribe to sensor_combined topic */
	int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
 	orb_set_interval(sensor_sub_fd, 10);
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
			printf("[gyrometer] Got no data within a second\n");
		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				printf("[gyrometer] ERROR return value from poll(): %d\n"
					, poll_ret);
			}
			error_counter++;
		} else {
 
			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct sensor_combined_s raw;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
/*				printf("\n[Gyro] Gyrometer:\t%8.4f\t%8.4f\t%8.4f\n",
					(double)raw.gyro_rad_s[0],
					(double)raw.gyro_rad_s[1],
					(double)raw.gyro_rad_s[2]); */
				

					x=raw.gyro_rad_s[0];
					y=raw.gyro_rad_s[1];
					z=raw.gyro_rad_s[2];
			}
			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
		}
		
		angle_rad_x = angle_rad_x + (x * 10.0) ;
		angle_deg_x = angle_rad_x * 180.0 / 3140.0 ;
		
		angle_rad_y = angle_rad_y + (y * 10.0) ;
		angle_deg_y = angle_rad_y * 180.0 / 3140.0 ;		

		angle_rad_z = angle_rad_z + (z * 10.0) ;
		angle_deg_z = angle_rad_z * 180.0 / 3140.0 ;
		
		if (count >= 10) 
			{
			printf ("\nAngle_X : %.4f \tAngle_Y : %.4f \tAngle_Z : %.4f",angle_deg_x,angle_deg_y,angle_deg_z) ;		
			count = 0 ;
			}
		
		if (angle_deg_x >= 90.0 && flag_x_p == 0)
			{
			printf ("\n90 degrees on X axis !!\n") ;
			flag_x_p = 1 ;
			FLAG++ ;
			}	

		if (angle_deg_y >= 90.0 && flag_y_p == 0)
			{
			printf ("\n90 degrees on Y axis !!\n") ;
			flag_y_p = 1 ;
			FLAG++ ;
			}		
	

		if (angle_deg_z >= 90.0 && flag_z_p == 0)
			{
			printf ("\n90 degrees on Z axis !!\n") ;
			flag_z_p = 1 ;
			FLAG++ ;
			}		

		if (angle_deg_x <= -90.0 && flag_x_n == 0)
			{
			printf ("\n-90 degrees on X axis !!\n") ;
			flag_x_n = 1 ;
			FLAG++ ;
			}	

		if (angle_deg_y <= -90.0 && flag_y_n == 0)
			{
			printf ("\n-90 degrees on Y axis !!\n") ;
			flag_y_n = 1 ;
			FLAG++ ;
			}		
	

		if (angle_deg_z <= -90.0 && flag_z_n == 0)
			{
			printf ("\n-90 degrees on Z axis !!\n") ;
			flag_z_n = 1 ;
			FLAG++ ;
			}

		if (FLAG >= 6)
			{
			printf ("\nGyrometer Tested Successfully\n") ;
			break ;
			}

		if (end_count > 10000)
		{
			printf("\nAGyrometer testing failed.\n");			
			break;
		}
			
	end_count++ ;
	count++ ;
	}
 
	return 0;
}


