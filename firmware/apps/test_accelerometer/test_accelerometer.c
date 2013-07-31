#include <nuttx/config.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
 
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>


int test_axes_x(double x,int flag_x,int *p_x);
int test_axes_y(double y,int flag_y,int flag_x,int *p_y);
int test_axes_z(double z,int flag_z,int flag_x,int flag_y,int *p,int *p_z);
__EXPORT int test_accelerometer_main(int argc, char *argv[]);
 
int test_accelerometer_main(int argc, char *argv[])
{	
	double x=0,y=0,z=0;
	int flag_x_n=0,flag_y_n=0,flag_z_n=0,FLAG=0,flag_x_p=0,flag_y_p=0,flag_z_p=0,end_count=0;

	printf("\nRotate the board along all the 3 axes. The accelerometer will acquire a lock on all three axes.\n");
	usleep(3000000) ;
 
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
			printf("[accelerometer] Got no data within a second\n");
		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				printf("[accelerometer] ERROR return value from poll(): %d\n"
					, poll_ret);
			}
			error_counter++;
		} else {
 
			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct sensor_combined_s raw;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
				printf("[accelerometer] Accelerometer:\t%8.4f\t%8.4f\t%8.4f\n",
					(double)raw.accelerometer_m_s2[0],
					(double)raw.accelerometer_m_s2[1],
					(double)raw.accelerometer_m_s2[2]);

					x=(double)raw.accelerometer_m_s2[0];
					y=(double)raw.accelerometer_m_s2[1];
					z=(double)raw.accelerometer_m_s2[2];
			}
			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
		}


			if((x > 9) && (flag_x_p == 0))
				{	
				printf("\nPositive X-axis tested.\n");
				flag_x_p = 1 ;
				FLAG++ ;
				}	
			if((x < -9) && (flag_x_n == 0))
				{	
				printf("\nNegative X-axis tested.\n");
				flag_x_n = 1 ;				
				FLAG++ ;
				}

			if((y > 9) && (flag_y_p == 0))
				{	
				printf("\nPositive Y-axis tested.\n");
				flag_y_p = 1 ;
				FLAG++ ;
				}	
			if((y < -9) && (flag_y_n == 0))
				{	
				printf("\nNegative Y-axis tested.\n");
				flag_y_n = 1 ;				
				FLAG++ ;
				}
	
			
			if((z > 9) && (flag_z_p == 0))
				{	
				printf("\nPositive Z-axis tested.\n");
				flag_z_p = 1 ;
				FLAG++ ;
				}	
			if((z < -9) && (flag_z_n == 0))
				{	
				printf("\nNegative Z-axis tested.\n");
				flag_z_n = 1 ;				
				FLAG++ ;
				}
			
		if(FLAG >= 6)
		{
			printf("\nAccelerometer tested successfully.\n");			
			break;
		}

		if (end_count > 1000)
		{
			printf("\nAccelerometer testing failed.\n");			
			break;
		}
			
	end_count++ ;
	}
 
	return 0;
}


