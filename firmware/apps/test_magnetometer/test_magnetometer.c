#include <nuttx/config.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <math.h>
 
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>

#define PI 3.14159265

__EXPORT int test_magnetometer_main(int argc, char *argv[]);

float my_atan2f(float y, float x)  ;
 
int test_magnetometer_main(int argc, char *argv[])
{	
	float acc_x=0,acc_y=0,acc_z=0,mag_x=0,mag_y=0,mag_z=0,result,degree;
	float heading,initial_heading, angles[3], cosRoll, sinRoll, cosPitch, sinPitch ;
	int flag_x=0,flag_y=0,flag_z=0,FLAG=0,count=0,end_count=0;

	printf("\nHold the board in a horizontal plane and rotate it through 360 degree. You should see the headings changing through 360 degrees in one complete rotation.\n");
	usleep (4000000) ;
 
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
			printf("[magnetometer] Got no data within a second\n");
		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				printf("[magnetometer] ERROR return value from poll(): %d\n"
					, poll_ret);
			}
			error_counter++;
		} else {
 
			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct sensor_combined_s raw;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
		/*		printf("\n[Magno] magnetometer:\t%8.4f\t%8.4f\t%8.4f\n",
					raw.magnetometer_raw[0],
					raw.magnetometer_raw[1],
					raw.magnetometer_raw[2]);   */
			
					acc_x=raw.accelerometer_m_s2[0] ;
					acc_y=raw.accelerometer_m_s2[1] ;
					acc_z=raw.accelerometer_m_s2[2] ;

					mag_x=raw.magnetometer_raw[0];
					mag_y=raw.magnetometer_raw[1];
					mag_z=raw.magnetometer_raw[2];
			}
			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
		}
		
		

		  angles[0] = asinf(-0.1f * acc_x) ;
		  angles[0] *= 180.0 ;
		  angles[0] /= PI ;
		//	printf ("\nAngle[0] = %.4f",angles[0]) ;

		  angles[1] = atanf(acc_y / acc_z) ;
		  angles[1] *= 180.0 ;
		  angles[1] /= PI ;
		//	printf ("\nAngle[1] = %.4f",angles[1]) ;

		  angles[2] = 0.0 ;		

		 angles[0] = (angles[0] * PI / 180.0) ;
		  angles[1] = (angles[1] * PI / 180.0) ;
		  cosPitch = cosf(angles[0]) ;
		  sinPitch = sinf(angles[0]) ;
		  cosRoll  = cosf(angles[1]) ;
		  sinRoll  = sinf(angles[1]) ;

		  float Xh = (mag_x * cosPitch) + (mag_z * sinPitch) ;
		  float Yh = (mag_x * sinRoll * sinPitch) + (mag_y * cosRoll) - (mag_z * sinRoll * cosPitch) ;
		  heading = my_atan2f(Yh, Xh) ;
		//printf ("\nYh = %.4f\tXh = %.4f",Yh,Xh) ; 		
	
		  heading *= 180.0 / PI ;
		printf ("\nHeading : %.4f",heading);

		if (count == 0)
			initial_heading = heading ;

		count++ ;

		if((heading > (initial_heading - 5)) && (heading < (initial_heading + 5)) && (count > 30))
		{
			printf("\nMagnetometer tested successfully.\n");			
			break;
		}

		if (end_count > 1000)
		{
			printf("\nMagnetometer testing failed.\n");			
			break;
		}
			
	end_count++ ;
	}
 
	return 0;
}

float my_atan2f(float y, float x) {
  if((x > 0) && (y >= 0))
    return atanf(y / x) ;
  else if(x < 0)
    return (PI + atanf(y / x)) ;
  else if((x > 0) && (y < 0))
    return (2*PI + atanf(y / x)) ;
  else if((x == 0) && (y < 0))
    return PI / 2 ;
  else if((x == 0) && (y > 0))
    return (3 * PI/2) ;
  else
    return -32768.0 ;
}
