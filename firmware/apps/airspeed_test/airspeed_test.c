#include <nuttx/config.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
 
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/airspeed.h>


__EXPORT int airspeed_test_main(int argc, char *argv[]);
 
int airspeed_test_main(int argc, char *argv[])
{	
	printf("\nAIRSPEED SENSOR READINGS!\n");
 
	/* subscribe to airspeed topic */
	int airspeed_sub_fd = orb_subscribe(ORB_ID(airspeed));
 	orb_set_interval(airspeed_sub_fd, 1000);
	/* one could wait for multiple topics with this technique, just using one here */
	struct pollfd fds[] = {
		{ .fd = airspeed_sub_fd,   .events = POLLIN },
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
			printf("[airspeed] Got no data within a second\n");
		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				printf("[airspeed] ERROR return value from poll(): %d\n"
					, poll_ret);
			}
			error_counter++;
		} else {
 
			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct airspeed_s raw;
				/* copy airspeed raw data into local buffer */
				orb_copy(ORB_ID(airspeed), airspeed_sub_fd, &raw);
				printf("[airspeed] Airspeed:\t%8.4f\t%8.4f\n",
					(float)raw.indicated_airspeed_m_s,
					(float)raw.true_airspeed_m_s
					);

			
			}
			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
		}
		
	}
 
	return 0;
}

