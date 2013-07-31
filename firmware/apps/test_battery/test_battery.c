#include <uORB/topics/battery_status.h>
#include <stdio.h>

__EXPORT int  test_battery_main(int argc, char *argv[]);

int	test_battery_main(int argc, char *argv[])
{
	int battery_sub_fd = orb_subscribe(ORB_ID(battery_status));
 
		struct battery_status_s raw;

		/* copy sensors raw data into local buffer */
		orb_copy(ORB_ID(battery_status), battery_sub_fd, &raw);
		printf("\nVoltage : %.4f\n",raw.voltage_v);
		printf("\nCurrent : %.4f\n",raw.current_a);
}

