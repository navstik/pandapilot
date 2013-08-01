#include "gps.h"
#include <nuttx/config.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <fcntl.h>
#include "ubx.h" //header files for the ubx protocol
#include <termios.h>
#include <signal.h>
#include <pthread.h>
#include <sys/prctl.h>
#include <errno.h>
#include <signal.h>
#include <string.h>  /* String function definitions */
#include <poll.h>
#include <systemlib/systemlib.h>

bool thread_should_exit;				/**< Deamon status flag */
static bool thread_running = false;				/**< Deamon status flag */
static int deamon_task;							/**< Handle of deamon task */
 
__EXPORT int gps_config_main(int argc, char *argv[]);

/**
 * Mainloop of deamon.
 */
int gps_config_thread_main(int argc, char *argv[]);
int gps_config_ubx_main(int argc, char *argv[]);
/**
 * Print the correct usage.
 */
static void usage(const char *reason);

/****************************************************************************
 * Definitions
 ****************************************************************************/
#define IMPORTANT_GPS_BAUD_RATES_N		2
#define RETRY_INTERVAL_SECONDS			10

bool gps_mode_try_all;
bool gps_baud_try_all;
bool gps_mode_success;
bool terminate_gps_thread;
bool gps_verbose;
int current_gps_speed = B38400;

enum GPS_MODES {
	GPS_MODE_START = 0,
	GPS_MODE_UBX = 1,
	GPS_MODE_MTK = 2,
	GPS_MODE_NMEA = 3,
	GPS_MODE_END = 4
};

/****************************************************************************
 * Private functions
 ****************************************************************************/
int open_port(char *port);

void close_port(int *fd);

void setup_port(char *device, int speed, int *fd);

 
int gps_config_main(int argc, char *argv[])
{
	int fd; /* File descriptor for the port */
 
/*GPS is connected to the controller via UART2 and the port for uart 2 is /dev/ttyS1 */
	fd = open("/dev/ttyS1", O_RDWR | O_NOCTTY | O_NDELAY);/*Opening the port */
 
/* Try to set baud rate */
	struct termios uart_config;
 
	tcgetattr(fd, &uart_config); // get what is the current baud rate
 
/* Setting baud rate of UART to 9600 for communicating with GPS at its default baudrate of 9600 */
	cfsetispeed(&uart_config, B9600);
	cfsetospeed(&uart_config, B9600);
 
	tcsetattr(fd, TCSANOW, &uart_config); //confirming that the baud rate is set
 
	int n=0;

/*changing baud rate of gps to 38400 - >  We can use either such hex messages or decimal messages */
char gps_baud[]="$PUBX,41,1,0007,0003,38400,0*20\r\n"; /*String sent to GPS for changing its baud rate.*/
	n = write(fd,gps_baud,sizeof(gps_baud));/*Writing the string to gps*/
 
	if (n < 0)
        printf("baud write failed!");
 
	close(fd);/*Closing the port*/
 
	fd = open("/dev/ttyS1", O_RDWR | O_NOCTTY | O_NDELAY); // Opening the port again
 
/*Now the Gps is set at 38400 baud rate due to the message sent above , So we need to set Uart port also at te same rate for communicating with gps */
        tcgetattr(fd, &uart_config);
 
/* Set baud rate of UART to 38400 now */
	cfsetispeed(&uart_config, B38400);
	cfsetospeed(&uart_config, B38400);
 
        tcsetattr(fd, TCSANOW, &uart_config);
 
//UBX=>CFG=>RATE(Rates) - change the Measurement Period to 200ms. This gives a 5 Hz position update since 5 x 200ms is one second.
char gps_rate[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};
 
	n = write(fd,gps_rate,sizeof(gps_rate));
 
	if (n < 0)
	        printf("gps_rate write failed!");
 
        close(fd);

	gps_config_ubx_main(argc,argv);
	
}

int gps_config_ubx_main(int argc, char *argv[])
{	
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("gps already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		deamon_task = task_spawn("gps_config",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_DEFAULT,
					 4096,
					 gps_config_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		thread_running = true;
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\tgps_config is running\n");
		} else {	
			printf("\tgps_config not started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

/*
 * Main function of gps_config app.
 */
int gps_config_thread_main(int argc, char *argv[]) {


	/* default values */
	const char *commandline_usage = "\tusage: %s {start|stop|status}\n";
	char *device = "/dev/ttyS1";
	gps_mode_success = true;
	terminate_gps_thread = false;
	bool retry = false;
	gps_verbose = false;

	enum GPS_MODES current_gps_mode = GPS_MODE_UBX;
	/* Declare file descriptor for gps here, open port later in setup_port */
	int fd;

	while (!thread_should_exit) {
		if (current_gps_mode == GPS_MODE_UBX) {

			if (gps_verbose) printf("[gps] Trying UBX mode at %d baud\n", current_gps_speed);
			             
				setup_port(device, current_gps_speed, &fd);
                
				/* start ubx thread and watchdog */
				pthread_t ubx_thread;
				pthread_t ubx_watchdog_thread;

				pthread_mutex_t  ubx_mutex_d;
				ubx_mutex = &ubx_mutex_d;
				pthread_mutex_init(ubx_mutex, NULL);
				gps_bin_ubx_state_t ubx_state_d;
				ubx_state = &ubx_state_d;
				ubx_decode_init();

				pthread_attr_t ubx_loop_attr;
				pthread_attr_init(&ubx_loop_attr);
				pthread_attr_setstacksize(&ubx_loop_attr, 3000);

				struct arg_struct args;
				args.fd_ptr = &fd;
				args.thread_should_exit_ptr = &thread_should_exit;

				pthread_create(&ubx_thread, &ubx_loop_attr, ubx_loop, (void *)&args);
                
				pthread_attr_t ubx_wd_attr;
				pthread_attr_init(&ubx_wd_attr);
				pthread_attr_setstacksize(&ubx_wd_attr, 1400);
				int pthread_create_res = pthread_create(&ubx_watchdog_thread, &ubx_wd_attr, ubx_watchdog_loop, (void *)&args);

				if (pthread_create_res != 0) fprintf(stderr, "[gps] ERROR: could not create ubx watchdog thread, pthread_create =%d\n", pthread_create_res);

				/* wait for threads to complete */
				pthread_join(ubx_watchdog_thread, NULL);

				if (gps_mode_success == false) {
					if (gps_verbose) printf("[gps] no success with UBX mode and %d baud\n", current_gps_speed);

					terminate_gps_thread = true;
					pthread_join(ubx_thread, NULL);

					gps_mode_success = true;
					terminate_gps_thread = false;

				/* maybe the watchdog was stopped through the thread_should_exit flag */
				} else if (thread_should_exit) {
					pthread_join(ubx_thread, NULL);
					if (gps_verbose) printf("[gps] ubx watchdog and ubx loop threads have been terminated, exiting.");
					//close(mavlink_fd);
					close_port(&fd);
					thread_running = false;
					return 0;
				}

				close_port(&fd);
			} 

			/* exit quickly if stop command has been received */
			if (thread_should_exit) {
				printf("[gps] stopped, exiting.\n");
				//close(mavlink_fd);
				thread_running = false;
				return 0;
	        }

			
	        
		if (retry) {
			printf("[gps] No configuration was successful, retrying in %d seconds \n", RETRY_INTERVAL_SECONDS);
			//mavlink_log_info(mavlink_fd, "[gps] No configuration was successful, retrying...");
			fflush(stdout);

		} else {
			fprintf(stderr, "[gps] No configuration was successful, exiting... \n");
			fflush(stdout);
			//mavlink_log_info(mavlink_fd, "[gps] No configuration was successful, exiting...");
			break;
		}

		sleep(RETRY_INTERVAL_SECONDS);
	}

	printf("[gps] exiting.\n");
	//close(mavlink_fd);
	thread_running = false;
	return 0;
}


static void usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);
	fprintf(stderr, "\tusage: gps {start|status|stop} -d devicename -b baudrate -m mode\n\tmodes are:\n\t\tubx\n\t\tmtkcustom\n\t\tnmea\n\t\tall\n");
	exit(1);
}

int open_port(char *port)
{
	int fd; /**< File descriptor for the gps port */

	/* Open serial port */
	fd = open(port, O_CREAT | O_RDWR | O_NOCTTY); /* O_RDWR - Read and write O_NOCTTY - Ignore special chars like CTRL-C */
	return (fd);
}


void close_port(int *fd)
{
	/* Close serial port */
	close(*fd);
}

void setup_port(char *device, int speed, int *fd)
{
	/* open port (baud rate is set in defconfig file) */
	*fd = open_port(device);

	if (*fd != -1) {
		if (gps_verbose) printf("[gps] Port opened: %s at %d baud\n", device, speed);

	} else {
		fprintf(stderr, "[gps] Could not open port, exiting gps app!\n");
		fflush(stdout);
	}

	/* Try to set baud rate */
	struct termios uart_config;
	int termios_state;

	if ((termios_state = tcgetattr(*fd, &uart_config)) < 0) {
		fprintf(stderr, "[gps] ERROR getting baudrate / termios config for %s: %d\n", device, termios_state);
		close(*fd);
	}
	if (gps_verbose) printf("[gps] Try to set baud rate %d now\n",speed);
	/* Set baud rate */
	cfsetispeed(&uart_config, speed);
	cfsetospeed(&uart_config, speed);
	if ((termios_state = tcsetattr(*fd, TCSANOW, &uart_config)) < 0) {
		fprintf(stderr, "[gps] ERROR setting baudrate / termios config for %s (tcsetattr)\n", device);
		close(*fd);
	}
}
