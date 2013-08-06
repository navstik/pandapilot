#include <nuttx/config.h>
#include <stdio.h>
#include <errno.h>
 
__EXPORT int print_rc_buffer_main(int argc, char *argv[]);
extern uint16_t rc_buffer[6];
 
int print_rc_buffer_main(int argc, char *argv[])
{
	int i ;

	for (i=0;i<6;i++)
	printf("%u\n",rc_buffer[i]);
	
	return OK;
}
