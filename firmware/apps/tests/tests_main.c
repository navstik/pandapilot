/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Lorenz Meier <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file tests_main.c
 * Tests main file, loads individual tests.
 */

#include <nuttx/config.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include <nuttx/spi.h>

#include <systemlib/perf_counter.h>

#include "tests.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int test_help(int argc, char *argv[]);
static int test_all(int argc, char *argv[]);

/****************************************************************************
 * Private Data
 ****************************************************************************/

const struct {
	const char 	*name;
	int	(* fn)(int argc, char *argv[]);
	unsigned	options;
#define OPT_NOHELP	(1<<0)
#define OPT_NOALLTEST	(1<<1)
} tests[] = {
	{"accelerometer",	test_accelerometer,	0},	
	{"barometer",		test_barometer, 	0},
	{"battery",		test_battery,		0},	
	{"led",			test_led,		0},
	{"gpio",		test_gpio,		0},
	{"gyrometer",		test_gyrometer,		0},	
	{"magnetometer",	test_magnetometer,	0},	
	{"pwm",			test_pwm,		0},
	{"pwren",		test_pwren,		0},
	{"usb",			test_usb,		0},
	{"help",		test_help,		0},
	{"all",			test_all,		OPT_NOALLTEST},
	{NULL,			NULL, 			0}
};

#define NTESTS (sizeof(tests) / sizeof(tests[0]))

static int
test_help(int argc, char *argv[])
{
	unsigned	i;

	printf("Available tests:\n");

	for (i = 0; tests[i].name; i++)
		printf("  %s\n", tests[i].name);

	return 0;
}

static int
test_all(int argc, char *argv[])
{
	unsigned	i;
	char		*args[2] = {"all", NULL};
	unsigned int failcount = 0;
	unsigned int testcount = 0;
	bool		passed[NTESTS];

	printf("\nRunning all tests...\n\n");

	for (i = 0; tests[i].name; i++) {
		/* Only run tests that are not excluded */
		if (!(tests[i].options & OPT_NOALLTEST)) {
			printf("  [%s] \t\t\tSTARTING TEST\n", tests[i].name);
			fflush(stdout);

			/* Execute test */
			if (tests[i].fn(1, args) != 0) {
				fprintf(stderr, "  [%s] \t\t\tFAIL\n", tests[i].name);
				fflush(stderr);
				failcount++;
				passed[i] = false;
			} else {
				printf("  [%s] \t\t\tPASS\n", tests[i].name);
				fflush(stdout);
				passed[i] = true;
			}
			testcount++;
		}
	}

	/* Print summary */
	printf("\n");
	int j;

	for (j = 0; j < 40; j++) {
		printf("-");
	}

	printf("\n\n");

	printf("     T E S T    S U M M A R Y\n\n");

	if (failcount == 0) {
		printf("  ______     __         __            ______     __  __    \n");
		printf(" /\\  __ \\   /\\ \\       /\\ \\          /\\  __ \\   /\\ \\/ /    \n");
		printf(" \\ \\  __ \\  \\ \\ \\____  \\ \\ \\____     \\ \\ \\/\\ \\  \\ \\  _\"-.  \n");
		printf("  \\ \\_\\ \\_\\  \\ \\_____\\  \\ \\_____\\     \\ \\_____\\  \\ \\_\\ \\_\\ \n");
		printf("   \\/_/\\/_/   \\/_____/   \\/_____/      \\/_____/   \\/_/\\/_/ \n");
		printf("\n");
		printf(" All tests passed (%d of %d)\n", testcount, testcount);

	} else {
		printf("  ______   ______     __     __ \n");
		printf(" /\\  ___\\ /\\  __ \\   /\\ \\   /\\ \\    \n");
		printf(" \\ \\  __\\ \\ \\  __ \\  \\ \\ \\  \\ \\ \\__\n");
		printf("  \\ \\_\\    \\ \\_\\ \\_\\  \\ \\_\\  \\ \\_____\\ \n");
		printf("   \\/_/     \\/_/\\/_/   \\/_/   \\/_____/ \n");
		printf("\n");
		printf(" Some tests failed (%d of %d)\n", failcount, testcount);
	}

	printf("\n");

	/* Print failed tests */
	if (failcount > 0) printf(" Failed tests:\n\n");

	unsigned int k;

	for (k = 0; k < i; k++) {
		if (!passed[k] && !(tests[k].options & OPT_NOALLTEST)) {
			printf(" [%s] to obtain details, please re-run with\n\t nsh> tests %s\n\n", tests[k].name, tests[k].name);
		}
	}

	fflush(stdout);

	return 0;
}

__EXPORT int tests_main(int argc, char *argv[]);

/**
 * Executes system tests.
 */
int tests_main(int argc, char *argv[])
{
	unsigned	i;

	if (argc < 2) {
		printf("tests: missing test name - 'tests help' for a list of tests\n");
		return 1;
	}

	for (i = 0; tests[i].name; i++) {
		if (!strcmp(tests[i].name, argv[1]))
			return tests[i].fn(argc - 1, argv + 1);
	}

	printf("tests: no test called '%s' - 'tests help' for a list of tests\n", argv[1]);
	return ERROR;
}
