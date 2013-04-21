/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_NAVSTIK

#include "AP_HAL_NAVSTIK.h"
#include "Scheduler.h"

#include <unistd.h>
#include <stdlib.h>
#include <sched.h>
#include <errno.h>
#include <stdio.h>
#include <drivers/drv_hrt.h>
#include <nuttx/arch.h>
#include <systemlib/systemlib.h>
#include <poll.h>

#include "UARTDriver.h"
#include "Storage.h"
#include "RCOutput.h"

using namespace Navstik;

extern const AP_HAL::HAL& hal;

extern bool _navstik_thread_should_exit;

NavstikScheduler::NavstikScheduler() :
    _perf_timers(perf_alloc(PC_ELAPSED, "APM_timers")),
	_perf_delay(perf_alloc(PC_ELAPSED, "APM_delay"))
{}

void NavstikScheduler::init(void *unused) 
{
    _sketch_start_time = hrt_absolute_time();

    // setup the timer thread - this will call tasks at 1kHz
	pthread_attr_t thread_attr;
	struct sched_param param;

	pthread_attr_init(&thread_attr);
	pthread_attr_setstacksize(&thread_attr, 2048);

	param.sched_priority = APM_TIMER_PRIORITY;
	(void)pthread_attr_setschedparam(&thread_attr, &param);
    pthread_attr_setschedpolicy(&thread_attr, SCHED_FIFO);

	pthread_create(&_timer_thread_ctx, &thread_attr, (pthread_startroutine_t)&Navstik::NavstikScheduler::_timer_thread, this);

    // the IO thread runs at lower priority
	pthread_attr_init(&thread_attr);
	pthread_attr_setstacksize(&thread_attr, 2048);

	param.sched_priority = APM_IO_PRIORITY;
	(void)pthread_attr_setschedparam(&thread_attr, &param);
    pthread_attr_setschedpolicy(&thread_attr, SCHED_FIFO);

	pthread_create(&_io_thread_ctx, &thread_attr, (pthread_startroutine_t)&Navstik::NavstikScheduler::_io_thread, this);
}

uint32_t NavstikScheduler::micros() 
{
    return (uint32_t)(hrt_absolute_time() - _sketch_start_time);
}

uint32_t NavstikScheduler::millis() 
{
    return hrt_absolute_time() / 1000;
}

void NavstikScheduler::delay_microseconds(uint16_t usec) 
{
    if (_in_timer_proc) {
        ::printf("ERROR: delay_microseconds() from timer process\n");
        return;
    }
    perf_begin(_perf_delay);
	uint32_t start = micros();
	while (micros() - start < usec) {
		up_udelay(usec - (micros() - start));
	}
    perf_end(_perf_delay);
}

void NavstikScheduler::delay(uint16_t ms)
{
    if (_in_timer_proc) {
        ::printf("ERROR: delay() from timer process\n");
        return;
    }
    perf_begin(_perf_delay);
	uint64_t start = hrt_absolute_time();
    
    while ((hrt_absolute_time() - start)/1000 < ms && 
           !_navstik_thread_should_exit) {
        // this yields the CPU to other apps
        poll(NULL, 0, 1);
        if (_min_delay_cb_ms <= ms) {
            if (_delay_cb) {
                _delay_cb();
            }
        }
    }
    perf_end(_perf_delay);
    if (_navstik_thread_should_exit) {
        exit(1);
    }
}

void NavstikScheduler::register_delay_callback(AP_HAL::Proc proc,
                                            uint16_t min_time_ms) 
{
    _delay_cb = proc;
    _min_delay_cb_ms = min_time_ms;
}

void NavstikScheduler::register_timer_process(AP_HAL::TimedProc proc) 
{
    for (uint8_t i = 0; i < _num_timer_procs; i++) {
        if (_timer_proc[i] == proc) {
            return;
        }
    }

    if (_num_timer_procs < NAVSTIK_SCHEDULER_MAX_TIMER_PROCS) {
        _timer_proc[_num_timer_procs] = proc;
        _num_timer_procs++;
    } else {
        hal.console->printf("Out of timer processes\n");
    }
}

void NavstikScheduler::register_timer_failsafe(AP_HAL::TimedProc failsafe, uint32_t period_us) 
{
    _failsafe = failsafe;
}

void NavstikScheduler::suspend_timer_procs() 
{
    _timer_suspended = true;
}

void NavstikScheduler::resume_timer_procs() 
{
    _timer_suspended = false;
    if (_timer_event_missed == true) {
        _run_timers(false);
        _timer_event_missed = false;
    }
}

void NavstikScheduler::reboot() 
{
	up_systemreset();
}

void NavstikScheduler::_run_timers(bool called_from_timer_thread)
{
    uint32_t tnow = micros();
    if (_in_timer_proc) {
        return;
    }
    _in_timer_proc = true;

    if (!_timer_suspended) {
        // now call the timer based drivers
        for (int i = 0; i < _num_timer_procs; i++) {
            if (_timer_proc[i] != NULL) {
                _timer_proc[i](tnow);
            }
        }
    } else if (called_from_timer_thread) {
        _timer_event_missed = true;
    }

    // and the failsafe, if one is setup
    if (_failsafe != NULL) {
        _failsafe(tnow);
    }

    _in_timer_proc = false;
}

void *NavstikScheduler::_timer_thread(void)
{
    while (!_navstik_thread_should_exit) {
        poll(NULL, 0, 1);

        // run registered timers
        perf_begin(_perf_timers);
        _run_timers(true);
        perf_end(_perf_timers);

        // process any pending RC output requests
        ((NavstikRCOutput *)hal.rcout)->_timer_tick();
    }
    return NULL;
}

void *NavstikScheduler::_io_thread(void)
{
    while (!_navstik_thread_should_exit) {
        poll(NULL, 0, 1);

        // process any pending serial bytes
        ((NavstikUARTDriver *)hal.uartA)->_timer_tick();
        ((NavstikUARTDriver *)hal.uartB)->_timer_tick();

        // process any pending storage writes
        ((NavstikStorage *)hal.storage)->_timer_tick();
    }
    return NULL;
}

void NavstikScheduler::panic(const prog_char_t *errormsg) 
{
    write(1, errormsg, strlen(errormsg));
    write(1, "\n", 1);
    hal.scheduler->delay_microseconds(10000);
    _navstik_thread_should_exit = true;
    exit(1);
}

bool NavstikScheduler::in_timerprocess() {
    return _in_timer_proc;
}

bool NavstikScheduler::system_initializing() {
    return !_initialized;
}

void NavstikScheduler::system_initialized() {
    if (_initialized) {
        panic(PSTR("PANIC: scheduler::system_initialized called"
                   "more than once"));
    }
    _initialized = true;
}

#endif
