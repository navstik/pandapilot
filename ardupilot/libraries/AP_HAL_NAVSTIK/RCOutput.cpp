/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_NAVSTIK
#include "RCOutput.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <drivers/drv_pwm_output.h>

extern const AP_HAL::HAL& hal;

using namespace Navstik;

void NavstikRCOutput::init(void* unused) 
{
    _perf_rcout = perf_alloc(PC_ELAPSED, "APM_rcout");
    _pwm_fd = open(PWM_OUTPUT_DEVICE_PATH, O_RDWR);
    if (_pwm_fd == -1) {
	hal.scheduler->panic("Unable to open " PWM_OUTPUT_DEVICE_PATH);
    }
    ioctl(_pwm_fd, PWM_SERVO_ARM, 0);
}

void NavstikRCOutput::set_freq(uint32_t chmask, uint16_t freq_hz) 
{
    if (freq_hz == _freq_hz) {
        // avoid the ioctl() if possible
        return;
    }
    // we can't set this per channel yet
    if (ioctl(_pwm_fd, PWM_SERVO_SET_UPDATE_RATE, (unsigned long)freq_hz) == 0) {
        _freq_hz = freq_hz;
    }
}

uint16_t NavstikRCOutput::get_freq(uint8_t ch) 
{
	return _freq_hz;
}

void NavstikRCOutput::enable_ch(uint8_t ch)
{
    // channels are always enabled ...
}

void NavstikRCOutput::enable_mask(uint32_t chmask)
{
    // channels are always enabled ...
}

void NavstikRCOutput::disable_ch(uint8_t ch)
{
    // channels are always enabled ...
}

void NavstikRCOutput::disable_mask(uint32_t chmask)
{
    // channels are always enabled ...
}

void NavstikRCOutput::write(uint8_t ch, uint16_t period_us)
{
    if (ch >= NAVSTIK_NUM_OUTPUT_CHANNELS) {
        return;
    }
    if (ch > _max_channel) {
        _max_channel = ch;
    }
    if (period_us != _period[ch]) {
        _period[ch] = period_us;
        _need_update = true;
        up_pwm_servo_set(ch, period_us);    
    }
}

void NavstikRCOutput::write(uint8_t ch, uint16_t* period_us, uint8_t len)
{
    for (uint8_t i=0; i<len; i++) {
        write(i, period_us[i]);
    }
}

uint16_t NavstikRCOutput::read(uint8_t ch) 
{
    if (ch >= NAVSTIK_NUM_OUTPUT_CHANNELS) {
        return 0;
    }
    return _period[ch];
}

void NavstikRCOutput::read(uint16_t* period_us, uint8_t len)
{
    for (uint8_t i=0; i<len; i++) {
        period_us[i] = read(i);
    }
}

void NavstikRCOutput::_timer_tick(void)
{
    uint32_t now = hal.scheduler->micros();

    // always send at least at 20Hz, otherwise the IO board may think
    // we are dead
    if (now - _last_output > 50000) {
        _need_update = true;
    }

    if (_need_update && _pwm_fd != -1) {
        _need_update = false;
        perf_begin(_perf_rcout);
        ::write(_pwm_fd, _period, _max_channel*sizeof(_period[0]));
        perf_end(_perf_rcout);
        _last_output = now;
    }
}

#endif // CONFIG_HAL_BOARD
