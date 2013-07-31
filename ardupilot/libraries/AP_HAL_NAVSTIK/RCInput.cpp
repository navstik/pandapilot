#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_NAVSTIK
#include "RCInput.h"
#include <drivers/drv_hrt.h>

using namespace Navstik;

extern const AP_HAL::HAL& hal;

#if CONFIG_HRT_PPM
extern uint16_t rc_buffer[6];
//extern uint16_t ppm_buffer[32];
extern unsigned ppm_decoded_channels;

extern uint64_t ppm_last_valid_decode;
#else
uint16_t ppm_buffer[32];
unsigned ppm_decoded_channels;
uint64_t ppm_last_valid_decode;
#endif

void NavstikRCInput::init(void* unused)
{
	clear_overrides();
}

uint8_t NavstikRCInput::valid() 
{
	return ppm_last_valid_decode != _last_read || _override_valid;
}

uint16_t NavstikRCInput::read(uint8_t ch) 
{
	_last_read = ppm_last_valid_decode;
	_override_valid = false;
	if (ch >= NAVSTIK_NUM_RCINPUT_CHANNELS) {
		return 0;
	}
	if (_override[ch]) {
		return _override[ch];
	}
	return rc_buffer[ch];
}

uint8_t NavstikRCInput::read(uint16_t* periods, uint8_t len) 
{
	if (len > NAVSTIK_NUM_RCINPUT_CHANNELS) {
		len = NAVSTIK_NUM_RCINPUT_CHANNELS;
	}
	for (uint8_t i = 0; i < len; i++){
		periods[i] = read(i);
	}
	return len;
}

bool NavstikRCInput::set_overrides(int16_t *overrides, uint8_t len) 
{
	bool res = false;
	for (uint8_t i = 0; i < len; i++) {
		res |= set_override(i, overrides[i]);
	}
	return res;
}

bool NavstikRCInput::set_override(uint8_t channel, int16_t override) {
	if (override < 0) {
		return false; /* -1: no change. */
	}
	if (channel >= NAVSTIK_NUM_RCINPUT_CHANNELS) {
		return false;
	}
	_override[channel] = override;
	if (override != 0) {
		_override_valid = true;
		return true;
	}
	return false;
}

void NavstikRCInput::clear_overrides()
{
	for (uint8_t i = 0; i < NAVSTIK_NUM_RCINPUT_CHANNELS; i++) {
		set_override(i, 0);
	}
}

#endif
