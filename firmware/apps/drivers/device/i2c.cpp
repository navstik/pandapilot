/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file i2c.cpp
 *
 * Base class for devices attached via the I2C bus.
 *
 * @todo Bus frequency changes; currently we do nothing with the value
 *       that is supplied.  Should we just depend on the bus knowing?
 */

#include "i2c.h"

namespace device
{

I2C::I2C(const char *name,
	 const char *devname,
	 int bus,
	 uint16_t address,
	 uint32_t frequency,
	 int irq) :
	// base class
	CDev(name, devname, irq),
	// public
	// protected
	_retries(0),
	// private
	_bus(bus),
	_address(address),
	_frequency(frequency),
	_dev(nullptr)
{
}

I2C::~I2C()
{
	if (_dev)
		up_i2cuninitialize(_dev);
}

int
I2C::init()
{
	int ret = OK;

	/* attach to the i2c bus */
	_dev = up_i2cinitialize(_bus);

	if (_dev == nullptr) {
		debug("failed to init I2C");
		ret = -ENOENT;
		goto out;
	}

	// call the probe function to check whether the device is present
	ret = probe();

	if (ret != OK) {
		debug("probe failed");
		goto out;
	}

	// do base class init, which will create device node, etc
	ret = CDev::init();

	if (ret != OK) {
		debug("cdev init failed");
		goto out;
	}

	// tell the world where we are
	log("on I2C bus %d at 0x%02x", _bus, _address);

out:
	return ret;
}

int
I2C::probe()
{
	// Assume the device is too stupid to be discoverable.
	return OK;
}

int
I2C::transfer(const uint8_t *send, unsigned send_len, uint8_t *recv, unsigned recv_len)
{
	struct i2c_msg_s msgv[2];
	unsigned msgs;
	int ret;
	unsigned tries = 0;

	do {
		//	debug("transfer out %p/%u  in %p/%u", send, send_len, recv, recv_len);

		msgs = 0;

		if (send_len > 0) {
			msgv[msgs].addr = _address;
			msgv[msgs].flags = 0;
			msgv[msgs].buffer = const_cast<uint8_t *>(send);
			msgv[msgs].length = send_len;
			msgs++;
		}

		if (recv_len > 0) {
			msgv[msgs].addr = _address;
			msgv[msgs].flags = I2C_M_READ;
			msgv[msgs].buffer = recv;
			msgv[msgs].length = recv_len;
			msgs++;
		}

		if (msgs == 0)
			return -EINVAL;

		/*
		 * I2C architecture means there is an unavoidable race here
		 * if there are any devices on the bus with a different frequency
		 * preference.  Really, this is pointless.
		 */
		I2C_SETFREQUENCY(_dev, _frequency);
		ret = I2C_TRANSFER(_dev, &msgv[0], msgs);

		if (ret == OK)
			break;

		// reset the I2C bus to unwedge on error
		up_i2creset(_dev);

	} while (tries++ < _retries);

	return ret;

}

int
I2C::transfer(i2c_msg_s *msgv, unsigned msgs)
{
	for (unsigned i = 0; i < msgs; i++)
		msgv[i].addr = _address;

	/*
	 * I2C architecture means there is an unavoidable race here
	 * if there are any devices on the bus with a different frequency
	 * preference.  Really, this is pointless.
	 */
	I2C_SETFREQUENCY(_dev, _frequency);
	int ret = I2C_TRANSFER(_dev, msgv, msgs);

	if (ret != OK)
		up_i2creset(_dev);

	return ret;
}

} // namespace device