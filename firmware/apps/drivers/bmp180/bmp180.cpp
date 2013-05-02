/****************************************************************************
 *
 *   Copyright (C) 2013 Navstik Development Team. Based on PX4 port.
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
 * @file bmp180.cpp
 * Driver for the BMP180 barometric pressure sensor connected via I2C.
 */

#include <nuttx/config.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <arch/board/board.h>

#include <drivers/drv_hrt.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_baro.h>

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

/**
 * Calibration PROM as reported by the device.
 */
#pragma pack(push,1)
struct bmp180_data {
        uint8_t OSS ;
  	int16_t DELAY_PRESSURE ;
  	int16_t DELAY_TEMP ;
  	int16_t MEASUREMENT_MODE ;
  	int16_t AC1 ;
  	int16_t AC2 ;
  	int16_t AC3 ;
  	uint16_t AC4 ;
  	uint16_t AC5 ;
  	uint16_t AC6 ;
  	int16_t B1  ;
  	int16_t B2  ;
  	int16_t MB  ;
  	int16_t MC  ;
  	int16_t MD  ;
  	float PRESSURE ;
  	float TEMP ;
};

#pragma pack(pop)

class BMP180 : public device::I2C
{
public:
	BMP180(int bus);
	~BMP180();

	virtual int		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

protected:
	virtual int		probe();

private:
	struct bmp180_data	_data;

	struct work_s		_work;
	unsigned		_measure_ticks;

	unsigned		_num_reports;
	volatile unsigned	_next_report;
	volatile unsigned	_oldest_report;
	struct baro_report	*_reports;

	bool			_collect_phase;
	unsigned		_measure_phase;

	/* intermediate temperature values per BMP180 datasheet */
	int32_t			_TEMP;
	uint32_t		_UT;
	int64_t			_OFF;
	int64_t			_SENS;

	/* altitude conversion calibration */
	unsigned		_msl_pressure;	/* in kPa */

	orb_advert_t		_baro_topic;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;
	perf_counter_t		_buffer_overflows;

	/**
	 * Test whether the device supported by the driver is present at a
	 * specific address.
	 *
	 * @param address	The I2C bus address to probe.
	 * @return		True if the device is present.
	 */
	int			probe_address(uint8_t address);

	/**
	 * Initialise the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void			start();

	/**
	 * Stop the automatic measurement state machine.
	 */
	void			stop();

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 *
	 * This is the heart of the measurement state machine.  This function
	 * alternately starts a measurement, or collects the data from the
	 * previous measurement.
	 *
	 * When the interval between measurements is greater than the minimum
	 * measurement interval, a gap is inserted between collection
	 * and measurement to provide the most recent measurement possible
	 * at the next interval.
	 */
	void			cycle();

	/**
	 * Static trampoline from the workq context; because we don't have a
	 * generic workq wrapper yet.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		cycle_trampoline(void *arg);

	/**
	 * Issue a measurement command for the current state.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			measure();

	/**
	 * Collect the result of the most recent measurement.
	 */
	int			collect();

	/**
	 * Send a reset command to the BMP180.
	 *
	 * This is required after any bus reset.
	 */
	int			cmd_reset();

	/**
	 * Read the BMP180 PROM
	 *
	 * @return		OK if the PROM reads successfully.
	 */
	int			read_prom();

	/**
	 * PROM CRC routine ported from BMP180 application note
	 *
	 * @param n_prom	Pointer to words read from PROM.
	 * @return		True if the CRC matches.
	 */
	bool			crc4(uint16_t *n_prom);

};

/* helper macro for handling report buffer indices */
#define INCREMENT(_x, _lim)	do { _x++; if (_x >= _lim) _x = 0; } while(0)

/* helper macro for arithmetic - returns the square of the argument */
#define POW2(_x)		((_x) * (_x))

/*
 * BMP180 internal constants and data structures.
 */

/* internal conversion time: 9.17 ms, so should not be read at rates higher than 100 Hz */
//#define BMP180_CONVERSION_INTERVAL	10000	/* microseconds */
#define BMP180_MEASUREMENT_RATIO	3	/* pressure measurements per temperature measurement */

#define BMP180_BUS		NAVSTIK_I2C_BUS_SENSORS
#define BMP180_ADDRESS		NAVSTIK_I2C_OBDEV_BMP180 /* address select pins pulled high */

#define BMP180_CTL        	0xF4 // Measurement Control Register.
#define BMP180_ADC_MSB      	0xF6 // Read Only Register. Contains Output Data.
#define BMP180_ADC_LSB      	0xF7 // Read Only Register. Contains Output Data.
#define BMP180_ADC_XLSB      	0xF8 // Read Only Register. Contains Output Data.
#define BMP180_ID        	0xD0 // Read Only Register. Value is 0x55 can be used to check communication.
#define BMP180_SOFT_RESET        	0xE0 // Write Only Register. If set to 0xB6 performs same sequence as power on reset

#define BMP180_TEMP        	0x2E // Value of Control Register for temperature measurment
#define BMP180_OSS0        	0x34 // Value of Control Register for Conversion time 4.5ms
#define BMP180_OSS1        	0x74 // Value of Control Register for Conversion time 7.5ms
#define BMP180_OSS2        	0xB4 // Value of Control Register for Conversion time 13.5ms
#define BMP180_OSS3        	0xF4 // Value of Control Register for Conversion time 25.5ms

#define BMP180_PRES        	0x34 // base value for pressure measurement (page 15 of datasheet)

#define ULTRA_LOW_POWER     	0
#define STANDARD        	1
#define HIGH_RESOLUTION      	2
#define ULTRA_HIGH_RESOLUTION  	3

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int bmp180_main(int argc, char *argv[]);


BMP180::BMP180(int bus) :
	I2C("BMP180", BARO_DEVICE_PATH, bus, 0, 400000),
	_measure_ticks(0),
	_num_reports(0),
	_next_report(0),
	_oldest_report(0),
	_reports(nullptr),
	_collect_phase(false),
	_measure_phase(0),
	_TEMP(0),
	_OFF(0),
	_SENS(0),
	_msl_pressure(101325),
	_baro_topic(-1),
	_sample_perf(perf_alloc(PC_ELAPSED, "bmp180_read")),
	_comms_errors(perf_alloc(PC_COUNT, "bmp180_comms_errors")),
	_buffer_overflows(perf_alloc(PC_COUNT, "bmp180_buffer_overflows"))
{
	// enable debug() calls
	_debug_enabled = true;

	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

BMP180::~BMP180()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr)
		delete[] _reports;
}

int
BMP180::init()
{
	int ret = ERROR;

	/* do I2C init (and probe) first */
	if (I2C::init() != OK)
		goto out;

	/* allocate basic report buffers */
	_num_reports = 2;
	_reports = new struct baro_report[_num_reports];

	if (_reports == nullptr)
		goto out;

	_oldest_report = _next_report = 0;

	/* get a publish handle on the baro topic */
	memset(&_reports[0], 0, sizeof(_reports[0]));
	_baro_topic = orb_advertise(ORB_ID(sensor_baro), &_reports[0]);

	if (_baro_topic < 0)
		debug("failed to create sensor_baro object");

	ret = OK;
out:
	return ret;
}

int
BMP180::probe()
{
	_retries = 10;

	if (OK == probe_address(BMP180_ADDRESS)) {
		_retries = 1;
		return OK;
	}

	return -EIO;
}

int
BMP180::probe_address(uint8_t address)
{
	int	ret;
	uint8_t bmp_txbuf[2];

	/* select the address we are going to try */
	set_address(address);

	/* send reset command */
	if (OK != cmd_reset())
		return -EIO;

	/* read PROM */
	if (OK != read_prom())
		return -EIO;

	_data.DELAY_TEMP = 5000;
	_data.OSS = STANDARD;
	_data.MEASUREMENT_MODE = BMP180_OSS1;
	_data.DELAY_PRESSURE = 9000;

	bmp_txbuf[0] =  BMP180_CTL;
	bmp_txbuf[1] = _data.MEASUREMENT_MODE;
	ret = transfer(bmp_txbuf, 2, nullptr, 0);
	if (OK != ret)
		perf_count(_comms_errors);
	usleep(_data.DELAY_PRESSURE);

	return OK;
}

ssize_t
BMP180::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct baro_report);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1)
		return -ENOSPC;

	/* if automatic measurement is enabled */
	if (_measure_ticks > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_oldest_report != _next_report) {
				memcpy(buffer, _reports + _oldest_report, sizeof(*_reports));
				ret += sizeof(_reports[0]);
				INCREMENT(_oldest_report, _num_reports);
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	/* XXX really it'd be nice to lock against other readers here */
	do {
		_measure_phase = 0;
		_oldest_report = _next_report = 0;

		/* do temperature first */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		usleep(_data.DELAY_TEMP);

		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* now do a pressure measurement */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		usleep(_data.DELAY_PRESSURE);

		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* state machine will have generated a report, copy it out */
		memcpy(buffer, _reports, sizeof(*_reports));
		ret = sizeof(*_reports);

	} while (0);

	return ret;
}

int
BMP180::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

				/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_measure_ticks = 0;
				return OK;

				/* external signalling not supported */
			case SENSOR_POLLRATE_EXTERNAL:

				/* zero would be bad */
			case 0:
				return -EINVAL;

				/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(_data.DELAY_PRESSURE);

					/* if we need to start the poll state machine, do it */
					if (want_start)
						start();

					return OK;
				}

				/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					unsigned ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(_data.DELAY_PRESSURE))
						return -EINVAL;

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start)
						start();

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_measure_ticks == 0)
			return SENSOR_POLLRATE_MANUAL;

		return (1000 / _measure_ticks);

	case SENSORIOCSQUEUEDEPTH: {
			/* add one to account for the sentinel in the ring */
			arg++;

			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 2) || (arg > 100))
				return -EINVAL;

			/* allocate new buffer */
			struct baro_report *buf = new struct baro_report[arg];

			if (nullptr == buf)
				return -ENOMEM;

			/* reset the measurement state machine with the new buffer, free the old */
			stop();
			delete[] _reports;
			_num_reports = arg;
			_reports = buf;
			start();

			return OK;
		}

	case SENSORIOCGQUEUEDEPTH:
		return _num_reports - 1;

	case SENSORIOCRESET:
		/* XXX implement this */
		return -EINVAL;

	case BAROIOCSMSLPRESSURE:

		/* range-check for sanity */
		if ((arg < 80000) || (arg > 120000))
			return -EINVAL;

		_msl_pressure = arg;
		return OK;

	case BAROIOCGMSLPRESSURE:
		return _msl_pressure;

	default:
		break;
	}

	/* give it to the superclass */
	return I2C::ioctl(filp, cmd, arg);
}

void
BMP180::start()
{

	/* reset the report ring and state machine */
	_collect_phase = false;
	_measure_phase = 0;
	_oldest_report = _next_report = 0;

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&BMP180::cycle_trampoline, this, 1);
}

void
BMP180::stop()
{
	work_cancel(HPWORK, &_work);
}

void
BMP180::cycle_trampoline(void *arg)
{
	BMP180 *dev = (BMP180 *)arg;

	dev->cycle();
}

void
BMP180::cycle()
{

	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		if (OK != collect()) {
			log("collection error");
			/* reset the collection state machine and try again */
			start();
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;

		/*
		 * Is there a collect->measure gap?
		 * Don't inject one after temperature measurements, so we can keep
		 * doing pressure measurements at something close to the desired rate.
		 */
		if ((_measure_phase != 0) &&
		    (_measure_ticks > USEC2TICK(_data.DELAY_PRESSURE))) {

			/* schedule a fresh cycle call when we are ready to measure again */
			work_queue(HPWORK,
				   &_work,
				   (worker_t)&BMP180::cycle_trampoline,
				   this,
				   _measure_ticks - USEC2TICK(_data.DELAY_PRESSURE));

			return;
		}
	}

	/* measurement phase */
	if (OK != measure()) {
		/* 
		 * We failed to send the I2C command to start the next
		 * reading. Hopefully this is a transient bus
		 * error. Schedule a fresh cycle call to try the
		 * command again in one tick
		 */		
		log("measure error");
		work_queue(HPWORK,
			   &_work,
			   (worker_t)&BMP180::cycle_trampoline,
			   this, 1);
		return;
	}

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&BMP180::cycle_trampoline,
		   this,
		   USEC2TICK(_data.DELAY_PRESSURE));
}

int
BMP180::measure()
{
	int ret;

	/*
	 * In phase zero, request temperature; in other phases, request pressure.
	 */
	//uint8_t	cmd_data = (_measure_phase == 0) ? ADDR_CMD_CONVERT_D2 : ADDR_CMD_CONVERT_D1;
	uint8_t bmp_txbuf[2];

	bmp_txbuf[0] = BMP180_CTL;
	bmp_txbuf[1] = (_measure_phase == 0) ? BMP180_TEMP : (BMP180_PRES + (_data.OSS << 6));

	/*
	 * Send the command to begin measuring.
	 */
	ret = transfer(bmp_txbuf, 2, nullptr, 0);

	if (OK != ret)
		perf_count(_comms_errors);

	return ret;
}

int
BMP180::collect()
{
  	uint8_t bmp_txbuf[1], bmp_rxbuf[3] ;
	
	perf_begin(_sample_perf);

	/* this should be fairly close to the end of the conversion, so the best approximation of the time */
	_reports[_next_report].timestamp = hrt_absolute_time();

//	if (OK != transfer(&cmd, 1, &data[0], 3)) {
//		perf_count(_comms_errors);
//		return -EIO;
//   	}

    	bmp_txbuf[0] = BMP180_ADC_MSB ;
	/* handle a measurement */
	if (_measure_phase == 0) {
    		uint32_t ut ;
    		int32_t x1, x2, b5 ;
		if (OK != transfer(bmp_txbuf, 1, bmp_rxbuf, 2)) {
	              perf_count(_comms_errors);
        	      return -EIO;
		}

		ut = (bmp_rxbuf[0] << 8) + bmp_rxbuf[1] ;
    		/*All calculations here are black magic from the
     		*BMP180 Datasheet. DO NOT CHANGE.
     		*/
    		x1 = (ut - _data.AC6) * _data.AC5 >> 15 ;
    		x2 = ((int32_t)_data.MC << 11) / (x1 + _data.MD) ;
    		b5 = x1 + x2 ;
    		_data.TEMP = (float)((b5 + 8) >> 4) / 10.0f;
		_TEMP = _data.TEMP*100;
		_UT = ut;
  	} else {
		int32_t x1, x2, x3, b3, b5, b6, p ;
    		uint32_t b4, b7, up, ut ;

		if (OK != transfer(bmp_txbuf, 1, bmp_rxbuf, 3)) {
                      perf_count(_comms_errors);
                      return -EIO;
		}
		
		ut = _UT;		
		up = ((bmp_rxbuf[0] << 16) + (bmp_rxbuf[1] << 8) + bmp_rxbuf[2]) >> (8 - _data.OSS) ;
      		x1 = (ut - _data.AC6) * _data.AC5 >> 15 ;
      		x2 = ((int32_t)_data.MC << 11) / (x1 + _data.MD) ;
      		b5 = x1 + x2 ;
      		
		b6 = b5 - 4000 ;
      		x1 = (_data.B2 * (b6 * b6 >> 12)) >> 11 ;
      		x2 = _data.AC2 * b6 >> 11 ;
      		x3 = x1 + x2 ;
      		b3 = ((((int32_t)_data.AC1 * 4 + x3) << _data.OSS) + 2) >> 2 ;

      		x1 = _data.AC3 * b6 >> 13 ;
      		x2 = (_data.B1 * (b6 * b6 >> 12)) >> 16 ;
      		x3 = ((x1 + x2) + 2) >> 2 ;
      		b4 = (_data.AC4 * (uint32_t)(x3 + 32768)) >> 15 ;
      		b7 = ((uint32_t)up - b3) * (50000 >> _data.OSS) ;

      		if(b7 < 0x80000000)
        		p = (b7 * 2) / b4 ;
      		else
        		p = (b7 / b4) * 2 ;

      		x1 = (p >> 8) * (p >> 8) ;
      		x1 = (x1 * 3038) >> 16 ;
      		x2 = (-7357 * p) >> 16 ;
      		_data.PRESSURE = (float)(p + ((x1 + x2 + 3791) >> 4)) ;

		/* pressure calculation, result in Pa */
		int32_t P = _data.PRESSURE;

		/* generate a new report */
		_reports[_next_report].temperature = _TEMP / 100.0f;
		_reports[_next_report].pressure = P / 100.0f;		/* convert to millibar */

		/* altitude calculations based on http://www.kansasflyer.org/index.asp?nav=Avi&sec=Alti&tab=Theory&pg=1 */

		/*
		 * PERFORMANCE HINT:
		 *
		 * The single precision calculation is 50 microseconds faster than the double
		 * precision variant. It is however not obvious if double precision is required.
		 * Pending more inspection and tests, we'll leave the double precision variant active.
		 *
		 * Measurements:
		 * 	double precision: bmp180_read: 992 events, 258641us elapsed, min 202us max 305us
		 *	single precision: bmp180_read: 963 events, 208066us elapsed, min 202us max 241us
		 */
#if 0/* USE_FLOAT */
		/* tropospheric properties (0-11km) for standard atmosphere */
		const float T1 = 15.0f + 273.15f;	/* temperature at base height in Kelvin */
		const float a  = -6.5f / 1000f;	/* temperature gradient in degrees per metre */
		const float g  = 9.80665f;	/* gravity constant in m/s/s */
		const float R  = 287.05f;	/* ideal gas constant in J/kg/K */

		/* current pressure at MSL in kPa */
		float p1 = _msl_pressure / 1000.0f;

		/* measured pressure in kPa */
		float p = P / 1000.0f;

		/*
		 * Solve:
		 *
		 *     /        -(aR / g)     \
		 *    | (p / p1)          . T1 | - T1
		 *     \                      /
		 * h = -------------------------------  + h1
		 *                   a
		 */
		_reports[_next_report].altitude = (((powf((p / p1), (-(a * R) / g))) * T1) - T1) / a;
#else
		/* tropospheric properties (0-11km) for standard atmosphere */
		const double T1 = 15.0 + 273.15;	/* temperature at base height in Kelvin */
		const double a  = -6.5 / 1000;	/* temperature gradient in degrees per metre */
		const double g  = 9.80665;	/* gravity constant in m/s/s */
		const double R  = 287.05;	/* ideal gas constant in J/kg/K */

		/* current pressure at MSL in kPa */
		double p1 = _msl_pressure / 1000.0;

		/* measured pressure in kPa */
		double pp = P / 1000.0;

		/*
		 * Solve:
		 *
		 *     /        -(aR / g)     \
		 *    | (p / p1)          . T1 | - T1
		 *     \                      /
		 * h = -------------------------------  + h1
		 *                   a
		 */
		_reports[_next_report].altitude = (((pow((pp / p1), (-(a * R) / g))) * T1) - T1) / a;
#endif
		/* publish it */
		orb_publish(ORB_ID(sensor_baro), _baro_topic, &_reports[_next_report]);

		/* post a report to the ring - note, not locked */
		INCREMENT(_next_report, _num_reports);

		/* if we are running up against the oldest report, toss it */
		if (_next_report == _oldest_report) {
			perf_count(_buffer_overflows);
			INCREMENT(_oldest_report, _num_reports);
		}

		/* notify anyone waiting for data */
		poll_notify(POLLIN);
	}

	/* update the measurement state machine */
	INCREMENT(_measure_phase, BMP180_MEASUREMENT_RATIO + 1);

	perf_end(_sample_perf);

	return OK;
}

int
BMP180::cmd_reset()
{
	unsigned	old_retrycount = _retries;
	uint8_t		bmp_txbuf[2];
	int		result;
	
	bmp_txbuf[0] = BMP180_SOFT_RESET; 
	bmp_txbuf[1] = 0xB6;

	/* bump the retry count */
	_retries = 10;
	result = transfer(bmp_txbuf, 2, nullptr, 0);
	_retries = old_retrycount;

	return result;
}

int
BMP180::read_prom()
{

	uint8_t bmp_txbuf[1], bmp_rxbuf[22] ;

	usleep(10000);
  
	bmp_txbuf[0] = 0xAA ;

	if (OK == transfer(bmp_txbuf, 1, bmp_rxbuf, 22)) {
		_data.AC1 = (bmp_rxbuf[0] << 8) + bmp_rxbuf[1] ;    /*AC1 */
   		_data.AC2 = (bmp_rxbuf[2] << 8) + bmp_rxbuf[3] ;    /*AC2 */
  		_data.AC3 = (bmp_rxbuf[4] << 8) + bmp_rxbuf[5] ;    /*AC3 */
  		_data.AC4 = (bmp_rxbuf[6] << 8) + bmp_rxbuf[7] ;    /*AC4 */
  		_data.AC5 = (bmp_rxbuf[8] << 8) + bmp_rxbuf[9] ;    /*AC5 */
  		_data.AC6 = (bmp_rxbuf[10] << 8) + bmp_rxbuf[11] ;    /*AC6 */
  		_data.B1  = (bmp_rxbuf[12] << 8) + bmp_rxbuf[13] ;    /*B1 */
  		_data.B2  = (bmp_rxbuf[14] << 8) + bmp_rxbuf[15] ;    /*B2 */
  		_data.MB  = (bmp_rxbuf[16] << 8) + bmp_rxbuf[17] ;    /*MB */
  		_data.MC  = (bmp_rxbuf[18] << 8) + bmp_rxbuf[19] ;    /*MC */
  		_data.MD  = (bmp_rxbuf[20] << 8) + bmp_rxbuf[21] ;    /*MD */
	}
	return OK;
}

void
BMP180::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_buffer_overflows);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	printf("report queue:   %u (%u/%u @ %p)\n",
	       _num_reports, _oldest_report, _next_report, _reports);
	printf("TEMP:           %d\n", _TEMP);
	printf("SENS:           %lld\n", _SENS);
	printf("OFF:            %lld\n", _OFF);
	printf("MSL pressure:   %10.4f\n", (double)(_msl_pressure / 100.f));
}

/**
 * Local functions in support of the shell command.
 */
namespace bmp180
{

BMP180	*g_dev;

void	start();
void	test();
void	reset();
void	info();
void	calibrate(unsigned altitude);

/**
 * Start the driver.
 */
void
start()
{
	int fd;

	if (g_dev != nullptr)
		errx(1, "already started");

	/* create the driver */
	g_dev = new BMP180(BMP180_BUS);

	if (g_dev == nullptr)
		goto fail;

	if (OK != g_dev->init())
		goto fail;

	/* set the poll rate to default, starts automatic data collection */
	fd = open(BARO_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		goto fail;

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
		goto fail;

	exit(0);

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{
	struct baro_report report;
	ssize_t sz;
	int ret;

	int fd = open(BARO_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		err(1, "%s open failed (try 'bmp180 start' if the driver is not running)", BARO_DEVICE_PATH);

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report))
		err(1, "immediate read failed");

	warnx("single read");
	warnx("pressure:    %10.4f", (double)report.pressure);
	warnx("altitude:    %11.4f", (double)report.altitude);
	warnx("temperature: %8.4f", (double)report.temperature);
	warnx("time:        %lld", report.timestamp);

	/* set the queue depth to 10 */
	if (OK != ioctl(fd, SENSORIOCSQUEUEDEPTH, 10))
		errx(1, "failed to set queue depth");

	/* start the sensor polling at 2Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2))
		errx(1, "failed to set 2Hz poll rate");

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 15; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1)
			errx(1, "timed out waiting for sensor data");

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report))
			err(1, "periodic read failed");

		warnx("periodic read %u", i);
		warnx("pressure:    %10.4f", (double)report.pressure);
		warnx("altitude:    %11.4f", (double)report.altitude);
		warnx("temperature: %8.4f", (double)report.temperature);
		warnx("time:        %lld", report.timestamp);
	}

	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(BARO_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		err(1, "failed ");

	if (ioctl(fd, SENSORIOCRESET, 0) < 0)
		err(1, "driver reset failed");

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
		err(1, "driver poll restart failed");

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	if (g_dev == nullptr)
		errx(1, "driver not running");

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

/**
 * Calculate actual MSL pressure given current altitude
 */
void
calibrate(unsigned altitude)
{
	struct baro_report report;
	float	pressure;
	float	p1;

	int fd = open(BARO_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		err(1, "%s open failed (try 'bmp180 start' if the driver is not running)", BARO_DEVICE_PATH);

	/* start the sensor polling at max */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MAX))
		errx(1, "failed to set poll rate");

	/* average a few measurements */
	pressure = 0.0f;

	for (unsigned i = 0; i < 20; i++) {
		struct pollfd fds;
		int ret;
		ssize_t sz;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 1000);

		if (ret != 1)
			errx(1, "timed out waiting for sensor data");

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report))
			err(1, "sensor read failed");

		pressure += report.pressure;
	}

	pressure /= 20;		/* average */
	pressure /= 10;		/* scale from millibar to kPa */

	/* tropospheric properties (0-11km) for standard atmosphere */
	const float T1 = 15.0 + 273.15;	/* temperature at base height in Kelvin */
	const float a  = -6.5 / 1000;	/* temperature gradient in degrees per metre */
	const float g  = 9.80665f;	/* gravity constant in m/s/s */
	const float R  = 287.05f;	/* ideal gas constant in J/kg/K */

	warnx("averaged pressure %10.4fkPa at %um", pressure, altitude);

	p1 = pressure * (powf(((T1 + (a * (float)altitude)) / T1), (g / (a * R))));

	warnx("calculated MSL pressure %10.4fkPa", p1);

	/* save as integer Pa */
	p1 *= 1000.0f;

	if (ioctl(fd, BAROIOCSMSLPRESSURE, (unsigned long)p1) != OK)
		err(1, "BAROIOCSMSLPRESSURE");

	exit(0);
}

} // namespace

int
bmp180_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start"))
		bmp180::start();

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test"))
		bmp180::test();

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset"))
		bmp180::reset();

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info"))
		bmp180::info();

	/*
	 * Perform MSL pressure calibration given an altitude in metres
	 */
	if (!strcmp(argv[1], "calibrate")) {
		if (argc < 2)
			errx(1, "missing altitude");

		long altitude = strtol(argv[2], nullptr, 10);

		bmp180::calibrate(altitude);
	}

	errx(1, "unrecognised command, try 'start', 'test', 'reset' or 'info'");
}
