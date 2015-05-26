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
 * @file adc121_vspb.cpp
 * Driver for the ADC121_VSPB voltage sensor connected via I2C.
 *
 * Author: Amir Melzer  <amir.melzer@mavt.ethz.ch>
 * 		   Lorenz Meier <lm@inf.ethz.ch>
 *
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

#include <board_config.h>

#include <drivers/drv_hrt.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_voltage_current.h>


/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif


class ADC121_VSPB : public device::I2C
{
public:
	ADC121_VSPB(int bus);
	virtual ~ADC121_VSPB();

	virtual int		init();

	virtual ssize_t	read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

protected:
	virtual int		probe();

private:

	struct work_s				_work;
	unsigned					_measure_ticks;

	unsigned					_num_reports;
	volatile unsigned			_next_report;
	volatile unsigned			_oldest_report;
	struct adc121_vspb_report	*_reports;

	bool						_measurement_phase;

	float 						_voltage;

	float						_bias_cal_term;
	float						_SF_cal_term;

	orb_advert_t				_voltage_sensor_pb_topic;

	perf_counter_t				_sample_perf;
	perf_counter_t				_comms_errors;
	perf_counter_t				_buffer_overflows;

	adc121_cal_term				_adc121_vspb_cal_term;

	/**
	 * Test whether the device supported by the driver is present at a
	 * specific address.
	 *
	 * @param address	The I2C bus address to probe.
	 * @return			True if the device is present.
	 */
	int			probe_address(uint8_t address);

	/**
	 * Initialize the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time. It might make sense
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
	 * This is the measurement state machine. This function
	 * alternately starts a measurement, or collects the data from the
	 * previous measurement.
	 *
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
	 * Issue a voltage measurement command for the current state.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			voltage_measurement();

};

/* helper macro for handling report buffer indices */
#define INCREMENT(_x, _lim)	do { _x++; if (_x >= _lim) _x = 0; } while(0)

/* helper macro for arithmetic - returns the square of the argument */
#define POW2(_x)		((_x) * (_x))

/*
 * ADC121_VSPB internal constants and data structures.
 */

/* internal conversion time: 5 ms  */
//#define ADC121_VSPB_CONVERSION_INTERVAL	5000	/* microseconds */
#define ADC121_VSPB_CONVERSION_INTERVAL	100000	/* microseconds */ // set sample rate to 10 Hz -> write a function in the driver that can set it from the sensors.cpp

#define ADC121_VSPB_BUS			PX4_I2C_BUS_EXPANSION

#define ADC121_VSPB_ADDRESS		0x58    /* ADC121_VSPB I2C address 										   */

#define CONV_RES_ADD			0x00	/* Pointer to the address of the Conversion Result register 	   */
#define ALT_STAT_ADD			0x01	/* Pointer to the address of the Alert Status register	   		   */
#define CONF_ADD				0x02	/* Pointer to the address of the Configuration register 		   */
#define LOW_LIM_REG_ADD			0x03	/* Pointer to the address of the Low limit register 		  	   */
#define HIGH_LIM_REG_ADD		0x04	/* Pointer to the address of the High limit register 		  	   */
#define HYS_REG_ADD				0x05	/* Pointer to the address of the Hysteresis register 		  	   */
#define LOW_CONV_REG_ADD		0x06	/* Pointer to the address of the Lowest conversion register 	   */
#define HIGH_CONV_REG_ADD		0x07	/* Pointer to the address of the Highest conversion register 	   */

/* Measurement definitions: */

#define VOLTAGE_MAX				33.0f
#define VOLTAGE_MIN				-10.0f
#define VOLTAGE_FULLSCALE		3.3f
#define VOLTAGE_MEASUREMENT_RES	4096.0f
#define VOLTAGE_SCALLING		10.02654867f

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int adc121_vspb_main(int argc, char *argv[]);


ADC121_VSPB::ADC121_VSPB(int bus) :
	I2C("ADC121_VSPB", ADC121_VSPB_DEVICE_PATH, bus, 0, 100000),							/* set I2C rate to 100KHz */
	_measure_ticks(0),
	_num_reports(0),
	_next_report(0),
	_oldest_report(0),
	_reports(nullptr),
	_measurement_phase(false),
	_voltage(0),
	_voltage_sensor_pb_topic(-1),
	_sample_perf(perf_alloc(PC_ELAPSED, "ADC121_VSPB_read")),
	_comms_errors(perf_alloc(PC_COUNT, "ADC121_VSPB_comms_errors")),
	_buffer_overflows(perf_alloc(PC_COUNT, "ADC121_VSPB_buffer_overflows")),
	_adc121_vspb_cal_term{}
{
	_debug_enabled = true;

	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));

	_adc121_vspb_cal_term.bias = 0.0f;
	_adc121_vspb_cal_term.SF   = 1.0f;

	_bias_cal_term = 0.0f;
	_SF_cal_term = 1.0f;
}

ADC121_VSPB::~ADC121_VSPB()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr)
		delete[] _reports;
}

int
ADC121_VSPB::init()
{
	int ret = ERROR;

	/* do I2C init (and probe) first */
	if (I2C::init() != OK)
		goto out;

	/* allocate basic report buffers */
	_num_reports = 2;
	_reports = new struct adc121_vspb_report[_num_reports];

	if (_reports == nullptr)
		goto out;

	_oldest_report = _next_report = 0;

	/* get a publish handle on the adc121_vspb topic */
	memset(&_reports[0], 0, sizeof(_reports[0]));
	_voltage_sensor_pb_topic = orb_advertise(ORB_ID(sensor_adc121_vspb), &_reports[0]);

	if (_voltage_sensor_pb_topic < 0)
		debug("failed to create sensor_adc121_vspb object");

	ret = OK;
out:
	return ret;
}

int
ADC121_VSPB::probe()
{
	_retries = 10;

	if (OK == probe_address(ADC121_VSPB_ADDRESS)){
		_retries = 1;
		return OK;
	}

	return -EIO;
}

int
ADC121_VSPB::probe_address(uint8_t address)
{
	set_address(address);

	/* send reset command */
	if (OK != voltage_measurement())
		return -EIO;

	/* Initialization functions: */

	return OK;
}

ssize_t
ADC121_VSPB::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct adc121_vspb_report);
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
	do {
		_measurement_phase = 0;
		_oldest_report = _next_report = 0;

		/* Take a voltage measurement */

		if (OK != voltage_measurement()) {
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
ADC121_VSPB::ioctl(struct file *filp, int cmd, unsigned long arg)
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
					_measure_ticks = USEC2TICK(ADC121_VSPB_CONVERSION_INTERVAL);

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
					if (ticks < USEC2TICK(ADC121_VSPB_CONVERSION_INTERVAL))
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
			struct adc121_vspb_report *buf = new struct adc121_vspb_report[arg];

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
		/* reset the measurement state machine */
		stop();

		/* free any existing reports */
		if (_reports != nullptr)
			delete[] _reports;

		start();
		return OK;
		//return -EINVAL;

	 case VSPBIOCSSCALE: {
		 /* copy adc121_vspb bias and SF */
		 struct adc121_vspb_cal_term *s = (struct adc121_vspb_cal_term *) arg;
		 memcpy(&_adc121_vspb_cal_term, s, sizeof(_adc121_vspb_cal_term));
		 _bias_cal_term = _adc121_vspb_cal_term.bias;
		 _SF_cal_term = _adc121_vspb_cal_term.SF;
		 return OK;
	 	 }

	default:
		break;
	}

	/* give it to the superclass */
	return I2C::ioctl(filp, cmd, arg);
}

void
ADC121_VSPB::start()
{
	/* reset the report ring and state machine */
	_measurement_phase = true;
	_oldest_report = _next_report = 0;

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&ADC121_VSPB::cycle_trampoline, this, 1);
}

void
ADC121_VSPB::stop()
{
	work_cancel(HPWORK, &_work);
}

void
ADC121_VSPB::cycle_trampoline(void *arg)
{
	ADC121_VSPB *dev = (ADC121_VSPB *)arg;

	dev->cycle();
}

void
ADC121_VSPB::cycle()
{

	/* collection phase? */
	if (_measurement_phase) {
		/* perform voltage measurement */
		if (OK != voltage_measurement()) {
			start();
			return;
		}

		/* next phase is measurement */
		_measurement_phase = true;

		/* schedule a fresh cycle call when the measurement is done */
		work_queue(HPWORK,
			   &_work,
			   (worker_t)&ADC121_VSPB::cycle_trampoline,
			   this,
			   _measure_ticks);										// ADC121_VSPB_CONVERSION_INTERVAL replaced with _measure_ticks
	}
}

/* voltage measurement: */
int
ADC121_VSPB::voltage_measurement()
{
	uint8_t ptr = CONV_RES_ADD;
	uint8_t data[2];
	union {
		uint8_t	b[2];
		uint32_t w;
	} cvt;

	/* read the most recent measurement */
	perf_begin(_sample_perf);

	/* this should be fairly close to the end of the conversion, so the best approximation of the time */
	_reports[_next_report].timestamp = hrt_absolute_time();

	/* fetch the raw value */

	if (OK != transfer(&ptr, 1, &data[0], 2)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	/* assemble 16 bit value and convert from big endian (sensor) to little endian (MCU) */
	cvt.b[0] = data[1];
	cvt.b[1] = data[0];

	/* voltage calculation, result in [v] */
	_voltage = (((float)((uint16_t)(cvt.w & 0x0fff)) * VOLTAGE_SCALLING * VOLTAGE_FULLSCALE / VOLTAGE_MEASUREMENT_RES) - _bias_cal_term) * _SF_cal_term;

	if ( (_voltage > VOLTAGE_MAX) | (_voltage < VOLTAGE_MIN) ) {
			warnx("ADC121_VSPB: voltage measured by power board is out of range: %3.2f [v]", (double) _voltage);
			return -EIO;
	}
	//warnx("measured voltage by the ADC121 by the power board sensor: %3.2f [v]", (double) voltage);  				// remove display!!!!!!!!

	/* generate a new report */
		_reports[_next_report].voltage = _voltage;					/* report in [v] */

	/* publish it */
	orb_publish(ORB_ID(sensor_adc121_vspb), _voltage_sensor_pb_topic, &_reports[_next_report]);

	/* post a report to the ring - note, not locked */
	INCREMENT(_next_report, _num_reports);

	/* if we are running up against the oldest report, toss it */
	if (_next_report == _oldest_report) {
		perf_count(_buffer_overflows);
		INCREMENT(_oldest_report, _num_reports);
	}

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	perf_end(_sample_perf);

return OK;
}

void
ADC121_VSPB::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_buffer_overflows);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	printf("report queue:   %u (%u/%u @ %p)\n",
	       _num_reports, _oldest_report, _next_report, _reports);
	printf("voltage:   %10.4f\n", (double)(_voltage));
}

/**
 * Local functions in support of the shell command.
 */
namespace adc121_vspb
{

ADC121_VSPB	*g_dev;

void	start();
void	test();
void	reset();
void	info();

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
	g_dev = new ADC121_VSPB(ADC121_VSPB_BUS);

	if (g_dev == nullptr)
		goto fail;

	if (OK != g_dev->init())
		goto fail;

	/* set the poll rate to default, starts automatic data collection */
	fd = open(ADC121_VSPB_DEVICE_PATH, O_RDONLY);

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

void
test()
{
	struct adc121_vspb_report report;
	ssize_t sz;
	int ret;

	int fd = open(ADC121_VSPB_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		err(1, "%s open failed (try 'ADC121_VSPB start' if the driver is not running)", ADC121_VSPB_DEVICE_PATH);

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report))
		err(1, "immediate read failed");

	warnx("single read");
	warnx("voltage:   %10.4f", (double)report.voltage);
	warnx("time:        %lld", report.timestamp);

	/* set the queue depth to 10 */
	if (OK != ioctl(fd, SENSORIOCSQUEUEDEPTH, 10))
		errx(1, "failed to set queue depth");

	/* start the sensor polling at 2Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2))
		errx(1, "failed to set 2Hz poll rate");

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
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

		warnx("voltage:   %10.4f", (double)report.voltage);
		warnx("time:        %lld", report.timestamp);

	}

	reset();
	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(ADC121_VSPB_DEVICE_PATH, O_RDONLY);

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


} // namespace

int
adc121_vspb_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start"))
		adc121_vspb::start();

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test"))
		adc121_vspb::test();

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset"))
		adc121_vspb::reset();

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info"))
		adc121_vspb::info();

	errx(1, "unrecognised command, try 'start', 'test', 'reset' or 'info'");
}
