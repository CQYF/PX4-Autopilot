/****************************************************************************
 *
 *   Copyright (C) 2020-2021 PX4 Development Team. All rights reserved.
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
 * @file diffpressure_main.cpp
 * @author cy
 *
 * Driver for the DiffPressure connected via I2C.
 */

#include "DiffPressure.h"
#include <px4_platform_common/module.h>
// #include <drivers/drv_adc.h>

DiffPressure::DiffPressure(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": diff_pressure"))
{

}

DiffPressure::~DiffPressure()
{
	ScheduleClear();
	perf_free(_cycle_perf);
}

void DiffPressure::exit_and_cleanup()
{
	I2CSPIDriverBase::exit_and_cleanup();	// nothing to do
}

void DiffPressure::RunImpl()
{
	if (should_exit()) {
		PX4_INFO("stopping");
		return;	// stop and return immediately to avoid unexpected schedule from stopping procedure
	}

	perf_begin(_cycle_perf);

	loop();

	perf_end(_cycle_perf);
}

void DiffPressure::print_usage()
{

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Driver to enable an external DiffPressure connected via I2C.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("diffpressure", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x70);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

void DiffPressure::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_cycle_perf);
}

extern "C" int diffpressure_main(int argc, char *argv[])
{
	using ThisDriver = DiffPressure;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 400000;
	cli.i2c_address = 0x70;

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	//最后一个参数是某种id号，随便找了一个没有定义的，详见DRV_ADC_DEVTYPE_ADS1115
	BusInstanceIterator iterator(MODULE_NAME, cli, 0x13);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
