/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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

#pragma once

#include <stdint.h>
#include <drivers/device/i2c.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/topics/diff_pressure.h>
#include <uORB/PublicationMulti.hpp>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>

#define FAKE_ADDR    0x00

using namespace time_literals;

class DiffPressure : public device::I2C, public I2CSPIDriver<DiffPressure>
{
public:
	DiffPressure(const I2CSPIDriverConfig &config);
	~DiffPressure() override;

	int init() override;

	static void print_usage();

	void RunImpl();

	int probe() override;

	uint8_t get_device_address() const;

protected:

	void print_status() override;

	void exit_and_cleanup() override;

private:

	uORB::Publication<diff_pressure_s>		_diff_pressure_pub{ORB_ID(diff_pressure)};

	static const hrt_abstime	SAMPLE_INTERVAL{100_ms};
	static const uint8_t sensor_num{2};

	diff_pressure_s _diff_pressure{};

	perf_counter_t			_cycle_perf;

	typedef enum {
		MUX,
		SENSOR
	} AddStatus_t;

	AddStatus_t add_status;

	void loop(void);

	int setchannel(uint8_t ch);
	int readReg(uint8_t addr, uint8_t *buf, size_t len);
	int writeReg(uint8_t addr, uint8_t *buf, size_t len);

	void rwMux(void) {add_status = MUX;};
	void rwSensor(void) {add_status = SENSOR;};
};
