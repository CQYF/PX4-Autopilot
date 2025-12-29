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

#include "DiffPressure.h"
#include <cassert>

int DiffPressure::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		return ret;
	}

	uint8_t cnt = 0;

	while (cnt < sensor_num){
		setchannel(cnt);
		uint8_t buf = 0b01000000;
		ret = writeReg(0x10, &buf, 1);
		if (ret != PX4_OK) {
			PX4_ERR("writeReg failed (%i)", ret);
			return ret;
		}

		cnt++;
	}

	ScheduleOnInterval(SAMPLE_INTERVAL, SAMPLE_INTERVAL);

	return PX4_OK;
}

int DiffPressure::probe()
{
	uint8_t cnt = 0;

	while (cnt < sensor_num){
		setchannel(cnt);
		uint8_t buf;
		int ret = readReg(0x0F, &buf, 1); // 随便读了一下
		if (ret != PX4_OK) {
			DEVICE_DEBUG("readReg failed (%i)", ret);
			return ret;
		}

		cnt++;
	}

	return PX4_OK;
}

void DiffPressure::loop(void)
{
	uint8_t cnt = 0;

	while (cnt < sensor_num){
		setchannel(cnt);
		uint8_t buf[5] = {};
		int ret = readReg(0x28, buf, 5);
		if (ret == PX4_OK) {
			uint32_t p = 	(static_cast<uint32_t>(buf[2]) << 16) |
					(static_cast<uint32_t>(buf[1]) << 8)  |
					(static_cast<uint32_t>(buf[0]) << 0);
			uint32_t t = 	(static_cast<uint32_t>(buf[4]) << 8)  |
					(static_cast<uint32_t>(buf[3]) << 0);

			_diff_pressure.pressure[cnt] = static_cast<float>(p) / 4096.0f;
			_diff_pressure.temperature[cnt] = static_cast<float>(t) / 100.0f;


		}
		else {
			DEVICE_DEBUG("readReg failed (%i)", ret);
		}

		cnt++;
	}
	_diff_pressure.num = sensor_num;
	_diff_pressure.timestamp = hrt_absolute_time();
	_diff_pressure_pub.publish(_diff_pressure);
}

int DiffPressure::setchannel(uint8_t ch)
{
	rwMux();
	if(ch < 8)
	{
		uint8_t buf = 1 << ch;
		return transfer(&buf, 1, nullptr, 0);
	}
	else return PX4_ERROR;
}

int DiffPressure::readReg(uint8_t addr, uint8_t *buf, size_t len)
{
	rwSensor();
	return transfer(&addr, 1, buf, len);
}

int DiffPressure::writeReg(uint8_t addr, uint8_t *buf, size_t len)
{
	rwSensor();
	uint8_t buffer[len + 1];
	buffer[0] = addr;
	memcpy(buffer + 1, buf, sizeof(uint8_t)*len);
	return transfer(buffer, len + 1, nullptr, 0);
}
