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

#include "WaterLevel.h"
#include <cassert>

int WaterLevel::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		return ret;
	}

	uint8_t fake_config[1] = {0};
	ret = writeReg(FAKE_ADDR, fake_config, 1);

	if (ret != PX4_OK) {
		PX4_ERR("writeReg failed (%i)", ret);
		return ret;
	}

	ScheduleOnInterval(SAMPLE_INTERVAL, SAMPLE_INTERVAL);

	return PX4_OK;
}

int WaterLevel::probe()
{
	uint8_t buf[9] = {};
	int ret = readReg(FAKE_ADDR, buf, 9);

	if (ret != PX4_OK) {
		DEVICE_DEBUG("readReg failed (%i)", ret);
		return ret;
	}

	return PX4_OK;
}

void WaterLevel::measure(void)
{
	uint8_t buf[9] = {};
	int ret = readReg(FAKE_ADDR, buf, 9);
	if (ret == PX4_OK) {
		_water_level.h = buf[0];
		_water_level.b[0] = 	(static_cast<uint32_t>(buf[4]) << 24) |
					(static_cast<uint32_t>(buf[3]) << 16) |
					(static_cast<uint32_t>(buf[2]) << 8) |
					(static_cast<uint32_t>(buf[1]) << 0);
		_water_level.b[1] = 	(static_cast<uint32_t>(buf[8]) << 24) |
					(static_cast<uint32_t>(buf[7]) << 16) |
					(static_cast<uint32_t>(buf[6]) << 8) |
					(static_cast<uint32_t>(buf[5]) << 0);
		_water_level.lv = 0.002f * static_cast<float>(_water_level.h);
		_water_level.timestamp = hrt_absolute_time();
		_water_level_pub.publish(_water_level);
	}
	else {
		DEVICE_DEBUG("readReg failed (%i)", ret);
	}
}

int WaterLevel::readReg(uint8_t addr, uint8_t *buf, size_t len)
{
	return transfer(&addr, 1, buf, len);
}

int WaterLevel::writeReg(uint8_t addr, uint8_t *buf, size_t len)
{
	uint8_t buffer[len + 1];
	buffer[0] = addr;
	memcpy(buffer + 1, buf, sizeof(uint8_t)*len);
	return transfer(buffer, len + 1, nullptr, 0);
}
