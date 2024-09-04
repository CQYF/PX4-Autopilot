/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include "VTrack.hpp"

#include <matrix/math.hpp>

using namespace matrix;

using namespace time_literals;

VTrack::VTrack() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_visual_buffer = new RingBuffer<VisualSample>(5);
	parameters_update(true);
}

VTrack::~VTrack()
{
	delete _visual_buffer;
	perf_free(_loop_perf);
}

bool
VTrack::init()
{
	if (!_vehicle_visual_odometry_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void
VTrack::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}
}

void VTrack::Run()
{
	if (should_exit()) {
		_vehicle_visual_odometry_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	if (_vehicle_visual_odometry_sub.update(&_vehicle_visual_odometry_msg))
	{
		VisualSample sample;
		sample.time_us = _vehicle_visual_odometry_msg.timestamp;
		sample.timestamp_sample = _vehicle_visual_odometry_msg.timestamp_sample;
		memcpy(sample.position, _vehicle_visual_odometry_msg.position, 3 * sizeof(float));
		memcpy(sample.q, _vehicle_visual_odometry_msg.q, 4 * sizeof(float));
		if(_first_write)
		{
			_visual_buffer->push(sample);
			_first_write = false;
		}
		else
		{
			VisualSample last_sample = _visual_buffer->get_newest();
			float dt = (float)(sample.timestamp_sample - last_sample.timestamp_sample) / 1000000.0f;

			if (dt > 0.0f) {
				// 速度计算
				Vector3f curr_p(sample.position);
				Vector3f last_p(last_sample.position);
				Vector3f curr_v = (curr_p - last_p) * (1.0f / dt);

				//四元数微分
				Quatf curr_q(sample.q);
				Quatf last_q(last_sample.q);
				Quatf delta_q = curr_q * last_q.inversed();
				Vector3f curr_w = delta_q.imag() * (2.0f / dt);

				//EMA
				Vector3f last_v(_diffed_visual_odometry_msg.velocity);
				Vector3f last_w(_diffed_visual_odometry_msg.angular_velocity);

				float alpha = _param_vtrack_alpha.get();

				Vector3f filted_v = (curr_v * alpha) + (last_v * (1-alpha));
				Vector3f filted_w = (curr_w * alpha) + (last_w * (1-alpha));

				//赋值
				_diffed_visual_odometry_msg = _vehicle_visual_odometry_msg;
				filted_v.copyTo(_diffed_visual_odometry_msg.velocity);
				filted_w.copyTo(_diffed_visual_odometry_msg.angular_velocity);

				_visual_buffer->push(sample);

				//发布
				_diffed_visual_odometry_pub.publish(_diffed_visual_odometry_msg);
			}
		}
	}

	parameters_update();

	// backup schedule
	ScheduleDelayed(20_ms);

	perf_end(_loop_perf);
}

int VTrack::task_spawn(int argc, char *argv[])
{
	VTrack *instance = new VTrack();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int VTrack::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int VTrack::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
vtrack.

)DESCR_STR");

	return 0;
}

extern "C" __EXPORT int vtrack_main(int argc, char *argv[])
{
	return VTrack::main(argc, argv);
}
