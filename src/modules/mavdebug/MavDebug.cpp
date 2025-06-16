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

#include "MavDebug.hpp"

using namespace time_literals;

MavDebug::MavDebug() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	parameters_update(true);
}

MavDebug::~MavDebug()
{
	perf_free(_loop_perf);
}

bool
MavDebug::init()
{
	if (! _slide_estimated_sub.registerCallback() || ! _hydro_depth_control_message_sub.registerCallback()) {
		_slide_estimated_sub.unregisterCallback();
		_hydro_depth_control_message_sub.unregisterCallback();
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void
MavDebug::parameters_update(bool force)
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

void MavDebug::Run()
{
	if (should_exit()) {
		_slide_estimated_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	slide_estimated_s slide_estimated;
	if (_slide_estimated_sub.update(&slide_estimated))
	{
		_debug_array.data[0] =  slide_estimated.lv_measure;
		_debug_array.data[1] =  slide_estimated.pr_measure;
		_debug_array.data[2] =  slide_estimated.acc_measure;
		_debug_array.data[3] =  slide_estimated.x1_fusion;
		_debug_array.data[4] =  slide_estimated.x2_fusion;
		_debug_array.data[5] =  slide_estimated.x3_fusion;
		_debug_array.data[6] =  slide_estimated.x4_fusion;
	}

	hydro_depth_control_message_s hydro_depth_control_message;
	if (_hydro_depth_control_message_sub.update(&hydro_depth_control_message))
	{
		_debug_array.data[10] = hydro_depth_control_message.depth;
		_debug_array.data[11] = hydro_depth_control_message.depth_rate;
		_debug_array.data[12] = hydro_depth_control_message.depth_setpoint;
		_debug_array.data[13] = hydro_depth_control_message.throttle_limited;
		_debug_array.data[14] = hydro_depth_control_message.horizontal_thrust;

		//非光滑反馈
		_debug_array.data[20] = hydro_depth_control_message.nsf_e1;
		_debug_array.data[21] = hydro_depth_control_message.nsf_e2;
		_debug_array.data[22] = hydro_depth_control_message.nsf_s;
		_debug_array.data[23] = hydro_depth_control_message.nsf_s_norm;
		_debug_array.data[24] = hydro_depth_control_message.nsf_s_norm_power;
		_debug_array.data[25] = hydro_depth_control_message.nsf_u;
		_debug_array.data[26] = hydro_depth_control_message.nsf_u_limited;

		//高阶滑模
		_debug_array.data[30] = hydro_depth_control_message.hsmc_x1;
		_debug_array.data[31] = hydro_depth_control_message.hsmc_x2;
		_debug_array.data[32] = hydro_depth_control_message.hsmc_x3;
		_debug_array.data[33] = hydro_depth_control_message.hsmc_surface;
		_debug_array.data[34] = hydro_depth_control_message.hsmc_u;

		//增益调度控制
		_debug_array.data[41] = hydro_depth_control_message.gsc_err;
		_debug_array.data[42] = hydro_depth_control_message.gsc_gain_b;
		_debug_array.data[43] = hydro_depth_control_message.gsc_gain_v1;
		_debug_array.data[44] = hydro_depth_control_message.gsc_gain_v2;
		_debug_array.data[45] = hydro_depth_control_message.gsc_gain_t;
		_debug_array.data[46] = hydro_depth_control_message.gsc_gain;
		_debug_array.data[47] = hydro_depth_control_message.gsc_gain_limited;
		_debug_array.data[48] = hydro_depth_control_message.gsc_u;
		_debug_array.data[49] = hydro_depth_control_message.gsc_u_limited;


		_debug_array.data[50] = hydro_depth_control_message.vertical_thrust;
		_debug_array.data[51] = hydro_depth_control_message.vertical_thrust_limited;
	}

	_debug_array.id = 1;
	_debug_array.timestamp = hrt_absolute_time();
	_debug_array_pub.publish(_debug_array);


	parameters_update();

	// backup schedule
	ScheduleDelayed(100_ms);

	perf_end(_loop_perf);
}

int MavDebug::task_spawn(int argc, char *argv[])
{
	MavDebug *instance = new MavDebug();

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

int MavDebug::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MavDebug::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
MavDebug.

)DESCR_STR");

	return 0;
}

extern "C" __EXPORT int mavdebug_main(int argc, char *argv[])
{
	return MavDebug::main(argc, argv);
}
