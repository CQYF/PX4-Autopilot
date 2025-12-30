/****************************************************************************
 *
 *   Copyright (c) 2013-2023 PX4 Development Team. All rights reserved.
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

#include "SpeedEstimator.hpp"
#include <lib/ai/X-CUBE-AI/App/app_x-cube-ai.h>

using namespace time_literals;

SpeedEstimator::SpeedEstimator() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	/* fetch initial parameter values */
	updateParams();
}

SpeedEstimator::~SpeedEstimator()
{
	perf_free(_loop_perf);
}

bool
SpeedEstimator::init()
{
	if (!_diff_pressure_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	MX_X_CUBE_AI_Init();
	ai_set_input_pointer(ai_inputs);
	ai_set_output_pointer(ai_outputs);

	return true;
}

void SpeedEstimator::Run()
{
	if (should_exit()) {
		_diff_pressure_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// only update parameters if they changed
	bool params_updated = _parameter_update_sub.updated();

	// check for parameter updates
	if (params_updated) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}

	diff_pressure_s diff_pressure;
	speed_estimated_s speed_estimated;
	if(_diff_pressure_sub.update(&diff_pressure)){

		//运行神经网络
		ai_inputs[0] = diff_pressure.pressure[0];
		ai_inputs[1] = diff_pressure.pressure[1];
		ai_inputs[2] = diff_pressure.pressure[2];
		ai_inputs[3] = diff_pressure.pressure[3];
		ai_inputs[4] = diff_pressure.pressure[4];

		MX_X_CUBE_AI_Process();

		speed_estimated.speed[0] = ai_outputs[0];
		speed_estimated.speed[1] = ai_outputs[1];
		speed_estimated.speed[2] = ai_outputs[2];

		speed_estimated.timestamp = hrt_absolute_time();
		_speed_estimated_pub.publish(speed_estimated);
	}

	// backup schedule
	ScheduleDelayed(10_ms);

	perf_end(_loop_perf);
}

int SpeedEstimator::task_spawn(int argc, char *argv[])
{
	SpeedEstimator *instance = new SpeedEstimator();

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

int SpeedEstimator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int SpeedEstimator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
speed_estimator module is used to estimate the speed.

)DESCR_STR");

	return 0;
}

extern "C" __EXPORT int speed_estimator_main(int argc, char *argv[])
{
	return SpeedEstimator::main(argc, argv);
}
