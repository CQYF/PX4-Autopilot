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

#include "HydroAllocator.hpp"

using namespace time_literals;

HydroAllocator::HydroAllocator() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	for (int i = 0; i < 2; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "HY_RT%u_IDX", i);
		_param_handles.hy_rt_idx[i] = param_find(buffer);
	}
	for (int i = 0; i < 2; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "HY_SV%u_IDX", i);
		_param_handles.hy_sv_idx[i] = param_find(buffer);
	}
	for (int i = 0; i < 2; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "HY_VZRT%u_PIT_R", i);
		_param_handles.hy_vzrt_pit_r[i] = param_find(buffer);
	}
	for (int i = 0; i < 2; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "HY_VZRT%u_YAW_R", i);
		_param_handles.hy_vzrt_yaw_r[i] = param_find(buffer);
	}
	for (int i = 0; i < 2; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "HY_VZRT%u_ROL_R", i);
		_param_handles.hy_vzrt_rol_r[i] = param_find(buffer);
	}
	for (int i = 0; i < 2; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "HY_VXRT%u_PIT_R", i);
		_param_handles.hy_vxrt_pit_r[i] = param_find(buffer);
	}
	for (int i = 0; i < 2; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "HY_VXRT%u_YAW_R", i);
		_param_handles.hy_vxrt_yaw_r[i] = param_find(buffer);
	}
	for (int i = 0; i < 2; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "HY_VXRT%u_ROL_R", i);
		_param_handles.hy_vxrt_rol_r[i] = param_find(buffer);
	}


	parameters_update(true);
}

HydroAllocator::~HydroAllocator()
{
	perf_free(_loop_perf);
}

bool
HydroAllocator::init()
{
	if (!_hydro_torque_setpoint_sub.registerCallback() || !_hydro_thrust_setpoint_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void
HydroAllocator::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();

		//更新多实例参数
		for (int i = 0; i < 2; ++i) {
			param_get(_param_handles.hy_rt_idx[i], &_params.hy_rt_idx[i]);
			param_get(_param_handles.hy_sv_idx[i], &_params.hy_sv_idx[i]);
			param_get(_param_handles.hy_vzrt_pit_r[i], &_params.hy_vzrt_pit_r[i]);
			param_get(_param_handles.hy_vzrt_yaw_r[i], &_params.hy_vzrt_yaw_r[i]);
			param_get(_param_handles.hy_vzrt_rol_r[i], &_params.hy_vzrt_rol_r[i]);
			param_get(_param_handles.hy_vxrt_pit_r[i], &_params.hy_vxrt_pit_r[i]);
			param_get(_param_handles.hy_vxrt_yaw_r[i], &_params.hy_vxrt_yaw_r[i]);
			param_get(_param_handles.hy_vxrt_rol_r[i], &_params.hy_vxrt_rol_r[i]);
		}
	}
}

void HydroAllocator::Run()
{
	if (should_exit()) {
		_hydro_torque_setpoint_sub.unregisterCallback();
		_hydro_thrust_setpoint_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	if (_hydro_torque_setpoint_sub.update(&_hydro_torque_setpoint_msg))
	{
		;
	}
	if (_hydro_thrust_setpoint_sub.update(&_hydro_thrust_setpoint_msg))
	{
		;
	}
	//0是左边，1是右边
	// float thrust_x[2];
	// float thrust_z[2];
	// thrust_x[0] = _hydro_thrust_setpoint_msg.xyz[0] / 2;
	// thrust_x[1] = thrust_x[0];
	// thrust_z[0] = _hydro_thrust_setpoint_msg.xyz[2] / 2;
	// thrust_z[1] = thrust_z[0];
	// float yaw_ratio[2];
	// float pit_ratio[2];
	// float rol_ratio[2];




	parameters_update();

	// backup schedule
	ScheduleDelayed(100_ms);

	perf_end(_loop_perf);
}

int HydroAllocator::task_spawn(int argc, char *argv[])
{
	HydroAllocator *instance = new HydroAllocator();

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

int HydroAllocator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int HydroAllocator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
HydroAllocator.

)DESCR_STR");

	return 0;
}

extern "C" __EXPORT int hydro_allocator_main(int argc, char *argv[])
{
	return HydroAllocator::main(argc, argv);
}
