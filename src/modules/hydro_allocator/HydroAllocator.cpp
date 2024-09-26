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

#include <lib/matrix/matrix/math.hpp>

using namespace time_literals;
using namespace matrix;

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

//待优化求解的方程，优化求解的目标就是让这个方程等于0
Vector2f HydroAllocator::func(Vector2f x, NfParams p)
{
	float alpha = x(0);
	float T = x(1);

	Vector2f out;

	out(0) = p.Fx - p.KL*p.v*p.v*(alpha+p.alpha0)*std::sin(p.alpha0) - T*std::cos(alpha);
	out(1) = p.Fz + p.KL*p.v*p.v*(alpha+p.alpha0)*std::cos(p.alpha0) + T*std::sin(alpha);

	return out;
}

//待优化方程的雅可比矩阵
SquareMatrix<float, 2> HydroAllocator::J_func(Vector2f x, NfParams p)
{
	float alpha = x(0);
	float T = x(1);

	SquareMatrix<float, 2> J;

	J(0, 0) = -p.KL*p.v*p.v*std::sin(p.alpha0) + T*std::sin(alpha);
	J(0, 1) = -std::cos(alpha);
	J(1, 0) = p.KL*p.v*p.v*std::cos(p.alpha0) + T*std::cos(alpha);
	J(1, 1) = std::sin(alpha);

	return J;
}

//使用牛顿法优化求解
void HydroAllocator::optim(float x_array[2], NfParams p)
{
	Vector2f x(x_array);
	SquareMatrix<float, 2> J;
	Vector2f func_out;
	SquareMatrix<float, 2> J_inv;
	Vector2f delta_x;
	Vector2f x_new;

	for(int i=0; i<10; i++)
	{
		J = J_func(x, p);
		func_out = func(x, p);

		inv(J, J_inv);

		delta_x = - J_inv * func_out;

		if(delta_x.norm_squared() < (float)1e-6)
			break;

		x_new = x + delta_x;

		x_new(0) = math::constrain(x_new(0), - _param_hy_wing_max_a.get(), _param_hy_wing_max_a.get());
		x_new(1) = math::constrain(x_new(1), 0.f, 100.0f);//TODO 此处的约束要改

		x = x_new;
	}

	x.copyTo(x_array);
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
	float thrust_x[2];
	float thrust_z[2];
	thrust_x[0] = _hydro_thrust_setpoint_msg.xyz[0] / 2;
	thrust_x[1] = thrust_x[0];
	thrust_z[0] = _hydro_thrust_setpoint_msg.xyz[2] / 2;
	thrust_z[1] = thrust_z[0];

	//执行器顺序：x0 z0 x1 z1
	//力矩顺序：  rol pit yaw
	float allocate_matrix_array[4][3] = {
		{_params.hy_vxrt_rol_r[0], _params.hy_vxrt_pit_r[0], _params.hy_vxrt_yaw_r[0]},
		{_params.hy_vzrt_rol_r[0], _params.hy_vzrt_pit_r[0], _params.hy_vzrt_yaw_r[0]},
		{_params.hy_vxrt_rol_r[1], _params.hy_vxrt_pit_r[1], _params.hy_vxrt_yaw_r[1]},
		{_params.hy_vzrt_rol_r[1], _params.hy_vzrt_pit_r[1], _params.hy_vzrt_yaw_r[1]}
	};

	//计算torque_setpoint导致的各虚拟执行器的期望推力变化量
	Matrix<float, 4, 3> allocate_matrix(allocate_matrix_array);
	Vector3f torque_vector(_hydro_torque_setpoint_msg.xyz);
	Vector4f delta_force_vector;

	delta_force_vector = allocate_matrix * torque_vector;

	thrust_x[0] += delta_force_vector(0);
	thrust_z[0] += delta_force_vector(1);
	thrust_x[1] += delta_force_vector(2);
	thrust_z[1] += delta_force_vector(3);

	//定义待优化求解的变量，第一行是左侧水翼，第二行是右侧水翼，第一列是攻角，第二列是推力
	//攻角的初始值为0，推力初始值为x方向期望推力的一半
	float x[2][2] = {
		{0, _hydro_thrust_setpoint_msg.xyz[0] / 2},
		{0, _hydro_thrust_setpoint_msg.xyz[0] / 2}
	};

	//填入方程的参数
	_nf_params.alpha0 = 0; //TODO

	_nf_params.KL = _param_hy_wing_kl.get();

	if(_param_hy_speed_select.get() == 0)//使用替代速度
	{
		_nf_params.v = _param_hy_alt_speed.get();
	}
	else
	{
		_nf_params.v = _param_hy_alt_speed.get();//TODO 暂时不支持真实速度，都用替代速度
	}

	//代入虚拟执行器的期望力，并进行优化求解
	_nf_params.Fx = thrust_x[0];
	_nf_params.Fz = thrust_z[0];
	optim(x[0], _nf_params);
	_nf_params.Fx = thrust_x[1];
	_nf_params.Fz = thrust_z[1];
	optim(x[1], _nf_params);

	//归一化，并再次限幅确保安全
	x[0][0] = math::constrain(x[0][0] / _param_hy_wing_max_a.get(), -1.f, 1.f);
	x[1][0] = math::constrain(x[1][0] / _param_hy_wing_max_a.get(), -1.f, 1.f);
	x[0][1] = math::constrain(x[0][1] / _param_hy_rt_max_thrust.get(), 0.f, 1.f);
	x[1][1] = math::constrain(x[1][1] / _param_hy_rt_max_thrust.get(), 0.f, 1.f);

	//根据参数设置的对应关系填入数据并发送
	actuator_motors_s hydro_motors_msg{0};
	actuator_servos_s hydro_servos_msg{0};

	hydro_motors_msg.timestamp = hrt_absolute_time();
	hydro_motors_msg.timestamp_sample = _hydro_thrust_setpoint_msg.timestamp_sample;

	hydro_servos_msg.timestamp = hrt_absolute_time();
	hydro_servos_msg.timestamp_sample = _hydro_torque_setpoint_msg.timestamp_sample;

	hydro_motors_msg.control[_params.hy_rt_idx[0]] = x[0][1];
	hydro_motors_msg.control[_params.hy_rt_idx[1]] = x[1][1];

	hydro_servos_msg.control[_params.hy_sv_idx[0]] = x[0][0];
	hydro_servos_msg.control[_params.hy_sv_idx[1]] = x[1][0];

	_hydro_motors_pub.publish(hydro_motors_msg);
	_hydro_servos_pub.publish(hydro_servos_msg);

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
