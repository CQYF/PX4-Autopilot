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

#include "HydroRateControl.hpp"

#include <include/HyModeName.hpp>
#include <math.h>

using namespace time_literals;
using namespace matrix;

using math::constrain;
using math::interpolate;
using math::radians;

HydroRateControl::HydroRateControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	/* fetch initial parameter values */
	parameters_update();
}

HydroRateControl::~HydroRateControl()
{
	perf_free(_loop_perf);
}

bool
HydroRateControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

int
HydroRateControl::parameters_update()
{
	const Vector3f rate_p = Vector3f(_param_hy_rr_p.get(), _param_hy_pr_p.get(), _param_hy_yr_p.get());
	const Vector3f rate_i = Vector3f(_param_hy_rr_i.get(), _param_hy_pr_i.get(), _param_hy_yr_i.get());
	const Vector3f rate_d = Vector3f(_param_hy_rr_d.get(), _param_hy_pr_d.get(), _param_hy_yr_d.get());

	_rate_control.setPidGains(rate_p, rate_i, rate_d);

	_rate_control.setIntegratorLimit(
		Vector3f(_param_hy_rr_imax.get(), _param_hy_pr_imax.get(), _param_hy_yr_imax.get()));

	_rate_control.setFeedForwardGain(
		// set FF gains to 0 as we add the FF control outside of the rate controller
		Vector3f(0.f, 0.f, 0.f));

	return PX4_OK;
}

float HydroRateControl::get_airspeed_and_update_scaling()
{
	// if no airspeed measurement is available out best guess is to use the trim airspeed
	float airspeed = _param_hy_airspd_trim.get();

	const float airspeed_constrained = constrain(constrain(airspeed, _param_hy_airspd_stall.get(),
					   _param_hy_airspd_max.get()), 0.1f, 1000.0f);

	_airspeed_scaling = (_param_hy_arsp_scale_en.get()) ? (_param_hy_airspd_trim.get() / airspeed_constrained) : 1.0f;

	return airspeed_constrained;
}

void HydroRateControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// only run controller if angular velocity changed
	if (_vehicle_angular_velocity_sub.updated() || (hrt_elapsed_time(&_last_run) > 20_ms)) {

		// only update parameters if they changed
		bool params_updated = _parameter_update_sub.updated();

		// check for parameter updates
		if (params_updated) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			updateParams();
			parameters_update();
		}

		float dt = 0.f;

		static constexpr float DT_MIN = 0.002f;
		static constexpr float DT_MAX = 0.04f;

		vehicle_angular_velocity_s vehicle_angular_velocity{};

		if (_vehicle_angular_velocity_sub.copy(&vehicle_angular_velocity)) {
			dt = math::constrain((vehicle_angular_velocity.timestamp_sample - _last_run) * 1e-6f, DT_MIN, DT_MAX);
			_last_run = vehicle_angular_velocity.timestamp_sample;
		}

		if (dt < DT_MIN || dt > DT_MAX) {
			const hrt_abstime time_now_us = hrt_absolute_time();
			dt = math::constrain((time_now_us - _last_run) * 1e-6f, DT_MIN, DT_MAX);
			_last_run = time_now_us;
		}

		vehicle_angular_velocity_s angular_velocity{};
		_vehicle_angular_velocity_sub.copy(&angular_velocity);

		Vector3f rates(angular_velocity.xyz);
		Vector3f angular_accel{angular_velocity.xyz_derivative};

		_vehicle_status_sub.update(&_vehicle_status);

		//读手动操控信息
		_manual_control_setpoint_sub.copy(&_manual_control_setpoint);

		//特技模式下，操控量映射到rate_sp
		if (_vehicle_status.nav_state == HYDRO_MODE_ACRO)
		{
			_rates_sp.roll = _manual_control_setpoint.roll * radians(_param_hy_acro_x_max.get());
			_rates_sp.yaw = _manual_control_setpoint.yaw * radians(_param_hy_acro_z_max.get());
			_rates_sp.pitch = -_manual_control_setpoint.pitch * radians(_param_hy_acro_y_max.get());
			_rates_sp.timestamp = hrt_absolute_time();
			_rates_sp.thrust_body[0] = (_manual_control_setpoint.throttle + 1.f) * .5f;

			_rate_sp_pub.publish(_rates_sp);

		}
		//手动模式下，操控量映射到torque和thrust
		else if(_vehicle_status.nav_state == HYDRO_MODE_MANUAL)
		{
			_vehicle_torque_setpoint.xyz[0] = math::constrain(_manual_control_setpoint.roll * _param_hy_man_r_sc.get() +
							_param_trim_roll.get(), -1.f, 1.f);
			_vehicle_torque_setpoint.xyz[1] = math::constrain(-_manual_control_setpoint.pitch * _param_hy_man_p_sc.get() +
							_param_trim_pitch.get(), -1.f, 1.f);
			_vehicle_torque_setpoint.xyz[2] = math::constrain(_manual_control_setpoint.yaw * _param_hy_man_y_sc.get() +
							_param_trim_yaw.get(), -1.f, 1.f);

			_vehicle_thrust_setpoint.xyz[0] = math::constrain((_manual_control_setpoint.throttle + 1.f) * .5f, 0.f, 1.f);
		}

		//在非手动的自定义模式下，执行控制算法
		if (_vehicle_status.nav_state == HYDRO_MODE_STABILIZED || _vehicle_status.nav_state == HYDRO_MODE_AUTO_DIVE || _vehicle_status.nav_state == HYDRO_MODE_ACRO) {

			const float airspeed = get_airspeed_and_update_scaling();

			/* reset integrals where needed */
			if (_rates_sp.reset_integral) {
				_rate_control.resetIntegral();
			}

			/* bi-linear interpolation over airspeed for actuator trim scheduling */
			Vector3f trim(_param_trim_roll.get(), _param_trim_pitch.get(), _param_trim_yaw.get());

			if (airspeed < _param_hy_airspd_trim.get()) {
				trim(0) += interpolate(airspeed, _param_hy_airspd_min.get(), _param_hy_airspd_trim.get(),
						       _param_hy_dtrim_r_vmin.get(),
						       0.0f);
				trim(1) += interpolate(airspeed, _param_hy_airspd_min.get(), _param_hy_airspd_trim.get(),
						       _param_hy_dtrim_p_vmin.get(),
						       0.0f);
				trim(2) += interpolate(airspeed, _param_hy_airspd_min.get(), _param_hy_airspd_trim.get(),
						       _param_hy_dtrim_y_vmin.get(),
						       0.0f);

			} else {
				trim(0) += interpolate(airspeed, _param_hy_airspd_trim.get(), _param_hy_airspd_max.get(), 0.0f,
						       _param_hy_dtrim_r_vmax.get());
				trim(1) += interpolate(airspeed, _param_hy_airspd_trim.get(), _param_hy_airspd_max.get(), 0.0f,
						       _param_hy_dtrim_p_vmax.get());
				trim(2) += interpolate(airspeed, _param_hy_airspd_trim.get(), _param_hy_airspd_max.get(), 0.0f,
						       _param_hy_dtrim_y_vmax.get());
			}

			_rates_sp_sub.update(&_rates_sp);

			Vector3f body_rates_setpoint = Vector3f(_rates_sp.roll, _rates_sp.pitch, _rates_sp.yaw);

			// Run attitude RATE controllers which need the desired attitudes from above, add trim.
			const Vector3f angular_acceleration_setpoint = _rate_control.update(rates, body_rates_setpoint, angular_accel, dt,
					false);

			const Vector3f gain_ff(_param_hy_rr_ff.get(), _param_hy_pr_ff.get(), _param_hy_yr_ff.get());
			const Vector3f feedforward = gain_ff.emult(body_rates_setpoint) * _airspeed_scaling;

			Vector3f control_u = angular_acceleration_setpoint * _airspeed_scaling * _airspeed_scaling + feedforward;

			// Special case yaw in Acro: if the parameter HY_ACRO_YAW_CTL is not set then don't control yaw
			if (_vehicle_status.nav_state == HYDRO_MODE_ACRO && !_param_hy_acro_yaw_en.get()) {
				control_u(2) = _manual_control_setpoint.yaw * _param_hy_man_y_sc.get();
				_rate_control.resetIntegral(2);
			}

			if (control_u.isAllFinite()) {
				matrix::constrain(control_u + trim, -1.f, 1.f).copyTo(_vehicle_torque_setpoint.xyz);

			} else {
				_rate_control.resetIntegral();
				trim.copyTo(_vehicle_torque_setpoint.xyz);
			}

			/* throttle passed through if it is finite */
			_vehicle_thrust_setpoint.xyz[0] = PX4_ISFINITE(_rates_sp.thrust_body[0]) ? _rates_sp.thrust_body[0] : 0.0f;

			/* scale effort by battery status */
			if (_param_hy_bat_scale_en.get() && _vehicle_thrust_setpoint.xyz[0] > 0.1f) {

				if (_battery_status_sub.updated()) {
					battery_status_s battery_status{};

					if (_battery_status_sub.copy(&battery_status) && battery_status.connected && battery_status.scale > 0.f) {
						_battery_scale = battery_status.scale;
					}
				}

				_vehicle_thrust_setpoint.xyz[0] *= _battery_scale;
			}

		} else {
			_rate_control.resetIntegral();
		}

		//在全部的自定义模式下，执行补偿操作和滑行深度控制
		if (_vehicle_status.nav_state == HYDRO_MODE_STABILIZED || _vehicle_status.nav_state == HYDRO_MODE_AUTO_DIVE ||
			_vehicle_status.nav_state == HYDRO_MODE_ACRO || _vehicle_status.nav_state == HYDRO_MODE_MANUAL)
		{
			// Add feed-forward from roll control output to yaw control output
			// This can be used to counteract the adverse yaw effect when rolling the plane
			_vehicle_torque_setpoint.xyz[2] = math::constrain(_vehicle_torque_setpoint.xyz[2] + _param_hy_rll_to_yaw_ff.get() *
							_vehicle_torque_setpoint.xyz[0], -1.f, 1.f);

			//推力前馈到pit轴力矩上
			_vehicle_torque_setpoint.xyz[1] = math::constrain(_vehicle_torque_setpoint.xyz[1] + _param_thr_to_pit_ff.get() *
							_vehicle_thrust_setpoint.xyz[0], -1.f, 1.f);

			//获取姿态
			_vehicle_attitude_sub.update(&_vehicle_attitude);
			Eulerf euler_angles(Quatf(_vehicle_attitude.q));
			//px4的算法忽略了机身的攻角对操纵面实际攻角的影响，认为操纵面的实际攻角始终等于操纵面相对于机身的角度，但这对于滑行控制而言是不可接受的，
			//飞机在滑行时机身总是保持一定的正攻角，此时水翼的0控制量也就对应了这个正攻角，在高速滑行时，水翼只需要很小的实际攻角就可以实现控制，
			//这个正攻角就会将实际平衡点向上大幅推移，并且一般会直接推移到平衡范围之外，也就出现了滑行中在水面上跳动的情况（当然这只是因素之一）
			//这项增益的目的是让水翼的攻角始终以水平面为0，实现原理是利用机身的实际pit角度（也就是机身的攻角）制造一个线性项，加到俯仰力矩当中
			//这项增益在手动模式下依然有效，并且由于控制分配机制的存在，会对小平尾产生附带影响。
			//! 这项增益会在起飞后造成麻烦（大概），暂时只用于滑行，如果要起飞，应当将增益置0
			//! 这项增益在手动模式下依然存在，调节增益系数的方法是在手动模式下俯仰飞机，直到在任何角度下水翼都保持水平
			// _vehicle_torque_setpoint.xyz[1] = math::constrain(_vehicle_torque_setpoint.xyz[1] + _param_attack_ff.get() *
			// 				euler_angles.theta(), -1.f, 1.f);
			//（使用新的控制和分配逻辑时，不需要此补偿）

			_depth_fusion_sub.update(&_depth_fusion);

			// TODO PID和PID参数
			float depth = _depth_fusion.fudepth * 0.01f;//换算单位
			float depth_setpoint = _param_hy_d_sp.get();

			//! 注意，水翼的推力计算都用真值，单位是N，但力矩仍用归一化值
			//水平推力，向前为正，和原来的推力一致，最大为水下推进器推力的2倍
			float hydro_horizontal_thrust_setpoint = _vehicle_thrust_setpoint.xyz[0] * 2 * _param_hy_rt_max_thrust.get();
			//竖直推力，向!下!为正，等于深度控制的输出加上重力补偿
			float hydro_vertical_thrust_setpoint = _param_hy_d_p.get() * (depth_setpoint - depth) + _param_hy_d_ff.get();
			//滑行时，机身的俯仰角近似为自然攻角，实际攻角等于翼面偏转角度加上自然攻角
			float alpha0 = euler_angles.theta();

			//水平和竖直推力转换为机身坐标系下的推力，使用二维坐标转换
			_hydro_thrust_setpoint.xyz[0] = hydro_horizontal_thrust_setpoint * std::cos(alpha0) - hydro_vertical_thrust_setpoint * std::sin(alpha0);
			_hydro_thrust_setpoint.xyz[2] = hydro_horizontal_thrust_setpoint * std::sin(alpha0) + hydro_vertical_thrust_setpoint * std::cos(alpha0);

			//y方向推力始终为0
			_hydro_thrust_setpoint.xyz[1] = 0;

			//力矩和原来保持一致
			_hydro_torque_setpoint = _vehicle_torque_setpoint;
		}

		//根据遥控器上某个辅助通道的状态，判断当前的运行模式
		if(_manual_control_setpoint.aux2 < -0.5f)
		{
			_hydro_running_state = HydroRunningState::WaterOnly;
		}
		else if(_manual_control_setpoint.aux2 > 0.5f)
		{
			_hydro_running_state = HydroRunningState::AirOnly;
		}
		else
		{
			_hydro_running_state = HydroRunningState::WaterAir;
		}

		//在全部的自定义模式下，要发布vehicle的setpoint（其余模式下px4自带的模块会发布setpoint），但在仅水下部分运行时要发送0
		if (_vehicle_status.nav_state == HYDRO_MODE_STABILIZED || _vehicle_status.nav_state == HYDRO_MODE_AUTO_DIVE ||
			_vehicle_status.nav_state == HYDRO_MODE_ACRO || _vehicle_status.nav_state == HYDRO_MODE_MANUAL)
		{
			if(_hydro_running_state == HydroRunningState::WaterOnly)
			{
				_vehicle_thrust_setpoint.xyz[0] = 0;
				_vehicle_thrust_setpoint.xyz[1] = 0;
				_vehicle_thrust_setpoint.xyz[2] = 0;
				_vehicle_torque_setpoint.xyz[0] = 0;
				_vehicle_torque_setpoint.xyz[1] = 0;
				_vehicle_torque_setpoint.xyz[2] = 0;
			}
			_vehicle_thrust_setpoint.timestamp = hrt_absolute_time();
			_vehicle_thrust_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			_vehicle_thrust_setpoint_pub.publish(_vehicle_thrust_setpoint);

			_vehicle_torque_setpoint.timestamp = hrt_absolute_time();
			_vehicle_torque_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			_vehicle_torque_setpoint_pub.publish(_vehicle_torque_setpoint);
		}

		//在全部的自定义模式下且水下部分需要运行时，正常发布hydro的setpoint；其他情况，也要发布0
		if((_vehicle_status.nav_state == HYDRO_MODE_STABILIZED || _vehicle_status.nav_state == HYDRO_MODE_AUTO_DIVE ||
			_vehicle_status.nav_state == HYDRO_MODE_ACRO || _vehicle_status.nav_state == HYDRO_MODE_MANUAL) && (_hydro_running_state != HydroRunningState::AirOnly))
		{
			;
		}
		else
		{
			_hydro_thrust_setpoint.xyz[0] = 0;
			_hydro_thrust_setpoint.xyz[1] = 0;
			_hydro_thrust_setpoint.xyz[2] = 0;
			_hydro_torque_setpoint.xyz[0] = 0;
			_hydro_torque_setpoint.xyz[1] = 0;
			_hydro_torque_setpoint.xyz[2] = 0;
		}
		_hydro_thrust_setpoint.timestamp = hrt_absolute_time();
		_hydro_thrust_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
		_hydro_thrust_setpoint_pub.publish(_hydro_thrust_setpoint);

		_hydro_torque_setpoint.timestamp = hrt_absolute_time();
		_hydro_torque_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
		_hydro_torque_setpoint_pub.publish(_hydro_torque_setpoint);
	}

	// backup schedule
	ScheduleDelayed(20_ms);

	perf_end(_loop_perf);
}

int HydroRateControl::task_spawn(int argc, char *argv[])
{
	HydroRateControl *instance = new HydroRateControl();

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

int HydroRateControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int HydroRateControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
hydro_rate_control is the water-air cross medium rate controller.

)DESCR_STR");

	/*PRINT_MODULE_USAGE_NAME("hydro_rate_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();*/

	return 0;
}

extern "C" __EXPORT int hydro_rate_control_main(int argc, char *argv[])
{
	return HydroRateControl::main(argc, argv);
}
