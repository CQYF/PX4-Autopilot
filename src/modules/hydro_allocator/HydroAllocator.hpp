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

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/module_params.h>
#include <uORB/Publication.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionInterval.hpp>

#include <uORB/topics/parameter_update.h>

#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_servos.h>

#include <drivers/drv_hrt.h>

using namespace time_literals;

class HydroAllocator final : public ModuleBase<HydroAllocator>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	HydroAllocator();
	~HydroAllocator() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionCallbackWorkItem 	_hydro_torque_setpoint_sub{this, ORB_ID(hydro_torque_setpoint)};
	uORB::SubscriptionCallbackWorkItem 	_hydro_thrust_setpoint_sub{this, ORB_ID(hydro_thrust_setpoint)};

	vehicle_torque_setpoint_s		_hydro_torque_setpoint_msg{0};
	vehicle_thrust_setpoint_s		_hydro_thrust_setpoint_msg{0};

	uORB::Publication<actuator_motors_s>	_hydro_motors_pub{ORB_ID(hydro_motors)};
	uORB::Publication<actuator_servos_s>	_hydro_servos_pub{ORB_ID(hydro_servos)};

	actuator_motors_s			_hydro_motors_msg{0};
	actuator_servos_s			_hydro_servos_msg{0};

	perf_counter_t _loop_perf;

	void parameters_update(bool force = false);

	struct ParamHandles {
		param_t hy_rt_idx[2];
		param_t hy_sv_idx[2];
		param_t hy_vzrt_pit_r[2];
		param_t hy_vzrt_yaw_r[2];
		param_t hy_vzrt_rol_r[2];
		param_t hy_vxrt_pit_r[2];
		param_t hy_vxrt_yaw_r[2];
		param_t hy_vxrt_rol_r[2];
	};

	struct Params {
		float hy_rt_idx[2];
		float hy_sv_idx[2];
		float hy_vzrt_pit_r[2];
		float hy_vzrt_yaw_r[2];
		float hy_vzrt_rol_r[2];
		float hy_vxrt_pit_r[2];
		float hy_vxrt_yaw_r[2];
		float hy_vxrt_rol_r[2];
	};

	ParamHandles _param_handles{};
	Params _params{};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::HY_ALT_SPEED>) _param_hy_alt_speed,
		(ParamInt<px4::params::HY_SPEED_SELECT>) _param_hy_speed_select,
		(ParamFloat<px4::params::HY_RT_MAX_THRUST>) _param_hy_rt_max_thrust,
		(ParamFloat<px4::params::HY_WING_KL>) _param_hy_wing_kl
	)
};
