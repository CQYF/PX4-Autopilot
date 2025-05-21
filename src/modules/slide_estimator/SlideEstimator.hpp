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

#pragma once

#include <lib/rate_control/rate_control.hpp>

#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <lib/slew_rate/SlewRate.hpp>
#include <matrix/math.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>

#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/water_level.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>

#include <uORB/topics/slide_estimated.h>

#include <lib/mathlib/math/filter/MedianFilter.hpp>
#include <lib/mathlib/mathlib.h>
#include <lib/loosely_kalman_filter/loosely_kalman_filter.hpp>
#include <matrix/matrix/math.hpp>

// using uORB::SubscriptionData;

using namespace time_literals;
using namespace math;
using namespace matrix;

class SlideEstimator final : public ModuleBase<SlideEstimator>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	SlideEstimator();
	~SlideEstimator() override;

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

	uORB::SubscriptionCallbackWorkItem _sensor_baro_sub{this, ORB_ID(sensor_baro)};
	uORB::SubscriptionCallbackWorkItem _water_level_sub{this, ORB_ID(water_level)};
	uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub{this, ORB_ID(vehicle_attitude)};
	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};
	uORB::SubscriptionCallbackWorkItem _vehicle_acceleration_sub{this, ORB_ID(vehicle_acceleration)};

	uORB::Publication<slide_estimated_s>		_slide_estimated_pub{ORB_ID(slide_estimated)};

	perf_counter_t _loop_perf;

	LooselyKalmanFilter<double, 150, 4> _kf;

	Matrix<double, 4, 1> _x_out; //* 位置 速度 加速度 水位计累计误差
	Matrix<double, 4, 4> _P_out;

	static float hy_se_q_hgt;
	static float hy_se_q_err;
	static void calc_F(Matrix<double, 4, 4>& F, uint64_t& dt);
	static void calc_Q(Matrix<double, 4, 4>& Q, uint64_t& dt);

	static bool run_info;

	/**
	 * pr:压强计  lv:水位计
	 * 高度和深度是同一个东西，都是以水面为0，向下为正
	 * b系是机体坐标系，e系是地球坐标系（世界坐标系）
	 */

	// 用于检查深度计合法性的中值滤波器
	MedianFilter<float, 15> _medfilter_pr_depth;

	// 记录姿态，由于计算位置和速度都需要姿态数据，因此在这里存一份，当没有新数据时就用这份数据
	Quatf _q;

	// 记录角速度，计算速度需要角速度数据，因此在这里存一份，当没有新数据时就用这份数据
	Vector3f _w;

	// 水位计浸水长度
	float _lv_immersion;
	// 水位计饱和程度
	float _lv_satuation;
	// 水位计测量得到的高度
	float _lv_height;

	// 根据浸水长度计算水位计饱和程度评估值
	void calc_lv_saturation();
	// 根据浸水长度、姿态和几何关系计算高度
	void calc_lv_height();

	// 根据压强计测得的深度测量值
	float _pr_depth;
	// 根据压强计算出的中心高度
	float _pr_height;

	// 压强转换为深度
	float pressure2depth(float pressure);
	// 深度测量值的合法性检查
	bool is_pr_depth_legal();
	// 根据压强计深度、姿态和几何关系计算高度
	void calc_pr_height();


	// IMU测得的高度加速度
	float _imu_height_acc;


	// 发布数据
	slide_estimated_s _slide_estimated{0};


	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::HY_SE_PR_P0>) _param_hy_se_pr_p0,
		(ParamFloat<px4::params::HY_SE_PR_RHO>) _param_hy_se_pr_rho,
		(ParamFloat<px4::params::HY_SE_G_PR>) _param_hy_se_g_pr,
		(ParamFloat<px4::params::HY_SE_G_ACC>) _param_hy_se_g_acc,
		(ParamFloat<px4::params::HY_SE_PR_MAXD>) _param_hy_se_pr_maxd,
		(ParamFloat<px4::params::HY_SE_PR_MAXX>) _param_hy_se_pr_maxx,
		(ParamFloat<px4::params::HY_SE_PR_MINX>) _param_hy_se_pr_minx,
		(ParamFloat<px4::params::HY_SE_LV_SAT_UUP>) _param_hy_se_lv_sat_uup,
		(ParamFloat<px4::params::HY_SE_LV_SAT_UP>) _param_hy_se_lv_sat_up,
		(ParamFloat<px4::params::HY_SE_LV_SAT_DN>) _param_hy_se_lv_sat_dn,
		(ParamFloat<px4::params::HY_SE_LV_SAT_DDN>) _param_hy_se_lv_sat_ddn,
		(ParamFloat<px4::params::HY_SE_LV_LEN>) _param_hy_se_lv_len,
		(ParamFloat<px4::params::HY_SE_C_LV_X>) _param_hy_se_c_lv_x,
		(ParamFloat<px4::params::HY_SE_C_LV_Y>) _param_hy_se_c_lv_y,
		(ParamFloat<px4::params::HY_SE_C_LV_Z>) _param_hy_se_c_lv_z,
		(ParamFloat<px4::params::HY_SE_C_PR_X>) _param_hy_se_c_pr_x,
		(ParamFloat<px4::params::HY_SE_C_PR_Y>) _param_hy_se_c_pr_y,
		(ParamFloat<px4::params::HY_SE_C_PR_Z>) _param_hy_se_c_pr_z,
		(ParamInt<px4::params::HY_SE_LIFESPAN>) _param_hy_se_lifespan,
		(ParamFloat<px4::params::HY_SE_R_ACC>) _param_hy_se_r_acc,
		(ParamFloat<px4::params::HY_SE_R_PR>) _param_hy_se_r_pr,
		(ParamFloat<px4::params::HY_SE_R_LV>) _param_hy_se_r_lv,
		(ParamFloat<px4::params::HY_SE_R_LVSAT>) _param_hy_se_r_lvsat,
		(ParamFloat<px4::params::HY_SE_Q_HGT>) _param_hy_se_q_hgt,
		(ParamFloat<px4::params::HY_SE_Q_ERR>) _param_hy_se_q_err
	)
};
