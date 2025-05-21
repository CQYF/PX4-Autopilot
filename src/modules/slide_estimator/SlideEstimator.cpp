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

#include "SlideEstimator.hpp"

#include <math.h>

using namespace time_literals;

float SlideEstimator::hy_se_q_hgt = 0.0f;
float SlideEstimator::hy_se_q_err = 0.0f;
bool SlideEstimator::run_info = false;

void SlideEstimator::calc_F(Matrix<double, 4, 4>& F, uint64_t& dt)
{
	double t = (double)dt;
	t /= 1000000;
	double t2 = t*t;

	F.setZero();
	F(0,0) = 1;
	F(1,1) = 1;
	F(2,2) = 1;
	F(0,1) = t;
	F(1,2) = t;
	F(0,2) = t2/2;
	F(3,3) = 1;
}

void SlideEstimator::calc_Q(Matrix<double, 4, 4>& Q, uint64_t& dt)
{
	double t = (double)dt;
	t /= 1000000;
	double t2 = t*t;

	Matrix<double, 4, 2> G;
	G.setZero();
	G(0,0) = t2/2;
	G(1,0) = t;
	G(2,0) = 1;
	G(3,1) = 1;

	Matrix<double, 2, 2> M;
	M.setZero();
	M(0,0) = t * (double)hy_se_q_hgt;
	M(1,1) = t * (double)hy_se_q_err;

	Q.setZero();
	Q = G*M*G.transpose();
}

SlideEstimator::SlideEstimator() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_kf(calc_F, calc_Q, (uint64_t)100000)
{
	/* fetch initial parameter values */
	updateParams();
	_kf.set_lifespan((uint64_t)_param_hy_se_lifespan.get());
	hy_se_q_hgt = _param_hy_se_q_hgt.get();
	hy_se_q_err = _param_hy_se_q_err.get();
}

SlideEstimator::~SlideEstimator()
{
	perf_free(_loop_perf);
}

bool
SlideEstimator::init()
{
	if (!_sensor_baro_sub.registerCallback() || !_water_level_sub.registerCallback() ||\
	!_vehicle_attitude_sub.registerCallback() || !_vehicle_angular_velocity_sub.registerCallback() ||\
	!_vehicle_acceleration_sub.registerCallback() ) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void SlideEstimator::Run()
{
	if (should_exit()) {
		_sensor_baro_sub.unregisterCallback();
		_water_level_sub.unregisterCallback();
		_vehicle_attitude_sub.unregisterCallback();
		_vehicle_angular_velocity_sub.unregisterCallback();
		_vehicle_acceleration_sub.unregisterCallback();
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

		_kf.set_lifespan((uint64_t)_param_hy_se_lifespan.get());
		hy_se_q_hgt = _param_hy_se_q_hgt.get();
		hy_se_q_err = _param_hy_se_q_err.get();
	}

	// 读取四元数姿态并保存
	vehicle_attitude_s vehicle_attitude;
	while(_vehicle_attitude_sub.update(&vehicle_attitude))
	{
		Quatf q_new(vehicle_attitude.q);
		_q = q_new;
	}

	// 读取角速度并保存
	vehicle_angular_velocity_s vehicle_angular_velocity;
	while(_vehicle_angular_velocity_sub.update(&vehicle_angular_velocity))
	{
		Vector3f w_new(vehicle_angular_velocity.xyz);
		_w = w_new;
	}

	// 清理过期数据
	_kf.clear_outofdate_data();

	// 收到加速度数据
	vehicle_acceleration_s vehicle_acceleration;
	while(_vehicle_acceleration_sub.update(&vehicle_acceleration))
	{
		// 加速度矢量，在b系下表示。
		Vector3f vb_a(vehicle_acceleration.xyz);

		// 加速度矢量，在n系下表示。
		Vector3f vn_a = _q.rotateVector(vb_a);

		// 计算高度的加速度
		_imu_height_acc = vn_a(2) + _param_hy_se_g_acc.get();

		// 调用kalman
		uint64_t t = vehicle_acceleration.timestamp_sample;
		double z_list[] = {_imu_height_acc};
		double H_list[] = {0,0,1,0};
		double R_list[] = {_param_hy_se_r_acc.get()};
		Matrix<double, 1, 1> z(z_list);
		Matrix<double, 1, 4> H(H_list);
		Matrix<double, 1, 1> R(R_list);
		_kf.insert_data(t, z, H, R);

		_slide_estimated.acc_measure = _imu_height_acc;
	}

	// 收到压强计数据
	sensor_baro_s sensor_baro;
	while(_sensor_baro_sub.update(&sensor_baro))
	{
		// 根据压强计读数计算深度测量值
		_pr_depth = pressure2depth(sensor_baro.pressure);
		// 如果深度测量值合法
		if(is_pr_depth_legal())
		{
			calc_pr_height();

			// 调用kalman
			uint64_t t = sensor_baro.timestamp_sample;
			double z_list[] = {_pr_height};
			double H_list[] = {1,0,0,1};
			double R_list[] = {_param_hy_se_r_pr.get()};
			Matrix<double, 1, 1> z(z_list);
			Matrix<double, 1, 4> H(H_list);
			Matrix<double, 1, 1> R(R_list);
			_kf.insert_data(t, z, H, R);

			_slide_estimated.pr_measure = _pr_height;
		}
	}

	// 收到水位计数据
	water_level_s water_level;
	while(_water_level_sub.update(&water_level))
	{
		_lv_immersion = water_level.lv;
		calc_lv_saturation();
		calc_lv_height();

		// 调用kalman
		uint64_t t = water_level.timestamp;
		double z_list[] = {_lv_height};
		double H_list[] = {1,0,0,0};
		double R_list[] = {_param_hy_se_r_lv.get() + _param_hy_se_r_lvsat.get()*_lv_satuation};
		Matrix<double, 1, 1> z(z_list);
		Matrix<double, 1, 4> H(H_list);
		Matrix<double, 1, 1> R(R_list);
		_kf.insert_data(t, z, H, R);

		_slide_estimated.lv_satuation = _lv_satuation;
		_slide_estimated.lv_measure = _lv_height;
	}

	if(_kf.update(_x_out, _P_out))
	{
		_slide_estimated.x1_fusion = (float)_x_out(0, 0);
		_slide_estimated.x2_fusion = (float)_x_out(1, 0);
		_slide_estimated.x3_fusion = (float)_x_out(2, 0);
		_slide_estimated.x4_fusion = (float)_x_out(3, 0);
		_slide_estimated.timestamp = hrt_absolute_time();
		_slide_estimated_pub.publish(_slide_estimated);
	}

	if(run_info) {
		_kf.info();
		run_info = false;
	}

	// backup schedule
	ScheduleDelayed(5_ms);

	perf_end(_loop_perf);
}

// 根据浸水长度计算水位计饱和程度评估值
void SlideEstimator::calc_lv_saturation()
{
	float uup = _param_hy_se_lv_sat_uup.get();
	float up = _param_hy_se_lv_sat_up.get();
	float dn = _param_hy_se_lv_sat_dn.get();
	float ddn = _param_hy_se_lv_sat_ddn.get();

	float x = _lv_immersion;
	float sat;

	if(x > uup)
		sat = 1.0f;
	else if(x > up)
		sat = (x - up) / (uup - up);
	else if(x > dn)
		sat = 0.0f;
	else if(x > ddn)
		sat = (dn - x) / (dn - ddn);
	else
		sat = 1.0f;

	_lv_satuation = sat;
}

// 根据浸水长度、姿态和几何关系计算高度
void SlideEstimator::calc_lv_height()
{
	// 中心（加速度计安装位置为中心）到水位计顶部的矢量，在b系下表示。
	Vector3f vb_c_lvtop(_param_hy_se_c_lv_x.get(), _param_hy_se_c_lv_y.get(), _param_hy_se_c_lv_z.get());
	// 水位计顶部到水位线的矢量，在b系下表示
	Vector3f vb_lvtop_waterline(0.0f, 0.0f, - _param_hy_se_lv_len.get() + _lv_immersion);
	// 中心到水位线的矢量，在b系下表示
	Vector3f vb_c_waterline = vb_c_lvtop + vb_lvtop_waterline;

	// 姿态四元数
	Quatf q = _q;

	// 中心到水位线的矢量，在n系下表示
	Vector3f vn_c_waterline = q.rotateVector(vb_c_waterline);

	// 取矢量的最后一项的负值，即为高度（水面为0，出水为负）
	_lv_height = - vn_c_waterline(2);
}

// 压强转换为深度
float SlideEstimator::pressure2depth(float pressure)
{
	float depth = (pressure - _param_hy_se_pr_p0.get()) /	(_param_hy_se_pr_rho.get() * _param_hy_se_g_pr.get());
	return depth;
}

// 深度测量值的合法性检查
bool SlideEstimator::is_pr_depth_legal()
{
	float x = _pr_depth;
	float med = _medfilter_pr_depth.apply(x);
	float maxd = _param_hy_se_pr_maxd.get();

	float x_max = _param_hy_se_pr_maxx.get();
	float x_min = _param_hy_se_pr_minx.get();

	if(isInRange(x - med, -maxd, maxd) && isInRange(x, x_min, x_max))
	{
		return true;
	}
	else
	{
		return false;
	}
}

// 根据压强计深度、姿态和几何关系计算高度
void SlideEstimator::calc_pr_height()
{
	// 中心到压强计的矢量，在b系下表示。
	Vector3f vb_c_pr(_param_hy_se_c_pr_x.get(), _param_hy_se_c_pr_y.get(), _param_hy_se_c_pr_z.get());

	// 姿态四元数
	Quatf q = _q;

	// 中心到压强计的矢量，在n系下表示。
	Vector3f vn_c_pr = q.rotateVector(vb_c_pr);

	// 压强计到水面的矢量，在n系下表示
	Vector3f vn_pr_waterface(0.0f, 0.0f, - _pr_depth);

	// 中心到水面的矢量，在n系下表示
	Vector3f vn_c_waterface = vn_c_pr + vn_pr_waterface;

	// 取矢量的最后一项的负值，即为高度（水面为0，出水为负）
	_pr_height = - vn_c_waterface(2);
}

int SlideEstimator::task_spawn(int argc, char *argv[])
{
	SlideEstimator *instance = new SlideEstimator();

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

int SlideEstimator::custom_command(int argc, char *argv[])
{
	if (!strcmp(argv[0], "show")) {
		run_info = true;
		return 0;
	}
	return print_usage("unknown command");
}

int SlideEstimator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
slide_estimator module is used to estimate the current slide.

)DESCR_STR");

	return 0;
}

extern "C" __EXPORT int slide_estimator_main(int argc, char *argv[])
{
	return SlideEstimator::main(argc, argv);
}
