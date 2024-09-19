#include <drivers/drv_adc.h>
#include <drivers/drv_hrt.h>
#include <lib/cdev/CDev.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_arch/adc.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/adc_report.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/depth_fusion.h>
#include <math.h>

using namespace time_literals;

class KalmanFilter {
public:
    KalmanFilter() {
        // 初始化状态 [位置, 速度]
        x_hat[0] = 0; // 位置
        x_hat[1] = 0; // 速度

        // 状态转移矩阵 A
        A[0][0] = 1; A[0][1] = dt;
        A[1][0] = 0; A[1][1] = 1;

        // 观测矩阵 H
        H[0][0] = 1; H[0][1] = 0;

        // 过程噪声协方差 Q
        Q[0][0] = 0.1; Q[0][1] = 0;
        Q[1][0] = 0; Q[1][1] = 0.1;

        // 测量噪声协方差 R
        R[0][0] = 1;

        // 估计误差协方差 P
        P[0][0] = 1; P[0][1] = 0;
        P[1][0] = 0; P[1][1] = 1;
    }

    void predict() {
        // 预测步骤
        double x_hat_temp[2];
        x_hat_temp[0] = A[0][0] * x_hat[0] + A[0][1] * x_hat[1]; // 位置预测
        x_hat_temp[1] = A[1][0] * x_hat[0] + A[1][1] * x_hat[1]; // 速度预测

        x_hat[0] = x_hat_temp[0];
        x_hat[1] = x_hat_temp[1];

        // 误差协方差预测
        double P_temp[2][2];
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                P_temp[i][j] = A[i][0] * P[0][j] + A[i][1] * P[1][j];
            }
        }
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                P[i][j] = P_temp[i][0] + Q[i][j];
            }
        }
    }

    void update(double z) {
        // 更新步骤
        double y[1]; // 测量残差
        y[0] = z - (H[0][0] * x_hat[0] + H[0][1] * x_hat[1]);

        // 残差协方差
        double S[1][1];
        S[0][0] = H[0][0] * P[0][0] + H[0][1] * P[1][0] + R[0][0];

        // 卡尔曼增益
        double K[2][1];
        K[0][0] = (P[0][0] * H[0][0] + P[0][1] * H[0][1]) / S[0][0];
        K[1][0] = (P[1][0] * H[0][0] + P[1][1] * H[0][1]) / S[0][0];

        // 更新状态
        x_hat[0] += K[0][0] * y[0];
        x_hat[1] += K[1][0] * y[0];

        // 更新误差协方差
        double P_temp[2][2];
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                P_temp[i][j] = P[i][j] - K[i][0] * H[0][j] * P[i][j];
            }
        }
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                P[i][j] = P_temp[i][j];
            }
        }
    }

    void getState(double &position, double &velocity) {
        position = x_hat[0];
        velocity = x_hat[1];
    }

    void setDeltaTime(double delta_time) {
        dt = delta_time;
        A[0][1] = dt; // 更新状态转移矩阵
    }

private:
    double x_hat[2]; // 状态估计 [位置, 速度]
    double A[2][2];  // 状态转移矩阵
    double H[1][2];  // 观测矩阵
    double Q[2][2];  // 过程噪声协方差
    double R[1][1];  // 测量噪声协方差
    double P[2][2];  // 估计误差协方差
    double dt = 1.0; // 时间间隔
};

class DepthFusion : public ModuleBase<DepthFusion>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	DepthFusion();
	~DepthFusion() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();
private:
	void Run() override;
	void Fusion(hrt_abstime now);
	uORB::Subscription _vehicle_air_data_sub{ORB_ID(vehicle_air_data)};
	uORB::Subscription _adc_report_sub{ORB_ID(adc_report)};
	uORB::Publication<depth_fusion_s> _depth_fusion_pub{ORB_ID(depth_fusion)};
	perf_counter_t			_loop_perf;
	static const hrt_abstime	interval_us{100_ms};
	KalmanFilter kf1;
    KalmanFilter kf2;
};

