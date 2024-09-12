#include"depth_fusion.hpp"
using namespace time_literals;
using namespace matrix;

using math::constrain;
using math::radians;

DepthFusion::DepthFusion():ModuleParams(nullptr),//初始化基类ModuleParams，用于管理模块参数
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	kf1.setDeltaTime(0.02);
	kf2.setDeltaTime(0.02);
	_depth_fusion_pub.advertise();
}
DepthFusion::~DepthFusion()
{
	perf_free(_loop_perf);
}

bool DepthFusion::init()
{
	ScheduleOnInterval(interval_us);
	return true;
}

/*void DepthFusion::Run()
{
	hrt_abstime now = hrt_absolute_time();
	Fusion(now);
}*/
void DepthFusion::Run()
{
	vehicle_air_data_s vehicle_air_data;
	adc_report_s adc_report;
	_vehicle_air_data_sub.update(&vehicle_air_data);
	_adc_report_sub.update(&adc_report);

	float depth1=vehicle_air_data.baro_alt_meter;
	kf1.predict();
	kf1.update(depth1);

	float depth2=adc_report.depth;
	kf2.predict();
	kf2.update(depth2);

	double d1,d2;
	kf1.getState(d1,d2);
	double d3,d4;
	kf2.getState(d3,d4);

	depth_fusion_s out={};
	out.timestamp=hrt_absolute_time();
	out.fudepth=d1*0.7+d3*0.3;
	out.depth1=d1;
	out.depth2=d3;
	out.depth1_or=depth1;
	out.depth2_or=depth2;
	_depth_fusion_pub.publish(out);
}
/*void DepthFusion::Fusion(hrt_abstime now)
{
	depth_fusion_s out={};
	out.timestamp=now;
	bool x=(now <= now + 1_s);
	while (x) {
		vehicle_air_data_s air_data;
		adc_report_s adc_data;
		if (_vehicle_air_data_sub.update(&air_data)){
			//vehicle_air_data_s air_data;
			orb_copy(ORB_ID(vehicle_air_data), orb_subscribe(ORB_ID(vehicle_air_data)), &air_data);
			//_vehicle_air_data_sub.update(&air_data);
			float depth1=air_data.baro_alt_meter;

			kf.predict();
			kf.update(depth1);
		}
		if (_adc_report_sub.update(&adc_data)){
			//adc_report_s adc_data;
			orb_copy(ORB_ID(adc_report), orb_subscribe(ORB_ID(adc_report)), &adc_data);
			//_vehicle_air_data_sub.update(&adc_data);
			float depth2=adc_data.depth;

			kf.predict();
			kf.update(depth2);
		}
		double d1,d2;
		kf.getState(d1, d2);
		out.fudepth=d1*0.7+d2*0.3;
		_depth_fusion_pub.publish(out);

		px4_sleep(20000);
	}
}*/

int DepthFusion::task_spawn(int argc, char *argv[])
{
	DepthFusion *instance = new DepthFusion();

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

int DepthFusion::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int DepthFusion::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("depth_fusion", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int depth_fusion_main(int argc, char *argv[])
{
	return DepthFusion::main(argc, argv);
}
