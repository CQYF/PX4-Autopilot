

uint64 timestamp                # time since system start (microseconds)

uint64 timestamp_sample_pr	# 深度计采样时间 (microseconds)
uint64 timestamp_sample_lv      # 水位计采样时间 (microseconds)

float32 depth_estimated		# 最终输出的估计值

float32 depth_origin_pr		# 原始值
float32 depth_origin_lv

float32 depth_medfilted_pr	# 中值滤波的结果

float32 depth_invalid_pr	# 不合法的值
float32 depth_invalid_lv

# TOPICS slide_estimated
