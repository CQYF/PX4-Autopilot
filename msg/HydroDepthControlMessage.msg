uint64 timestamp                # time since system start (microseconds)

float32 depth
float32 depth_rate
float32 depth_setpoint
float32 throttle_limited
float32 horizontal_thrust

#非光滑反馈
float32 nsf_e1
float32 nsf_e2
float32 nsf_s
float32 nsf_s_norm
float32 nsf_s_norm_power
float32 nsf_u
float32 nsf_u_limited

#高阶滑模
float32 hsmc_x1
float32 hsmc_x2
float32 hsmc_x3
float32 hsmc_surface
float32 hsmc_u

#增益调度控制
float32 gsc_err
float32 gsc_gain_b
float32 gsc_gain_v1
float32 gsc_gain_v2
float32 gsc_gain_t
float32 gsc_gain
float32 gsc_gain_limited
float32 gsc_u
float32 gsc_u_limited


float32 vertical_thrust
float32 vertical_thrust_limited

