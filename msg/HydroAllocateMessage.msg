uint64 timestamp                # time since system start (microseconds)

float32[4] thrust_xz_control	# 控制得到的四个推力，0-lx 1-lz 2-rx 3-rz
float32[4] thrust_xz_limited	# 限幅后的四个推力，顺序同上

float32[4] actuator_optimed	# 优化求解的执行器期望值，0-左翼 1-右翼 2-左推 3-右推
