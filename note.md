飞行任务
飞行模式
misson模式下执行特定的任务

qgc是AppImage文件，似乎是某种镜像，使用./QGroundControl.AppImage直接运行
mission planner是exe文件，需要使用mono运行，命令为mono MissionPlanner.exe
mono环境需要安装mono-complete

我的任务是做飞行任务，即给飞行器一个命令序列，让它完成特定的任务
仔细看px4文档的Development/Concepts/Flight Tasks页面，这有助于完成我的任务。该页面最下方有某个视频，等黄老师从youtube下载，该页面最下方还有一个pdf，里面是px4的整体原理

飞行器的命令可以在mission planner里面编辑和查看？

在docker环境下使用make px4_sitl gazebo-classic_advanced_plane可以调出gazebo仿真，之后可以用qgc连接到飞机，问题：这条指令的含义和原理是什么？

bing国际版

PX4 flight_review拉取到本地，然后在docker里面运行，就可以通过网页分析数据
需要修改app/Dockerfile，添加apt和pip的下载和更新路径，改为清华和华为的，避免太卡
需要修改.env，把DOMAIN和BOKEH_ALLOW_WS_ORIGIN改为127.0.0.1，使本地能够访问

imu的噪声会对飞机性能造成严重影响，会导致电机发热。原因：电机油门快速跳变，导致发热严重

在qgc中配置使用sd卡记录accel和gyro的原始数据和high_rate的数据，然后可以从qgc上下载ulg文件，每个文件从飞控解锁开始记录，到上锁结束记录
flight_review里可以看到记录信号的频域曲线，如果有高频噪声，就调整某个截止频率参数，另外给pid的d分量进行计算的信号有独立的截止频率参数，命名都是IMU……CUTOFF，截止频率不能太小，否则性能会严重下降
对于共振的情况，可以使用陷波滤波器进行滤波，配置时需要指定基频和带宽，并且可以设置对多少个协频进行滤波，比如35Hz，3个协频，则35、70、105Hz都会被滤波
桨叶动平衡不好也会导致噪声，这种噪声可以利用一个独立的自适应滤波器进行消除，该滤波器的原理是转速总是等于噪声频率，比如100rps就对应100Hz，配置方式是将某个IMU……EN参数置1，如果电机转速可以反馈得到，则使用普通滤波器，如果不能就使用FFT滤波器，该滤波器会在飞控板上运行一个FFT，将一定的时间窗口内的信号变换到频域，并辨识出噪声的频率。从性能上看，普通滤波器性能更好。


研究代码，如何在现有FlightTask的基础上扩展出自己的新任务
研究代码，如何使用Mavlink触发新任务
新任务直接绑定在一个模式上 or 使用遥控器在某个模式中触发任务

Task中setpoint的更新频率是多少

是否可以直接在原有setpoint的基础上逐渐将position的z轴下降3m再上浮，以实现潜行的目的

看一个现有task的setpoint更新原理

如何使用遥控器切换模式？如何使用Mavlink切换模式？如何自动切换模式？
是否可以只切换任务，不切换模式？


px4的FlightMode到底是什么？FlightModeManager完全不涉及FlightMode的变动
FlightModeManager主要根据vehicle_control_mode和vehicle_status来切换任务，根据vehicle_command来动态调整任务的参数
vehicle_command似乎来自于Mavlink的命令，原因是该消息中有许多命名为paramx的变量。该消息是否有可能来自于别处，比如被其他程序生成，或使用遥控器触发？FlightModeManager不根据该消息切换任务
vehicle_control_mode消息和FlightMode没有任何关系

Q：px4的VehicleStatus消息和VehicleControlMode消息有什么区别？
A：
在PX4飞控系统中，`VehicleStatus` 和 `VehicleControlMode` 消息代表了两种不同层面的飞行器状态信息：

**VehicleStatus 消息**:
`VehicleStatus` 消息提供了飞行器整体运行状态的综合视图，涵盖了多种维度的状态信息，例如：
- 飞行模式（虽然也包含在内，但更为全面地反映了整个系统的状态）
- 锁定/解锁状态
- 电池电压和剩余电量
- GPS和其他定位系统的状态
- 飞行控制器内部的状态标志
- 飞行器的故障信息
- 飞行器的物理状态，如高度、速度等基本飞行参数

**VehicleControlMode 消息**:
相比之下，`VehicleControlMode` 消息更加聚焦于飞行器当前的控制模式，这是决定飞行器如何响应输入信号和自主行为的核心信息。它通常包括：
- 当前激活的飞行模式（手动、自稳、自动、OFFBOARD等）
- 是否处于机载计算机控制下的特殊模式
- 控制权来源（遥控器、自动驾驶仪、外部控制等）

总结来说，`VehicleStatus` 是一个综合性的状态报告，其中包含了 `VehicleControlMode` 作为一部分，但它还包括了飞行器更多的系统级状态数据；而 `VehicleControlMode` 更专注于描述当前生效的控制策略和决策层级。

px4的FlightMode分为main mode和sub mode（在Mavlink中使用:分隔，但在程序中仅使用一个量表示）（似乎程序里面还有一个base_mode？Commander.cpp line 737），看上去FlightMode是由vehicle_status中的nav_state决定的

理论上可以：
error: 自己改Mavlink的指令，往里面添加自己定义的新指令
在VehicleCommand.msg里同步地添加自己的指令
error: （Mavlink和飞控交互并不会原封不动地上传，而是会先进行处理，处理后的东西发过去以后会变成飞控上的一条vehicle_command消息，里面的command部分是一个数）
在Commander.cpp的handle_command()函数中添加对该命令的处理，（如果是切换模式可以直接添加在VEHICLE_CMD_DO_SET_MODE的命令下？自己仅新建参数？）
对于模式切换的命令，可以先在VehicleStatus.msg中的nav_state下新建自己的模式，然后在上一步中仿照原有的代码切换为对应的模式
nav_state的切换没有直接效果，vehicle_status消息会被FlightModeManager接收，并在start_flight_task()函数中根据nav_state对Task进行切换
自己实现了Task之后，就可以根据上述步骤来实现利用Mavlink切换飞控的模式
显然，自己实现模式的方法比较繁琐，因此可以考虑使用现有的模式，仅自己实现Task

问题：nav_state是怎么自动切换的？
vehicle_command可以由程序模块产生，比如CollisionPrevention模块。原因：vehicle_command消息并不是持续存在，而是需要进行某项命令的时候才会出现

模式切换还有多种方式，比如通过ManualControl模块使用ActionRequest触发。具体有哪些方式可以在Commander.cpp中搜索“_user_mode_intention.change：”

vehicle_control_mode消息的产生较为简单，它是由nav_state、vehicle_type等变量直接得到的，详情可以查看Commander.cpp中的updateControlMode()函数

Mavlink的信息似乎是由output_mavlink产生的，而这个程序似乎又是gimbal模块的一部分，这一点非常奇怪

Mavlink命令的处理方式和先前所想的不太一样，其传递的就是原始信息，并不会做什么预处理。
每个module都要实现一些函数，其中就包括用户命令的处理函数custom_command()，在module.h中可以找到每个module必须要实现的一些函数

Mavlink的命令传递到飞控后是如何找到其操作的对应模块的（搁置）
各module是如何启动的，包括自动启动和Mavlink启动（搁置）

如何将自己的Task挂载到auto模式上？or 如何将自己的Task挂载到某个模式上？
将这个任务挂载到特定的模式上会有什么问题？

使用以下命令启动仿真程序：
./Tools/simulation/gazebo-classic/sitl_run.sh /workspaces/PX4-Autopilot-docker/build/px4_sitl_default/bin/px4 none plane none /workspaces/PX4-Autopilot-docker /workspaces/PX4-Autopilot-docker/build/px4_sitl_default
用法：
usage: sitl_run.sh sitl_bin debugger model world src_path build_path
将plane换成iris就是四旋翼的
备注：
希望将遥控信号引入仿真环境，目前的方法是在后面加上 -d /dev/ttyACM0，但没有用（已经找到了qgc里面的虚拟摇杆，暂时不需要这个了）

问题：
仿真模型采用iris时，一切正常，自定义任务被成功激活；换成plane后，FlightModeManager直接不运行了
不能运行top命令怎么办？

根据对启动文件的查询（ROMFS下的rcS、rc.vehicle_setup），
commander和navigator是任何机型都要启动的module，
机型为旋翼时，flight_mode_manager正常启动（rc.mc_apps），机型为固定翼时，flight_mode_manager不启动（rc.fw_apps），
使用mavlink命令手动启动flight_mode_manager，没有有意义的响应

旋翼的工作流程可以查看PX4 Developer Summit 2020 - Overview of multicopter control from sensors to motors.pdf
由于上述的原因，固定翼的工作流程必然不同于旋翼，根据目前的研究，工作流程大概如下：
自动模式：
navigator模块发布position_setpoint_triplet，由fw_pos_control接收
之后便是pos->att->rate->控制分配
实验：将position_setpoint_triplet的发布（navigator_main.cpp line 1100）注释掉后，Hold模式失效
疑点：为什么还能正常起飞？
手动模式：
待研究
**因此，根据目前的研究结果，无法直接在固定翼上使用FlightTask，需要在navigator方面研究新方法**

**实际情况可能还有出入，根据最新的发现，position_setpoint_triplet的发布频率非常低，甚至可能之在准备前往下一个路径点的时候才发送一次（但也要注意先前注释发布实验的结果）**

如果想要搜索一个消息是谁发布和接收的，可以按这个例子搜索 ORB_ID(vehicle_local_position_setpoint)

根据测试，固定翼模式下，vehicle_local_position_setpoint不发布没有影响，旋翼模式下也没有影响


以下是固定翼某时刻的uorb消息情况：
TOPIC NAME                         INST #SUB RATE #Q SIZE
actuator_armed                        0   15    2  1   16
actuator_motors                       0    2  251  1   72
actuator_outputs                      0    4  250  1   80
actuator_outputs_sim                  0    1  250  1   80
actuator_servos                       0    2  251  1   48
airspeed                              0    3   10  1   32
airspeed_validated                    0   10   10  1   32
airspeed_wind                         0    1   10  1   64
airspeed_wind                         1    1   10  1   64
battery_status                        0    9  100  1  168
control_allocator_status              0    2  125  1   56
cpuload                               0    5    2  1   16
differential_pressure                 0    8   10  1   32
ekf2_timestamps                       0    2  250  1   24
esc_status                            0    9  250  1  336
estimator_aid_src_baro_hgt            0    1   16  1   56
estimator_aid_src_gnss_hgt            0    1    2  1   56
estimator_aid_src_gnss_pos            0    1    2  1   72
estimator_aid_src_gnss_vel            0    1    2  1   96
estimator_aid_src_mag                 0    1   14  1   96
estimator_aid_src_mag_heading         0    1   14  1   56
estimator_attitude                    0    2  249  1   56
estimator_baro_bias                   0    1    5  1   40
estimator_event_flags                 0    1    1  1   56
estimator_global_position             0    2  125  1   64
estimator_gps_status                  0    1    2  1   40
estimator_innovation_test_ratios      0    1  125  1  152
estimator_innovation_variances        0    1  125  1  152
estimator_innovations                 0    1  125  1  152
estimator_local_position              0    2  125  1  168
estimator_odometry                    0    1  125  1  112
estimator_selector_status             0   11    1  1  160
estimator_sensor_bias                 0    8    1  1  120
estimator_states                      0    1  125  1  216
estimator_status                      0    9  125  1  120
estimator_status_flags                0    2    1  1   96
failsafe_flags                        0    1    2  1   88
failure_detector_status               0    2    2  1   24
flaps_setpoint                        0    2  250  1   16
geofence_result                       0    1    4  1   16
manual_control_input                  0    1   25  1   64
manual_control_setpoint               0    8   25  1   64
npfg_status                           0    1   50  1   64
position_controller_status            0    5   50  1   48
rate_ctrl_status                      0    1  250  1   24
rtl_time_estimate                     0    5    1  1   24
sensor_accel                          0    6  250  8   48
sensor_baro                           0    6   41  4   32
sensor_baro                           1    3   41  4   32
sensor_combined                       0    4  250  1   48
sensor_gps                            0    5    2  1  128
sensor_gyro                           0    7  249  8   48
sensor_gyro_fifo                      0    2  249  4  224
sensor_mag                            0    3   83  4   40
sensor_preflight_mag                  0    1   84  1   16
sensors_status_imu                    0    3  250  1   96
spoilers_setpoint                     0    2  250  1   16
tecs_status                           0    4   50  1   96
telemetry_status                      0    2    1  1   88
telemetry_status                      1    2    1  1   88
telemetry_status                      2    2    1  1   88
telemetry_status                      3    2    1  1   88
vehicle_acceleration                  0    3  250  1   32
vehicle_air_data                      0   22   16  1   40
vehicle_angular_velocity              0   14  249  1   40
vehicle_angular_velocity_groundtruth  0    1  249  1   40
vehicle_attitude                      0   17  250  1   56
vehicle_attitude_groundtruth          0    1  249  1   56
vehicle_attitude_setpoint             0    6   50  1   56
vehicle_control_mode                  0   19    2  1   24
vehicle_global_position               0   18  125  1   64
vehicle_global_position_groundtruth   0    1  249  1   64
vehicle_gps_position                  0   13    2  1  128
vehicle_imu                           0    5  249  1   56
vehicle_imu_status                    0    8   10  1  136
vehicle_land_detected                 0   18    1  1   24
vehicle_local_position                0   39  125  1  168
vehicle_local_position_groundtruth    0    1  250  1  168
vehicle_magnetometer                  0    6   14  1   40
vehicle_odometry                      0    2  125  1  112
vehicle_rates_setpoint                0    5  250  1   40
vehicle_status                        0   45    2  1   72
vehicle_thrust_setpoint               0    6  250  1   32
vehicle_torque_setpoint               0    3  250  1   32
