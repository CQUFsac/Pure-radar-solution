# Pure-radar-solution

这套工作空间用于无人驾驶方程式算法开发和 SCOUT MINI 实车验证。目前主要有两条运行链路：

- 在 FSSIM 中调试锥桶赛道局部规划、路径跟踪、直线加速和八字绕环；
- 在 SCOUT MINI 四轮差速底盘上运行纯激光雷达感知、局部规划和 `/cmd_vel` 控制。

当前实车不是阿克曼赛车。SCOUT MINI 控制器输出线速度 `v` 和角速度 `ω`，使用的关系是 `ω = v × κ`。FSSIM 使用单独的赛车控制适配器输出 `fssim_common/Cmd`。两套控制器不能混用。

这份仓库保存的是完整 catkin 工作空间源码。`build/`、`devel/` 和 `install/` 属于本机编译结果，不提交到 Git。

## 1. 当前完成情况

已经接通的部分：

- RoboSense 雷达驱动；
- ROI 裁剪和当前版本的地面高度过滤；
- 自适应欧式聚类；
- 几何特征提取、锥桶候选筛选和位置估计；
- 统一锥桶消息 `driverless_msgs/ConeObservationArray`；
- Delaunay 候选边、连续中点搜索、路径平滑和历史路径短时降级；
- SCOUT MINI Pure Pursuit 控制，输出 `/cmd_vel`；
- FSSIM 锥桶、位姿和控制接口适配；
- 直线加速任务状态机；
- 八字绕环右两圈、左两圈和结束停车状态机；
- 可选的锥桶地图管理器接口。

仍需继续完成或用实车数据确认的部分：

- 当前 `roi_filter` 的地面处理仍是高度阈值法，不是最终室外地面分割算法；
- 16 线雷达对 29 cm 小锥桶的有效点数、强度阈值和远距离参数需要 rosbag 标定；
- `serial_imu` 只是现有串口 IMU 驱动，CH110 的厂家协议、波特率、时间戳和坐标方向必须实测确认；
- 锥桶多帧跟踪、定位建图和闭环还没有作为实车强依赖；
- 相机颜色识别和 LiDAR/相机目标级后融合只保留了消息接口，当前纯雷达版本不依赖相机。

## 2. 目录说明

```text
fase_ws_sim/
├── src/
│   ├── lidar_cone_detector/
│   │   ├── driverless_msgs/          统一锥桶消息
│   │   ├── rslidar_sdk/              RoboSense 雷达驱动
│   │   ├── roi_filter/               ROI 和当前地面高度过滤
│   │   ├── lidar_cone_detector/      聚类、特征判断、锥桶位置
│   │   ├── local_path_planner/       候选边、中线搜索和路径平滑
│   │   ├── scout_path_controller/    SCOUT 差速底盘控制器
│   │   ├── fssim_path_controller/    FSSIM 赛车控制适配器
│   │   ├── fssim_bridge/             FSSIM 数据和控制桥接
│   │   ├── scout_mission_manager/    直线加速、八字绕环状态机
│   │   ├── cone_map_manager/         可选锥桶地图接口
│   │   ├── imu/serial_imu/           串口 IMU 驱动
│   │   └── driverless_bringup/       两个平台的一键启动入口
│   ├── fssim*/                       FSSIM 及 Gazebo 模型、插件和消息
│   ├── cone_car_core/                队友公共接口包
│   ├── scout_base/                   SCOUT ROS 驱动、描述和消息
│   └── ugv_sdk/                      松灵底盘通信 SDK
├── .catkin_workspace
└── 启动命令.txt                      实车现场使用的简要命令
```

`fssim_test` 目录带有 `CATKIN_IGNORE`，只保留作旧接口参考，不参与当前编译。当前有效的模拟器适配包是 `fssim_bridge`。

## 3. 环境

模拟器当前使用：

- Ubuntu 20.04；
- ROS Noetic；
- Gazebo 11；
- FSSIM：`Huguet57/fssim-2021`。

SCOUT MINI 端仍是 ROS1。实车电脑如果使用 ROS Melodic，必须在对应 Ubuntu 18.04 环境中重新安装依赖，不能同时 source Noetic 和 Melodic 的工作空间。

首次拉取后建议先安装当前系统能够解析到的依赖：

```bash
cd ~/fase_ws_sim
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

RoboSense SDK 还会用到 libpcap，串口 IMU 包会用到 ROS `serial`。如果 rosdep 没有自动补齐，再按编译错误安装对应 ROS 版本的软件包。

## 4. 下载和编译

```bash
git clone -b lidar_cone_detector https://github.com/CQUFsac/Pure-radar-solution.git fase_ws_sim
cd fase_ws_sim
```

不要直接把 FSSIM 和 SCOUT 的所有包混在一起盲目编译。仓库提供两个编译配置：

### FSSIM

```bash
cd ~/fase_ws_sim
bash src/lidar_cone_detector/driverless_bringup/scripts/build_workspace_profile.sh fssim
source devel/setup.bash
```

### SCOUT MINI

```bash
cd ~/fase_ws_sim
bash src/lidar_cone_detector/driverless_bringup/scripts/build_workspace_profile.sh scout
source devel/setup.bash
```

切换 ROS 环境或更新消息文件后，建议清理本机的 `build/`、`devel/`、`install/` 再重新编译。这三个目录不能从其他电脑复制过来。

## 5. FSSIM 使用方法

### 5.1 高速避障/赛道测试

```bash
cd ~/fase_ws_sim
source devel/setup.bash
roslaunch fssim_bridge fssim_integration.launch
```

如果 FSSIM 已经在另一个终端运行：

```bash
roslaunch fssim_bridge fssim_integration.launch start_fssim:=false
```

终端出现 `Sending RES GO` 后，模拟器才会接受驱动命令。

### 5.2 直线加速

```bash
roslaunch fssim_bridge fssim_acceleration.launch
```

当前仿真目标速度是 15 m/s，可在启动时临时修改：

```bash
roslaunch fssim_bridge fssim_acceleration.launch cruise_speed:=12.0
```

这个参数是目标车速，不代表一定能达到对应的全程平均速度。直线加速配置使用较长前视距离和较小转角限制，防止高速时因厘米级路径抖动左右摆动。终点停车优先使用模拟器任务完成信号，同时保留终点位置判断。

### 5.3 八字绕环

```bash
roslaunch fssim_bridge fssim_skidpad.launch
```

默认顺序是：入口、右一圈、右二圈、左一圈、左二圈、出口、停车。八字交叉口有多条局部可行路线，因此该赛项使用独立任务方向提示和模拟器八字路径生成器，不能只靠普通 Delaunay 最近路径决定方向。

重新测试前执行：

```bash
rosservice call /mission/reset
rosservice call /mission/start
```

### 5.4 仿真检查话题

```bash
rostopic hz /lidar/cones
rostopic hz /perception/lidar/cones_raw
rostopic hz /planning/local_path
rostopic echo /planning/path_status
rostopic echo /fssim_bridge/status
rostopic echo /fssim/cmd
```

正常情况下锥桶和局部路径约为 10 Hz，控制命令约为 20 Hz。FSSIM 给的是模拟锥桶观测，并没有真正模拟 RS-Helios-16P 的原始扫描，因此它只能验证规划、任务状态和控制，不能代替真实点云测试。

## 6. SCOUT MINI 使用方法

实车启动分成硬件层和算法层。当前 bringup 不会自动启动雷达硬件和底盘 CAN，避免算法异常时直接接管车辆。

启动前先确认：

1. SCOUT 硬件急停可用，测试人员能够随时按下；
2. 雷达网络正常，`/rslidar_points` 持续发布；
3. 底盘驱动正常，手动小速度 `/cmd_vel` 测试方向正确；
4. `base_link`、雷达和 IMU 的 TF 方向正确；
5. 场地没有人员站在车辆预计路径上。

启动雷达和底盘驱动后，运行算法：

### 6.1 直线加速

```bash
roslaunch driverless_bringup scout_acceleration.launch
```

首次实车从低速开始：

```bash
roslaunch driverless_bringup scout_acceleration.launch \
  approach_speed:=0.15 run_speed:=0.25 max_linear_speed:=0.30
```

确认点云、行车线、停车门和控制方向都正确后，再逐级测试：

```text
第二阶段：approach_speed=0.20，run_speed=0.40，max_linear_speed=0.50
第三阶段：approach_speed=0.25，run_speed=0.60，max_linear_speed=0.70
```

这些都是初始测试值，不是最终比赛速度。每次修改速度后都要重新检查感知距离、停车距离和轮胎打滑。

### 6.2 八字绕环

```bash
roslaunch driverless_bringup scout_skidpad.launch
```

八字状态机需要 `/imu/data.angular_velocity.z` 判断累计转角。第一次上车必须手动让车体左转和右转，确认 ROS 约定下左转为正、右转为负；符号不对时先修正 IMU 配置，不能靠交换左右圈顺序掩盖问题。

### 6.3 开始和复位

启动 launch 后任务不会无限自动重跑：

```bash
rosservice call /mission/start
```

测试结束或故障处理后：

```bash
rosservice call /mission/reset
```

如果再次 start 提示 `reset mission before starting again`，说明上一次任务状态还没有复位。

## 7. 数据链路

### SCOUT MINI 纯雷达链路

```text
/rslidar_points
  -> roi_filter
  -> /lidar/non_ground_points
  -> lidar_cone_detector
  -> /perception/lidar/cones_raw
  -> local_path_planner
  -> /planning/local_path
  -> scout_path_controller
  -> /cmd_vel
```

当前控制器不强制依赖 `/odom`。以后定位模块稳定后，可启用里程计接口做历史路径运动补偿，但 `odom -> base_link` 只能由一个节点发布。

### FSSIM 链路

```text
/lidar/cones
  -> fssim_bridge
  -> /perception/lidar/cones_raw
  -> local_path_planner
  -> /planning/local_path
  -> fssim_path_controller
  -> /fssim/cmd
```

## 8. 主要话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/rslidar_points` | `sensor_msgs/PointCloud2` | 实车原始雷达点云 |
| `/lidar/roi_points` | `sensor_msgs/PointCloud2` | ROI 调试点云 |
| `/lidar/ground_points` | `sensor_msgs/PointCloud2` | 当前地面点输出 |
| `/lidar/non_ground_points` | `sensor_msgs/PointCloud2` | 锥桶检测输入 |
| `/perception/lidar/cones_raw` | `driverless_msgs/ConeObservationArray` | 单帧雷达锥桶 |
| `/planning/local_path` | `nav_msgs/Path` | `base_link` 下的局部路径 |
| `/planning/path_confidence` | `std_msgs/Float32` | 路径置信度 0～1 |
| `/planning/path_status` | `std_msgs/String` | 规划状态和降级原因 |
| `/cmd_vel` | `geometry_msgs/Twist` | SCOUT 线速度和角速度 |
| `/fssim/cmd` | `fssim_common/Cmd` | FSSIM 驱动和转角命令 |
| `/mission/turn_hint` | `std_msgs/Int8` | 右转、直行、左转提示 |
| `/mission/speed_limit` | `std_msgs/Float32` | 当前任务速度上限 |
| `/mission/stop` | `std_msgs/Bool` | 软件停车请求 |
| `/mission/status` | `std_msgs/String` | 任务状态诊断 |

规划模块后期应统一订阅 `/perception/cones`。融合模块未启动时，该话题可以由选择器转发纯雷达结果；相机掉线不能阻塞纯雷达链路。

## 9. 常用参数位置

- ROI：`src/lidar_cone_detector/roi_filter/config/roi_params.yaml`
- 聚类和锥桶判断：`src/lidar_cone_detector/lidar_cone_detector/config/clustering.yaml`
- 局部规划：`src/lidar_cone_detector/local_path_planner/config/local_path_planner.yaml`
- SCOUT 控制：`src/lidar_cone_detector/scout_path_controller/config/controller.yaml`
- FSSIM 控制：`src/lidar_cone_detector/fssim_path_controller/config/controller.yaml`
- 直线加速任务：`src/lidar_cone_detector/scout_mission_manager/config/acceleration.yaml`
- 实车八字任务：`src/lidar_cone_detector/scout_mission_manager/config/skidpad.yaml`
- 仿真八字任务：`src/lidar_cone_detector/scout_mission_manager/config/fssim_skidpad.yaml`
- 雷达型号和网络：`src/lidar_cone_detector/rslidar_sdk/config/config.yaml`

所有阈值都只是当前初始值。改参数前先录 rosbag，改完后只动一组参数并保留测试记录，不要同时修改 ROI、地面、聚类和分类阈值。

## 10. 常见问题

### 有锥桶但没有行车线

依次检查：

```bash
rostopic hz /lidar/non_ground_points
rostopic hz /perception/lidar/cones_raw
rostopic echo -n 1 /perception/lidar/cones_raw
rostopic echo /planning/path_status
rostopic hz /planning/local_path
```

常见原因是地面阈值把 29 cm 锥桶删掉、聚类最少点数过高、赛道宽度参数与场地不符、输入 `frame_id` 错误或 TF 查不到。

### RViz 有路径但车不动

检查任务是否 start、`/mission/stop` 是否为 false、控制节点是否发布，以及底盘是否接收同一个话题：

```bash
rostopic echo /mission/status
rostopic echo /mission/stop
rostopic hz /cmd_vel
rostopic echo /scout_path_controller/status
```

### FSSIM 出现重复 TF

同一个 child frame 不能由两个节点发布。`fssim_map -> base_link` 只保留 `fssim_odom_bridge`，不要同时启动另一套 odom/TF 桥。

### 编译时提示同名包

不要把备份源码放在 `src` 下面。catkin 会递归扫描，只要备份中还有 `package.xml`，即使目录名不同也会按同名 ROS 包处理。旧版本请用 Git 分支或放到工作空间外。

### 修改 YAML 后没有效果

先确认实际启动的 launch 是否加载了该 YAML，再用 `rosparam get` 检查运行参数。只改源码常量需要重新编译；只改启动时加载的 YAML 通常重启 launch 即可。

## 11. 安全要求

- 软件发布零速度不等于硬件急停；
- 首次控制测试应架空驱动轮或使用低速空旷场地；
- 感知超时、路径超时、任务故障时必须停止发布运动命令；
- 不允许通过 Wi-Fi 传输底盘实时控制链路；
- 提速前必须通过直线、缓弯、急弯、单侧漏桶、连续漏桶和假目标测试；
- 实车测试必须录制原始点云、锥桶、路径、任务状态和控制命令。

建议录制：

```bash
rosbag record -O driverless_test.bag \
  /rslidar_points /lidar/non_ground_points \
  /perception/lidar/cones_raw \
  /planning/local_path /planning/path_status /planning/path_confidence \
  /mission/status /mission/stop /cmd_vel /tf /tf_static
```

## 12. 提交代码约定

- 不提交 `build/`、`devel/`、`install/`、rosbag、pcap 和日志；
- 不在 `src` 里复制 `backup`、`before`、`old` 版本；
- 改消息后要重新编译依赖它的包；
- 新模块优先复用 `driverless_msgs/ConeObservationArray`，不要再增加第三套锥桶消息；
- SCOUT 和 FSSIM 的控制适配器保持分开，上层规划继续输出路径和曲率；
- 每次提交前至少检查 `git status`、ROS 包是否重名以及 launch XML 是否完整。

更细的模块说明在各功能包自己的 README 中。实车固定赛项注意事项见：

```text
src/lidar_cone_detector/driverless_bringup/docs/SCOUT_FIXED_EVENTS.md
src/lidar_cone_detector/driverless_bringup/docs/TWO_PLATFORM_ARCHITECTURE.md
src/lidar_cone_detector/driverless_bringup/docs/TEAM_CODE_INTEGRATION.md
```
