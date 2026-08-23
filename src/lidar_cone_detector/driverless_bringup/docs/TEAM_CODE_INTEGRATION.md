# 队友代码联通说明

## 统一接口

```text
serial_imu
  -> /imu/data                         sensor_msgs/Imu
       |-> scout_mission_manager       八字圈数
       +-> 队友定位模块                状态估计输入

LiDAR detector / perception selector
  -> /perception/lidar/cones_raw
     或 /perception/cones              driverless_msgs/ConeObservationArray
       |-> local_path_planner          局部中线
       |-> scout_mission_manager       门线识别
       +-> cone_map_manager            地标地图

队友定位模块
  -> map -> base_link TF
     或 /localization/pose             geometry_msgs/PoseWithCovarianceStamped
       -> cone_map_manager

cone_map_manager
  -> /mapping/cones                    driverless_msgs/ConeObservationArray
```

## 已替换的重复内容

- 删除 `imu/imu_launch`：它只重复启动 `serial_imu`；
- 删除调试用 `imu_subscriber`：使用 `rostopic echo /imu/data`；
- `Cone_Map_Manager` 改为 ROS 小写包名 `cone_map_manager`；
- 删除 `CarPosition.msg`、`ConePosition.msg`，统一使用标准几何消息和
  `driverless_msgs/ConeObservationArray`；
- 建图节点不再重复承担 IMU 位置积分；定位只由定位模块负责；
- 规划、任务和建图共用一个可配置 `cone_topic`，以后可切换融合输出。

## 启动方式

IMU 协议确认后，可让固定赛项启动文件一并启动串口驱动：

```bash
roslaunch driverless_bringup scout_skidpad.launch \
  start_imu_driver:=true imu_port:=/dev/imu
```

没有定位时保持：

```text
start_mapper:=false
```

定位提供 `/localization/pose` 或 `map -> base_link` 后：

```bash
roslaunch driverless_bringup scout_skidpad.launch \
  start_mapper:=true localization_pose_topic:=/localization/pose
```

建图未就绪不会阻止纯局部雷达规划和 SCOUT 控制。
