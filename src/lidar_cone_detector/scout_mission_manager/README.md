# SCOUT MINI 固定赛项任务管理器

这个包只服务于当前的四轮差速 SCOUT MINI 算法验证平台。它不输出前轮转角，
也不直接发布 `/cmd_vel`。任务管理器只告诉规划器和控制器：

- 当前应该直行、右转还是左转；
- 当前允许的最大线速度；
- 是否必须停车；
- 当前处于第几圈、是否已经完成。

当前支持两个任务：

- `acceleration`：起步门线 → 直线加速 → 终点门线 → 减速停车；
- `skidpad`：入口 → 右一圈 → 右二圈 → 切换左环 → 左一圈 →
  左二圈 → 出口 → 减速停车。

锥桶输入话题是启动参数。当前纯雷达使用
`/perception/lidar/cones_raw`；后期目标级融合完成后，可把 `cone_topic` 改为
`/perception/cones`，任务状态机、规划器和控制器不需要修改。

## 为什么需要独立任务状态机

局部三角剖分只能知道“眼前哪些锥桶可以组成中点”，不知道八字交叉处本圈必须
走右环还是左环。把圈数逻辑写进三角剖分会导致规划器越来越难调试。因此使用
`/mission/turn_hint` 对候选中线施加软约束：

- `-1`：当前期望右转；
- `0`：当前期望直行；
- `+1`：当前期望左转。

如果局部感知只剩一条可行路径，规划器仍可输出；方向提示不是无条件硬编码转弯。

## ROS 接口

输入：

| 话题 | 类型 | 用途 |
|---|---|---|
| `/perception/lidar/cones_raw` | `driverless_msgs/ConeObservationArray` | 纯雷达锥桶和门线识别 |
| `/imu/data` | `sensor_msgs/Imu` | 八字赛项积分 `angular_velocity.z` |

输出：

| 话题 | 类型 | 用途 |
|---|---|---|
| `/mission/state` | `std_msgs/String` | 当前状态 |
| `/mission/turn_hint` | `std_msgs/Int8` | 右转/直行/左转提示 |
| `/mission/speed_limit` | `std_msgs/Float32` | 当前线速度上限，单位 m/s |
| `/mission/stop` | `std_msgs/Bool` | 硬停车许可开关 |
| `/mission/finished` | `std_msgs/Bool` | 任务正常完成 |
| `/mission/status` | `std_msgs/String` | 门线、积分航向和故障诊断 |

服务：

```bash
rosservice call /mission/start
rosservice call /mission/reset
```

`/mission/test_gate` 只用于顶车架或离线联调时人工注入门线事件，实车正式运行
不能依赖它。

## 纯雷达可识别门线

激光雷达无法识别地面上的油漆计时线。SCOUT MINI 测试场必须在赛道两侧布置
“双排边界门”：

```text
左边界       行驶区域       右边界
  ● ---------------------- ●   第一排
  ● ---------------------- ●   第二排
        两排纵向距离 0.2~0.9 m
```

四个锥桶都位于边界外侧，不应挡住车辆。连续三帧识别到两条横向配对边后才产生
门线事件，随后必须清除三帧并经过防抖时间才能再次计数。所有距离均为初始建议值，
需要按真实赛道宽度和 rosbag 调整。

## 八字圈数判定

圈数不是只靠“看见一次门线”增加，而是同时满足：

1. 当前任务状态要求右圈或左圈；
2. CH110 的 `angular_velocity.z` 积分达到设定航向变化；
3. 本圈持续时间大于最小值；
4. 再次稳定识别到中心双排门线。

默认 ROS 约定为逆时针正、顺时针负。上车前必须手动左转和右转检查
`/imu/data.angular_velocity.z`；只有驱动符号确实相反时，才把
`imu/yaw_sign` 改为 `-1.0`。

## 启动和验收

直线加速：

```bash
roslaunch driverless_bringup scout_acceleration.launch
rostopic echo /mission/status
rosservice call /mission/start
```

八字绕环：

```bash
roslaunch driverless_bringup scout_skidpad.launch
rostopic echo /mission/status
rosservice call /mission/start
```

首次实车必须架空驱动轮检查停车链路，然后使用不超过 0.3 m/s 的限速完成单圈
状态切换。确认右圈时 `segment_yaw` 为负、左圈时为正后，再使用配置中的初始速度。

任何锥桶输入或必需 IMU 超时都会进入 `FAULT`，`/mission/stop=true`，且不会自动
恢复；排查后调用 `/mission/reset` 再重新开始。软件停车不能代替 SCOUT 的硬件
急停，场边测试人员必须始终掌握硬件急停。
