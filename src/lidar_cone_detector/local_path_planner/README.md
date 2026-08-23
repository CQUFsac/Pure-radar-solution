# local_path_planner

用于 SCOUT MINI 纯激光雷达锥桶赛道的 ROS1 局部路径规划器。当前底盘是四轮差速底盘，本包只输出局部路径，不直接计算方向盘转角，也不直接发布 `/cmd_vel`。

## 处理流程

```text
锥桶观测
→ TF 转换到 base_link
→ ROI、置信度和重复目标过滤
→ Delaunay 三角剖分生成候选横向边
→ 赛道宽度、横向程度和历史路径过滤
→ Beam Search 搜索连续中点
→ 三次 Hermite 曲线和平滑重采样
→ 曲率、连续性、自交和锥桶间距检查
→ nav_msgs/Path
```

规划器不根据 LiDAR 数据编造蓝色或黄色。纯雷达阶段使用几何关系推断赛道中线。

## ROS 接口

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 输入 | `/perception/lidar/cones_raw` | `driverless_msgs/ConeObservationArray` | 当前雷达锥桶观测 |
| 输入 | `/odometry/filtered` | `nav_msgs/Odometry` | 只用于补偿历史路径，不参与底盘控制 |
| 输出 | `/planning/local_path` | `nav_msgs/Path` | `base_link` 下的局部路径 |
| 输出 | `/planning/path_confidence` | `std_msgs/Float32` | 0～1 路径置信度 |
| 输出 | `/planning/path_status` | `std_msgs/String` | `OK`、`DEGRADED_HISTORY`、`INVALID` 或 `STALE_INPUT` |
| 调试 | `/planning/debug/path_markers` | `visualization_msgs/MarkerArray` | 候选边、中点和最终路径 |

锥桶跟踪模块完成后，建议把输入话题改为统一的 `/perception/cones` 或 `/perception/cones_tracked`，消息类型继续使用 `driverless_msgs/ConeObservationArray`。

## 坐标系与时间

- 规划坐标系默认是 `base_link`：x 向前，y 向左。
- 输入不在 `base_link` 时，节点按数组消息的 `header.stamp` 查询 TF 并转换。
- 所有锥桶必须共享数组消息中的时间戳和坐标系。
- 输出沿用输入时间戳，输出坐标系固定为 `expected_frame`。
- `/odometry/filtered` 应来自 `robot_localization`，用于把上一帧路径转换到当前车辆坐标系。

如果 TF 不可用、消息坐标系为空或路径检查失败，节点不会发布未经验证的新路径。

## 路径稳定与降级

上一条有效路径会先保存到里程计坐标系。下一帧规划时，它只作为候选边和路径搜索的软约束，防止单帧漏检造成路线左右跳变。

新路径暂时生成失败时，节点最多复用少量历史帧，并按帧降低置信度。历史路径超过 `history/fallback_max_age` 后立即失效。雷达话题超过 `input_timeout` 没有更新时，无论是否存在历史路径，都发布空路径和 `STALE_INPUT`。

控制节点必须执行以下规则：

- 收到空路径：停车；
- 路径超时：停车；
- `DEGRADED_HISTORY`：降速；
- 路径置信度过低：降速或停车。

## 主要参数

`config/local_path_planner.yaml` 中的数值均为初始建议值，需要使用实际锥桶尺寸、赛道宽度和 rosbag 调整。

- `edge/min_width`、`edge/max_width`：允许的赛道宽度范围。
- `edge/expected_width`：候选横向边的期望宽度。
- `edge/max_longitudinal_offset`：限制同侧前后锥桶被错误配对。
- `search/beam_width`：保留的候选搜索分支数。
- `search/max_turn_angle_deg`：相邻中点允许的最大转向变化。
- `history/max_age`：历史路径参与评分的最长时间。
- `history/fallback_max_frames`：新路径失败时最多复用多少帧。
- `validator/max_abs_curvature`：允许的最大路径曲率。
- `validator/min_cone_clearance`：路径与任意锥桶的最小距离。
- `input_timeout`：锥桶输入超时阈值。

SCOUT MINI 初期测试建议把车速限制在 0.5～1.0 m/s，再根据感知距离、路径置信度和打滑情况逐步提高。

## 编译与启动

```bash
cd ~/fase_ws
catkin_make
source devel/setup.bash
roslaunch local_path_planner local_path_planner.launch
```

离线调试时建议同时观察：

```bash
rostopic echo /planning/path_status
rostopic echo /planning/path_confidence
rostopic hz /planning/local_path
```

RViz 中显示 `/planning/local_path` 和 `/planning/debug/path_markers`。

## 当前边界

- 本包是局部规划，不负责锥桶地图、闭环定位和全局最优圈速。
- 连续单侧漏检、回头弯和赛道外密集备用锥桶仍可能产生错误候选。
- 历史路径只用于短时稳定，不能替代感知。
- 真车闭环前必须完成直线、缓弯、急弯、漏桶、假目标、TF 错误和输入掉线测试。
- 后期接入相机颜色后，可以在候选边代价中增加蓝黄边界一致性，但无需修改路径输出接口。
