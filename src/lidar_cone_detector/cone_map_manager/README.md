# cone_map_manager

队友原始建图代码已经接入统一消息，但不再使用 IMU 加速度二次积分直接生成车辆
位置。没有去除重力、零偏、姿态误差和初始速度时，这种位置会迅速发散，不能作为
锥桶地图坐标。

输入：

- `/perception/lidar/cones_raw`：当前纯雷达锥桶；
- `/localization/pose`：`geometry_msgs/PoseWithCovarianceStamped`，
  表示 `base_link` 在 `map` 下的位姿；
- 或由定位节点直接提供可查询的 `map -> base_link` TF。

输出：

- `/mapping/cones`：`driverless_msgs/ConeObservationArray`，固定在 `map`；
- `/mapping/cone_markers`：RViz 标记；
- `/mapping/status`：运行、等待位姿或输入超时状态。

未来接入视觉融合时，把 `input_topic` 改为 `/perception/cones` 即可。定位或 TF
未就绪时节点不会伪造地图坐标。当前规划器仍使用局部感知，不强依赖本包。
