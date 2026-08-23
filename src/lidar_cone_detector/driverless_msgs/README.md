# driverless_msgs

SCOUT MINI 无人驾驶算法验证平台的公共 ROS1 消息包。

本包只存放跨模块使用的数据接口，不包含检测、跟踪、建图、规划或控制算法。这样 `cone_tracker`、`cone_mapper`、`lidar_camera_fusion` 和 `perception_selector` 不需要依赖 `lidar_cone_detector` 算法包。

## 当前消息

- `ConeObservation.msg`：单个锥桶观测。
- `ConeObservationArray.msg`：同一传感器帧中的锥桶观测数组。

LiDAR 单帧检测输出话题保持为：

```text
/perception/lidar/cones_raw
driverless_msgs/ConeObservationArray
```

纯 LiDAR 节点只能将 `semantic_class` 填为 `SEMANTIC_UNKNOWN`。颜色类别由后续相机检测与目标级融合模块补充。
