# LiDAR 锥桶聚类与单帧检测模块说明

## 1. 模块定位

`lidar_cone_detector` 是面向 SCOUT MINI 无人方程式算法验证平台的 ROS1 Melodic 功能包。

当前模块使用 RS-Helios-16P 的非地面点云，完成：

1. 距离自适应欧式聚类；
2. 点簇几何与 PCA 特征提取；
3. 基于规则和置信度的锥桶候选判断；
4. 锥桶底部中心位置估计；
5. 位置协方差计算；
6. ROS 锥桶候选消息和调试点云发布。

本模块只负责**单帧检测**，不包含多帧跟踪、锥桶地图、左右边界、路径规划或车辆控制。

纯 LiDAR 无法识别蓝锥、黄锥和橙锥，因此当前输出语义固定为 `UNKNOWN`，不根据车辆左右位置编造颜色。

## 2. 总体数据流

```text
/lidar/non_ground_points
sensor_msgs/PointCloud2
        │
        ▼
PointCloud2 字段解析
保留 x、y、z、intensity、ring、time/timestamp
        │
        ▼
按雷达平面距离分成四段
0.5～5 m / 5～10 m / 10～15 m / 15～20 m
        │
        ▼
每一段独立建立 KD-Tree
        │
        ▼
PCL EuclideanClusterExtraction
        │
        ▼
ClusterResult[]
        │
        ▼
几何特征与 PCA 特征提取
        │
        ▼
规则硬过滤 + 柔性评分
        │
        ├── REJECTED：删除
        ├── WEAK：弱候选，等待跟踪确认
        └── STRONG：当前帧强候选
        │
        ▼
可见表面距离 + 横向中位数 + 半径修正
        │
        ▼
锥桶底部中心和位置协方差
        │
        ▼
/perception/lidar/cones_raw
driverless_msgs/ConeObservationArray
```

## 3. 上游输入要求

### 3.1 输入话题

| 项目 | 要求 |
|---|---|
| 话题 | `/lidar/non_ground_points` |
| 类型 | `sensor_msgs/PointCloud2` |
| 内容 | ROI 裁剪、车体点删除和地面分割后的非地面点 |
| 必需字段 | `x`、`y`、`z` |
| 建议保留字段 | `intensity`、`ring`、`time` 或 `timestamp` |

节点兼容名为 `time` 或 `timestamp` 的单点时间字段。字段的实际数据类型可以是常见整数或浮点类型，节点会统一转换到内部点类型。

### 3.2 Header 要求

上游必须保留雷达原始帧的：

- `header.stamp`；
- `header.frame_id`。

当前模块的所有输出继续使用同一时间戳和坐标系。

### 3.3 坐标系要求

特征提取默认假设：

- x 轴指向车辆前方；
- y 轴指向车辆左侧；
- z 轴指向上方。

推荐上游将点云转换到 `base_link`。如果输入仍为 `lidar_link`，必须保证雷达安装外参正确，并在参数中设置雷达原点。

## 4. ROS 输出话题

| 话题 | 类型 | 用途 |
|---|---|---|
| `/perception/lidar/cones_raw` | `driverless_msgs/ConeObservationArray` | 单帧 LiDAR 锥桶候选 |
| `/lidar/debug/clustered_points` | `sensor_msgs/PointCloud2` | 所有欧式聚类彩色显示 |
| `/lidar/debug/cone_candidate_points` | `sensor_msgs/PointCloud2` | 仅显示保留的锥桶候选 |
| `/lidar/debug/cone_positions` | `geometry_msgs/PoseArray` | 在 RViz 中显示估计位置 |

候选调试点云中：

- 绿色表示 `STRONG`；
- 橙色表示 `WEAK`。

## 5. 文件职责

### 5.1 ROS 主节点

`src/lidar_cone_detector_node.cpp`

这是本功能包可执行程序的主入口，包含唯一的 `main()`。主要负责：

- 初始化 ROS；
- 读取 YAML 参数；
- 订阅非地面点云；
- 创建各算法模块；
- 在回调中顺序调用聚类、特征、分类和位置估计；
- 发布锥桶消息和调试话题。

算法文件不需要各自编写 `main()`。

### 5.2 点类型

`include/lidar_cone_detector/point_type.hpp`

内部点类型 `PointXYZIRT` 保存：

```text
x、y、z、intensity、ring、time
```

### 5.3 距离自适应欧式聚类

```text
include/lidar_cone_detector/adaptive_euclidean_clusterer.hpp
src/adaptive_euclidean_clusterer.cpp
```

主要负责：

- 删除 NaN/Inf；
- 按平面距离分段；
- 每段使用独立聚类半径和点数限制；
- 建立 PCL KD-Tree；
- 执行欧式聚类；
- 保留点簇到原输入点云的索引映射；
- 为当前帧点簇分配临时 ID。

这里的 ID 每帧从零开始，不是跟踪 ID，也不能直接保存到地图中。

### 5.4 几何和 PCA 特征

```text
include/lidar_cone_detector/geometric_feature_extractor.hpp
src/geometric_feature_extractor.cpp
```

输出特征包括：

- 有效点数；
- 最小、最大和平均距离；
- 三维 min/max；
- 点簇质心；
- 高度；
- x/y 方向尺寸；
- 雷达径向深度；
- 与雷达观察方向垂直的横向宽度；
- intensity 最小值、最大值、均值和标准差；
- ring 数量和跨度；
- 协方差矩阵特征值；
- 线性度、平面度、散乱度和竖直度；
- 点簇上部、中部和下部宽度；
- 上下宽度比例。

特征提取器只负责测量，不负责判断目标是否为锥桶。

独立的 `tensor_feature_extractor` 当前不参与编译。所需的协方差和特征值张量计算已经整合到 `geometric_feature_extractor` 中，避免重复计算。

### 5.5 锥桶候选分类

```text
include/lidar_cone_detector/cone_cluster_classifier.hpp
src/cone_cluster_classifier.cpp
```

分类包含两个阶段。

第一阶段为硬过滤：

- 特征无效；
- 点数不足；
- 近中距离目标明显过矮或过窄；
- 所有距离目标明显过高、过宽或过深；
- 可选的接地高度异常。

第二阶段为柔性评分：

- 高度评分；
- 横向宽度评分；
- 径向深度评分；
- 可选接地评分；
- 上下宽度比例评分；
- PCA 形状评分；
- ring 数量评分。

最终置信度由几何结构分数与测量可靠性共同生成：

```text
confidence = structural_score × measurement_reliability
```

10 m 以后的目标即使形状得分较高，也只能成为 `WEAK`，必须由后续多帧跟踪确认。

### 5.6 锥桶位置估计

```text
include/lidar_cone_detector/cone_position_estimator.hpp
src/cone_position_estimator.cpp
```

位置估计不能直接使用全部点的平均值，因为雷达通常只能看到锥桶朝向雷达的一侧。

当前方法：

1. 根据点簇质心建立雷达径向和横向坐标；
2. 使用较近距离分位点估计可见表面；
3. 使用横向中位数估计横向中心；
4. 使用点簇宽度或标称锥桶半径进行径向补偿；
5. 估计底部中心 x、y；
6. 使用固定地面 z 或点簇最低点估计 z；
7. 根据距离段、点数和分类置信度生成 3×3 位置协方差。

如果地面分割删除了锥桶底部，使用点簇最低点得到的 z 会偏高。获得可靠地面模型后，应启用固定地面高度或将局部地面平面传入位置估计器。

## 6. ConeObservation 消息

`driverless_msgs/msg/ConeObservation.msg` 保存：

- Header；
- 单帧点簇 ID；
- 三维位置；
- 3×3 位置协方差；
- 来源；
- 语义类别；
- 候选等级；
- 存在概率；
- LiDAR 置信度；
- 相机颜色置信度；
- 高度、宽度和深度；
- 点数；
- 距离段；
- 是否经过跟踪确认。

当前纯 LiDAR 节点填写规则：

| 字段 | 当前值 |
|---|---|
| `source` | `SOURCE_LIDAR` |
| `semantic_class` | `SEMANTIC_UNKNOWN` |
| `camera_color_confidence` | 0 |
| `confirmed` | false |
| `candidate_level` | `WEAK` 或 `STRONG` |

后续相机融合节点可以保留 LiDAR 三维位置和协方差，只补充颜色语义与颜色置信度。

## 7. 主要配置参数

参数文件：`config/clustering.yaml`

### 7.1 欧式聚类初始建议值

| 距离段 | 范围 | 聚类半径 | 最少点数 |
|---|---:|---:|---:|
| near | 0.5～5 m | 0.10 m | 4 |
| middle | 5～10 m | 0.15 m | 3 |
| far | 10～15 m | 0.22 m | 2 |
| very_far | 15～20 m | 0.30 m | 2 |

这些数值均为**初始建议值**，必须通过实际 RS-Helios-16P rosbag 调整。

### 7.2 必须实测的参数

- `features/lidar_origin_x`；
- `features/lidar_origin_y`；
- 实际锥桶高度和底部宽度；
- `position/nominal_cone_radius`；
- `classifier/expected_ground_z`；
- 四段聚类半径；
- 分类硬范围与理想范围；
- `minimum_structural_score`；
- `strong_confidence_threshold`。

在地面高度没有可靠测量前，保持：

```yaml
classifier:
  use_ground_check: false
```

## 8. 调试顺序

建议按以下顺序检查，不能直接只看最终锥桶坐标。

### 8.1 输入检查

确认：

- 非地面点云有数据；
- `frame_id` 正确；
- 时间戳持续更新；
- x/y/z 字段存在；
- intensity、ring、time/timestamp 尽量保留。

### 8.2 聚类检查

观察 `/lidar/debug/clustered_points`：

- 一个锥桶尽量形成一个点簇；
- 相邻锥桶不要粘成一个簇；
- 同一个锥桶不要被拆成多个簇；
- 5 m、10 m、15 m 分段附近重点检查碎裂问题。

### 8.3 分类检查

观察 `/lidar/debug/cone_candidate_points`：

- 近距离完整锥桶应尽量为绿色强候选；
- 远距离少点目标应为橙色弱候选；
- 墙面、车体、路缘和大物体应被拒绝；
- 不应根据左右位置输出蓝色或黄色。

### 8.4 位置检查

观察 `/lidar/debug/cone_positions`：

- 位置应接近锥桶底部中心；
- 不应明显停留在锥桶朝向雷达的一侧表面；
- 横向位置不应因单个噪声点大幅跳动；
- 距离越远，位置协方差应越大。

## 9. 单帧验收建议

以下均为初始验收建议，需要实车确认。

| 测试 | 初始验收要求 |
|---|---|
| 3 m 单锥桶 | 连续帧大部分时间产生候选 |
| 5 m 单锥桶 | 不在分段边界持续分裂 |
| 8～10 m 单锥桶 | 允许弱候选，不要求每帧强候选 |
| 15 m 以上 | 只作为弱候选，不承诺稳定单帧检测 |
| 两个并排锥桶 | 尽量形成两个独立点簇 |
| 墙面或大纸箱 | 因尺寸过大被拒绝 |
| 无目标场景 | 假候选数量可控，不连续稳定出现 |

## 10. 已知限制

1. 16 线雷达在远距离锥桶上可能只有 2～3 个点，单帧几何特征不可靠。
2. 欧式聚类分段边界可能将同一个锥桶拆成两部分。
3. 强度受距离、材料和入射角影响，当前不作为硬判断条件。
4. 点簇最低点不一定是锥桶底面，尤其是在地面分割删除底部后。
5. 纯 LiDAR 无法判断锥桶颜色。
6. 单帧分类无法区分赛道锥桶和赛道外完全相同的备用锥桶。
7. 当前 ID 每帧重置，不能直接用于地图。
8. 当前没有点云运动补偿；速度提高后需要结合单点时间和车辆运动处理。

## 11. 后续模块接口

下一模块建议为 `cone_tracker`：

```text
/perception/lidar/cones_raw
        │
        ▼
车辆运动补偿
        │
        ▼
马氏距离门限 + 最近邻/匈牙利匹配
        │
        ▼
NEW / TENTATIVE / CONFIRMED / LOST / DELETED
        │
        ▼
/perception/cones_tracked
```

跟踪器负责：

- 分配跨帧稳定 ID；
- 利用多帧确认远距离 2～3 点目标；
- 删除短暂噪声；
- 输出 `confirmed=true` 的目标；
- 为后续锥桶地图和赛道边界提供输入。

相机融合后，统一感知选择器再输出：

```text
/perception/cones
```

规划和地图模块只订阅统一话题，不直接依赖相机节点是否运行。

## 12. 当前完成状态

已完成：

- ROS1 节点结构；
- PointCloud2 字段兼容转换；
- 四段欧式聚类；
- 几何和 PCA 特征；
- 距离自适应规则分类；
- 底部中心修正；
- 位置协方差；
- 公共 `driverless_msgs` ROS 消息；
- 调试点云和位置输出；
- CMake、package、launch 和 YAML 接口。

尚未完成：

- ROS Melodic 环境中的实际编译验证；
- RS-Helios-16P rosbag 参数标定；
- 多帧跟踪；
- 锥桶地图；
- 无颜色赛道边界；
- 相机颜色融合。

在实车使用前，必须先完成静态单锥桶、不同距离、并排双锥桶和假目标场景的数据采集与参数调整。
