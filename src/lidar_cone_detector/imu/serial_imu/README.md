# serial_imu

该包保留队友提交的串口协议解析器，但统一发布：

```text
/imu/data  sensor_msgs/Imu
frame_id: imu_link
```

串口、波特率、坐标系和输出话题均由 launch 参数配置。默认值：

```bash
roslaunch serial_imu imu_msg.launch \
  port:=/dev/imu baud_rate:=115200 output_topic:=/imu/data
```

重要：现有解析器最初标注为 HI226/IMUSOL 协议，不能仅凭“能打开串口”就认定与
CH110 兼容。接真车前必须使用 CH110 厂家协议核对帧头、量纲、轴向、波特率和校验，
并检查左转时 `angular_velocity.z > 0`、右转时 `< 0`。

当前时间戳是 Xavier 接收到完整数据后的 ROS 时间，不是设备硬件时间。
