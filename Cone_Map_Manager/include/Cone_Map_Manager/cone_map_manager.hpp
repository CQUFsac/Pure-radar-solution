#ifndef CONE_MAP_MANAGER_HPP
#define CONE_MAP_MANAGER_HPP

#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <driverless_msgs/ConeObservationArray.h>
#include <deque>

struct Point2D {
  double x;
  double y;
};

struct car_position {
  double x;
  double y;
  double timestamp;
};

struct cone_position {
  double x;
  double y;
  double timestamp;
  std::string color; // 锥桶颜色
};


class ConeMapManager
{
public:
  ConeMapManager(ros::NodeHandle& nh, ros::NodeHandle& private_nh);

  // Get the latest IMU data
  sensor_msgs::Imu getLatestImuData() const;

  // Get the latest LiDAR cone detection data
  driverless_msgs::ConeObservationArray getLatestLidarConesData() const;

  // Check if data has been received
  bool hasImuData() const;
  bool hasLidarConesData() const;
  void imuPosition_();
  void imuPosition_initialize();
  void LidarCones_initialize();
  bool CheckConeOverlap(const cone_position& , const std::vector<cone_position>& , double );

private:
  void imuCallback(const sensor_msgs::ImuConstPtr& msg);
  void lidarConesCallback(const driverless_msgs::ConeObservationArrayConstPtr& msg);


  ros::Subscriber imu_sub_;
  ros::Subscriber lidar_cones_sub_;

  std::deque<sensor_msgs::Imu> imu_data_buffer_;  // 存储多条 IMU 数据
  driverless_msgs::ConeObservationArray latest_lidar_cones_data_;

  bool imu_received_;
  bool lidar_cones_received_;

  mutable std::mutex imu_mutex_;
  mutable std::mutex lidar_cones_mutex_;

  Point2D origin_point;
  Point2D vehicle_point;

  double imu_sampling_interval;  // IMU采样时间间隔
  car_position vehicle_position;  // 车辆位置结构体

  std::vector<Cone_Map_Manager::ConePosition> cone_points;
};

#endif  // CONE_MAP_MANAGER_HPP