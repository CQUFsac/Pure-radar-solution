#include "Cone_Map_Manager/cone_map_manager.hpp"

/*
  无人车地图坐标点采用2维结构
 * 如需改动imu数据寄存器imu_data_buffer_的限制数量，请在cimuCallback中修改imu_data_buffer_的最大数量限制。
*/


ConeMapManager::ConeMapManager(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
  : imu_received_(false)
  , lidar_cones_received_(false)
{
  // Read IMU subscriber parameters from parameter server
  std::string imu_topic;
  int imu_queue_size;
  private_nh.param<std::string>("imu_topic", imu_topic, "/imu/data");
  private_nh.param<int>("imu_queue_size", imu_queue_size, 10);

  // Read LiDAR cone detection subscriber parameters from parameter server
  std::string lidar_cones_topic;
  int lidar_cones_queue_size;
  private_nh.param<std::string>("lidar_cones_topic", lidar_cones_topic, "/perception/lidar/cones_raw");
  private_nh.param<int>("lidar_cones_queue_size", lidar_cones_queue_size, 5);

  private_nh.param<double>("imu_sampling_interval", imu_sampling_interval, 0.01);

  // Subscribe to IMU data
  imu_sub_ = nh.subscribe(imu_topic, imu_queue_size, &ConeMapManager::imuCallback, this);
  ROS_INFO_STREAM("ConeMapManager subscribed to IMU topic: " << imu_topic
                  << " (queue_size: " << imu_queue_size << ")");

  // Subscribe to LiDAR cone detection data
  lidar_cones_sub_ = nh.subscribe(lidar_cones_topic, lidar_cones_queue_size,
                                  &ConeMapManager::lidarConesCallback, this);
  ROS_INFO_STREAM("ConeMapManager subscribed to LiDAR cones topic: " << lidar_cones_topic
                  << " (queue_size: " << lidar_cones_queue_size << ")");

  // Initialize vehicle position in IMU coordinate system
  imuPosition_initialize();
  
  std::vector <cone_map_manager::ConePosition> cone_position; // Initialize cone position data
}

void ConeMapManager::imuCallback(const sensor_msgs::ImuConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(imu_mutex_);
  imu_data_buffer_.push_back(*msg);     // 追加到末尾
  if (imu_data_buffer_.size() > 10)   // 限制最大数量
    imu_data_buffer_.pop_front();       // 移除最旧的数据
  imu_received_ = true;

  // Update vehicle position based on IMU data
  imuPosition_();
}

void ConeMapManager::lidarConesCallback(const driverless_msgs::ConeObservationArrayConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(lidar_cones_mutex_);
  latest_lidar_cones_data_ = *msg;
  lidar_cones_received_ = true;


}

sensor_msgs::Imu ConeMapManager::getLatestImuData() const
{
  std::lock_guard<std::mutex> lock(imu_mutex_);
  return latest_imu_data_;
}

driverless_msgs::ConeObservationArray ConeMapManager::getLatestLidarConesData() const
{
  std::lock_guard<std::mutex> lock(lidar_cones_mutex_);
  return latest_lidar_cones_data_;
}

bool ConeMapManager::hasImuData() const
{
  std::lock_guard<std::mutex> lock(imu_mutex_);
  return imu_received_;
}

bool ConeMapManager::hasLidarConesData() const
{
  std::lock_guard<std::mutex> lock(lidar_cones_mutex_);
  return lidar_cones_received_;
}


//节点1: imu坐标变换下的车辆位置标定

//1.1 构建坐标系
void ConeMapManager::imuPosition_initialize()
{
  //构造imu坐标系下的车辆位置

  //构造原点坐标
  
  origin_point.x = 0.0;
  origin_point.y = 0.0;

  //构造车辆坐标点
  
  vehicle_point.x = 0; //初始化为0
  vehicle_point.y = 0; //初始化为0
}

void ConeMapManager::imuPosition_()
{
  std::lock_guard<std::mutex> lock(imu_mutex_);
  // Copy the IMU data buffer to avoid holding the lock for too long
  std::deque<sensor_msgs::Imu> imu_data_buffer_temp = imu_data_buffer_;
  //imu积分
  for (int i = 0; i < imu_data_buffer_temp.size(); ++i)
  {
    const auto& imu_data = imu_data_buffer_temp[i];
    //积分计算车辆位置
    vehicle_point.x += imu_data.linear_acceleration.x * pow(imu_sampling_interval, 2) ;
    vehicle_point.y += imu_data.linear_acceleration.y * pow(imu_sampling_interval, 2) ;
  }

  vehicle_position.x = vehicle_point.x;
  vehicle_position.y = vehicle_point.y;
  vehicle_position.timestamp = imu_data_buffer_temp.back().header.stamp.toSec(); //记录时间戳

}

void ConeMapManager::LidarCones_position()
{
  driverless_msgs::ConeObservationArray lidar_cones_data = getLatestLidarConesData();
  //在这里对lidar_cones_data进行处理，获取锥桶的位置数据，并将其转换为车辆坐标系下的坐标

  //1.坐标变换

  //提取锥桶2维位置数据
  for (const auto& cone : lidar_cones_data.cones)
  {
    double cone_x = cone.position.x;
    double cone_y = cone.position.y;

    //将锥桶位置转换为车辆坐标系下的坐标

    //这里的vehicle_position再读取的时候可能正在被更改，后续考虑锁存逻辑防止这种情况
    double transformed_x = cone_x + vehicle_position.x;
    double transformed_y = cone_y + vehicle_position.y;
    cone_position cone_temp;
    cone_temp.x = transformed_x;
    cone_temp.y = transformed_y;

    //将该锥桶坐标装入相应的数据类型中

    

    //先检查锥桶是否重合
  
    if (CheckConeOverlap(cone_temp,cone_points))
      {
        //如果重合，就把重合点融合为一个锥桶坐标（两个取中点）

      }

    cone_points.push_back({transformed_x, transformed_y, cone.header.stamp.toSec(), cone.color});
  }
  
}

bool ConeMapManager::CheckConeOverlap(const cone_position& new_cone, const std::vector<Cone_Map_Manager::ConePosition>& existing_cones, double threshold)
{
  int n = existing_cones.size();
  int cout = std::min(3,n);
  for (int i = existing_cones.size(); i >  existing_cones.size()- cout; i--)
  {
    double distance = std::sqrt(std::pow(new_cone.x - existing_cone[i].x, 2) + std::pow(new_cone.y - existing_cone[i].y, 2));
    if (distance < threshold)
    {
      return true; // Found an overlapping cone
    }
  }
  return false; // No overlap found
}





