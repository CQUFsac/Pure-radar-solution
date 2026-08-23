#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cstdint>
#include <string>

struct EIGEN_ALIGN16 RsPointXYZIRT
{
  PCL_ADD_POINT4D;
  std::uint8_t intensity = 0U;
  std::uint16_t ring = 0U;
  double timestamp = 0.0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
  RsPointXYZIRT,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (std::uint8_t, intensity, intensity)
  (std::uint16_t, ring, ring)
  (double, timestamp, timestamp)
)

class RoiFilterNode
{
public:
  RoiFilterNode()
    : private_nh_("~")
  {
    private_nh_.param("roi_xmin", roi_[0], 0.5F);
    private_nh_.param("roi_xmax", roi_[1], 20.0F);
    private_nh_.param("roi_ymin", roi_[2], -5.0F);
    private_nh_.param("roi_ymax", roi_[3], 5.0F);
    private_nh_.param("roi_zmin", roi_[4], -0.6F);
    private_nh_.param("roi_zmax", roi_[5], 1.5F);
    private_nh_.param("ground_z_threshold", ground_z_threshold_, -0.25F);

    std::string input_topic;
    std::string roi_topic;
    std::string ground_topic;
    std::string non_ground_topic;
    private_nh_.param<std::string>(
      "input_topic", input_topic, "/rslidar_points");
    private_nh_.param<std::string>(
      "roi_topic", roi_topic, "/lidar/roi_points");
    private_nh_.param<std::string>(
      "ground_topic", ground_topic, "/lidar/ground_points");
    private_nh_.param<std::string>(
      "non_ground_topic",
      non_ground_topic,
      "/lidar/non_ground_points");

    cloud_sub_ = nh_.subscribe(
      input_topic, 2, &RoiFilterNode::cloudCallback, this);
    roi_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(roi_topic, 2);
    ground_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>(ground_topic, 2);
    non_ground_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>(non_ground_topic, 2);

    ROS_INFO(
      "roi_filter: input=%s, non_ground=%s, ground_z=%.3f",
      input_topic.c_str(),
      non_ground_topic.c_str(),
      ground_z_threshold_);
  }

private:
  void cloudCallback(
    const sensor_msgs::PointCloud2ConstPtr& input_message)
  {
    pcl::PointCloud<RsPointXYZIRT> input;
    pcl::fromROSMsg(*input_message, input);

    pcl::PointCloud<RsPointXYZIRT> roi;
    pcl::PointCloud<RsPointXYZIRT> ground;
    pcl::PointCloud<RsPointXYZIRT> non_ground;
    roi.reserve(input.size());
    ground.reserve(input.size());
    non_ground.reserve(input.size());

    for (const RsPointXYZIRT& point : input)
    {
      const bool inside =
        point.x >= roi_[0] && point.x <= roi_[1] &&
        point.y >= roi_[2] && point.y <= roi_[3] &&
        point.z >= roi_[4] && point.z <= roi_[5];
      if (!inside)
      {
        continue;
      }

      roi.push_back(point);
      if (point.z < ground_z_threshold_)
      {
        ground.push_back(point);
      }
      else
      {
        non_ground.push_back(point);
      }
    }

    publishCloud(roi, input_message->header, roi_pub_);
    publishCloud(ground, input_message->header, ground_pub_);
    publishCloud(
      non_ground, input_message->header, non_ground_pub_);
  }

  static void publishCloud(
    const pcl::PointCloud<RsPointXYZIRT>& cloud,
    const std_msgs::Header& header,
    ros::Publisher& publisher)
  {
    sensor_msgs::PointCloud2 message;
    pcl::toROSMsg(cloud, message);
    message.header = header;
    publisher.publish(message);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber cloud_sub_;
  ros::Publisher roi_pub_;
  ros::Publisher ground_pub_;
  ros::Publisher non_ground_pub_;
  float roi_[6];
  float ground_z_threshold_ = -0.25F;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "roi_filter_node");
  RoiFilterNode node;
  ros::spin();
  return 0;
}
