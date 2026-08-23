#include <ros/ros.h>

#include <fssim_common/State.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <string>

class FssimOdomBridge
{
public:
  FssimOdomBridge()
    : nh_(),
      private_nh_("~")
  {
    private_nh_.param<std::string>(
      "input_state_topic", input_state_topic_, "/fssim/base_pose_ground_truth");
    private_nh_.param<std::string>(
      "output_odom_topic", output_odom_topic_, "/odometry/filtered");
    private_nh_.param<std::string>("odom_frame", odom_frame_, "fssim_map");
    private_nh_.param<std::string>("base_frame", base_frame_, "base_link");
    private_nh_.param("publish_tf", publish_tf_, true);
    private_nh_.param("publish_imu", publish_imu_, false);
    private_nh_.param<std::string>(
      "output_imu_topic", output_imu_topic_, "/imu/data");

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(output_odom_topic_, 10);
    if (publish_imu_)
    {
      imu_pub_ = nh_.advertise<sensor_msgs::Imu>(output_imu_topic_, 20);
    }
    state_sub_ = nh_.subscribe(
      input_state_topic_, 20, &FssimOdomBridge::stateCallback, this);

    ROS_INFO(
      "fssim_odom_bridge: %s -> %s, TF %s -> %s (%s)",
      input_state_topic_.c_str(),
      output_odom_topic_.c_str(),
      odom_frame_.c_str(),
      base_frame_.c_str(),
      publish_tf_ ? "on" : "off");
  }

private:
  void stateCallback(const fssim_common::State::ConstPtr& msg)
  {
    const ros::Time stamp =
      msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;

    // Some FSSIM runs publish the same ground-truth sample more than once.
    // Broadcasting every duplicate produces TF_REPEATED_DATA and makes RViz
    // appear unstable, so one timestamp is converted exactly once.
    if (!last_state_stamp_.isZero() && stamp == last_state_stamp_)
    {
      ROS_DEBUG_THROTTLE(
        2.0, "fssim_odom_bridge: duplicate state timestamp ignored");
      return;
    }
    last_state_stamp_ = stamp;

    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    odom.pose.pose.position.x = msg->x;
    odom.pose.pose.position.y = msg->y;
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, msg->yaw);
    orientation.normalize();
    odom.pose.pose.orientation.x = orientation.x();
    odom.pose.pose.orientation.y = orientation.y();
    odom.pose.pose.orientation.z = orientation.z();
    odom.pose.pose.orientation.w = orientation.w();

    odom.twist.twist.linear.x = msg->vx;
    odom.twist.twist.linear.y = msg->vy;
    odom.twist.twist.angular.z = msg->r;

    // These are initial simulation values, not real sensor covariances.
    odom.pose.covariance[0] = 0.01;
    odom.pose.covariance[7] = 0.01;
    odom.pose.covariance[35] = 0.0025;
    odom.twist.covariance[0] = 0.01;
    odom.twist.covariance[7] = 0.01;
    odom.twist.covariance[35] = 0.0025;

    odom_pub_.publish(odom);

    // The skidpad mission manager only needs yaw rate.  Publish the exact
    // simulated yaw rate as an IMU-shaped message so the normal mission
    // state machine can be reused without changing the planner interface.
    if (publish_imu_)
    {
      sensor_msgs::Imu imu;
      imu.header.stamp = stamp;
      imu.header.frame_id = base_frame_;
      imu.orientation = odom.pose.pose.orientation;
      imu.angular_velocity.z = msg->r;
      imu.orientation_covariance[8] = 0.0025;
      imu.angular_velocity_covariance[8] = 0.0025;
      imu.linear_acceleration_covariance[0] = -1.0;
      imu_pub_.publish(imu);
    }

    if (!publish_tf_)
    {
      return;
    }

    geometry_msgs::TransformStamped transform;
    transform.header = odom.header;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = msg->x;
    transform.transform.translation.y = msg->y;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_.sendTransform(transform);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber state_sub_;
  ros::Publisher odom_pub_;
  ros::Publisher imu_pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  std::string input_state_topic_;
  std::string output_odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string output_imu_topic_;
  bool publish_tf_;
  bool publish_imu_;
  ros::Time last_state_stamp_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fssim_odom_bridge");
  FssimOdomBridge bridge;
  ros::spin();
  return 0;
}
