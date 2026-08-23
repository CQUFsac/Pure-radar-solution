#include <algorithm>
#include <cmath>
#include <exception>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace
{

constexpr double kPi = 3.14159265358979323846;

double clamp(const double value, const double lower, const double upper)
{
  return std::max(lower, std::min(upper, value));
}

double smoothStep(const double value)
{
  const double t = clamp(value, 0.0, 1.0);
  return t * t * (3.0 - 2.0 * t);
}

struct WorldPoint
{
  double x = 0.0;
  double y = 0.0;
};

}  // namespace

class FssimSkidpadPath
{
public:
  FssimSkidpadPath()
    : nh_(), private_nh_("~")
  {
    loadParameters();

    odom_sub_ = nh_.subscribe(
      odom_topic_, 10, &FssimSkidpadPath::odometryCallback, this);
    state_sub_ = nh_.subscribe(
      mission_state_topic_, 5, &FssimSkidpadPath::missionStateCallback, this);
    path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic_, 1);
    status_pub_ = nh_.advertise<std_msgs::String>(path_status_topic_, 1, true);

    timer_ = nh_.createTimer(
      ros::Duration(1.0 / publish_rate_),
      &FssimSkidpadPath::timerCallback,
      this);

    ROS_INFO(
      "FSSIM skidpad path active: radius=%.3f m, centres=(%.3f,%+.3f)/(%.3f,%+.3f)",
      radius_,
      right_center_x_, right_center_y_,
      left_center_x_, left_center_y_);
  }

private:
  void loadParameters()
  {
    private_nh_.param<std::string>(
      "odometry_topic", odom_topic_, "/odometry/filtered");
    private_nh_.param<std::string>(
      "mission_state_topic", mission_state_topic_, "/mission/state");
    private_nh_.param<std::string>(
      "path_topic", path_topic_, "/planning/local_path");
    private_nh_.param<std::string>(
      "path_status_topic", path_status_topic_, "/planning/path_status");
    private_nh_.param<std::string>(
      "base_frame", base_frame_, "fssim/vehicle/base_link");

    private_nh_.param("publish_rate", publish_rate_, 30.0);
    private_nh_.param("path_length", path_length_, 22.0);
    private_nh_.param("point_spacing", point_spacing_, 0.20);
    private_nh_.param("recovery_length", recovery_length_, 3.0);
    private_nh_.param("entry_join_x", entry_join_x_, 0.0);
    private_nh_.param("entry_center_y", entry_center_y_, 0.0);
    private_nh_.param("right_center_x", right_center_x_, 0.0);
    private_nh_.param("right_center_y", right_center_y_, -9.125);
    private_nh_.param("left_center_x", left_center_x_, 0.0);
    private_nh_.param("left_center_y", left_center_y_, 9.125);
    private_nh_.param("radius", radius_, 9.125);

    publish_rate_ = std::max(5.0, publish_rate_);
    path_length_ = std::max(5.0, path_length_);
    point_spacing_ = clamp(point_spacing_, 0.05, 1.0);
    recovery_length_ = std::max(0.5, recovery_length_);
    radius_ = std::max(1.0, radius_);
  }

  void odometryCallback(const nav_msgs::Odometry::ConstPtr& message)
  {
    odom_x_ = message->pose.pose.position.x;
    odom_y_ = message->pose.pose.position.y;
    const auto& q = message->pose.pose.orientation;
    odom_yaw_ = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    last_odom_time_ = ros::Time::now();
    has_odometry_ = true;
  }

  void missionStateCallback(const std_msgs::String::ConstPtr& message)
  {
    mission_state_ = message->data;
    has_mission_state_ = true;
  }

  void appendEntry(std::vector<WorldPoint>& points) const
  {
    const double distance_to_join = std::max(0.0, entry_join_x_ - odom_x_);
    const double line_length = std::min(path_length_, distance_to_join);
    const int line_steps = std::max(
      1, static_cast<int>(std::ceil(line_length / point_spacing_)));

    for (int index = 0; index <= line_steps; ++index)
    {
      const double distance = std::min(
        line_length, static_cast<double>(index) * point_spacing_);
      const double blend = distance_to_join > 1.0e-6 ?
        smoothStep(distance / distance_to_join) : 1.0;
      points.push_back(WorldPoint{
        odom_x_ + distance,
        odom_y_ + blend * (entry_center_y_ - odom_y_)});
    }

    const double remaining = path_length_ - line_length;
    if (remaining <= point_spacing_)
    {
      return;
    }

    // The top point of the lower circle is tangent to the +X entry line.
    const double start_angle = kPi * 0.5;
    const int arc_steps = std::max(
      1, static_cast<int>(std::ceil(remaining / point_spacing_)));
    for (int index = 1; index <= arc_steps; ++index)
    {
      const double distance = std::min(
        remaining, static_cast<double>(index) * point_spacing_);
      const double angle = start_angle - distance / radius_;
      points.push_back(WorldPoint{
        right_center_x_ + radius_ * std::cos(angle),
        right_center_y_ + radius_ * std::sin(angle)});
    }
  }

  void appendCircle(
    const double center_x,
    const double center_y,
    const double direction,
    std::vector<WorldPoint>& points) const
  {
    const double dx = odom_x_ - center_x;
    const double dy = odom_y_ - center_y;
    const double current_radius = std::max(0.1, std::hypot(dx, dy));
    const double start_angle = std::atan2(dy, dx);
    const int steps = std::max(
      2, static_cast<int>(std::ceil(path_length_ / point_spacing_)));

    for (int index = 0; index <= steps; ++index)
    {
      const double distance = std::min(
        path_length_, static_cast<double>(index) * point_spacing_);
      const double blend = smoothStep(distance / recovery_length_);
      const double blended_radius =
        current_radius + blend * (radius_ - current_radius);
      const double angle =
        start_angle + direction * distance / radius_;
      points.push_back(WorldPoint{
        center_x + blended_radius * std::cos(angle),
        center_y + blended_radius * std::sin(angle)});
    }
  }

  void appendExit(std::vector<WorldPoint>& points) const
  {
    const int steps = std::max(
      2, static_cast<int>(std::ceil(path_length_ / point_spacing_)));
    for (int index = 0; index <= steps; ++index)
    {
      const double distance = std::min(
        path_length_, static_cast<double>(index) * point_spacing_);
      const double blend = smoothStep(distance / recovery_length_);
      points.push_back(WorldPoint{
        odom_x_ + distance,
        odom_y_ + blend * (entry_center_y_ - odom_y_)});
    }
  }

  bool buildWorldPath(std::vector<WorldPoint>& points) const
  {
    // Publish the entry reference before the mission manager finishes its
    // sensor-ready handshake.  /mission/stop still prevents vehicle motion;
    // path availability and drive authorisation are separate concerns.
    if (!has_mission_state_ ||
        mission_state_ == "WAIT_GO" ||
        mission_state_ == "ENTER")
    {
      if (odom_x_ < entry_join_x_)
      {
        appendEntry(points);
      }
      else
      {
        appendCircle(
          right_center_x_, right_center_y_, -1.0, points);
      }
      return true;
    }

    if (mission_state_ == "RIGHT_LAP_1" ||
        mission_state_ == "RIGHT_LAP_2")
    {
      appendCircle(right_center_x_, right_center_y_, -1.0, points);
      return true;
    }

    if (mission_state_ == "TRANSITION_TO_LEFT" ||
        mission_state_ == "LEFT_LAP_1" ||
        mission_state_ == "LEFT_LAP_2")
    {
      appendCircle(left_center_x_, left_center_y_, 1.0, points);
      return true;
    }

    if (mission_state_ == "EXIT")
    {
      appendExit(points);
      return true;
    }

    return false;
  }

  void publishStatus(const std::string& text)
  {
    std_msgs::String status;
    status.data = text;
    status_pub_.publish(status);
  }

  void timerCallback(const ros::TimerEvent&)
  {
    if (!has_odometry_ ||
        (ros::Time::now() - last_odom_time_).toSec() > 0.5)
    {
      publishStatus("INVALID_SKIDPAD: odometry unavailable");
      return;
    }
    std::vector<WorldPoint> world_points;
    world_points.reserve(
      static_cast<std::size_t>(path_length_ / point_spacing_) + 4U);
    if (!buildWorldPath(world_points) || world_points.size() < 2U)
    {
      publishStatus("INVALID_SKIDPAD: inactive state=" + mission_state_);
      return;
    }

    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = base_frame_;
    path.poses.reserve(world_points.size());

    const double cosine = std::cos(odom_yaw_);
    const double sine = std::sin(odom_yaw_);
    for (const WorldPoint& world : world_points)
    {
      const double dx = world.x - odom_x_;
      const double dy = world.y - odom_y_;

      geometry_msgs::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = cosine * dx + sine * dy;
      pose.pose.position.y = -sine * dx + cosine * dy;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }

    path_pub_.publish(path);
    const std::string active_state = has_mission_state_ ?
      mission_state_ : "PRESTART";
    publishStatus(
      "VALID_SKIDPAD_REFERENCE: state=" + active_state +
      "; points=" + std::to_string(path.poses.size()));
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber odom_sub_;
  ros::Subscriber state_sub_;
  ros::Publisher path_pub_;
  ros::Publisher status_pub_;
  ros::Timer timer_;

  std::string odom_topic_;
  std::string mission_state_topic_;
  std::string path_topic_;
  std::string path_status_topic_;
  std::string base_frame_;
  std::string mission_state_;

  bool has_odometry_ = false;
  bool has_mission_state_ = false;
  ros::Time last_odom_time_;
  double odom_x_ = 0.0;
  double odom_y_ = 0.0;
  double odom_yaw_ = 0.0;

  double publish_rate_ = 30.0;
  double path_length_ = 22.0;
  double point_spacing_ = 0.20;
  double recovery_length_ = 3.0;
  double entry_join_x_ = 0.0;
  double entry_center_y_ = 0.0;
  double right_center_x_ = 0.0;
  double right_center_y_ = -9.125;
  double left_center_x_ = 0.0;
  double left_center_y_ = 9.125;
  double radius_ = 9.125;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fssim_skidpad_path");
  try
  {
    FssimSkidpadPath node;
    ros::spin();
  }
  catch (const std::exception& error)
  {
    ROS_FATAL("fssim_skidpad_path failed: %s", error.what());
    return 1;
  }
  return 0;
}
