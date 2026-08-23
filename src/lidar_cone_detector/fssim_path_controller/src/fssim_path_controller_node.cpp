#include <ros/ros.h>

#include <fssim_common/Cmd.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

class FssimPathController
{
public:
  FssimPathController()
    : private_nh_("~")
  {
    loadParameters();

    path_sub_ = nh_.subscribe(
      input_path_topic_, 1, &FssimPathController::pathCallback, this);
    path_status_sub_ = nh_.subscribe(
      input_path_status_topic_,
      1,
      &FssimPathController::pathStatusCallback,
      this);
    odom_sub_ = nh_.subscribe(
      input_odom_topic_, 5, &FssimPathController::odometryCallback, this);

    cmd_pub_ = nh_.advertise<fssim_common::Cmd>(output_cmd_topic_, 1);
    status_pub_ = nh_.advertise<std_msgs::String>(
      output_status_topic_, 1, true);
    control_timer_ = nh_.createTimer(
      ros::Duration(1.0 / std::max(1.0, control_rate_)),
      &FssimPathController::controlTimerCallback,
      this);

    ROS_INFO(
      "FSSIM path controller started: path=%s, odom=%s, cmd=%s",
      input_path_topic_.c_str(),
      input_odom_topic_.c_str(),
      output_cmd_topic_.c_str());
  }

private:
  static double clamp(
    const double value,
    const double lower,
    const double upper)
  {
    return std::max(lower, std::min(upper, value));
  }

  void loadParameters()
  {
    private_nh_.param<std::string>(
      "input_path_topic", input_path_topic_, "/planning/local_path");
    private_nh_.param<std::string>(
      "input_path_status_topic",
      input_path_status_topic_,
      "/planning/path_status");
    private_nh_.param<std::string>(
      "input_odom_topic", input_odom_topic_, "/odometry/filtered");
    private_nh_.param<std::string>(
      "output_cmd_topic", output_cmd_topic_, "/fssim/cmd");
    private_nh_.param<std::string>(
      "output_status_topic",
      output_status_topic_,
      "/fssim_path_controller/status");

    private_nh_.param("control_rate", control_rate_, 20.0);
    private_nh_.param("odometry_timeout", odometry_timeout_, 1.0);
    private_nh_.param("path_timeout", path_timeout_, 0.60);
    private_nh_.param(
      "path_freshness_timeout", path_freshness_timeout_, 0.25);
    private_nh_.param("degraded_speed_scale", degraded_speed_scale_, 0.30);

    private_nh_.param("wheelbase", wheelbase_, 1.53);
    private_nh_.param("steering_sign", steering_sign_, 1.0);
    private_nh_.param("max_steer", max_steer_, 0.40);
    private_nh_.param("max_steer_rate", max_steer_rate_, 0.80);
    private_nh_.param("steering_filter", steering_filter_, 0.25);

    private_nh_.param("lookahead_distance", lookahead_distance_, 2.0);
    private_nh_.param(
      "min_lookahead_distance", min_lookahead_distance_, 1.15);
    private_nh_.param(
      "max_lookahead_distance", max_lookahead_distance_, 2.40);
    private_nh_.param("lookahead_speed_gain", lookahead_speed_gain_, 0.12);
    private_nh_.param(
      "lookahead_curvature_reduction",
      lookahead_curvature_reduction_,
      0.75);
    private_nh_.param("lookahead_filter", lookahead_filter_, 0.70);

    private_nh_.param("cruise_speed", cruise_speed_, 3.0);
    private_nh_.param(
      "minimum_corner_speed", minimum_corner_speed_, 1.0);
    private_nh_.param("speed_kp", speed_kp_, 0.08);
    private_nh_.param("drive_feedforward", drive_feedforward_, 0.04);
    private_nh_.param("max_drive_command", max_drive_command_, 0.40);
    private_nh_.param("max_brake_command", max_brake_command_, -0.30);

    private_nh_.param("creep/max_distance", creep_max_distance_, 1.50);
    private_nh_.param("creep/max_time", creep_max_time_, 2.50);
    private_nh_.param("creep/path_extension", creep_path_extension_, 1.20);

    steering_sign_ = steering_sign_ >= 0.0 ? 1.0 : -1.0;
    steering_filter_ = clamp(steering_filter_, 0.0, 0.95);
    lookahead_filter_ = clamp(lookahead_filter_, 0.0, 0.95);
    degraded_speed_scale_ = clamp(degraded_speed_scale_, 0.1, 1.0);
    min_lookahead_distance_ = std::max(0.5, min_lookahead_distance_);
    max_lookahead_distance_ =
      std::max(min_lookahead_distance_, max_lookahead_distance_);
    active_lookahead_ = clamp(
      lookahead_distance_,
      min_lookahead_distance_,
      max_lookahead_distance_);
    max_drive_command_ = std::max(0.0, max_drive_command_);
    max_brake_command_ = std::min(0.0, max_brake_command_);
  }

  void pathCallback(const nav_msgs::Path::ConstPtr& message)
  {
    if (message->poses.size() < 2U)
    {
      ROS_WARN_THROTTLE(
        1.0, "Empty local path received; retaining the last valid path");
      return;
    }

    path_ = *message;
    has_path_ = true;
    last_valid_path_time_ = ros::Time::now();
    if (has_odometry_)
    {
      path_odom_x_ = odom_x_;
      path_odom_y_ = odom_y_;
      path_odom_yaw_ = odom_yaw_;
      path_pose_valid_ = true;
    }
  }

  void odometryCallback(const nav_msgs::Odometry::ConstPtr& message)
  {
    current_speed_ = message->twist.twist.linear.x;
    odom_x_ = message->pose.pose.position.x;
    odom_y_ = message->pose.pose.position.y;
    const auto& orientation = message->pose.pose.orientation;
    odom_yaw_ = std::atan2(
      2.0 * (orientation.w * orientation.z +
             orientation.x * orientation.y),
      1.0 - 2.0 * (orientation.y * orientation.y +
                   orientation.z * orientation.z));
    has_odometry_ = std::isfinite(odom_x_) &&
      std::isfinite(odom_y_) && std::isfinite(odom_yaw_);
    last_odometry_time_ = ros::Time::now();
  }

  bool isRecoverableConeGap(const std::string& status) const
  {
    if (status.compare(0, 7, "INVALID") != 0)
    {
      return false;
    }
    return status.find("not enough valid cones") != std::string::npos ||
      status.find("no candidate edges") != std::string::npos ||
      status.find("no valid start edge") != std::string::npos ||
      status.find("not enough connected midpoints") != std::string::npos;
  }

  void pathStatusCallback(const std_msgs::String::ConstPtr& message)
  {
    const std::string& status = message->data;
    const bool degraded =
      status.compare(0, 16, "DEGRADED_HISTORY") == 0 ||
      status.compare(0, 20, "DEGRADED_SINGLE_SIDE") == 0 ||
      status.compare(0, 19, "DEGRADED_GAP_BRIDGE") == 0;
    const bool recoverable_gap = isRecoverableConeGap(status);
    const bool invalid = status.compare(0, 7, "INVALID") == 0;
    const bool stale = status.compare(0, 11, "STALE_INPUT") == 0;

    path_degraded_ = degraded || recoverable_gap;
    hard_path_fault_ = stale || (invalid && !recoverable_gap);

    if (recoverable_gap && !creep_requested_)
    {
      creep_distance_ = 0.0;
      creep_start_time_ = ros::Time::now();
      ROS_WARN("Cone-pair gap: starting distance-limited continuation");
    }
    else if (!recoverable_gap && creep_requested_)
    {
      ROS_INFO(
        "Cone-pair gap cleared after %.2f m; resuming normal tracking",
        creep_distance_);
      creep_distance_ = 0.0;
    }
    creep_requested_ = recoverable_gap;
  }

  void transformStoredPoint(
    const double stored_x,
    const double stored_y,
    double& current_x,
    double& current_y) const
  {
    if (!path_pose_valid_)
    {
      current_x = stored_x;
      current_y = stored_y;
      return;
    }

    const double path_cosine = std::cos(path_odom_yaw_);
    const double path_sine = std::sin(path_odom_yaw_);
    const double world_x =
      path_odom_x_ + path_cosine * stored_x - path_sine * stored_y;
    const double world_y =
      path_odom_y_ + path_sine * stored_x + path_cosine * stored_y;

    const double dx = world_x - odom_x_;
    const double dy = world_y - odom_y_;
    const double current_cosine = std::cos(odom_yaw_);
    const double current_sine = std::sin(odom_yaw_);
    current_x = current_cosine * dx + current_sine * dy;
    current_y = -current_sine * dx + current_cosine * dy;
  }

  double estimatePathCurvature() const
  {
    double maximum_curvature = 0.0;
    for (const auto& pose : path_.poses)
    {
      double x = 0.0;
      double y = 0.0;
      transformStoredPoint(
        pose.pose.position.x, pose.pose.position.y, x, y);
      const double squared_distance = x * x + y * y;
      if (x <= 0.10 || squared_distance < 0.25 ||
          squared_distance > 25.0)
      {
        continue;
      }
      maximum_curvature = std::max(
        maximum_curvature, std::abs(2.0 * y / squared_distance));
    }
    return maximum_curvature;
  }

  bool findLookaheadPoint(
    const double requested_lookahead,
    const bool allow_extension,
    double& target_x,
    double& target_y) const
  {
    bool found = false;
    double furthest_distance = -std::numeric_limits<double>::infinity();
    double previous_x = 0.0;
    double previous_y = 0.0;
    double last_x = 0.0;
    double last_y = 0.0;
    bool have_previous = false;
    bool have_last = false;

    for (const auto& pose : path_.poses)
    {
      double x = 0.0;
      double y = 0.0;
      transformStoredPoint(
        pose.pose.position.x, pose.pose.position.y, x, y);
      if (!std::isfinite(x) || !std::isfinite(y))
      {
        continue;
      }
      if (have_last)
      {
        previous_x = last_x;
        previous_y = last_y;
        have_previous = true;
      }
      last_x = x;
      last_y = y;
      have_last = true;
      if (x <= 0.05)
      {
        continue;
      }

      const double distance = std::hypot(x, y);
      if (distance >= requested_lookahead)
      {
        target_x = x;
        target_y = y;
        return true;
      }
      if (distance > furthest_distance)
      {
        furthest_distance = distance;
        target_x = x;
        target_y = y;
        found = true;
      }
    }

    if (allow_extension && have_previous && have_last)
    {
      const double dx = last_x - previous_x;
      const double dy = last_y - previous_y;
      const double length = std::hypot(dx, dy);
      if (length > 0.05)
      {
        const double tangent_x = dx / length;
        const double tangent_y = dy / length;
        for (double extension = 0.20;
             extension <= creep_path_extension_ + 1.0e-6;
             extension += 0.20)
        {
          const double x = last_x + tangent_x * extension;
          const double y = last_y + tangent_y * extension;
          if (x <= 0.05)
          {
            continue;
          }
          const double distance = std::hypot(x, y);
          if (distance >= requested_lookahead)
          {
            target_x = x;
            target_y = y;
            return true;
          }
          if (distance > furthest_distance)
          {
            furthest_distance = distance;
            target_x = x;
            target_y = y;
            found = true;
          }
        }
      }
    }
    return found;
  }

  void controlTimerCallback(const ros::TimerEvent& event)
  {
    const ros::Time now = ros::Time::now();
    if (!has_odometry_ ||
        (now - last_odometry_time_).toSec() > odometry_timeout_)
    {
      publishStop("WAITING_FOR_ODOMETRY");
      return;
    }
    if (hard_path_fault_)
    {
      publishStop("PATH_HARD_FAULT");
      return;
    }

    const double path_age = has_path_ ?
      (now - last_valid_path_time_).toSec() :
      std::numeric_limits<double>::infinity();
    const double creep_age = creep_start_time_.isZero() ?
      std::numeric_limits<double>::infinity() :
      (now - creep_start_time_).toSec();
    const bool creep_allowed =
      creep_requested_ && path_pose_valid_ &&
      creep_distance_ < creep_max_distance_ &&
      creep_age <= creep_max_time_;

    if (!has_path_ || (path_age > path_timeout_ && !creep_allowed))
    {
      if (creep_requested_ && creep_distance_ >= creep_max_distance_)
      {
        publishStop("CREEP_DISTANCE_LIMIT");
      }
      else if (creep_requested_ && creep_age > creep_max_time_)
      {
        publishStop("CREEP_TIME_LIMIT");
      }
      else
      {
        publishStop("WAITING_FOR_PATH");
      }
      return;
    }

    const double path_curvature = estimatePathCurvature();
    const double raw_lookahead = clamp(
      lookahead_distance_ +
        lookahead_speed_gain_ *
          std::max(0.0, std::abs(current_speed_) - 1.0) -
        lookahead_curvature_reduction_ *
          clamp(path_curvature / 0.8, 0.0, 1.0),
      min_lookahead_distance_,
      max_lookahead_distance_);
    active_lookahead_ =
      lookahead_filter_ * active_lookahead_ +
      (1.0 - lookahead_filter_) * raw_lookahead;

    double target_x = 0.0;
    double target_y = 0.0;
    if (!findLookaheadPoint(
          active_lookahead_, creep_allowed, target_x, target_y))
    {
      publishStop("NO_FORWARD_LOOKAHEAD");
      return;
    }

    const double lookahead_squared =
      target_x * target_x + target_y * target_y;
    if (lookahead_squared < 0.01)
    {
      publishStop("LOOKAHEAD_TOO_CLOSE");
      return;
    }

    const double curvature = 2.0 * target_y / lookahead_squared;
    double desired_steer =
      steering_sign_ * std::atan(wheelbase_ * curvature);
    desired_steer = clamp(desired_steer, -max_steer_, max_steer_);

    const double dt = std::min(
      0.20,
      std::max(0.001, (event.current_real - event.last_real).toSec()));
    if (creep_allowed)
    {
      creep_distance_ += std::abs(current_speed_) * dt;
    }

    desired_steer = clamp(
      desired_steer,
      last_steer_ - max_steer_rate_ * dt,
      last_steer_ + max_steer_rate_ * dt);
    const double filtered_steer =
      steering_filter_ * last_steer_ +
      (1.0 - steering_filter_) * desired_steer;
    last_steer_ = filtered_steer;

    const double corner_ratio = clamp(
      std::abs(filtered_steer) / std::max(0.01, max_steer_),
      0.0,
      1.0);
    double target_speed =
      cruise_speed_ -
      corner_ratio * (cruise_speed_ - minimum_corner_speed_);
    const bool degraded =
      path_degraded_ || path_age > path_freshness_timeout_;
    if (degraded)
    {
      target_speed *= degraded_speed_scale_;
    }

    const double drive_command = clamp(
      drive_feedforward_ + speed_kp_ * (target_speed - current_speed_),
      max_brake_command_,
      max_drive_command_);

    fssim_common::Cmd command;
    command.dc = drive_command;
    command.delta = filtered_steer;
    cmd_pub_.publish(command);

    publishStatus(
      creep_allowed ? "CREEPING_FOR_CONES" :
      (degraded ? "RUNNING_DEGRADED" : "RUNNING"));
  }

  void publishStop(const std::string& reason)
  {
    fssim_common::Cmd command;
    command.dc =
      std::abs(current_speed_) > 0.2 ? max_brake_command_ : 0.0;
    command.delta = 0.0;
    cmd_pub_.publish(command);
    last_steer_ = 0.0;
    publishStatus(reason);
    ROS_WARN_THROTTLE(
      1.0, "FSSIM path controller stopped: %s", reason.c_str());
  }

  void publishStatus(const std::string& status)
  {
    if (status == last_status_)
    {
      return;
    }
    std_msgs::String message;
    message.data = status;
    status_pub_.publish(message);
    last_status_ = status;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber path_sub_;
  ros::Subscriber path_status_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher cmd_pub_;
  ros::Publisher status_pub_;
  ros::Timer control_timer_;
  nav_msgs::Path path_;

  bool has_path_ = false;
  bool has_odometry_ = false;
  bool path_degraded_ = false;
  bool hard_path_fault_ = false;
  bool creep_requested_ = false;
  bool path_pose_valid_ = false;

  double current_speed_ = 0.0;
  double last_steer_ = 0.0;
  double active_lookahead_ = 0.0;
  double odom_x_ = 0.0;
  double odom_y_ = 0.0;
  double odom_yaw_ = 0.0;
  double path_odom_x_ = 0.0;
  double path_odom_y_ = 0.0;
  double path_odom_yaw_ = 0.0;
  double creep_distance_ = 0.0;

  ros::Time last_valid_path_time_;
  ros::Time last_odometry_time_;
  ros::Time creep_start_time_;
  std::string last_status_;

  std::string input_path_topic_;
  std::string input_path_status_topic_;
  std::string input_odom_topic_;
  std::string output_cmd_topic_;
  std::string output_status_topic_;

  double control_rate_ = 20.0;
  double odometry_timeout_ = 1.0;
  double path_timeout_ = 0.60;
  double path_freshness_timeout_ = 0.25;
  double degraded_speed_scale_ = 0.30;
  double wheelbase_ = 1.53;
  double steering_sign_ = 1.0;
  double max_steer_ = 0.40;
  double max_steer_rate_ = 0.80;
  double steering_filter_ = 0.25;
  double lookahead_distance_ = 2.0;
  double min_lookahead_distance_ = 1.15;
  double max_lookahead_distance_ = 2.40;
  double lookahead_speed_gain_ = 0.12;
  double lookahead_curvature_reduction_ = 0.75;
  double lookahead_filter_ = 0.70;
  double cruise_speed_ = 3.0;
  double minimum_corner_speed_ = 1.0;
  double speed_kp_ = 0.08;
  double drive_feedforward_ = 0.04;
  double max_drive_command_ = 0.40;
  double max_brake_command_ = -0.30;
  double creep_max_distance_ = 1.50;
  double creep_max_time_ = 2.50;
  double creep_path_extension_ = 1.20;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fssim_path_controller");
  FssimPathController controller;
  ros::spin();
  return 0;
}
