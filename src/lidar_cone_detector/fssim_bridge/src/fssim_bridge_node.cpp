#include <ros/ros.h>

#include <driverless_msgs/ConeObservation.h>
#include <driverless_msgs/ConeObservationArray.h>
#include <fssim_common/Cmd.h>
#include <fssim_common/Mission.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

class FssimBridge
{
public:
  FssimBridge()
    : nh_(),
      private_nh_("~"),
      has_path_(false),
      has_odometry_(false),
      path_degraded_(false),
      creep_requested_(false),
       path_pose_valid_(false),
       finish_stop_latched_(false),
       mission_stop_requested_(false),
       has_mission_speed_limit_(false),
      current_speed_(0.0),
      last_steer_(0.0),
      last_steer_rate_(0.0),
      active_lookahead_(0.0),
      filtered_preview_curvature_(0.0),
      preview_curvature_initialized_(false),
      current_odom_x_(0.0),
      current_odom_y_(0.0),
      current_odom_yaw_(0.0),
      path_odom_x_(0.0),
      path_odom_y_(0.0),
      path_odom_yaw_(0.0),
      creep_distance_(0.0)
  {
    loadParameters();

    cone_sub_ = nh_.subscribe(
      input_cones_topic_, 1, &FssimBridge::coneCallback, this);
    path_sub_ = nh_.subscribe(
      input_path_topic_, 1, &FssimBridge::pathCallback, this);
    path_status_sub_ = nh_.subscribe(
      input_path_status_topic_, 1, &FssimBridge::pathStatusCallback, this);
    odom_sub_ = nh_.subscribe(
      input_odom_topic_, 5, &FssimBridge::odometryCallback, this);
    mission_finished_sub_ = nh_.subscribe(
      input_mission_finished_topic_,
      1,
      &FssimBridge::missionFinishedCallback,
      this);
    mission_stop_sub_ = nh_.subscribe(
      input_mission_stop_topic_,
      1,
      &FssimBridge::missionStopCallback,
      this);
    mission_speed_limit_sub_ = nh_.subscribe(
      input_mission_speed_limit_topic_,
      1,
      &FssimBridge::missionSpeedLimitCallback,
      this);

    cone_pub_ = nh_.advertise<driverless_msgs::ConeObservationArray>(
      output_cones_topic_, 1);
    cmd_pub_ = nh_.advertise<fssim_common::Cmd>(output_cmd_topic_, 1);
    status_pub_ = nh_.advertise<std_msgs::String>(output_status_topic_, 1, true);

    control_timer_ = nh_.createTimer(
      ros::Duration(1.0 / std::max(1.0, control_rate_)),
      &FssimBridge::controlTimerCallback,
      this);

    ROS_INFO(
      "fssim_bridge: cones %s -> %s, path %s, odom %s, cmd %s",
      input_cones_topic_.c_str(),
      output_cones_topic_.c_str(),
      input_path_topic_.c_str(),
      input_odom_topic_.c_str(),
      output_cmd_topic_.c_str());
    ROS_INFO(
      "fssim_bridge: Pure Pursuit L=%.2f m, lookahead=%.2f m, target=%.2f m/s",
      wheelbase_,
      lookahead_distance_,
      cruise_speed_);
  }

private:
  void loadParameters()
  {
    private_nh_.param<std::string>(
      "input_cones_topic", input_cones_topic_, "/lidar/cones");
    private_nh_.param<std::string>(
      "input_path_topic", input_path_topic_, "/planning/local_path");
    private_nh_.param<std::string>(
      "input_path_status_topic",
      input_path_status_topic_,
      "/planning/path_status");
    private_nh_.param<std::string>(
      "input_odom_topic", input_odom_topic_, "/odometry/filtered");
    private_nh_.param<std::string>(
      "output_cones_topic",
      output_cones_topic_,
      "/perception/lidar/cones_raw");
    private_nh_.param<std::string>(
      "output_cmd_topic", output_cmd_topic_, "/fssim/cmd");
    private_nh_.param<std::string>(
      "output_status_topic", output_status_topic_, "/fssim_bridge/status");
    private_nh_.param<std::string>(
      "input_mission_finished_topic",
      input_mission_finished_topic_,
      "/fssim/mission_finished");
    private_nh_.param<std::string>(
      "input_mission_stop_topic",
      input_mission_stop_topic_,
      "/mission/stop");
    private_nh_.param<std::string>(
      "input_mission_speed_limit_topic",
      input_mission_speed_limit_topic_,
      "/mission/speed_limit");
    // Empty means keeping the FSSIM frame and letting the planner use TF.
    private_nh_.param<std::string>("output_cone_frame", output_cone_frame_, "");
    private_nh_.param("use_simulated_color", use_simulated_color_, true);
    private_nh_.param(
      "simulated_color_min_confidence",
      simulated_color_min_confidence_,
      0.55);

    private_nh_.param("control_rate", control_rate_, 20.0);
    private_nh_.param("path_timeout", path_timeout_, 0.6);
    private_nh_.param("path_freshness_timeout", path_freshness_timeout_, 0.25);
    private_nh_.param("path_fallback_speed_scale", path_fallback_speed_scale_, 0.4);
    private_nh_.param("odometry_timeout", odometry_timeout_, 0.3);
    private_nh_.param("creep/max_distance", creep_max_distance_, 2.5);
    private_nh_.param("creep/max_time", creep_max_time_, 4.0);
    private_nh_.param("creep/path_extension", creep_path_extension_, 2.0);
    private_nh_.param("creep/max_speed", creep_max_speed_, 0.8);

    // Gotthard wheelbase from the FSSIM model. Confirm again if the car model changes.
    private_nh_.param("wheelbase", wheelbase_, 1.53);
    private_nh_.param("lookahead_distance", lookahead_distance_, 2.0);
    private_nh_.param("min_lookahead_distance", min_lookahead_distance_, 1.15);
    private_nh_.param("max_lookahead_distance", max_lookahead_distance_, 2.40);
    private_nh_.param("lookahead_speed_gain", lookahead_speed_gain_, 0.12);
    private_nh_.param(
      "lookahead_curvature_reduction", lookahead_curvature_reduction_, 0.75);
    private_nh_.param("lookahead_filter", lookahead_filter_, 0.70);
    private_nh_.param("max_steer", max_steer_, 0.40);
    private_nh_.param("max_steer_rate", max_steer_rate_, 0.80);
    private_nh_.param(
      "max_steer_acceleration", max_steer_acceleration_, 1.50);
    private_nh_.param(
      "max_steer_return_rate", max_steer_return_rate_, 0.90);
    private_nh_.param(
      "max_steer_return_acceleration",
      max_steer_return_acceleration_,
      3.00);
    private_nh_.param("steering_filter", steering_filter_, 0.25);
    private_nh_.param("steering_sign", steering_sign_, 1.0);
    private_nh_.param("preview_distance", preview_distance_, 5.0);
    private_nh_.param(
      "preview_curvature_weight", preview_curvature_weight_, 0.30);
    private_nh_.param(
      "preview_curvature_filter", preview_curvature_filter_, 0.70);

    private_nh_.param("cruise_speed", cruise_speed_, 3.0);
    private_nh_.param("minimum_corner_speed", minimum_corner_speed_, 1.0);
    private_nh_.param("speed_kp", speed_kp_, 0.08);
    private_nh_.param("drive_feedforward", drive_feedforward_, 0.04);
    private_nh_.param("max_drive_command", max_drive_command_, 0.20);
    private_nh_.param("max_brake_command", max_brake_command_, -0.20);
    // Event-specific finish stopping is disabled by default.  Acceleration
    // enables it in its own launch wrapper; trackdrive therefore keeps its
    // multi-lap behaviour.
    private_nh_.param("finish_stop/enabled", finish_stop_enabled_, false);
    private_nh_.param(
      "finish_stop/use_mission_finished",
      finish_stop_use_mission_finished_,
      true);
    private_nh_.param(
      "finish_stop/use_world_x_fallback",
      finish_stop_use_world_x_fallback_,
      false);
    private_nh_.param("finish_stop/world_x", finish_stop_world_x_, 75.3);
    private_nh_.param(
      "mission_control/use_stop", mission_control_use_stop_, false);
    private_nh_.param(
      "mission_control/use_speed_limit",
      mission_control_use_speed_limit_,
      false);
    private_nh_.param(
      "finish_stop/stopped_speed_threshold",
      finish_stopped_speed_threshold_,
      0.05);
    private_nh_.param(
      "max_target_acceleration", max_target_acceleration_, 1.2);
    private_nh_.param(
      "max_target_deceleration", max_target_deceleration_, 1.8);

    steering_filter_ = clamp(steering_filter_, 0.0, 0.95);
    steering_sign_ = steering_sign_ >= 0.0 ? 1.0 : -1.0;
    path_fallback_speed_scale_ = clamp(path_fallback_speed_scale_, 0.1, 1.0);
    simulated_color_min_confidence_ =
      clamp(simulated_color_min_confidence_, 0.0, 1.0);
    max_brake_command_ = std::min(0.0, max_brake_command_);
    finish_stopped_speed_threshold_ = std::max(
      0.01, finish_stopped_speed_threshold_);
    max_drive_command_ = std::max(0.0, max_drive_command_);
    max_target_acceleration_ = std::max(0.1, max_target_acceleration_);
    max_target_deceleration_ = std::max(0.1, max_target_deceleration_);
    creep_max_distance_ = std::max(0.0, creep_max_distance_);
    creep_max_time_ = std::max(0.0, creep_max_time_);
    creep_path_extension_ = std::max(0.0, creep_path_extension_);
    min_lookahead_distance_ = std::max(0.5, min_lookahead_distance_);
    max_lookahead_distance_ =
      std::max(min_lookahead_distance_, max_lookahead_distance_);
    lookahead_filter_ = clamp(lookahead_filter_, 0.0, 0.95);
    max_steer_acceleration_ = std::max(0.1, max_steer_acceleration_);
    max_steer_return_rate_ = std::max(
      max_steer_rate_, max_steer_return_rate_);
    max_steer_return_acceleration_ = std::max(
      max_steer_acceleration_, max_steer_return_acceleration_);
    preview_distance_ = std::max(1.0, preview_distance_);
    preview_curvature_weight_ = clamp(preview_curvature_weight_, 0.0, 0.75);
    preview_curvature_filter_ = clamp(preview_curvature_filter_, 0.0, 0.95);
    active_lookahead_ = clamp(
      lookahead_distance_, min_lookahead_distance_, max_lookahead_distance_);
  }

  static double clamp(const double value, const double lower, const double upper)
  {
    return std::max(lower, std::min(upper, value));
  }

  static double normalizeAngle(double angle)
  {
    const double pi = 3.14159265358979323846;
    while (angle > pi)
    {
      angle -= 2.0 * pi;
    }
    while (angle < -pi)
    {
      angle += 2.0 * pi;
    }
    return angle;
  }

  static bool hasField(
    const sensor_msgs::PointCloud2& cloud,
    const std::string& name)
  {
    for (const auto& field : cloud.fields)
    {
      if (field.name == name)
      {
        return true;
      }
    }
    return false;
  }

  static bool readFloatField(
    const sensor_msgs::PointCloud2& cloud,
    const std::size_t point_index,
    const std::string& name,
    float& value)
  {
    const sensor_msgs::PointField* requested_field = nullptr;
    for (const auto& field : cloud.fields)
    {
      if (field.name == name &&
          field.datatype == sensor_msgs::PointField::FLOAT32)
      {
        requested_field = &field;
        break;
      }
    }
    if (requested_field == nullptr || cloud.width == 0U)
    {
      return false;
    }

    const std::size_t row = point_index / cloud.width;
    const std::size_t column = point_index % cloud.width;
    const std::size_t offset =
      row * cloud.row_step + column * cloud.point_step +
      requested_field->offset;
    if (offset + sizeof(float) > cloud.data.size())
    {
      return false;
    }
    std::memcpy(&value, cloud.data.data() + offset, sizeof(float));
    return std::isfinite(value);
  }

  void coneCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    if (!hasField(*msg, "x") || !hasField(*msg, "y") || !hasField(*msg, "z"))
    {
      ROS_ERROR_THROTTLE(1.0, "FSSIM cone cloud has no x/y/z fields");
      return;
    }

    driverless_msgs::ConeObservationArray output;
    output.header = msg->header;
    if (!output_cone_frame_.empty())
    {
      output.header.frame_id = output_cone_frame_;
    }

    try
    {
      sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*msg, "z");

      uint32_t id = 0;
      std::size_t point_index = 0U;
      for (; x != x.end(); ++x, ++y, ++z, ++point_index)
      {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z))
        {
          continue;
        }

        driverless_msgs::ConeObservation cone;
        cone.header = output.header;
        cone.id = id++;
        cone.position.x = *x;
        cone.position.y = *y;
        cone.position.z = *z;

        cone.source = driverless_msgs::ConeObservation::SOURCE_LIDAR;
        cone.semantic_class =
          driverless_msgs::ConeObservation::SEMANTIC_UNKNOWN;
        cone.candidate_level =
          driverless_msgs::ConeObservation::CANDIDATE_STRONG;
        cone.existence_probability = 0.95F;
        cone.lidar_confidence = 0.95F;
        cone.camera_color_confidence = 0.0F;

        if (use_simulated_color_)
        {
          float blue = 0.0F;
          float yellow = 0.0F;
          float orange = 0.0F;
          const bool has_probabilities =
            readFloatField(*msg, point_index, "probability_blue", blue) &&
            readFloatField(*msg, point_index, "probability_yellow", yellow) &&
            readFloatField(*msg, point_index, "probability_orange", orange);
          if (has_probabilities)
          {
            const float best = std::max(blue, std::max(yellow, orange));
            cone.camera_color_confidence = best;
            if (best >= simulated_color_min_confidence_)
            {
              if (blue > yellow && blue > orange)
              {
                cone.semantic_class =
                  driverless_msgs::ConeObservation::SEMANTIC_BLUE;
              }
              else if (yellow > orange)
              {
                cone.semantic_class =
                  driverless_msgs::ConeObservation::SEMANTIC_YELLOW;
              }
              else
              {
                cone.semantic_class =
                  driverless_msgs::ConeObservation::SEMANTIC_SMALL_ORANGE;
              }
              cone.source =
                driverless_msgs::ConeObservation::SOURCE_FUSED;
            }
          }
        }
        cone.position_covariance[0] = 0.0025;
        cone.position_covariance[4] = 0.0025;
        cone.position_covariance[8] = 0.01;
        cone.point_count = 1;
        cone.confirmed = true;
        output.cones.push_back(cone);
      }
    }
    catch (const std::runtime_error& error)
    {
      ROS_ERROR_THROTTLE(1.0, "Cannot read FSSIM cone cloud: %s", error.what());
      return;
    }

    cone_pub_.publish(output);
    ROS_INFO_THROTTLE(
      2.0,
      "FSSIM cones: %zu, source frame: %s, output frame: %s",
      output.cones.size(),
      msg->header.frame_id.c_str(),
      output.header.frame_id.c_str());
  }

  void pathCallback(const nav_msgs::Path::ConstPtr& msg)
  {
    if (msg->poses.size() < 2)
    {
      // A single invalid planning frame should not erase the last good path.
      ROS_WARN_THROTTLE(1.0, "Empty local path received, keeping the last valid path briefly");
      return;
    }

    path_ = *msg;
    has_path_ = true;
    last_valid_path_time_ = ros::Time::now();
    if (has_odometry_)
    {
      path_odom_x_ = current_odom_x_;
      path_odom_y_ = current_odom_y_;
      path_odom_yaw_ = current_odom_yaw_;
      path_pose_valid_ = true;
    }
  }

  void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    current_speed_ = msg->twist.twist.linear.x;
    current_odom_x_ = msg->pose.pose.position.x;
    current_odom_y_ = msg->pose.pose.position.y;
    const auto& orientation = msg->pose.pose.orientation;
    current_odom_yaw_ = std::atan2(
      2.0 * (orientation.w * orientation.z +
             orientation.x * orientation.y),
      1.0 - 2.0 * (orientation.y * orientation.y +
                   orientation.z * orientation.z));
    has_odometry_ = true;
    last_odometry_receive_time_ = ros::Time::now();

    // The acceleration track uses the FSSIM map x axis as its longitudinal
    // direction.  This is only a fallback in case the simulator's mission
    // completion message is lost; it is not enabled for other events.
    if (finish_stop_enabled_ && finish_stop_use_world_x_fallback_ &&
        current_odom_x_ >= finish_stop_world_x_)
    {
      finish_stop_latched_ = true;
    }
  }

  void missionFinishedCallback(const fssim_common::Mission::ConstPtr& msg)
  {
    if (!finish_stop_enabled_ || !finish_stop_use_mission_finished_ ||
        !msg->finished)
    {
      return;
    }

    // The launch switch is event-specific.  Once the finish signal arrives,
    // keep braking even if later path or cone messages are still valid.
    finish_stop_latched_ = true;
    ROS_INFO("fssim_bridge: mission finished, finish stop latched");
  }

  void missionStopCallback(const std_msgs::Bool::ConstPtr& msg)
  {
    if (mission_control_use_stop_)
    {
      mission_stop_requested_ = msg->data;
    }
  }

  void missionSpeedLimitCallback(const std_msgs::Float32::ConstPtr& msg)
  {
    if (!mission_control_use_speed_limit_ || !std::isfinite(msg->data))
    {
      return;
    }
    mission_speed_limit_ = std::max(0.0, static_cast<double>(msg->data));
    has_mission_speed_limit_ = true;
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
      status.find("not enough connected midpoints") != std::string::npos ||
      status.find("too few path points") != std::string::npos ||
      status.find("path is too short") != std::string::npos ||
      status.find("temporal path jump rejected") != std::string::npos;
  }

  void pathStatusCallback(const std_msgs::String::ConstPtr& msg)
  {
    const std::string& status = msg->data;
    path_degraded_ =
      status.compare(0, 16, "DEGRADED_HISTORY") == 0 ||
      status.compare(0, 20, "DEGRADED_SINGLE_SIDE") == 0 ||
      status.compare(0, 19, "DEGRADED_GAP_BRIDGE") == 0 ||
      status.compare(0, 26, "DEGRADED_BOUNDARY_RECOVERY") == 0 ||
      status.compare(0, 7, "INVALID") == 0 ||
      status.compare(0, 11, "STALE_INPUT") == 0;

    const bool request_creep = isRecoverableConeGap(status);
    if (request_creep && !creep_requested_)
    {
      creep_distance_ = 0.0;
      creep_start_time_ = ros::Time::now();
      ROS_WARN("Cone-pair gap detected: start distance-limited path continuation");
    }
    else if (!request_creep && creep_requested_)
    {
      ROS_INFO(
        "Cone-pair gap cleared after %.2f m, normal path tracking resumed",
        creep_distance_);
      creep_distance_ = 0.0;
    }
    creep_requested_ = request_creep;
  }

  void transformStoredPathPoint(
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

    const double dx = world_x - current_odom_x_;
    const double dy = world_y - current_odom_y_;
    const double current_cosine = std::cos(current_odom_yaw_);
    const double current_sine = std::sin(current_odom_yaw_);
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
      transformStoredPathPoint(
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

  double estimateSignedPreviewCurvature(const double preview_distance) const
  {
    struct LocalPoint
    {
      double x;
      double y;
    };
    std::vector<LocalPoint> points;
    points.reserve(path_.poses.size());
    for (const auto& pose : path_.poses)
    {
      double x = 0.0;
      double y = 0.0;
      transformStoredPathPoint(
        pose.pose.position.x, pose.pose.position.y, x, y);
      if (std::isfinite(x) && std::isfinite(y))
      {
        points.push_back(LocalPoint{x, y});
      }
    }
    if (points.size() < 3U)
    {
      return 0.0;
    }

    std::size_t start_segment = 0U;
    double start_ratio = 0.0;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0U; index + 1U < points.size(); ++index)
    {
      const double dx = points[index + 1U].x - points[index].x;
      const double dy = points[index + 1U].y - points[index].y;
      const double length_squared = dx * dx + dy * dy;
      if (length_squared < 1.0e-8)
      {
        continue;
      }
      const double ratio = clamp(
        -(points[index].x * dx + points[index].y * dy) / length_squared,
        0.0,
        1.0);
      const double projected_x = points[index].x + ratio * dx;
      const double projected_y = points[index].y + ratio * dy;
      const double distance = std::hypot(projected_x, projected_y);
      if (distance < nearest_distance)
      {
        nearest_distance = distance;
        start_segment = index;
        start_ratio = ratio;
      }
    }

    LocalPoint segment_start{
      points[start_segment].x + start_ratio *
        (points[start_segment + 1U].x - points[start_segment].x),
      points[start_segment].y + start_ratio *
        (points[start_segment + 1U].y - points[start_segment].y)};
    double remaining = std::max(1.0, preview_distance);
    double travelled = 0.0;
    double weighted_turn = 0.0;
    double weighted_distance = 0.0;
    double previous_heading = 0.0;
    bool have_previous_heading = false;
    for (std::size_t index = start_segment;
         index + 1U < points.size() && remaining > 1.0e-6;
         ++index)
    {
      const LocalPoint& segment_end = points[index + 1U];
      const double dx = segment_end.x - segment_start.x;
      const double dy = segment_end.y - segment_start.y;
      const double segment_length = std::hypot(dx, dy);
      if (segment_length > 1.0e-6)
      {
        const double heading = std::atan2(dy, dx);
        const double used_length = std::min(segment_length, remaining);
        // Near curvature must dominate.  Otherwise an S bend inside one
        // preview window can cancel itself and look like a straight line.
        const double preview_ratio = clamp(
          travelled / std::max(1.0, preview_distance), 0.0, 1.0);
        const double preview_weight = 1.0 - 0.75 * preview_ratio;
        if (have_previous_heading)
        {
          weighted_turn += preview_weight * normalizeAngle(
            heading - previous_heading) * used_length / segment_length;
        }
        weighted_distance += preview_weight * used_length;
        previous_heading = heading;
        have_previous_heading = true;
        travelled += used_length;
        remaining -= used_length;
        if (used_length + 1.0e-6 < segment_length)
        {
          break;
        }
      }
      segment_start = segment_end;
    }
    return weighted_distance > 0.3 ?
      weighted_turn / weighted_distance : 0.0;
  }

  bool findLookaheadPoint(
    const double requested_lookahead,
    const bool allow_path_extension,
    double& target_x,
    double& target_y) const
  {
    struct LocalPoint
    {
      double x;
      double y;
    };
    std::vector<LocalPoint> points;
    points.reserve(path_.poses.size());
    for (const auto& pose : path_.poses)
    {
      double x = 0.0;
      double y = 0.0;
      transformStoredPathPoint(
        pose.pose.position.x, pose.pose.position.y, x, y);
      if (!std::isfinite(x) || !std::isfinite(y))
      {
        continue;
      }
      points.push_back(LocalPoint{x, y});
    }
    if (points.size() < 2U)
    {
      return false;
    }

    // Project the vehicle origin onto the ordered path, then move forward by
    // arc length.  Radial distance can jump between different parts of a
    // hairpin and produces the observed steer/反打 oscillation.
    std::size_t start_segment = 0U;
    double start_ratio = 0.0;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0U; index + 1U < points.size(); ++index)
    {
      const double dx = points[index + 1U].x - points[index].x;
      const double dy = points[index + 1U].y - points[index].y;
      const double length_squared = dx * dx + dy * dy;
      if (length_squared < 1.0e-8)
      {
        continue;
      }
      const double ratio = clamp(
        -(points[index].x * dx + points[index].y * dy) /
          length_squared,
        0.0,
        1.0);
      const double projected_x = points[index].x + ratio * dx;
      const double projected_y = points[index].y + ratio * dy;
      const double distance = std::hypot(projected_x, projected_y);
      if (distance < nearest_distance)
      {
        nearest_distance = distance;
        start_segment = index;
        start_ratio = ratio;
      }
    }

    double remaining = std::max(0.05, requested_lookahead);
    LocalPoint segment_start{
      points[start_segment].x + start_ratio *
        (points[start_segment + 1U].x - points[start_segment].x),
      points[start_segment].y + start_ratio *
        (points[start_segment + 1U].y - points[start_segment].y)};
    for (std::size_t index = start_segment;
         index + 1U < points.size();
         ++index)
    {
      const LocalPoint& segment_end = points[index + 1U];
      const double dx = segment_end.x - segment_start.x;
      const double dy = segment_end.y - segment_start.y;
      const double segment_length = std::hypot(dx, dy);
      if (segment_length > 1.0e-6)
      {
        if (remaining <= segment_length)
        {
          const double ratio = remaining / segment_length;
          target_x = segment_start.x + ratio * dx;
          target_y = segment_start.y + ratio * dy;
          if (target_x > 0.02)
          {
            return true;
          }
          remaining = 0.0;
        }
        else
        {
          remaining -= segment_length;
        }
      }
      segment_start = segment_end;
      if (remaining <= 0.0 && segment_start.x > 0.02)
      {
        target_x = segment_start.x;
        target_y = segment_start.y;
        return true;
      }
    }

    const LocalPoint& last = points.back();
    const LocalPoint& previous = points[points.size() - 2U];
    if (allow_path_extension && creep_path_extension_ > 0.0)
    {
      const double dx = last.x - previous.x;
      const double dy = last.y - previous.y;
      const double length = std::hypot(dx, dy);
      if (length > 0.05)
      {
        const double extension = std::min(
          creep_path_extension_, std::max(0.0, remaining));
        target_x = last.x + extension * dx / length;
        target_y = last.y + extension * dy / length;
        return target_x > 0.02;
      }
    }
    target_x = last.x;
    target_y = last.y;
    return target_x > 0.02;
  }

  void controlTimerCallback(const ros::TimerEvent& event)
  {
    const ros::Time now = ros::Time::now();

    if (finish_stop_latched_)
    {
      publishFinishStop();
      return;
    }

    if (mission_control_use_stop_ && mission_stop_requested_)
    {
      publishStop("MISSION_STOP_REQUESTED");
      return;
    }

    if (!has_odometry_ ||
        (now - last_odometry_receive_time_).toSec() > odometry_timeout_)
    {
      publishStop("WAITING_FOR_ODOMETRY");
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

    double target_x = 0.0;
    double target_y = 0.0;
    const double raw_preview_curvature =
      estimateSignedPreviewCurvature(preview_distance_);
    if (!preview_curvature_initialized_)
    {
      filtered_preview_curvature_ = raw_preview_curvature;
      preview_curvature_initialized_ = true;
    }
    else
    {
      filtered_preview_curvature_ =
        preview_curvature_filter_ * filtered_preview_curvature_ +
        (1.0 - preview_curvature_filter_) * raw_preview_curvature;
    }
    const double path_curvature = std::max(
      std::abs(filtered_preview_curvature_), estimatePathCurvature());
    const double raw_lookahead = clamp(
      lookahead_distance_ +
        lookahead_speed_gain_ * std::max(0.0, std::abs(current_speed_) - 1.0) -
        lookahead_curvature_reduction_ *
          clamp(path_curvature / 0.8, 0.0, 1.0),
      min_lookahead_distance_,
      max_lookahead_distance_);
    active_lookahead_ =
      lookahead_filter_ * active_lookahead_ +
      (1.0 - lookahead_filter_) * raw_lookahead;
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

    // Path is expressed in base_link: x forward, y left.
    const double pursuit_curvature = 2.0 * target_y / lookahead_squared;
    // The nearby target corrects tracking error.  The signed curvature over
    // a longer horizon starts the turn progressively before the first close
    // cone pair, without aiming one long chord across a tight corner.
    const double curvature =
      (1.0 - preview_curvature_weight_) * pursuit_curvature +
      preview_curvature_weight_ * filtered_preview_curvature_;
    double desired_steer =
      steering_sign_ * std::atan(wheelbase_ * curvature);
    desired_steer = clamp(desired_steer, -max_steer_, max_steer_);

    const double dt = std::max(
      0.001, (event.current_real - event.last_real).toSec());
    if (creep_allowed)
    {
      creep_distance_ += std::abs(current_speed_) * dt;
    }
    const bool unwinding_steering =
      std::abs(desired_steer) + 1.0e-4 < std::abs(last_steer_) ||
      desired_steer * last_steer_ < 0.0;
    const double active_steer_rate = unwinding_steering ?
      max_steer_return_rate_ : max_steer_rate_;
    const double active_steer_acceleration = unwinding_steering ?
      max_steer_return_acceleration_ : max_steer_acceleration_;
    const double requested_steer_rate = clamp(
      (desired_steer - last_steer_) / dt,
      -active_steer_rate,
      active_steer_rate);
    const double maximum_rate_change = active_steer_acceleration * dt;
    const double limited_steer_rate = clamp(
      requested_steer_rate,
      last_steer_rate_ - maximum_rate_change,
      last_steer_rate_ + maximum_rate_change);
    const double rate_limited_steer = clamp(
      last_steer_ + limited_steer_rate * dt,
      -max_steer_,
      max_steer_);
    const double previous_steer = last_steer_;
    const double filtered_steer =
      steering_filter_ * previous_steer +
      (1.0 - steering_filter_) * rate_limited_steer;
    last_steer_ = filtered_steer;
    last_steer_rate_ = (filtered_steer - previous_steer) / dt;

    const double preview_steer = std::atan(
      wheelbase_ * filtered_preview_curvature_);
    const double corner_ratio = std::max(
      clamp(
        std::abs(filtered_steer) / std::max(0.01, max_steer_), 0.0, 1.0),
      clamp(
        std::abs(preview_steer) / std::max(0.01, max_steer_), 0.0, 1.0));
    double requested_target_speed =
      cruise_speed_ -
      corner_ratio * (cruise_speed_ - minimum_corner_speed_);
    // A single rejected planner frame must not pulse the throttle.  Path age
    // already tells us whether the stored path has genuinely become stale.
    const bool using_last_path =
      path_age > path_freshness_timeout_;
    if (using_last_path)
    {
      requested_target_speed *= path_fallback_speed_scale_;
    }
    if (creep_allowed)
    {
      // Continue a short planning gap without carrying normal race speed
      // beyond the end of the last validated path.
      requested_target_speed = std::min(
        requested_target_speed, std::max(0.1, creep_max_speed_));
    }
    if (mission_control_use_speed_limit_ && has_mission_speed_limit_)
    {
      requested_target_speed = std::min(
        requested_target_speed, mission_speed_limit_);
    }

    // Rate-limit the requested speed before the PI-like drive calculation.
    // This removes the stop-go behaviour when path quality changes between
    // adjacent 10 Hz planner frames.
    if (!target_speed_initialized_)
    {
      filtered_target_speed_ = std::max(0.0, current_speed_);
      target_speed_initialized_ = true;
    }
    filtered_target_speed_ = clamp(
      requested_target_speed,
      filtered_target_speed_ - max_target_deceleration_ * dt,
      filtered_target_speed_ + max_target_acceleration_ * dt);
    const double target_speed = filtered_target_speed_;

    const double drive_command = clamp(
      drive_feedforward_ + speed_kp_ * (target_speed - current_speed_),
      max_brake_command_,
      max_drive_command_);

    fssim_common::Cmd cmd;
    cmd.dc = drive_command;
    cmd.delta = filtered_steer;
    cmd_pub_.publish(cmd);
    if (creep_allowed)
    {
      publishStatus("CREEPING_FOR_CONES");
    }
    else
    {
      publishStatus(using_last_path ? "RUNNING_ON_LAST_PATH" : "RUNNING");
    }

    ROS_INFO_THROTTLE(
      0.5,
      "PurePursuit: target=(%.2f, %.2f), Ld=%.2f, k_pp=%.3f, k_preview=%.3f, k_cmd=%.3f, delta=%.3f, v=%.2f/%.2f, dc=%.3f",
      target_x,
      target_y,
      active_lookahead_,
      pursuit_curvature,
      filtered_preview_curvature_,
      curvature,
      filtered_steer,
      current_speed_,
      target_speed,
      drive_command);
  }

  void publishStop(const std::string& reason)
  {
    fssim_common::Cmd cmd;
    cmd.dc = std::abs(current_speed_) > 0.2 ? max_brake_command_ : 0.0;
    cmd.delta = 0.0;
    cmd_pub_.publish(cmd);
    last_steer_ = 0.0;
    last_steer_rate_ = 0.0;
    filtered_preview_curvature_ = 0.0;
    preview_curvature_initialized_ = false;
    // Keep the speed ramp aligned with the actual vehicle while stopped.
    filtered_target_speed_ = std::max(0.0, current_speed_);
    target_speed_initialized_ = true;
    publishStatus(reason);
    ROS_WARN_THROTTLE(1.0, "fssim_bridge stopped: %s", reason.c_str());
  }

  void publishFinishStop()
  {
    fssim_common::Cmd cmd;
    cmd.dc = std::abs(current_speed_) > finish_stopped_speed_threshold_ ?
      max_brake_command_ : 0.0;
    cmd.delta = 0.0;
    cmd_pub_.publish(cmd);

    last_steer_ = 0.0;
    last_steer_rate_ = 0.0;
    filtered_preview_curvature_ = 0.0;
    preview_curvature_initialized_ = false;
    filtered_target_speed_ = 0.0;
    target_speed_initialized_ = true;
    publishStatus(
      std::abs(current_speed_) > finish_stopped_speed_threshold_ ?
      "BRAKING_AFTER_FINISH" : "MISSION_FINISHED_STOPPED");
    ROS_INFO_THROTTLE(
      1.0,
      "fssim_bridge finish stop: x=%.2f m, speed=%.2f m/s, dc=%.2f",
      current_odom_x_,
      current_speed_,
      cmd.dc);
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
  ros::Subscriber cone_sub_;
  ros::Subscriber path_sub_;
  ros::Subscriber path_status_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber mission_finished_sub_;
  ros::Subscriber mission_stop_sub_;
  ros::Subscriber mission_speed_limit_sub_;
  ros::Publisher cone_pub_;
  ros::Publisher cmd_pub_;
  ros::Publisher status_pub_;
  ros::Timer control_timer_;

  nav_msgs::Path path_;
  bool has_path_;
  bool has_odometry_;
  bool path_degraded_;
  bool creep_requested_;
  bool path_pose_valid_;
  bool finish_stop_latched_;
  bool mission_stop_requested_;
  bool has_mission_speed_limit_;
  double current_speed_;
  double last_steer_;
  double last_steer_rate_;
  double active_lookahead_;
  double filtered_preview_curvature_;
  bool preview_curvature_initialized_;
  double current_odom_x_;
  double current_odom_y_;
  double current_odom_yaw_;
  double path_odom_x_;
  double path_odom_y_;
  double path_odom_yaw_;
  double creep_distance_;
  ros::Time last_valid_path_time_;
  ros::Time last_odometry_receive_time_;
  ros::Time creep_start_time_;
  std::string last_status_;

  std::string input_cones_topic_;
  std::string input_path_topic_;
  std::string input_path_status_topic_;
  std::string input_odom_topic_;
  std::string input_mission_finished_topic_;
  std::string input_mission_stop_topic_;
  std::string input_mission_speed_limit_topic_;
  std::string output_cones_topic_;
  std::string output_cmd_topic_;
  std::string output_status_topic_;
  std::string output_cone_frame_;
  bool use_simulated_color_ = true;
  double simulated_color_min_confidence_ = 0.55;

  double control_rate_;
  double path_timeout_;
  double path_freshness_timeout_;
  double path_fallback_speed_scale_;
  double odometry_timeout_;
  double creep_max_distance_;
  double creep_max_time_;
  double creep_path_extension_;
  double creep_max_speed_;
  double wheelbase_;
  double lookahead_distance_;
  double min_lookahead_distance_;
  double max_lookahead_distance_;
  double lookahead_speed_gain_;
  double lookahead_curvature_reduction_;
  double lookahead_filter_;
  double max_steer_;
  double max_steer_rate_;
  double max_steer_acceleration_;
  double max_steer_return_rate_;
  double max_steer_return_acceleration_;
  double steering_filter_;
  double steering_sign_;
  double preview_distance_;
  double preview_curvature_weight_;
  double preview_curvature_filter_;
  double cruise_speed_;
  double minimum_corner_speed_;
  double speed_kp_;
  double drive_feedforward_;
  double max_drive_command_;
  double max_brake_command_;
  bool finish_stop_enabled_ = false;
  bool mission_control_use_stop_ = false;
  bool mission_control_use_speed_limit_ = false;
  bool finish_stop_use_mission_finished_ = true;
  bool finish_stop_use_world_x_fallback_ = false;
  double finish_stop_world_x_ = 75.3;
  double finish_stopped_speed_threshold_ = 0.05;
  double mission_speed_limit_ = 0.0;
  double max_target_acceleration_;
  double max_target_deceleration_;
  double filtered_target_speed_ = 0.0;
  bool target_speed_initialized_ = false;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fssim_bridge_node");
  FssimBridge bridge;
  ros::spin();
  return 0;
}
