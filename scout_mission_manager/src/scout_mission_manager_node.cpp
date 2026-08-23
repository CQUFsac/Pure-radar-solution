#include <ros/ros.h>

#include <driverless_msgs/ConeObservationArray.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <cstddef>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

struct GateCone
{
  double x = 0.0;
  double y = 0.0;
};

struct GatePair
{
  std::size_t first = 0U;
  std::size_t second = 0U;
  double midpoint_x = 0.0;
  double midpoint_y = 0.0;
  double width = 0.0;
};

enum class MissionState
{
  WAIT_GO,
  APPROACH_START,
  ACCELERATING,
  ENTER,
  RIGHT_LAP_1,
  RIGHT_LAP_2,
  TRANSITION_TO_LEFT,
  LEFT_LAP_1,
  LEFT_LAP_2,
  EXIT,
  BRAKING,
  FINISHED,
  FAULT
};

std::string stateName(const MissionState state)
{
  switch (state)
  {
    case MissionState::WAIT_GO: return "WAIT_GO";
    case MissionState::APPROACH_START: return "APPROACH_START";
    case MissionState::ACCELERATING: return "ACCELERATING";
    case MissionState::ENTER: return "ENTER";
    case MissionState::RIGHT_LAP_1: return "RIGHT_LAP_1";
    case MissionState::RIGHT_LAP_2: return "RIGHT_LAP_2";
    case MissionState::TRANSITION_TO_LEFT: return "TRANSITION_TO_LEFT";
    case MissionState::LEFT_LAP_1: return "LEFT_LAP_1";
    case MissionState::LEFT_LAP_2: return "LEFT_LAP_2";
    case MissionState::EXIT: return "EXIT";
    case MissionState::BRAKING: return "BRAKING";
    case MissionState::FINISHED: return "FINISHED";
    case MissionState::FAULT: return "FAULT";
  }
  return "UNKNOWN";
}

}  // namespace

class ScoutMissionManager
{
public:
  ScoutMissionManager()
    : private_nh_("~"),
      tf_listener_(tf_buffer_)
  {
    loadParameters();

    cone_sub_ = nh_.subscribe(
      cone_topic_, 1, &ScoutMissionManager::coneCallback, this);
    imu_sub_ = nh_.subscribe(
      imu_topic_, 20, &ScoutMissionManager::imuCallback, this);
    odom_sub_ = nh_.subscribe(
      odom_topic_, 20, &ScoutMissionManager::odometryCallback, this);

    state_pub_ = nh_.advertise<std_msgs::String>(
      state_topic_, 1, true);
    turn_hint_pub_ = nh_.advertise<std_msgs::Int8>(
      turn_hint_topic_, 1, true);
    speed_limit_pub_ = nh_.advertise<std_msgs::Float32>(
      speed_limit_topic_, 1, true);
    stop_pub_ = nh_.advertise<std_msgs::Bool>(
      stop_topic_, 1, true);
    finished_pub_ = nh_.advertise<std_msgs::Bool>(
      finished_topic_, 1, true);
    status_pub_ = nh_.advertise<std_msgs::String>(
      status_topic_, 1, true);

    start_service_ = nh_.advertiseService(
      start_service_name_, &ScoutMissionManager::startCallback, this);
    reset_service_ = nh_.advertiseService(
      reset_service_name_, &ScoutMissionManager::resetCallback, this);
    test_gate_service_ = nh_.advertiseService(
      test_gate_service_name_,
      &ScoutMissionManager::testGateCallback,
      this);

    timer_ = nh_.createTimer(
      ros::Duration(1.0 / std::max(1.0, publish_rate_)),
      &ScoutMissionManager::timerCallback,
      this);

    resetMission();
    publishOutputs();
    ROS_INFO(
      "SCOUT mission manager started: mission=%s, cones=%s, imu=%s",
      mission_type_.c_str(), cone_topic_.c_str(), imu_topic_.c_str());
  }

private:
  void loadParameters()
  {
    private_nh_.param<std::string>(
      "mission_type", mission_type_, "acceleration");
    private_nh_.param<std::string>(
      "base_frame", base_frame_, "base_link");
    private_nh_.param<std::string>(
      "cone_topic", cone_topic_, "/perception/lidar/cones_raw");
    private_nh_.param<std::string>(
      "imu_topic", imu_topic_, "/imu/data");
    private_nh_.param<std::string>(
      "odom_topic", odom_topic_, "/odometry/filtered");
    private_nh_.param<std::string>(
      "state_topic", state_topic_, "/mission/state");
    private_nh_.param<std::string>(
      "turn_hint_topic", turn_hint_topic_, "/mission/turn_hint");
    private_nh_.param<std::string>(
      "speed_limit_topic", speed_limit_topic_, "/mission/speed_limit");
    private_nh_.param<std::string>(
      "stop_topic", stop_topic_, "/mission/stop");
    private_nh_.param<std::string>(
      "finished_topic", finished_topic_, "/mission/finished");
    private_nh_.param<std::string>(
      "status_topic", status_topic_, "/mission/status");
    private_nh_.param<std::string>(
      "start_service", start_service_name_, "/mission/start");
    private_nh_.param<std::string>(
      "reset_service", reset_service_name_, "/mission/reset");
    private_nh_.param<std::string>(
      "test_gate_service",
      test_gate_service_name_,
      "/mission/test_gate");

    private_nh_.param("publish_rate", publish_rate_, 20.0);
    private_nh_.param("tf_timeout", tf_timeout_, 0.03);
    private_nh_.param("cone_timeout", cone_timeout_, 0.50);
    private_nh_.param("imu_timeout", imu_timeout_, 0.30);
    private_nh_.param(
      "require_cones_before_start", require_cones_before_start_, true);
    private_nh_.param(
      "monitor_cones_during_run", monitor_cones_during_run_, true);
    private_nh_.param("require_imu", require_imu_, false);
    private_nh_.param("auto_start", auto_start_, false);
    private_nh_.param("maximum_mission_time", maximum_mission_time_, 100.0);
    private_nh_.param("brake_duration", brake_duration_, 2.0);

    private_nh_.param("imu/calibrate_bias", calibrate_gyro_bias_, true);
    private_nh_.param("imu/gyro_bias_z", gyro_bias_z_, 0.0);
    private_nh_.param("imu/bias_samples", gyro_bias_samples_, 100);
    private_nh_.param(
      "imu/calibration_max_rate", calibration_max_rate_, 0.08);
    private_nh_.param("imu/max_integration_dt", max_integration_dt_, 0.10);
    private_nh_.param("imu/yaw_sign", imu_yaw_sign_, 1.0);

    private_nh_.param("gate/min_x", gate_min_x_, 0.2);
    private_nh_.param("gate/max_x", gate_max_x_, 4.0);
    private_nh_.param("gate/max_abs_y", gate_max_abs_y_, 2.5);
    private_nh_.param("gate/min_width", gate_min_width_, 1.0);
    private_nh_.param("gate/max_width", gate_max_width_, 2.6);
    private_nh_.param(
      "gate/max_longitudinal_offset",
      gate_max_longitudinal_offset_,
      0.35);
    private_nh_.param(
      "gate/max_midpoint_lateral_offset",
      gate_max_midpoint_lateral_offset_,
      0.45);
    private_nh_.param(
      "gate/min_cone_confidence", gate_min_cone_confidence_, 0.25);
    private_nh_.param("gate/require_double_gate", require_double_gate_, true);
    private_nh_.param(
      "gate/min_double_gate_spacing",
      min_double_gate_spacing_,
      0.20);
    private_nh_.param(
      "gate/max_double_gate_spacing",
      max_double_gate_spacing_,
      0.90);
    private_nh_.param(
      "gate/max_double_width_difference",
      max_double_width_difference_,
      0.45);
    private_nh_.param(
      "gate/required_present_frames", required_present_frames_, 3);
    private_nh_.param(
      "gate/required_clear_frames", required_clear_frames_, 3);
    private_nh_.param("gate/rearm_time", gate_rearm_time_, 1.0);

    private_nh_.param(
      "acceleration/use_start_gate", acceleration_use_start_gate_, true);
    private_nh_.param(
      "acceleration/approach_speed", acceleration_approach_speed_, 0.30);
    private_nh_.param(
      "acceleration/run_speed", acceleration_run_speed_, 0.30);
    private_nh_.param(
      "acceleration/minimum_run_time",
      acceleration_minimum_run_time_,
      8.0);
    private_nh_.param(
      "acceleration/start_gate_timeout",
      acceleration_start_gate_timeout_,
      15.0);

    private_nh_.param("skidpad/entry_speed", skidpad_entry_speed_, 0.25);
    private_nh_.param("skidpad/circle_speed", skidpad_circle_speed_, 0.30);
    private_nh_.param(
      "skidpad/transition_speed", skidpad_transition_speed_, 0.20);
    private_nh_.param("skidpad/exit_speed", skidpad_exit_speed_, 0.25);
    private_nh_.param(
      "skidpad/lap_yaw_threshold", skidpad_lap_yaw_threshold_, 5.20);
    private_nh_.param(
      "skidpad/minimum_lap_time", skidpad_minimum_lap_time_, 10.0);
    private_nh_.param(
      "skidpad/transition_minimum_time",
      skidpad_transition_minimum_time_,
      1.0);
    private_nh_.param(
      "skidpad/exit_duration", skidpad_exit_duration_, 20.0);
    private_nh_.param(
      "skidpad/use_odometry_crossing",
      skidpad_use_odometry_crossing_,
      false);
    private_nh_.param(
      "skidpad/crossover_world_x", skidpad_crossover_world_x_, 0.0);
    private_nh_.param(
      "skidpad/crossover_max_abs_y", skidpad_crossover_max_abs_y_, 3.0);
    private_nh_.param(
      "skidpad/crossover_rearm_distance",
      skidpad_crossover_rearm_distance_,
      1.0);
    private_nh_.param(
      "skidpad/crossover_min_interval",
      skidpad_crossover_min_interval_,
      4.0);
    private_nh_.param(
      "skidpad/exit_distance", skidpad_exit_distance_, 0.0);

    if (mission_type_ != "acceleration" && mission_type_ != "skidpad")
    {
      throw std::runtime_error(
        "mission_type must be acceleration or skidpad");
    }
    if (mission_type_ == "skidpad" && !require_imu_)
    {
      ROS_WARN(
        "Skidpad is running without required IMU. "
        "This is only suitable for interface testing.");
    }
    gyro_bias_samples_ = std::max(1, gyro_bias_samples_);
    required_present_frames_ = std::max(1, required_present_frames_);
    required_clear_frames_ = std::max(1, required_clear_frames_);
    skidpad_crossover_rearm_distance_ = std::max(
      0.2, skidpad_crossover_rearm_distance_);
    skidpad_crossover_min_interval_ = std::max(
      0.5, skidpad_crossover_min_interval_);
    skidpad_exit_distance_ = std::max(0.0, skidpad_exit_distance_);
  }

  bool startCallback(
    std_srvs::Trigger::Request&,
    std_srvs::Trigger::Response& response)
  {
    const ros::Time now = ros::Time::now();
    if (state_ != MissionState::WAIT_GO)
    {
      response.success = false;
      response.message = "reset mission before starting again";
      return true;
    }
    if (require_cones_before_start_ &&
        (last_cone_time_.isZero() ||
         (now - last_cone_time_).toSec() > cone_timeout_))
    {
      response.success = false;
      response.message = "cone input is not ready";
      return true;
    }
    if (require_imu_ &&
        (!imu_ready_ || last_imu_time_.isZero() ||
         (now - last_imu_time_).toSec() > imu_timeout_))
    {
      response.success = false;
      response.message = "IMU is not ready or gyro bias is not calibrated";
      return true;
    }

    resetRunData();
    mission_start_time_ = now;
    if (mission_type_ == "acceleration")
    {
      transitionTo(
        acceleration_use_start_gate_ ?
        MissionState::APPROACH_START :
        MissionState::ACCELERATING);
    }
    else
    {
      transitionTo(MissionState::ENTER);
    }
    response.success = true;
    response.message = "mission started";
    return true;
  }

  bool resetCallback(
    std_srvs::Trigger::Request&,
    std_srvs::Trigger::Response& response)
  {
    resetMission();
    response.success = true;
    response.message = "mission reset to WAIT_GO";
    return true;
  }

  bool testGateCallback(
    std_srvs::Trigger::Request&,
    std_srvs::Trigger::Response& response)
  {
    if (!isRunningState())
    {
      response.success = false;
      response.message = "mission is not running";
      return true;
    }
    new_gate_event_ = true;
    response.success = true;
    response.message = "test-only gate event injected";
    return true;
  }

  void resetMission()
  {
    state_ = MissionState::WAIT_GO;
    state_entry_time_ = ros::Time::now();
    mission_start_time_ = ros::Time(0);
    fault_reason_.clear();
    resetRunData();
  }

  void resetRunData()
  {
    segment_yaw_ = 0.0;
    last_imu_stamp_ = ros::Time(0);
    gate_present_frames_ = 0;
    gate_clear_frames_ = 0;
    gate_stable_ = false;
    gate_armed_ = true;
    new_gate_event_ = false;
    last_gate_event_time_ = ros::Time(0);
    last_gate_midpoint_x_ = std::numeric_limits<double>::quiet_NaN();
    new_crossover_event_ = false;
    skidpad_crossing_count_ = 0;
    crossover_armed_ = has_odometry_ &&
      current_odom_x_ <= skidpad_crossover_world_x_ -
        skidpad_crossover_rearm_distance_;
    last_crossover_time_ = ros::Time(0);
    exit_start_odom_valid_ = false;
  }

  void transitionTo(const MissionState next_state)
  {
    if (state_ == next_state)
    {
      return;
    }
    ROS_INFO(
      "Mission transition: %s -> %s",
      stateName(state_).c_str(),
      stateName(next_state).c_str());
    state_ = next_state;
    state_entry_time_ = ros::Time::now();
    segment_yaw_ = 0.0;
    last_imu_stamp_ = ros::Time(0);
    if (next_state == MissionState::EXIT && has_odometry_)
    {
      exit_start_odom_x_ = current_odom_x_;
      exit_start_odom_y_ = current_odom_y_;
      exit_start_odom_valid_ = true;
    }
  }

  void setFault(const std::string& reason)
  {
    if (state_ == MissionState::FAULT)
    {
      return;
    }
    fault_reason_ = reason;
    ROS_ERROR("Mission fault: %s", reason.c_str());
    transitionTo(MissionState::FAULT);
  }

  bool isRunningState() const
  {
    return state_ != MissionState::WAIT_GO &&
      state_ != MissionState::FINISHED &&
      state_ != MissionState::FAULT;
  }

  bool needsLiveSensors() const
  {
    return isRunningState() && state_ != MissionState::BRAKING;
  }

  double stateAge(const ros::Time& now) const
  {
    return state_entry_time_.isZero() ?
      0.0 : (now - state_entry_time_).toSec();
  }

  void imuCallback(const sensor_msgs::Imu::ConstPtr& message)
  {
    const double raw_rate = message->angular_velocity.z;
    if (!std::isfinite(raw_rate))
    {
      return;
    }
    const ros::Time stamp =
      message->header.stamp.isZero() ? ros::Time::now() : message->header.stamp;
    last_imu_time_ = ros::Time::now();

    if (calibrate_gyro_bias_ && !imu_ready_ &&
        state_ == MissionState::WAIT_GO)
    {
      if (std::abs(raw_rate) <= calibration_max_rate_)
      {
        gyro_bias_sum_ += raw_rate;
        ++gyro_bias_count_;
        if (gyro_bias_count_ >= gyro_bias_samples_)
        {
          gyro_bias_z_ =
            gyro_bias_sum_ / static_cast<double>(gyro_bias_count_);
          imu_ready_ = true;
          ROS_INFO(
            "IMU gyro-z bias calibrated: %.6f rad/s from %d samples",
            gyro_bias_z_, gyro_bias_count_);
        }
      }
      else
      {
        gyro_bias_sum_ = 0.0;
        gyro_bias_count_ = 0;
      }
    }
    else if (!calibrate_gyro_bias_)
    {
      imu_ready_ = true;
    }

    if (isRunningState() && !last_imu_stamp_.isZero())
    {
      const double dt = (stamp - last_imu_stamp_).toSec();
      if (dt > 0.0 && dt <= max_integration_dt_)
      {
        segment_yaw_ +=
          (raw_rate - gyro_bias_z_) * dt * imu_yaw_sign_;
      }
    }
    last_imu_stamp_ = stamp;
  }

  void odometryCallback(const nav_msgs::Odometry::ConstPtr& message)
  {
    current_odom_x_ = message->pose.pose.position.x;
    current_odom_y_ = message->pose.pose.position.y;
    last_odom_time_ = ros::Time::now();
    has_odometry_ = std::isfinite(current_odom_x_) &&
      std::isfinite(current_odom_y_);
    if (!has_odometry_ || mission_type_ != "skidpad" ||
        !skidpad_use_odometry_crossing_ || !isRunningState())
    {
      return;
    }

    // The skidpad timing line is crossed in the entry direction five times:
    // enter, finish right lap 1, finish right lap 2, finish left lap 1,
    // finish left lap 2.  Arm only on the negative-x side, so the opposite
    // half of either circle cannot be counted as a timing-line crossing.
    if (current_odom_x_ <= skidpad_crossover_world_x_ -
          skidpad_crossover_rearm_distance_)
    {
      crossover_armed_ = true;
    }

    const ros::Time now = ros::Time::now();
    const bool interval_ready = last_crossover_time_.isZero() ||
      (now - last_crossover_time_).toSec() >=
        skidpad_crossover_min_interval_;
    if (crossover_armed_ && interval_ready &&
        current_odom_x_ >= skidpad_crossover_world_x_ &&
        std::abs(current_odom_y_) <= skidpad_crossover_max_abs_y_)
    {
      crossover_armed_ = false;
      new_crossover_event_ = true;
      last_crossover_time_ = now;
      ROS_INFO(
        "Skidpad centre-line crossing detected at (%.2f, %.2f)",
        current_odom_x_, current_odom_y_);
    }
  }

  bool transformCones(
    const driverless_msgs::ConeObservationArray& message,
    std::vector<GateCone>& cones)
  {
    const std::string source_frame = message.header.frame_id;
    if (source_frame.empty())
    {
      ROS_WARN_THROTTLE(1.0, "Mission manager received cones without frame_id");
      return false;
    }

    geometry_msgs::TransformStamped transform;
    const bool needs_transform = source_frame != base_frame_;
    const ros::Time stamp =
      message.header.stamp.isZero() ? ros::Time(0) : message.header.stamp;
    if (needs_transform)
    {
      try
      {
        transform = tf_buffer_.lookupTransform(
          base_frame_,
          source_frame,
          stamp,
          ros::Duration(std::max(0.0, tf_timeout_)));
      }
      catch (const tf2::TransformException& exception)
      {
        ROS_WARN_THROTTLE(
          1.0, "Mission gate TF unavailable: %s", exception.what());
        return false;
      }
    }

    cones.clear();
    cones.reserve(message.cones.size());
    for (const auto& observation : message.cones)
    {
      const double confidence = std::max(
        static_cast<double>(observation.existence_probability),
        static_cast<double>(observation.lidar_confidence));
      if (confidence < gate_min_cone_confidence_)
      {
        continue;
      }

      geometry_msgs::Point position = observation.position;
      if (needs_transform)
      {
        geometry_msgs::PointStamped input;
        geometry_msgs::PointStamped output;
        input.header = message.header;
        input.header.stamp = stamp;
        input.point = observation.position;
        tf2::doTransform(input, output, transform);
        position = output.point;
      }

      if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
          position.x < gate_min_x_ || position.x > gate_max_x_ ||
          std::abs(position.y) > gate_max_abs_y_)
      {
        continue;
      }
      GateCone cone;
      cone.x = position.x;
      cone.y = position.y;
      cones.push_back(cone);
    }
    return true;
  }

  bool detectGate(
    const std::vector<GateCone>& cones,
    double& gate_midpoint_x) const
  {
    std::vector<GatePair> pairs;
    for (std::size_t first = 0U; first < cones.size(); ++first)
    {
      for (std::size_t second = first + 1U; second < cones.size(); ++second)
      {
        if (cones[first].y * cones[second].y >= 0.0)
        {
          continue;
        }
        const double width = std::abs(cones[first].y - cones[second].y);
        const double midpoint_x =
          0.5 * (cones[first].x + cones[second].x);
        const double midpoint_y =
          0.5 * (cones[first].y + cones[second].y);
        if (width < gate_min_width_ || width > gate_max_width_ ||
            std::abs(cones[first].x - cones[second].x) >
              gate_max_longitudinal_offset_ ||
            std::abs(midpoint_y) > gate_max_midpoint_lateral_offset_)
        {
          continue;
        }
        GatePair pair;
        pair.first = first;
        pair.second = second;
        pair.midpoint_x = midpoint_x;
        pair.midpoint_y = midpoint_y;
        pair.width = width;
        pairs.push_back(pair);
      }
    }

    if (!require_double_gate_)
    {
      if (pairs.empty())
      {
        return false;
      }
      gate_midpoint_x = pairs.front().midpoint_x;
      for (const GatePair& pair : pairs)
      {
        gate_midpoint_x = std::min(gate_midpoint_x, pair.midpoint_x);
      }
      return true;
    }

    bool found = false;
    double best_midpoint = std::numeric_limits<double>::infinity();
    for (std::size_t first = 0U; first < pairs.size(); ++first)
    {
      for (std::size_t second = first + 1U; second < pairs.size(); ++second)
      {
        const bool shares_cone =
          pairs[first].first == pairs[second].first ||
          pairs[first].first == pairs[second].second ||
          pairs[first].second == pairs[second].first ||
          pairs[first].second == pairs[second].second;
        const double spacing = std::abs(
          pairs[first].midpoint_x - pairs[second].midpoint_x);
        if (shares_cone ||
            spacing < min_double_gate_spacing_ ||
            spacing > max_double_gate_spacing_ ||
            std::abs(pairs[first].width - pairs[second].width) >
              max_double_width_difference_)
        {
          continue;
        }
        found = true;
        best_midpoint = std::min(
          best_midpoint,
          0.5 * (
            pairs[first].midpoint_x + pairs[second].midpoint_x));
      }
    }
    gate_midpoint_x = best_midpoint;
    return found;
  }

  void coneCallback(
    const driverless_msgs::ConeObservationArray::ConstPtr& message)
  {
    std::vector<GateCone> cones;
    if (!transformCones(*message, cones))
    {
      return;
    }
    last_cone_time_ = ros::Time::now();

    double midpoint_x = std::numeric_limits<double>::quiet_NaN();
    const bool gate_present = detectGate(cones, midpoint_x);
    if (gate_present)
    {
      last_gate_midpoint_x_ = midpoint_x;
      ++gate_present_frames_;
      gate_clear_frames_ = 0;
      if (!gate_stable_ &&
          gate_present_frames_ >= required_present_frames_)
      {
        gate_stable_ = true;
        if (gate_armed_)
        {
          new_gate_event_ = true;
          gate_armed_ = false;
          last_gate_event_time_ = ros::Time::now();
          ROS_INFO(
            "Stable double-cone gate detected at x=%.2f m",
            last_gate_midpoint_x_);
        }
      }
    }
    else
    {
      gate_present_frames_ = 0;
      ++gate_clear_frames_;
      if (gate_clear_frames_ >= required_clear_frames_)
      {
        gate_stable_ = false;
        new_gate_event_ = false;
        if (!last_gate_event_time_.isZero() &&
            (ros::Time::now() - last_gate_event_time_).toSec() >=
              gate_rearm_time_)
        {
          gate_armed_ = true;
        }
      }
    }
  }

  void acknowledgeGateEvent()
  {
    new_gate_event_ = false;
  }

  bool rightLapComplete(const bool gate_event, const double age) const
  {
    return gate_event &&
      age >= skidpad_minimum_lap_time_ &&
      segment_yaw_ <= -std::abs(skidpad_lap_yaw_threshold_);
  }

  bool leftLapComplete(const bool gate_event, const double age) const
  {
    return gate_event &&
      age >= skidpad_minimum_lap_time_ &&
      segment_yaw_ >= std::abs(skidpad_lap_yaw_threshold_);
  }

  void runAccelerationStateMachine(
    const ros::Time& now,
    const bool gate_event)
  {
    const double age = stateAge(now);
    if (state_ == MissionState::APPROACH_START)
    {
      if (gate_event)
      {
        acknowledgeGateEvent();
        transitionTo(MissionState::ACCELERATING);
      }
      else if (age > acceleration_start_gate_timeout_)
      {
        setFault("start gate timeout");
      }
    }
    else if (state_ == MissionState::ACCELERATING &&
             gate_event &&
             age >= acceleration_minimum_run_time_)
    {
      acknowledgeGateEvent();
      transitionTo(MissionState::BRAKING);
    }
  }

  void runSkidpadStateMachine(
    const ros::Time& now,
    const bool gate_event)
  {
    const double age = stateAge(now);
    switch (state_)
    {
      case MissionState::ENTER:
        if (gate_event)
        {
          acknowledgeGateEvent();
          transitionTo(MissionState::RIGHT_LAP_1);
        }
        break;
      case MissionState::RIGHT_LAP_1:
        if (rightLapComplete(gate_event, age))
        {
          acknowledgeGateEvent();
          transitionTo(MissionState::RIGHT_LAP_2);
        }
        break;
      case MissionState::RIGHT_LAP_2:
        if (rightLapComplete(gate_event, age))
        {
          acknowledgeGateEvent();
          transitionTo(MissionState::TRANSITION_TO_LEFT);
        }
        break;
      case MissionState::TRANSITION_TO_LEFT:
        if (!gate_stable_ && age >= skidpad_transition_minimum_time_)
        {
          transitionTo(MissionState::LEFT_LAP_1);
        }
        break;
      case MissionState::LEFT_LAP_1:
        if (leftLapComplete(gate_event, age))
        {
          acknowledgeGateEvent();
          transitionTo(MissionState::LEFT_LAP_2);
        }
        break;
      case MissionState::LEFT_LAP_2:
        if (leftLapComplete(gate_event, age))
        {
          acknowledgeGateEvent();
          transitionTo(MissionState::EXIT);
        }
        break;
      case MissionState::EXIT:
        if (age >= skidpad_exit_duration_)
        {
          transitionTo(MissionState::BRAKING);
        }
        break;
      default:
        break;
    }
  }

  void runSkidpadOdometryStateMachine(
    const ros::Time& now,
    const bool crossover_event)
  {
    if (crossover_event)
    {
      new_crossover_event_ = false;
      ++skidpad_crossing_count_;
      switch (skidpad_crossing_count_)
      {
        case 1:
          transitionTo(MissionState::RIGHT_LAP_1);
          break;
        case 2:
          transitionTo(MissionState::RIGHT_LAP_2);
          break;
        case 3:
          // The third passage changes directly from the right circle to the
          // left circle.  There is no extra straight-lap state in the rules.
          transitionTo(MissionState::LEFT_LAP_1);
          break;
        case 4:
          transitionTo(MissionState::LEFT_LAP_2);
          break;
        case 5:
          transitionTo(MissionState::EXIT);
          break;
        default:
          break;
      }
    }

    if (state_ == MissionState::EXIT)
    {
      bool exit_complete = false;
      if (skidpad_exit_distance_ > 0.0 && exit_start_odom_valid_ &&
          has_odometry_)
      {
        exit_complete = std::hypot(
          current_odom_x_ - exit_start_odom_x_,
          current_odom_y_ - exit_start_odom_y_) >= skidpad_exit_distance_;
      }
      else
      {
        exit_complete = stateAge(now) >= skidpad_exit_duration_;
      }
      if (exit_complete)
      {
        transitionTo(MissionState::BRAKING);
      }
    }
  }

  void timerCallback(const ros::TimerEvent&)
  {
    const ros::Time now = ros::Time::now();
    // Simulation-only convenience when enabled by configuration.  The
    // default remains false, so real-vehicle missions still require the
    // explicit /mission/start safety action.
    if (auto_start_ && state_ == MissionState::WAIT_GO)
    {
      const bool cones_ready =
        !require_cones_before_start_ ||
        (!last_cone_time_.isZero() &&
         (now - last_cone_time_).toSec() <= cone_timeout_);
      const bool imu_ready =
        !require_imu_ ||
        (imu_ready_ && !last_imu_time_.isZero() &&
         (now - last_imu_time_).toSec() <= imu_timeout_);
      if (cones_ready && imu_ready)
      {
        resetRunData();
        mission_start_time_ = now;
        transitionTo(
          mission_type_ == "acceleration" ?
            (acceleration_use_start_gate_ ?
              MissionState::APPROACH_START : MissionState::ACCELERATING) :
            MissionState::ENTER);
        ROS_INFO("Mission auto-started after simulation inputs became ready");
      }
    }
    if (needsLiveSensors())
    {
      if (monitor_cones_during_run_ &&
          (last_cone_time_.isZero() ||
           (now - last_cone_time_).toSec() > cone_timeout_))
      {
        setFault("cone input timeout");
      }
      else if (require_imu_ &&
               (last_imu_time_.isZero() ||
                (now - last_imu_time_).toSec() > imu_timeout_))
      {
        setFault("IMU timeout");
      }
      else if (!mission_start_time_.isZero() &&
               (now - mission_start_time_).toSec() >
                 maximum_mission_time_)
      {
        setFault("maximum mission time exceeded");
      }
    }

    const bool gate_event = new_gate_event_;
    if (state_ != MissionState::FAULT)
    {
      if (mission_type_ == "acceleration")
      {
        runAccelerationStateMachine(now, gate_event);
      }
      else
      {
        if (skidpad_use_odometry_crossing_)
        {
          runSkidpadOdometryStateMachine(now, new_crossover_event_);
        }
        else
        {
          runSkidpadStateMachine(now, gate_event);
        }
      }
    }

    if (state_ == MissionState::BRAKING &&
        stateAge(now) >= brake_duration_)
    {
      transitionTo(MissionState::FINISHED);
    }
    publishOutputs();
  }

  int desiredTurn() const
  {
    if (state_ == MissionState::ENTER ||
        state_ == MissionState::RIGHT_LAP_1 ||
        state_ == MissionState::RIGHT_LAP_2)
    {
      return -1;
    }
    if (state_ == MissionState::TRANSITION_TO_LEFT ||
        state_ == MissionState::LEFT_LAP_1 ||
        state_ == MissionState::LEFT_LAP_2)
    {
      return 1;
    }
    return 0;
  }

  double speedLimit() const
  {
    if (mission_type_ == "acceleration")
    {
      if (state_ == MissionState::APPROACH_START)
      {
        return acceleration_approach_speed_;
      }
      if (state_ == MissionState::ACCELERATING)
      {
        return acceleration_run_speed_;
      }
    }
    else
    {
      if (state_ == MissionState::ENTER)
      {
        return skidpad_entry_speed_;
      }
      if (state_ == MissionState::RIGHT_LAP_1 ||
          state_ == MissionState::RIGHT_LAP_2 ||
          state_ == MissionState::LEFT_LAP_1 ||
          state_ == MissionState::LEFT_LAP_2)
      {
        return skidpad_circle_speed_;
      }
      if (state_ == MissionState::TRANSITION_TO_LEFT)
      {
        return skidpad_transition_speed_;
      }
      if (state_ == MissionState::EXIT)
      {
        return skidpad_exit_speed_;
      }
    }
    return 0.0;
  }

  bool hardStopRequested() const
  {
    return state_ == MissionState::WAIT_GO ||
      state_ == MissionState::BRAKING ||
      state_ == MissionState::FINISHED ||
      state_ == MissionState::FAULT;
  }

  void publishOutputs()
  {
    std_msgs::String state_message;
    state_message.data = stateName(state_);
    state_pub_.publish(state_message);

    std_msgs::Int8 turn_message;
    turn_message.data = static_cast<int8_t>(desiredTurn());
    turn_hint_pub_.publish(turn_message);

    std_msgs::Float32 speed_message;
    speed_message.data = static_cast<float>(speedLimit());
    speed_limit_pub_.publish(speed_message);

    std_msgs::Bool stop_message;
    stop_message.data = hardStopRequested();
    stop_pub_.publish(stop_message);

    std_msgs::Bool finished_message;
    finished_message.data = state_ == MissionState::FINISHED;
    finished_pub_.publish(finished_message);

    std::ostringstream status;
    status << "state=" << stateName(state_)
           << "; turn_hint=" << desiredTurn()
           << "; crossing_count=" << skidpad_crossing_count_
           << "; speed_limit=" << speedLimit()
           << "; gate_stable=" << (gate_stable_ ? 1 : 0)
           << "; gate_armed=" << (gate_armed_ ? 1 : 0)
           << "; gate_x=" << last_gate_midpoint_x_
           << "; segment_yaw=" << segment_yaw_
           << "; imu_ready=" << (imu_ready_ ? 1 : 0);
    if (!fault_reason_.empty())
    {
      status << "; fault=" << fault_reason_;
    }
    std_msgs::String status_message;
    status_message.data = status.str();
    status_pub_.publish(status_message);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Subscriber cone_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher state_pub_;
  ros::Publisher turn_hint_pub_;
  ros::Publisher speed_limit_pub_;
  ros::Publisher stop_pub_;
  ros::Publisher finished_pub_;
  ros::Publisher status_pub_;
  ros::ServiceServer start_service_;
  ros::ServiceServer reset_service_;
  ros::ServiceServer test_gate_service_;
  ros::Timer timer_;

  MissionState state_ = MissionState::WAIT_GO;
  ros::Time state_entry_time_;
  ros::Time mission_start_time_;
  ros::Time last_cone_time_;
  ros::Time last_imu_time_;
  ros::Time last_imu_stamp_;
  ros::Time last_gate_event_time_;
  ros::Time last_odom_time_;
  ros::Time last_crossover_time_;
  std::string fault_reason_;

  std::string mission_type_;
  std::string base_frame_;
  std::string cone_topic_;
  std::string imu_topic_;
  std::string odom_topic_;
  std::string state_topic_;
  std::string turn_hint_topic_;
  std::string speed_limit_topic_;
  std::string stop_topic_;
  std::string finished_topic_;
  std::string status_topic_;
  std::string start_service_name_;
  std::string reset_service_name_;
  std::string test_gate_service_name_;

  bool require_cones_before_start_ = true;
  bool monitor_cones_during_run_ = true;
  bool require_imu_ = false;
  bool auto_start_ = false;
  bool calibrate_gyro_bias_ = true;
  bool imu_ready_ = false;
  bool require_double_gate_ = true;
  bool gate_stable_ = false;
  bool gate_armed_ = true;
  bool new_gate_event_ = false;
  bool acceleration_use_start_gate_ = true;
  bool skidpad_use_odometry_crossing_ = false;
  bool has_odometry_ = false;
  bool crossover_armed_ = false;
  bool new_crossover_event_ = false;
  bool exit_start_odom_valid_ = false;

  int gyro_bias_samples_ = 100;
  int gyro_bias_count_ = 0;
  int required_present_frames_ = 3;
  int required_clear_frames_ = 3;
  int gate_present_frames_ = 0;
  int gate_clear_frames_ = 0;
  int skidpad_crossing_count_ = 0;

  double publish_rate_ = 20.0;
  double tf_timeout_ = 0.03;
  double cone_timeout_ = 0.50;
  double imu_timeout_ = 0.30;
  double maximum_mission_time_ = 100.0;
  double brake_duration_ = 2.0;
  double gyro_bias_z_ = 0.0;
  double gyro_bias_sum_ = 0.0;
  double calibration_max_rate_ = 0.08;
  double max_integration_dt_ = 0.10;
  double imu_yaw_sign_ = 1.0;
  double segment_yaw_ = 0.0;
  double gate_min_x_ = 0.2;
  double gate_max_x_ = 4.0;
  double gate_max_abs_y_ = 2.5;
  double gate_min_width_ = 1.0;
  double gate_max_width_ = 2.6;
  double gate_max_longitudinal_offset_ = 0.35;
  double gate_max_midpoint_lateral_offset_ = 0.45;
  double gate_min_cone_confidence_ = 0.25;
  double min_double_gate_spacing_ = 0.20;
  double max_double_gate_spacing_ = 0.90;
  double max_double_width_difference_ = 0.45;
  double gate_rearm_time_ = 1.0;
  double last_gate_midpoint_x_ =
    std::numeric_limits<double>::quiet_NaN();
  double acceleration_approach_speed_ = 0.30;
  double acceleration_run_speed_ = 0.30;
  double acceleration_minimum_run_time_ = 8.0;
  double acceleration_start_gate_timeout_ = 15.0;
  double skidpad_entry_speed_ = 0.25;
  double skidpad_circle_speed_ = 0.30;
  double skidpad_transition_speed_ = 0.20;
  double skidpad_exit_speed_ = 0.25;
  double skidpad_lap_yaw_threshold_ = 5.20;
  double skidpad_minimum_lap_time_ = 10.0;
  double skidpad_transition_minimum_time_ = 1.0;
  double skidpad_exit_duration_ = 20.0;
  double skidpad_crossover_world_x_ = 0.0;
  double skidpad_crossover_max_abs_y_ = 3.0;
  double skidpad_crossover_rearm_distance_ = 1.0;
  double skidpad_crossover_min_interval_ = 4.0;
  double skidpad_exit_distance_ = 0.0;
  double current_odom_x_ = 0.0;
  double current_odom_y_ = 0.0;
  double exit_start_odom_x_ = 0.0;
  double exit_start_odom_y_ = 0.0;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scout_mission_manager");
  try
  {
    ScoutMissionManager manager;
    ros::spin();
  }
  catch (const std::exception& exception)
  {
    ROS_FATAL(
      "Failed to start scout_mission_manager: %s",
      exception.what());
    return 1;
  }
  return 0;
}
