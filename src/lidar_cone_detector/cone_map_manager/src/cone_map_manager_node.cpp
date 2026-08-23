#include <ros/ros.h>

#include <driverless_msgs/ConeObservationArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

namespace
{

double clamp(const double value, const double lower, const double upper)
{
  return std::max(lower, std::min(upper, value));
}

struct Landmark
{
  driverless_msgs::ConeObservation cone;
  ros::Time last_seen;
  std::uint32_t hits = 0U;
};

}  // namespace

class ConeMapManager
{
public:
  ConeMapManager()
    : private_nh_("~"),
      tf_listener_(tf_buffer_)
  {
    loadParameters();

    cone_sub_ = nh_.subscribe(
      input_topic_, 2, &ConeMapManager::coneCallback, this);
    if (use_pose_topic_)
    {
      pose_sub_ = nh_.subscribe(
        pose_topic_, 20, &ConeMapManager::poseCallback, this);
    }

    map_pub_ = nh_.advertise<driverless_msgs::ConeObservationArray>(
      output_topic_, 1, true);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      marker_topic_, 1);
    status_pub_ = nh_.advertise<std_msgs::String>(
      status_topic_, 1, true);
    reset_service_ = nh_.advertiseService(
      "/mapping/reset", &ConeMapManager::resetCallback, this);
    timer_ = nh_.createTimer(
      ros::Duration(1.0 / std::max(1.0, publish_rate_)),
      &ConeMapManager::timerCallback,
      this);

    publishStatus("WAITING_FOR_INPUT");
    ROS_INFO(
      "cone_map_manager started: input=%s, output=%s, frame=%s",
      input_topic_.c_str(), output_topic_.c_str(), map_frame_.c_str());
  }

private:
  void loadParameters()
  {
    private_nh_.param<std::string>(
      "input_topic", input_topic_, "/perception/lidar/cones_raw");
    private_nh_.param<std::string>(
      "pose_topic", pose_topic_, "/localization/pose");
    private_nh_.param<std::string>(
      "output_topic", output_topic_, "/mapping/cones");
    private_nh_.param<std::string>(
      "marker_topic", marker_topic_, "/mapping/cone_markers");
    private_nh_.param<std::string>(
      "status_topic", status_topic_, "/mapping/status");
    private_nh_.param<std::string>("map_frame", map_frame_, "map");
    private_nh_.param<std::string>("base_frame", base_frame_, "base_link");
    private_nh_.param("use_pose_topic", use_pose_topic_, true);
    private_nh_.param("tf_timeout", tf_timeout_, 0.03);
    private_nh_.param("pose_max_age", pose_max_age_, 0.20);
    private_nh_.param("input_timeout", input_timeout_, 0.50);
    private_nh_.param("publish_rate", publish_rate_, 10.0);
    private_nh_.param(
      "min_observation_confidence",
      min_observation_confidence_,
      0.35);
    private_nh_.param(
      "association_distance", association_distance_, 0.70);
    private_nh_.param(
      "minimum_confirm_hits", minimum_confirm_hits_, 3);
    private_nh_.param(
      "maximum_landmark_age", maximum_landmark_age_, 30.0);
    private_nh_.param(
      "minimum_update_gain", minimum_update_gain_, 0.15);
    private_nh_.param(
      "maximum_update_gain", maximum_update_gain_, 0.60);

    minimum_confirm_hits_ = std::max(1, minimum_confirm_hits_);
    association_distance_ = std::max(0.05, association_distance_);
    minimum_update_gain_ = clamp(minimum_update_gain_, 0.01, 1.0);
    maximum_update_gain_ = clamp(
      maximum_update_gain_, minimum_update_gain_, 1.0);
  }

  void poseCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& message)
  {
    if (message->header.frame_id != map_frame_)
    {
      ROS_WARN_THROTTLE(
        1.0,
        "Ignoring localization pose in frame '%s'; expected '%s'",
        message->header.frame_id.c_str(),
        map_frame_.c_str());
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    latest_pose_ = *message;
    has_pose_ = true;
    last_pose_wall_time_ = ros::WallTime::now();
  }

  bool lookupMapTransform(
    const std::string& source_frame,
    const ros::Time& stamp,
    geometry_msgs::TransformStamped& transform)
  {
    try
    {
      transform = tf_buffer_.lookupTransform(
        map_frame_,
        source_frame,
        stamp,
        ros::Duration(std::max(0.0, tf_timeout_)));
      return true;
    }
    catch (const tf2::TransformException&)
    {
      return false;
    }
  }

  bool buildTransformFromPose(
    const ros::Time& stamp,
    geometry_msgs::TransformStamped& transform)
  {
    geometry_msgs::PoseWithCovarianceStamped pose;
    ros::WallTime pose_wall_time;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!has_pose_)
      {
        return false;
      }
      pose = latest_pose_;
      pose_wall_time = last_pose_wall_time_;
    }

    const double wall_age =
      (ros::WallTime::now() - pose_wall_time).toSec();
    const double stamp_age =
      stamp.isZero() || pose.header.stamp.isZero() ?
      0.0 : std::abs((stamp - pose.header.stamp).toSec());
    if (wall_age > pose_max_age_ || stamp_age > pose_max_age_)
    {
      return false;
    }

    transform.header = pose.header;
    transform.header.stamp = stamp;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = pose.pose.pose.position.x;
    transform.transform.translation.y = pose.pose.pose.position.y;
    transform.transform.translation.z = pose.pose.pose.position.z;
    transform.transform.rotation = pose.pose.pose.orientation;
    return true;
  }

  bool transformObservation(
    const driverless_msgs::ConeObservation& observation,
    const std_msgs::Header& array_header,
    geometry_msgs::Point& map_point)
  {
    const std::string source_frame = array_header.frame_id;
    if (source_frame.empty())
    {
      return false;
    }
    const ros::Time stamp =
      array_header.stamp.isZero() ? ros::Time(0) : array_header.stamp;

    geometry_msgs::TransformStamped direct_transform;
    if (lookupMapTransform(source_frame, stamp, direct_transform))
    {
      geometry_msgs::PointStamped input;
      geometry_msgs::PointStamped output;
      input.header = array_header;
      input.header.stamp = stamp;
      input.point = observation.position;
      tf2::doTransform(input, output, direct_transform);
      map_point = output.point;
      return true;
    }

    if (!use_pose_topic_)
    {
      return false;
    }

    geometry_msgs::Point base_point = observation.position;
    if (source_frame != base_frame_)
    {
      geometry_msgs::TransformStamped base_transform;
      try
      {
        base_transform = tf_buffer_.lookupTransform(
          base_frame_,
          source_frame,
          stamp,
          ros::Duration(std::max(0.0, tf_timeout_)));
      }
      catch (const tf2::TransformException&)
      {
        return false;
      }
      geometry_msgs::PointStamped input;
      geometry_msgs::PointStamped output;
      input.header = array_header;
      input.header.stamp = stamp;
      input.point = observation.position;
      tf2::doTransform(input, output, base_transform);
      base_point = output.point;
    }

    geometry_msgs::TransformStamped pose_transform;
    if (!buildTransformFromPose(stamp, pose_transform))
    {
      return false;
    }
    geometry_msgs::PointStamped input;
    geometry_msgs::PointStamped output;
    input.header.stamp = stamp;
    input.header.frame_id = base_frame_;
    input.point = base_point;
    tf2::doTransform(input, output, pose_transform);
    map_point = output.point;
    return true;
  }

  int findAssociation(const geometry_msgs::Point& point) const
  {
    int best_index = -1;
    double best_distance = association_distance_;
    for (std::size_t index = 0U; index < landmarks_.size(); ++index)
    {
      const double distance = std::hypot(
        point.x - landmarks_[index].cone.position.x,
        point.y - landmarks_[index].cone.position.y);
      if (distance < best_distance)
      {
        best_distance = distance;
        best_index = static_cast<int>(index);
      }
    }
    return best_index;
  }

  void updateLandmark(
    Landmark& landmark,
    const driverless_msgs::ConeObservation& observation,
    const geometry_msgs::Point& map_point,
    const ros::Time& stamp)
  {
    const double confidence = clamp(
      std::max(
        static_cast<double>(observation.existence_probability),
        static_cast<double>(observation.lidar_confidence)),
      0.0,
      1.0);
    const double gain = clamp(
      confidence, minimum_update_gain_, maximum_update_gain_);
    const double residual_x = map_point.x - landmark.cone.position.x;
    const double residual_y = map_point.y - landmark.cone.position.y;

    landmark.cone.position.x += gain * residual_x;
    landmark.cone.position.y += gain * residual_y;
    landmark.cone.position.z +=
      gain * (map_point.z - landmark.cone.position.z);
    landmark.cone.position_covariance[0] =
      (1.0 - gain) * landmark.cone.position_covariance[0] +
      gain * residual_x * residual_x;
    landmark.cone.position_covariance[4] =
      (1.0 - gain) * landmark.cone.position_covariance[4] +
      gain * residual_y * residual_y;
    landmark.cone.position_covariance[8] =
      std::max(
        1.0e-4,
        (1.0 - gain) * landmark.cone.position_covariance[8]);

    landmark.cone.existence_probability = static_cast<float>(
      std::max(
        static_cast<double>(landmark.cone.existence_probability),
        confidence));
    landmark.cone.lidar_confidence = std::max(
      landmark.cone.lidar_confidence,
      observation.lidar_confidence);
    landmark.cone.height =
      std::max(landmark.cone.height, observation.height);
    landmark.cone.width =
      std::max(landmark.cone.width, observation.width);
    landmark.cone.depth =
      std::max(landmark.cone.depth, observation.depth);
    landmark.cone.point_count =
      std::max(landmark.cone.point_count, observation.point_count);
    landmark.cone.candidate_level =
      std::max(landmark.cone.candidate_level, observation.candidate_level);

    if (observation.semantic_class !=
          driverless_msgs::ConeObservation::SEMANTIC_UNKNOWN &&
        observation.camera_color_confidence >=
          landmark.cone.camera_color_confidence)
    {
      landmark.cone.semantic_class = observation.semantic_class;
      landmark.cone.camera_color_confidence =
        observation.camera_color_confidence;
      landmark.cone.source = observation.source;
    }

    ++landmark.hits;
    landmark.cone.confirmed =
      landmark.hits >= static_cast<std::uint32_t>(minimum_confirm_hits_);
    landmark.cone.header.stamp = stamp;
    landmark.cone.header.frame_id = map_frame_;
    landmark.last_seen = stamp;
  }

  void coneCallback(
    const driverless_msgs::ConeObservationArray::ConstPtr& message)
  {
    const ros::Time stamp =
      message->header.stamp.isZero() ? ros::Time::now() : message->header.stamp;
    std::vector<std::pair<
      driverless_msgs::ConeObservation,
      geometry_msgs::Point>> transformed;
    transformed.reserve(message->cones.size());

    for (const auto& observation : message->cones)
    {
      const double confidence = std::max(
        static_cast<double>(observation.existence_probability),
        static_cast<double>(observation.lidar_confidence));
      if (confidence < min_observation_confidence_)
      {
        continue;
      }
      geometry_msgs::Point point;
      if (!transformObservation(observation, message->header, point) ||
          !std::isfinite(point.x) ||
          !std::isfinite(point.y) ||
          !std::isfinite(point.z))
      {
        publishStatus("WAITING_FOR_MAP_POSE_OR_TF");
        return;
      }
      transformed.emplace_back(observation, point);
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto& item : transformed)
      {
        const int match = findAssociation(item.second);
        if (match >= 0)
        {
          updateLandmark(
            landmarks_[static_cast<std::size_t>(match)],
            item.first,
            item.second,
            stamp);
        }
        else
        {
          Landmark landmark;
          landmark.cone = item.first;
          landmark.cone.id = next_landmark_id_++;
          landmark.cone.position = item.second;
          landmark.cone.position_covariance[0] = std::max(
            0.04, landmark.cone.position_covariance[0]);
          landmark.cone.position_covariance[4] = std::max(
            0.04, landmark.cone.position_covariance[4]);
          landmark.cone.position_covariance[8] = std::max(
            0.01, landmark.cone.position_covariance[8]);
          landmark.cone.header.stamp = stamp;
          landmark.cone.header.frame_id = map_frame_;
          landmark.cone.confirmed = minimum_confirm_hits_ <= 1;
          landmark.hits = 1U;
          landmark.last_seen = stamp;
          landmarks_.push_back(landmark);
        }
      }
      last_input_time_ = stamp;
      last_input_wall_time_ = ros::WallTime::now();
    }
    publishStatus("RUNNING");
  }

  void removeExpired(const ros::Time& now)
  {
    landmarks_.erase(
      std::remove_if(
        landmarks_.begin(),
        landmarks_.end(),
        [&](const Landmark& landmark)
        {
          return !landmark.last_seen.isZero() &&
            (now - landmark.last_seen).toSec() > maximum_landmark_age_;
        }),
      landmarks_.end());
  }

  void timerCallback(const ros::TimerEvent&)
  {
    const ros::Time now = ros::Time::now();
    driverless_msgs::ConeObservationArray output;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      removeExpired(now);
      output.header.stamp = now;
      output.header.frame_id = map_frame_;
      output.cones.reserve(landmarks_.size());
      for (const Landmark& landmark : landmarks_)
      {
        driverless_msgs::ConeObservation cone = landmark.cone;
        cone.header = output.header;
        output.cones.push_back(cone);
      }
    }
    map_pub_.publish(output);
    publishMarkers(output);

    ros::WallTime last_input_wall_time;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      last_input_wall_time = last_input_wall_time_;
    }
    if (!last_input_wall_time.isZero() &&
        (ros::WallTime::now() - last_input_wall_time).toSec() >
          input_timeout_)
    {
      publishStatus("INPUT_TIMEOUT");
    }
  }

  void publishMarkers(
    const driverless_msgs::ConeObservationArray& map)
  {
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(clear);

    for (std::size_t index = 0U; index < map.cones.size(); ++index)
    {
      const auto& cone = map.cones[index];
      visualization_msgs::Marker marker;
      marker.header = map.header;
      marker.ns = "cone_map";
      marker.id = static_cast<int>(index);
      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position = cone.position;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = std::max(0.12, static_cast<double>(cone.width));
      marker.scale.y = marker.scale.x;
      marker.scale.z = std::max(0.25, static_cast<double>(cone.height));
      marker.pose.position.z += 0.5 * marker.scale.z;
      marker.color.a = cone.confirmed ? 1.0F : 0.45F;
      marker.color.r = 0.9F;
      marker.color.g = 0.55F;
      marker.color.b = 0.1F;
      if (cone.semantic_class ==
          driverless_msgs::ConeObservation::SEMANTIC_BLUE)
      {
        marker.color.r = 0.1F;
        marker.color.g = 0.25F;
        marker.color.b = 1.0F;
      }
      else if (cone.semantic_class ==
               driverless_msgs::ConeObservation::SEMANTIC_YELLOW)
      {
        marker.color.r = 1.0F;
        marker.color.g = 0.9F;
        marker.color.b = 0.1F;
      }
      markers.markers.push_back(marker);
    }
    marker_pub_.publish(markers);
  }

  bool resetCallback(
    std_srvs::Trigger::Request&,
    std_srvs::Trigger::Response& response)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      landmarks_.clear();
      next_landmark_id_ = 1U;
    }
    response.success = true;
    response.message = "cone map cleared";
    publishStatus("RESET");
    return true;
  }

  void publishStatus(const std::string& state)
  {
    std::size_t count = 0U;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      count = landmarks_.size();
    }
    std::ostringstream stream;
    stream << state << "; landmarks=" << count;
    const std::string value = stream.str();
    if (value == last_status_)
    {
      return;
    }
    std_msgs::String message;
    message.data = value;
    status_pub_.publish(message);
    last_status_ = value;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Subscriber cone_sub_;
  ros::Subscriber pose_sub_;
  ros::Publisher map_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher status_pub_;
  ros::ServiceServer reset_service_;
  ros::Timer timer_;
  mutable std::mutex mutex_;
  std::vector<Landmark> landmarks_;
  geometry_msgs::PoseWithCovarianceStamped latest_pose_;
  bool has_pose_ = false;
  std::uint32_t next_landmark_id_ = 1U;
  ros::Time last_input_time_;
  ros::WallTime last_input_wall_time_;
  ros::WallTime last_pose_wall_time_;
  std::string last_status_;

  std::string input_topic_;
  std::string pose_topic_;
  std::string output_topic_;
  std::string marker_topic_;
  std::string status_topic_;
  std::string map_frame_;
  std::string base_frame_;
  bool use_pose_topic_ = true;
  double tf_timeout_ = 0.03;
  double pose_max_age_ = 0.20;
  double input_timeout_ = 0.50;
  double publish_rate_ = 10.0;
  double min_observation_confidence_ = 0.35;
  double association_distance_ = 0.70;
  int minimum_confirm_hits_ = 3;
  double maximum_landmark_age_ = 30.0;
  double minimum_update_gain_ = 0.15;
  double maximum_update_gain_ = 0.60;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cone_map_manager");
  ConeMapManager manager;
  ros::spin();
  return 0;
}
