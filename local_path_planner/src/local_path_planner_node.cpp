#include <ros/ros.h>

#include <driverless_msgs/ConeObservationArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <exception>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "local_path_planner/candidate_edge_generator.hpp"
#include "local_path_planner/path_searcher.hpp"
#include "local_path_planner/path_smoother.hpp"
#include "local_path_planner/path_validator.hpp"
#include "local_path_planner/single_side_recovery.hpp"

namespace local_path_planner
{
namespace
{

double degreesToRadians(double degrees)
{
    return degrees * 3.14159265358979323846 / 180.0;
}

double normalizeAngle(double angle)
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

geometry_msgs::Point toRosPoint(const Point2D& point, double z = 0.0)
{
    geometry_msgs::Point output;
    output.x = point.x;
    output.y = point.y;
    output.z = z;
    return output;
}

}  // namespace

class LocalPathPlannerNode
{
public:
    LocalPathPlannerNode()
        : private_nh_("~"),
          tf_listener_(tf_buffer_),
          last_input_wall_time_(ros::WallTime::now())
    {
        std::string input_topic;
        std::string output_topic;
        std::string debug_topic;
        std::string odometry_topic;
        std::string confidence_topic;
        std::string status_topic;
        int queue_size = 1;

        private_nh_.param<std::string>(
            "input_topic", input_topic, "/perception/lidar/cones_raw");
        private_nh_.param<std::string>(
            "output_topic", output_topic, "/planning/local_path");
        private_nh_.param<std::string>(
            "debug_topic", debug_topic, "/planning/debug/path_markers");
        private_nh_.param<std::string>(
            "odometry_topic", odometry_topic, "/odometry/filtered");
        private_nh_.param<std::string>(
            "confidence_topic", confidence_topic, "/planning/path_confidence");
        private_nh_.param<std::string>(
            "status_topic", status_topic, "/planning/path_status");
        private_nh_.param<std::string>(
            "expected_frame", expected_frame_, "base_link");
        private_nh_.param("queue_size", queue_size, 1);
        private_nh_.param("publish_debug", publish_debug_, true);
        private_nh_.param("min_input_cones", min_input_cones_, 4);
        private_nh_.param("roi/min_x", roi_min_x_, 0.3);
        private_nh_.param("roi/max_x", roi_max_x_, 10.0);
        private_nh_.param("roi/max_abs_y", roi_max_abs_y_, 5.0);
        private_nh_.param("roi/min_confidence", min_confidence_, 0.35);
        private_nh_.param("roi/min_cone_separation", min_cone_separation_, 0.08);
        private_nh_.param(
            "roi/require_confirmed", require_confirmed_, false);
        private_nh_.param(
            "roi/unconfirmed_confidence_scale",
            unconfirmed_confidence_scale_,
            0.75);
        private_nh_.param("history/enabled", history_enabled_, true);
        private_nh_.param("history/max_age", history_max_age_, 0.6);
        private_nh_.param(
            "history/fallback_enabled", fallback_enabled_, true);
        private_nh_.param(
            "history/fallback_max_age", fallback_max_age_, 0.35);
        private_nh_.param(
            "history/fallback_max_frames", fallback_max_frames_, 2);
        private_nh_.param(
            "history/fallback_confidence_decay",
            fallback_confidence_decay_,
            0.65);
        private_nh_.param(
            "single_side/max_age", single_side_max_age_, 1.2);
        private_nh_.param(
            "single_side/max_frames", single_side_max_frames_, 12);
        private_nh_.param("odometry/max_age", odometry_max_age_, 0.25);
        private_nh_.param(
            "odometry/buffer_duration", odometry_buffer_duration_, 2.0);
        private_nh_.param(
            "odometry/reset_translation", odometry_reset_translation_, 2.0);
        double odometry_reset_yaw_deg = 60.0;
        private_nh_.param(
            "odometry/reset_yaw_deg",
            odometry_reset_yaw_deg,
            odometry_reset_yaw_deg);
        odometry_reset_yaw_ = degreesToRadians(odometry_reset_yaw_deg);
        private_nh_.param(
            "tf_lookup_timeout", tf_lookup_timeout_, 0.03);
        private_nh_.param("input_timeout", input_timeout_, 0.35);
        private_nh_.param("watchdog_rate", watchdog_rate_, 10.0);

        edge_generator_.reset(new CandidateEdgeGenerator(loadEdgeConfig()));
        path_searcher_.reset(new PathSearcher(loadSearchConfig()));
        path_smoother_.reset(new PathSmoother(loadSmootherConfig()));
        path_validator_.reset(new PathValidator(loadValidatorConfig()));
        single_side_recovery_.reset(
            new SingleSideRecovery(loadSingleSideConfig()));

        cone_subscriber_ = nh_.subscribe(
            input_topic,
            std::max(1, queue_size),
            &LocalPathPlannerNode::coneCallback,
            this);
        odometry_subscriber_ = nh_.subscribe(
            odometry_topic,
            10,
            &LocalPathPlannerNode::odometryCallback,
            this);
        path_publisher_ = nh_.advertise<nav_msgs::Path>(output_topic, 1);
        debug_publisher_ =
            nh_.advertise<visualization_msgs::MarkerArray>(debug_topic, 1);
        confidence_publisher_ =
            nh_.advertise<std_msgs::Float32>(confidence_topic, 1);
        status_publisher_ = nh_.advertise<std_msgs::String>(status_topic, 1);
        watchdog_timer_ = nh_.createWallTimer(
            ros::WallDuration(1.0 / std::max(1.0, watchdog_rate_)),
            &LocalPathPlannerNode::watchdogCallback,
            this);

        ROS_INFO_STREAM(
            "Local path planner started. input=" << input_topic
            << ", output=" << output_topic);
    }

private:
    struct OdomState
    {
        bool valid = false;
        ros::Time stamp;
        std::string frame_id;
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
    };

    CandidateEdgeGeneratorConfig loadEdgeConfig()
    {
        CandidateEdgeGeneratorConfig config;
        private_nh_.param(
            "edge/use_delaunay", config.use_delaunay, config.use_delaunay);
        private_nh_.param(
            "edge/fallback_to_pairwise",
            config.fallback_to_pairwise,
            config.fallback_to_pairwise);
        private_nh_.param(
            "edge/min_delaunay_candidates",
            config.min_delaunay_candidates,
            config.min_delaunay_candidates);
        private_nh_.param("edge/min_width", config.min_width, config.min_width);
        private_nh_.param("edge/max_width", config.max_width, config.max_width);
        private_nh_.param(
            "edge/expected_width", config.expected_width, config.expected_width);
        private_nh_.param(
            "edge/max_longitudinal_offset",
            config.max_longitudinal_offset,
            config.max_longitudinal_offset);
        private_nh_.param(
            "edge/min_midpoint_x", config.min_midpoint_x, config.min_midpoint_x);
        private_nh_.param(
            "edge/max_reference_distance",
            config.max_reference_distance,
            config.max_reference_distance);
        private_nh_.param(
            "edge/width_cost_weight",
            config.width_cost_weight,
            config.width_cost_weight);
        private_nh_.param(
            "edge/longitudinal_cost_weight",
            config.longitudinal_cost_weight,
            config.longitudinal_cost_weight);
        private_nh_.param(
            "edge/confidence_cost_weight",
            config.confidence_cost_weight,
            config.confidence_cost_weight);
        private_nh_.param(
            "edge/reference_cost_weight",
            config.reference_cost_weight,
            config.reference_cost_weight);
        private_nh_.param(
            "edge/max_edges_per_cone",
            config.max_edges_per_cone,
            config.max_edges_per_cone);
        return config;
    }

    SingleSideRecoveryConfig loadSingleSideConfig()
    {
        SingleSideRecoveryConfig config;
        private_nh_.param(
            "single_side/enabled", config.enabled, config.enabled);
        private_nh_.param(
            "single_side/expected_track_width",
            config.expected_track_width,
            config.expected_track_width);
        private_nh_.param(
            "single_side/min_boundary_offset",
            config.min_boundary_offset,
            config.min_boundary_offset);
        private_nh_.param(
            "single_side/max_boundary_offset",
            config.max_boundary_offset,
            config.max_boundary_offset);
        private_nh_.param(
            "single_side/max_center_reference_distance",
            config.max_center_reference_distance,
            config.max_center_reference_distance);
        private_nh_.param(
            "single_side/min_forward_x",
            config.min_forward_x,
            config.min_forward_x);
        private_nh_.param(
            "single_side/max_forward_x",
            config.max_forward_x,
            config.max_forward_x);
        private_nh_.param(
            "single_side/min_point_spacing",
            config.min_point_spacing,
            config.min_point_spacing);
        private_nh_.param(
            "single_side/min_virtual_points",
            config.min_virtual_points,
            config.min_virtual_points);
        private_nh_.param(
            "single_side/confidence_scale",
            config.confidence_scale,
            config.confidence_scale);
        return config;
    }

    PathSearcherConfig loadSearchConfig()
    {
        PathSearcherConfig config;
        private_nh_.param(
            "search/start_min_distance",
            config.start_min_distance,
            config.start_min_distance);
        private_nh_.param(
            "search/start_max_distance",
            config.start_max_distance,
            config.start_max_distance);
        double max_start_heading_deg = config.max_start_heading * 180.0 /
            3.14159265358979323846;
        private_nh_.param(
            "search/max_start_heading_deg",
            max_start_heading_deg,
            max_start_heading_deg);
        config.max_start_heading = degreesToRadians(max_start_heading_deg);
        private_nh_.param(
            "search/min_step_distance",
            config.min_step_distance,
            config.min_step_distance);
        private_nh_.param(
            "search/max_step_distance",
            config.max_step_distance,
            config.max_step_distance);
        private_nh_.param(
            "search/max_gap_step_distance",
            config.max_gap_step_distance,
            config.max_gap_step_distance);
        private_nh_.param(
            "search/expected_step_distance",
            config.expected_step_distance,
            config.expected_step_distance);
        double max_turn_angle_deg = config.max_turn_angle * 180.0 /
            3.14159265358979323846;
        private_nh_.param(
            "search/max_turn_angle_deg",
            max_turn_angle_deg,
            max_turn_angle_deg);
        config.max_turn_angle = degreesToRadians(max_turn_angle_deg);
        double max_gap_turn_angle_deg =
            config.max_gap_turn_angle * 180.0 /
            3.14159265358979323846;
        private_nh_.param(
            "search/max_gap_turn_angle_deg",
            max_gap_turn_angle_deg,
            max_gap_turn_angle_deg);
        config.max_gap_turn_angle =
            degreesToRadians(max_gap_turn_angle_deg);
        private_nh_.param(
            "search/max_path_length",
            config.max_path_length,
            config.max_path_length);
        private_nh_.param(
            "search/max_reference_distance",
            config.max_reference_distance,
            config.max_reference_distance);
        private_nh_.param(
            "search/min_midpoints", config.min_midpoints, config.min_midpoints);
        private_nh_.param(
            "search/max_midpoints", config.max_midpoints, config.max_midpoints);
        private_nh_.param(
            "search/max_gap_bridges",
            config.max_gap_bridges,
            config.max_gap_bridges);
        private_nh_.param(
            "search/beam_width", config.beam_width, config.beam_width);
        private_nh_.param(
            "search/edge_cost_weight",
            config.edge_cost_weight,
            config.edge_cost_weight);
        private_nh_.param(
            "search/heading_cost_weight",
            config.heading_cost_weight,
            config.heading_cost_weight);
        private_nh_.param(
            "search/spacing_cost_weight",
            config.spacing_cost_weight,
            config.spacing_cost_weight);
        private_nh_.param(
            "search/history_cost_weight",
            config.history_cost_weight,
            config.history_cost_weight);
        private_nh_.param(
            "search/reused_cone_penalty",
            config.reused_cone_penalty,
            config.reused_cone_penalty);
        private_nh_.param(
            "search/progress_reward_weight",
            config.progress_reward_weight,
            config.progress_reward_weight);
        private_nh_.param(
            "search/gap_bridge_penalty",
            config.gap_bridge_penalty,
            config.gap_bridge_penalty);
        private_nh_.param(
            "search/gap_confidence_scale",
            config.gap_confidence_scale,
            config.gap_confidence_scale);
        return config;
    }

    PathSmootherConfig loadSmootherConfig()
    {
        PathSmootherConfig config;
        private_nh_.param(
            "smoother/use_cubic_spline",
            config.use_cubic_spline,
            config.use_cubic_spline);
        private_nh_.param(
            "smoother/min_input_spacing",
            config.min_input_spacing,
            config.min_input_spacing);
        private_nh_.param(
            "smoother/sample_spacing",
            config.sample_spacing,
            config.sample_spacing);
        private_nh_.param(
            "smoother/spline_tension",
            config.spline_tension,
            config.spline_tension);
        private_nh_.param(
            "smoother/max_spline_deviation",
            config.max_spline_deviation,
            config.max_spline_deviation);
        private_nh_.param(
            "smoother/smoothing_passes",
            config.smoothing_passes,
            config.smoothing_passes);
        return config;
    }

    PathValidatorConfig loadValidatorConfig()
    {
        PathValidatorConfig config;
        private_nh_.param(
            "validator/min_path_points",
            config.min_path_points,
            config.min_path_points);
        private_nh_.param(
            "validator/min_path_length",
            config.min_path_length,
            config.min_path_length);
        private_nh_.param(
            "validator/min_forward_progress",
            config.min_forward_progress,
            config.min_forward_progress);
        private_nh_.param(
            "validator/min_path_confidence",
            config.min_path_confidence,
            config.min_path_confidence);
        private_nh_.param(
            "validator/max_point_spacing",
            config.max_point_spacing,
            config.max_point_spacing);
        double max_heading_change_deg = config.max_heading_change * 180.0 /
            3.14159265358979323846;
        private_nh_.param(
            "validator/max_heading_change_deg",
            max_heading_change_deg,
            max_heading_change_deg);
        config.max_heading_change = degreesToRadians(max_heading_change_deg);
        private_nh_.param(
            "validator/max_abs_curvature",
            config.max_abs_curvature,
            config.max_abs_curvature);
        private_nh_.param(
            "validator/max_curvature_change",
            config.max_curvature_change,
            config.max_curvature_change);
        private_nh_.param(
            "validator/min_cone_clearance",
            config.min_cone_clearance,
            config.min_cone_clearance);
        return config;
    }

    bool prepareCones(
        const driverless_msgs::ConeObservationArray& message,
        std::vector<ConePoint>& cones,
        std_msgs::Header& output_header,
        std::string& reason)
    {
        const std::string source_frame = message.header.frame_id;
        output_header = message.header;
        if (output_header.stamp.isZero())
        {
            output_header.stamp = ros::Time::now();
        }
        output_header.frame_id = expected_frame_.empty() ?
            source_frame : expected_frame_;
        if (source_frame.empty())
        {
            reason = "input frame is empty";
            return false;
        }

        const std::string target_frame =
            expected_frame_.empty() ? source_frame : expected_frame_;
        output_header.frame_id = target_frame;

        const bool needs_transform = source_frame != target_frame;
        geometry_msgs::TransformStamped transform;
        if (needs_transform)
        {
            try
            {
                transform = tf_buffer_.lookupTransform(
                    target_frame,
                    source_frame,
                    output_header.stamp,
                    ros::Duration(std::max(0.0, tf_lookup_timeout_)));
            }
            catch (const tf2::TransformException& exception)
            {
                reason = std::string("TF unavailable: ") + exception.what();
                return false;
            }
        }

        cones.clear();
        cones.reserve(message.cones.size());
        for (const driverless_msgs::ConeObservation& observation : message.cones)
        {
            geometry_msgs::Point position = observation.position;
            if (needs_transform)
            {
                geometry_msgs::PointStamped input;
                geometry_msgs::PointStamped transformed;
                input.header = message.header;
                input.header.stamp = output_header.stamp;
                input.point = observation.position;
                tf2::doTransform(input, transformed, transform);
                position = transformed.point;
            }

            const double x = position.x;
            const double y = position.y;
            if (!std::isfinite(x) || !std::isfinite(y) ||
                x < roi_min_x_ || x > roi_max_x_ ||
                std::abs(y) > roi_max_abs_y_)
            {
                continue;
            }

            const double confidence = std::max(
                static_cast<double>(observation.existence_probability),
                static_cast<double>(observation.lidar_confidence));
            if (require_confirmed_ && !observation.confirmed)
            {
                continue;
            }
            const double adjusted_confidence = observation.confirmed ?
                confidence : confidence * std::max(
                    0.0, std::min(1.0, unconfirmed_confidence_scale_));
            if (adjusted_confidence < min_confidence_)
            {
                continue;
            }

            ConePoint cone;
            cone.id = observation.id;
            cone.position.x = x;
            cone.position.y = y;
            cone.confidence =
                std::max(0.0, std::min(1.0, adjusted_confidence));
            cone.confirmed = observation.confirmed;

            bool merged = false;
            for (ConePoint& existing : cones)
            {
                if (std::hypot(
                        existing.position.x - cone.position.x,
                        existing.position.y - cone.position.y) <
                    min_cone_separation_)
                {
                    if (cone.confidence > existing.confidence)
                    {
                        existing = cone;
                    }
                    merged = true;
                    break;
                }
            }
            if (!merged)
            {
                cones.push_back(cone);
            }
        }
        reason = "ok";
        return true;
    }

    void odometryCallback(const nav_msgs::Odometry::ConstPtr& message)
    {
        OdomState state;
        state.stamp = message->header.stamp.isZero() ?
            ros::Time::now() : message->header.stamp;
        state.frame_id = message->header.frame_id;
        state.x = message->pose.pose.position.x;
        state.y = message->pose.pose.position.y;
        state.yaw = tf2::getYaw(message->pose.pose.orientation);
        state.valid = !state.frame_id.empty() &&
            std::isfinite(state.x) && std::isfinite(state.y) &&
            std::isfinite(state.yaw);

        std::lock_guard<std::mutex> lock(history_mutex_);
        const bool frame_changed = latest_odom_.valid && state.valid &&
            latest_odom_.frame_id != state.frame_id;
        const bool pose_jumped = latest_odom_.valid && state.valid &&
            (std::hypot(
                 state.x - latest_odom_.x,
                 state.y - latest_odom_.y) >
                 std::max(0.1, odometry_reset_translation_) ||
             std::abs(normalizeAngle(state.yaw - latest_odom_.yaw)) >
                 std::max(0.1, odometry_reset_yaw_));
        if (frame_changed || pose_jumped)
        {
            odometry_history_.clear();
            previous_world_path_.clear();
            previous_path_stamp_ = ros::Time(0);
            previous_path_confidence_ = 0.0;
            ROS_WARN(
                "Odometry frame/reset changed, cleared planner history.");
        }
        latest_odom_ = state;
        if (state.valid)
        {
            odometry_history_.push_back(state);
            while (!odometry_history_.empty() &&
                   (state.stamp - odometry_history_.front().stamp).toSec() >
                       std::max(0.1, odometry_buffer_duration_))
            {
                odometry_history_.pop_front();
            }
        }
    }

    bool getCurrentOdom(const ros::Time& message_stamp, OdomState& output)
    {
        {
            std::lock_guard<std::mutex> lock(history_mutex_);
            if (odometry_history_.empty())
            {
                output = latest_odom_;
            }
            else
            {
                const ros::Time target_stamp = message_stamp.isZero() ?
                    ros::Time::now() : message_stamp;
                output = odometry_history_.front();
                double best_age =
                    std::abs((target_stamp - output.stamp).toSec());
                for (const OdomState& candidate : odometry_history_)
                {
                    const double age =
                        std::abs((target_stamp - candidate.stamp).toSec());
                    if (age < best_age)
                    {
                        output = candidate;
                        best_age = age;
                    }
                }
            }
        }
        if (!history_enabled_ || !output.valid)
        {
            return false;
        }

        const ros::Time target_stamp = message_stamp.isZero() ?
            ros::Time::now() : message_stamp;
        return std::abs((target_stamp - output.stamp).toSec()) <=
            odometry_max_age_;
    }

    std::vector<Point2D> buildReferencePath(
        const OdomState& current_odom,
        const ros::Time& message_stamp)
    {
        std::vector<Point2D> world_path;
        ros::Time path_stamp;
        {
            std::lock_guard<std::mutex> lock(history_mutex_);
            world_path = previous_world_path_;
            path_stamp = previous_path_stamp_;
        }
        const ros::Time target_stamp = message_stamp.isZero() ?
            ros::Time::now() : message_stamp;
        if (world_path.size() < 2U || path_stamp.isZero() ||
            (target_stamp - path_stamp).toSec() < 0.0 ||
            (target_stamp - path_stamp).toSec() > history_max_age_)
        {
            return std::vector<Point2D>();
        }

        const double cosine = std::cos(current_odom.yaw);
        const double sine = std::sin(current_odom.yaw);
        std::vector<Point2D> reference;
        reference.reserve(world_path.size());
        for (const Point2D& world_point : world_path)
        {
            const double dx = world_point.x - current_odom.x;
            const double dy = world_point.y - current_odom.y;
            Point2D local;
            local.x = cosine * dx + sine * dy;
            local.y = -sine * dx + cosine * dy;
            if (local.x >= -0.3 &&
                std::hypot(local.x, local.y) <= roi_max_x_ + 2.0)
            {
                reference.push_back(local);
            }
        }
        if (reference.size() < 2U)
        {
            reference.clear();
        }
        else if (std::hypot(reference.front().x, reference.front().y) > 0.3)
        {
            reference.insert(reference.begin(), Point2D());
        }
        return reference;
    }

    double previousPathAge(const ros::Time& message_stamp)
    {
        std::lock_guard<std::mutex> lock(history_mutex_);
        if (previous_path_stamp_.isZero())
        {
            return std::numeric_limits<double>::infinity();
        }
        const ros::Time target_stamp = message_stamp.isZero() ?
            ros::Time::now() : message_stamp;
        return (target_stamp - previous_path_stamp_).toSec();
    }

    double previousPathConfidence()
    {
        std::lock_guard<std::mutex> lock(history_mutex_);
        return previous_path_confidence_;
    }

    void storePreviousPath(
        const std::vector<PathPoint>& path,
        const OdomState& odom,
        const ros::Time& message_stamp)
    {
        if (!history_enabled_ || !odom.valid)
        {
            return;
        }

        const double cosine = std::cos(odom.yaw);
        const double sine = std::sin(odom.yaw);
        std::vector<Point2D> world_path;
        world_path.reserve(path.size());
        for (const PathPoint& point : path)
        {
            Point2D world;
            world.x = odom.x + cosine * point.x - sine * point.y;
            world.y = odom.y + sine * point.x + cosine * point.y;
            world_path.push_back(world);
        }

        std::lock_guard<std::mutex> lock(history_mutex_);
        previous_world_path_.swap(world_path);
        previous_path_stamp_ = message_stamp.isZero() ?
            ros::Time::now() : message_stamp;
        previous_path_confidence_ = path.empty() ? 0.0 : path.front().confidence;
    }

    void publishDiagnostics(double confidence, const std::string& status)
    {
        std_msgs::Float32 confidence_message;
        confidence_message.data = static_cast<float>(
            std::max(0.0, std::min(1.0, confidence)));
        confidence_publisher_.publish(confidence_message);

        std_msgs::String status_message;
        status_message.data = status;
        status_publisher_.publish(status_message);
    }

    void publishPath(
        const std_msgs::Header& header,
        const std::vector<PathPoint>& points)
    {
        nav_msgs::Path path;
        path.header = header;
        path.poses.reserve(points.size());
        for (const PathPoint& point : points)
        {
            geometry_msgs::PoseStamped pose;
            pose.header = header;
            pose.pose.position.x = point.x;
            pose.pose.position.y = point.y;
            pose.pose.position.z = 0.0;
            tf2::Quaternion orientation;
            orientation.setRPY(0.0, 0.0, point.yaw);
            pose.pose.orientation = tf2::toMsg(orientation);
            path.poses.push_back(pose);
        }
        path_publisher_.publish(path);
    }

    void publishDebug(
        const std_msgs::Header& header,
        const std::vector<ConePoint>& cones,
        const std::vector<CandidateEdge>& edges,
        const SearchResult& search_result,
        const std::vector<PathPoint>& path)
    {
        if (!publish_debug_)
        {
            return;
        }

        visualization_msgs::MarkerArray markers;
        visualization_msgs::Marker clear;
        clear.action = visualization_msgs::Marker::DELETEALL;
        markers.markers.push_back(clear);

        visualization_msgs::Marker edge_marker;
        edge_marker.header = header;
        edge_marker.ns = "candidate_edges";
        edge_marker.id = 0;
        edge_marker.type = visualization_msgs::Marker::LINE_LIST;
        edge_marker.action = visualization_msgs::Marker::ADD;
        edge_marker.pose.orientation.w = 1.0;
        edge_marker.scale.x = 0.025;
        edge_marker.color.r = 0.15F;
        edge_marker.color.g = 0.55F;
        edge_marker.color.b = 1.0F;
        edge_marker.color.a = 0.65F;
        for (const CandidateEdge& edge : edges)
        {
            if (edge.first_cone_index >= cones.size() ||
                edge.second_cone_index >= cones.size())
            {
                continue;
            }
            edge_marker.points.push_back(
                toRosPoint(cones[edge.first_cone_index].position, 0.04));
            edge_marker.points.push_back(
                toRosPoint(cones[edge.second_cone_index].position, 0.04));
        }
        markers.markers.push_back(edge_marker);

        visualization_msgs::Marker raw_marker;
        raw_marker.header = header;
        raw_marker.ns = "selected_midpoints";
        raw_marker.id = 1;
        raw_marker.type = visualization_msgs::Marker::LINE_STRIP;
        raw_marker.action = visualization_msgs::Marker::ADD;
        raw_marker.pose.orientation.w = 1.0;
        raw_marker.scale.x = 0.06;
        raw_marker.color.r = 1.0F;
        raw_marker.color.g = 0.75F;
        raw_marker.color.b = 0.1F;
        raw_marker.color.a = 1.0F;
        for (const Point2D& point : search_result.midpoints)
        {
            raw_marker.points.push_back(toRosPoint(point, 0.08));
        }
        markers.markers.push_back(raw_marker);

        visualization_msgs::Marker path_marker;
        path_marker.header = header;
        path_marker.ns = "smoothed_path";
        path_marker.id = 2;
        path_marker.type = visualization_msgs::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::Marker::ADD;
        path_marker.pose.orientation.w = 1.0;
        path_marker.scale.x = 0.09;
        path_marker.color.r = 0.15F;
        path_marker.color.g = 1.0F;
        path_marker.color.b = 0.25F;
        path_marker.color.a = 1.0F;
        for (const PathPoint& point : path)
        {
            Point2D value;
            value.x = point.x;
            value.y = point.y;
            path_marker.points.push_back(toRosPoint(value, 0.12));
        }
        markers.markers.push_back(path_marker);
        debug_publisher_.publish(markers);
    }

    void watchdogCallback(const ros::WallTimerEvent&)
    {
        if ((ros::WallTime::now() - last_input_wall_time_).toSec() <=
            std::max(0.05, input_timeout_))
        {
            return;
        }

        if (!input_timed_out_)
        {
            std::lock_guard<std::mutex> lock(history_mutex_);
            previous_world_path_.clear();
            previous_path_stamp_ = ros::Time(0);
            previous_path_confidence_ = 0.0;
            input_timed_out_ = true;
        }

        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = expected_frame_;
        publishPath(header, std::vector<PathPoint>());
        publishDiagnostics(0.0, "STALE_INPUT");
        publishDebug(
            header,
            std::vector<ConePoint>(),
            std::vector<CandidateEdge>(),
            SearchResult(),
            std::vector<PathPoint>());
        consecutive_failures_ = std::max(
            consecutive_failures_, fallback_max_frames_ + 1);
        ROS_ERROR_STREAM_THROTTLE(
            1.0, "local path planner input timeout: "
            << (ros::WallTime::now() - last_input_wall_time_).toSec()
            << " s");
    }

    void coneCallback(
        const driverless_msgs::ConeObservationArray::ConstPtr& message)
    {
        last_input_wall_time_ = ros::WallTime::now();
        input_timed_out_ = false;
        const ros::WallTime start_time = ros::WallTime::now();
        std::vector<ConePoint> cones;
        std::vector<CandidateEdge> edges;
        SearchResult search_result;
        std::vector<PathPoint> path;
        std::string failure_reason;
        std_msgs::Header output_header;
        bool planning_input_valid = false;
        bool reused_history = false;
        bool reused_single_side = false;
        bool used_gap_bridge = false;
        OdomState current_odom;
        const bool have_current_odom =
            getCurrentOdom(message->header.stamp, current_odom);
        const std::vector<Point2D> reference_path = have_current_odom ?
            buildReferencePath(current_odom, message->header.stamp) :
            std::vector<Point2D>();

        if (!prepareCones(
                *message, cones, output_header, failure_reason))
        {
            planning_input_valid = false;
        }
        else
        {
            planning_input_valid = true;
            if (cones.size() < static_cast<std::size_t>(std::max(2, min_input_cones_)))
            {
                failure_reason = "not enough valid cones";
            }
            else
            {
                edges = edge_generator_->generate(cones, reference_path);
                search_result = path_searcher_->search(edges, reference_path);
                if (!search_result.success)
                {
                    failure_reason = search_result.reason;
                }
                else
                {
                    path = path_smoother_->smooth(
                        search_result.midpoints, search_result.confidence);
                    if (!path_validator_->validate(path, cones, failure_reason))
                    {
                        path.clear();
                    }
                    else
                    {
                        used_gap_bridge = search_result.gap_bridges > 0;
                    }
                }
            }
        }

        if (path.empty())
        {
            ++consecutive_failures_;
            const double history_age = previousPathAge(message->header.stamp);
            if (planning_input_valid && have_current_odom &&
                reference_path.size() >= 2U &&
                consecutive_failures_ <= std::max(0, single_side_max_frames_) &&
                history_age >= 0.0 &&
                history_age <= std::max(0.0, single_side_max_age_))
            {
                const SingleSideRecoveryResult recovery =
                    single_side_recovery_->recover(
                        cones,
                        reference_path,
                        previousPathConfidence());
                if (recovery.success)
                {
                    std::vector<PathPoint> recovery_path =
                        path_smoother_->smooth(
                            recovery.centerline, recovery.confidence);
                    std::string recovery_reason;
                    if (path_validator_->validate(
                            recovery_path, cones, recovery_reason))
                    {
                        path.swap(recovery_path);
                        reused_single_side = true;
                        search_result.success = true;
                        search_result.reason = "single-side recovery";
                        search_result.midpoints = recovery.centerline;
                        search_result.confidence = recovery.confidence;
                    }
                }
            }

            if (path.empty() && planning_input_valid && fallback_enabled_ &&
                have_current_odom && reference_path.size() >= 2U &&
                consecutive_failures_ <= std::max(0, fallback_max_frames_) &&
                history_age >= 0.0 &&
                history_age <= std::max(0.0, fallback_max_age_))
            {
                const double fallback_confidence =
                    previousPathConfidence() * std::pow(
                        std::max(0.0, std::min(1.0, fallback_confidence_decay_)),
                        consecutive_failures_);
                std::vector<PathPoint> fallback_path =
                    path_smoother_->smooth(reference_path, fallback_confidence);
                std::string fallback_reason;
                if (path_validator_->validate(
                        fallback_path, cones, fallback_reason))
                {
                    path.swap(fallback_path);
                    reused_history = true;
                }
            }
        }
        else
        {
            consecutive_failures_ = 0;
            if (have_current_odom)
            {
                storePreviousPath(path, current_odom, output_header.stamp);
            }
        }

        publishPath(output_header, path);
        publishDebug(output_header, cones, edges, search_result, path);

        const double elapsed_ms =
            (ros::WallTime::now() - start_time).toSec() * 1000.0;
        const double path_confidence =
            path.empty() ? 0.0 : path.front().confidence;
        std::string status;
        if (path.empty())
        {
            status = std::string("INVALID: ") + failure_reason;
        }
        else if (reused_single_side)
        {
            status = std::string("DEGRADED_SINGLE_SIDE: ") + failure_reason;
        }
        else if (reused_history)
        {
            status = std::string("DEGRADED_HISTORY: ") + failure_reason;
        }
        else if (used_gap_bridge)
        {
            status = "DEGRADED_GAP_BRIDGE";
        }
        else
        {
            status = "OK";
        }
        status += "; cones=" + std::to_string(cones.size()) +
            "; edges=" + std::to_string(edges.size()) +
            "; ms=" + std::to_string(elapsed_ms);
        publishDiagnostics(path_confidence, status);

        if (path.empty())
        {
            ROS_WARN_STREAM_THROTTLE(
                1.0, "local path invalid: " << failure_reason
                << ", cones=" << cones.size()
                << ", edges=" << edges.size());
        }
        else
        {
            ROS_INFO_STREAM_THROTTLE(
                1.0, "local path points=" << path.size()
                << ", cones=" << cones.size()
                << ", edges=" << edges.size()
                << ", elapsed_ms=" << elapsed_ms);
        }
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber cone_subscriber_;
    ros::Subscriber odometry_subscriber_;
    ros::Publisher path_publisher_;
    ros::Publisher debug_publisher_;
    ros::Publisher confidence_publisher_;
    ros::Publisher status_publisher_;
    ros::WallTimer watchdog_timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::unique_ptr<CandidateEdgeGenerator> edge_generator_;
    std::unique_ptr<PathSearcher> path_searcher_;
    std::unique_ptr<PathSmoother> path_smoother_;
    std::unique_ptr<PathValidator> path_validator_;
    std::unique_ptr<SingleSideRecovery> single_side_recovery_;
    std::mutex history_mutex_;
    OdomState latest_odom_;
    std::deque<OdomState> odometry_history_;
    std::vector<Point2D> previous_world_path_;
    ros::Time previous_path_stamp_;
    double previous_path_confidence_ = 0.0;
    ros::WallTime last_input_wall_time_;
    std::string expected_frame_;
    bool publish_debug_ = true;
    bool require_confirmed_ = false;
    bool history_enabled_ = true;
    bool fallback_enabled_ = true;
    bool input_timed_out_ = false;
    int min_input_cones_ = 4;
    int fallback_max_frames_ = 2;
    int single_side_max_frames_ = 12;
    int consecutive_failures_ = 0;
    double roi_min_x_ = 0.3;
    double roi_max_x_ = 10.0;
    double roi_max_abs_y_ = 5.0;
    double min_confidence_ = 0.35;
    double min_cone_separation_ = 0.08;
    double unconfirmed_confidence_scale_ = 0.75;
    double history_max_age_ = 0.6;
    double fallback_max_age_ = 0.35;
    double fallback_confidence_decay_ = 0.65;
    double single_side_max_age_ = 1.2;
    double odometry_max_age_ = 0.25;
    double odometry_buffer_duration_ = 2.0;
    double odometry_reset_translation_ = 2.0;
    double odometry_reset_yaw_ = 1.05;
    double tf_lookup_timeout_ = 0.03;
    double input_timeout_ = 0.35;
    double watchdog_rate_ = 10.0;
};

}  // namespace local_path_planner

int main(int argc, char** argv)
{
    ros::init(argc, argv, "local_path_planner_node");
    try
    {
        local_path_planner::LocalPathPlannerNode node;
        ros::spin();
    }
    catch (const std::exception& exception)
    {
        ROS_FATAL("Failed to start local_path_planner_node: %s", exception.what());
        return 1;
    }
    return 0;
}
