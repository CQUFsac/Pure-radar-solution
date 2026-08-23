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
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <array>
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

std::vector<double> cumulativeLengths(const std::vector<Point2D>& points)
{
    std::vector<double> cumulative(points.size(), 0.0);
    for (std::size_t index = 1U; index < points.size(); ++index)
    {
        cumulative[index] = cumulative[index - 1U] + std::hypot(
            points[index].x - points[index - 1U].x,
            points[index].y - points[index - 1U].y);
    }
    return cumulative;
}

bool samplePolyline(
    const std::vector<Point2D>& points,
    const std::vector<double>& cumulative,
    double progress,
    Point2D& output)
{
    if (points.size() < 2U || cumulative.size() != points.size() ||
        progress < 0.0 || progress > cumulative.back() + 1.0e-6)
    {
        return false;
    }
    const auto upper = std::upper_bound(
        cumulative.begin(), cumulative.end(), progress);
    const std::size_t end_index = upper == cumulative.end() ?
        points.size() - 1U :
        static_cast<std::size_t>(upper - cumulative.begin());
    if (end_index == 0U)
    {
        output = points.front();
        return true;
    }
    const std::size_t start_index = end_index - 1U;
    const double segment_length =
        cumulative[end_index] - cumulative[start_index];
    const double ratio = segment_length > 1.0e-9 ?
        (progress - cumulative[start_index]) / segment_length : 0.0;
    output.x = points[start_index].x +
        ratio * (points[end_index].x - points[start_index].x);
    output.y = points[start_index].y +
        ratio * (points[end_index].y - points[start_index].y);
    return true;
}

double meanNearPolylineCurvature(
    const std::vector<Point2D>& points,
    double maximum_progress)
{
    double weighted_curvature = 0.0;
    double weight_sum = 0.0;
    double progress = 0.0;
    maximum_progress = std::max(0.2, maximum_progress);
    for (std::size_t index = 1U; index + 1U < points.size(); ++index)
    {
        const double first_x = points[index].x - points[index - 1U].x;
        const double first_y = points[index].y - points[index - 1U].y;
        const double second_x = points[index + 1U].x - points[index].x;
        const double second_y = points[index + 1U].y - points[index].y;
        const double first_length = std::hypot(first_x, first_y);
        const double second_length = std::hypot(second_x, second_y);
        if (first_length < 1.0e-5 || second_length < 1.0e-5)
        {
            continue;
        }
        progress += first_length;
        if (progress > maximum_progress)
        {
            break;
        }
        const double heading_change = std::atan2(
            first_x * second_y - first_y * second_x,
            first_x * second_x + first_y * second_y);
        const double local_length = std::max(
            0.1, 0.5 * (first_length + second_length));
        const double weight = std::min(
            local_length, maximum_progress - progress + first_length);
        weighted_curvature += (heading_change / local_length) * weight;
        weight_sum += weight;
    }
    return weight_sum > 1.0e-6 ? weighted_curvature / weight_sum : 0.0;
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
        std::string mission_turn_hint_topic;
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
            "mission/turn_hint_topic",
            mission_turn_hint_topic,
            "/mission/turn_hint");
        private_nh_.param<std::string>(
            "expected_frame", expected_frame_, "base_link");
        private_nh_.param("queue_size", queue_size, 1);
        private_nh_.param("publish_debug", publish_debug_, true);
        private_nh_.param("use_odometry", use_odometry_, true);
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
        private_nh_.param("temporal/enabled", temporal_enabled_, false);
        private_nh_.param(
            "temporal/new_path_weight", temporal_new_path_weight_, 0.30);
        private_nh_.param(
            "temporal/compare_distance", temporal_compare_distance_, 5.0);
        private_nh_.param(
            "temporal/min_compare_length", temporal_min_compare_length_, 1.0);
        private_nh_.param(
            "temporal/max_mean_deviation", temporal_max_mean_deviation_, 0.60);
        private_nh_.param(
            "temporal/max_near_deviation", temporal_max_near_deviation_, 0.85);
        private_nh_.param(
            "temporal/near_distance", temporal_near_distance_, 3.0);
        private_nh_.param(
            "temporal/high_curvature_threshold",
            temporal_high_curvature_threshold_,
            0.35);
        private_nh_.param(
            "temporal/high_curvature_new_path_weight",
            temporal_high_curvature_new_path_weight_,
            0.38);
        private_nh_.param(
            "temporal/corner_exit_new_path_weight",
            temporal_corner_exit_new_path_weight_,
            0.70);
        private_nh_.param(
            "temporal/corner_exit_detection_distance",
            temporal_corner_exit_detection_distance_,
            2.0);
        private_nh_.param(
            "temporal/far_new_path_weight",
            temporal_far_new_path_weight_,
            0.85);
        private_nh_.param(
            "temporal/far_blend_start_distance",
            temporal_far_blend_start_distance_,
            1.0);
        private_nh_.param(
            "temporal/far_blend_full_distance",
            temporal_far_blend_full_distance_,
            4.0);
        temporal_new_path_weight_ = std::max(
            0.05, std::min(1.0, temporal_new_path_weight_));
        temporal_high_curvature_new_path_weight_ = std::max(
            temporal_new_path_weight_,
            std::min(1.0, temporal_high_curvature_new_path_weight_));
        temporal_corner_exit_new_path_weight_ = std::max(
            temporal_new_path_weight_,
            std::min(1.0, temporal_corner_exit_new_path_weight_));
        temporal_corner_exit_detection_distance_ = std::max(
            0.5, temporal_corner_exit_detection_distance_);
        temporal_far_new_path_weight_ = std::max(
            temporal_new_path_weight_,
            std::min(1.0, temporal_far_new_path_weight_));
        temporal_far_blend_start_distance_ = std::max(
            0.0, temporal_far_blend_start_distance_);
        temporal_far_blend_full_distance_ = std::max(
            temporal_far_blend_start_distance_ + 0.1,
            temporal_far_blend_full_distance_);
        private_nh_.param("cone_memory/enabled", cone_memory_enabled_, false);
        private_nh_.param(
            "cone_memory/merge_distance", cone_memory_merge_distance_, 0.45);
        private_nh_.param(
            "cone_memory/max_age", cone_memory_max_age_, 5.0);
        private_nh_.param(
            "cone_memory/max_radius", cone_memory_max_radius_, 16.0);
        private_nh_.param(
            "cone_memory/min_confidence", cone_memory_min_confidence_, 0.22);
        private_nh_.param(
            "cone_memory/position_update_weight",
            cone_memory_position_update_weight_,
            0.35);
        private_nh_.param(
            "cone_memory/semantic_vote_decay",
            cone_memory_semantic_vote_decay_,
            0.96);
        private_nh_.param(
            "cone_memory/max_cones", cone_memory_max_cones_, 240);
        cone_memory_position_update_weight_ = std::max(
            0.05, std::min(1.0, cone_memory_position_update_weight_));
        cone_memory_semantic_vote_decay_ = std::max(
            0.50, std::min(1.0, cone_memory_semantic_vote_decay_));
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
        private_nh_.param(
            "mission/use_turn_hint", use_mission_turn_hint_, false);
        private_nh_.param(
            "mission/turn_hint_timeout", mission_turn_hint_timeout_, 0.50);
        private_nh_.param(
            "recovery/prefer_real_cross_edges",
            prefer_real_cross_edges_,
            true);
        private_nh_.param(
            "recovery/min_reference_hold_length",
            min_reference_hold_length_,
            2.0);
        private_nh_.param(
            "recovery/reference_hold_min_curvature",
            reference_hold_min_curvature_,
            0.08);
        min_reference_hold_length_ = std::max(
            0.5, min_reference_hold_length_);
        reference_hold_min_curvature_ = std::max(
            0.01, reference_hold_min_curvature_);

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
        if (use_odometry_)
        {
            odometry_subscriber_ = nh_.subscribe(
                odometry_topic,
                10,
                &LocalPathPlannerNode::odometryCallback,
                this);
        }
        if (use_mission_turn_hint_)
        {
            mission_turn_hint_subscriber_ = nh_.subscribe(
                mission_turn_hint_topic,
                1,
                &LocalPathPlannerNode::missionTurnHintCallback,
                this);
        }
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

    struct RememberedCone
    {
        std::uint32_t id = 0U;
        Point2D world_position;
        double confidence = 0.0;
        int hits = 0;
        ros::Time last_seen;
        std::array<double, 5U> semantic_votes{{0.0, 0.0, 0.0, 0.0, 0.0}};
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
            "edge/hard_reference_gate",
            config.hard_reference_gate,
            config.hard_reference_gate);
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
            "edge/use_semantic_pairing",
            config.use_semantic_pairing,
            config.use_semantic_pairing);
        private_nh_.param(
            "edge/allow_orange_cross_edges",
            config.allow_orange_cross_edges,
            config.allow_orange_cross_edges);
        private_nh_.param(
            "edge/enforce_semantic_one_to_one",
            config.enforce_semantic_one_to_one,
            config.enforce_semantic_one_to_one);
        private_nh_.param(
            "edge/unknown_semantic_penalty",
            config.unknown_semantic_penalty,
            config.unknown_semantic_penalty);
        private_nh_.param(
            "edge/use_semantic_boundary_recovery",
            config.use_semantic_boundary_recovery,
            config.use_semantic_boundary_recovery);
        private_nh_.param(
            "edge/semantic_boundary_requires_reference",
            config.semantic_boundary_requires_reference,
            config.semantic_boundary_requires_reference);
        private_nh_.param(
            "edge/expected_boundary_spacing",
            config.expected_boundary_spacing,
            config.expected_boundary_spacing);
        private_nh_.param(
            "edge/min_boundary_link_distance",
            config.min_boundary_link_distance,
            config.min_boundary_link_distance);
        private_nh_.param(
            "edge/max_boundary_link_distance",
            config.max_boundary_link_distance,
            config.max_boundary_link_distance);
        double max_boundary_heading_error_deg =
            config.max_boundary_heading_error * 180.0 /
            3.14159265358979323846;
        private_nh_.param(
            "edge/max_boundary_heading_error_deg",
            max_boundary_heading_error_deg,
            max_boundary_heading_error_deg);
        config.max_boundary_heading_error =
            degreesToRadians(max_boundary_heading_error_deg);
        double max_boundary_start_heading_error_deg =
            config.max_boundary_start_heading_error * 180.0 /
            3.14159265358979323846;
        private_nh_.param(
            "edge/max_boundary_start_heading_error_deg",
            max_boundary_start_heading_error_deg,
            max_boundary_start_heading_error_deg);
        config.max_boundary_start_heading_error =
            degreesToRadians(max_boundary_start_heading_error_deg);
        private_nh_.param(
            "edge/min_boundary_reference_offset",
            config.min_boundary_reference_offset,
            config.min_boundary_reference_offset);
        private_nh_.param(
            "edge/max_boundary_reference_offset",
            config.max_boundary_reference_offset,
            config.max_boundary_reference_offset);
        private_nh_.param(
            "edge/max_boundary_center_reference_distance",
            config.max_boundary_center_reference_distance,
            config.max_boundary_center_reference_distance);
        private_nh_.param(
            "edge/max_boundary_reference_progress_step",
            config.max_boundary_reference_progress_step,
            config.max_boundary_reference_progress_step);
        private_nh_.param(
            "edge/max_pair_reference_progress_difference",
            config.max_pair_reference_progress_difference,
            config.max_pair_reference_progress_difference);
        private_nh_.param(
            "edge/boundary_virtual_cost",
            config.boundary_virtual_cost,
            config.boundary_virtual_cost);
        private_nh_.param(
            "edge/max_boundary_edges_per_cone",
            config.max_boundary_edges_per_cone,
            config.max_boundary_edges_per_cone);
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
        private_nh_.param(
            "search/min_start_forward_x",
            config.min_start_forward_x,
            config.min_start_forward_x);
        private_nh_.param(
            "search/allow_reference_lateral_start",
            config.allow_reference_lateral_start,
            config.allow_reference_lateral_start);
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
            "search/hard_reference_gate",
            config.hard_reference_gate,
            config.hard_reference_gate);
        private_nh_.param(
            "search/require_reference_for_gap_bridge",
            config.require_reference_for_gap_bridge,
            config.require_reference_for_gap_bridge);
        private_nh_.param(
            "search/max_reference_progress_step",
            config.max_reference_progress_step,
            config.max_reference_progress_step);
        private_nh_.param(
            "search/reference_backtrack_tolerance",
            config.reference_backtrack_tolerance,
            config.reference_backtrack_tolerance);
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
            "search/width_change_cost_weight",
            config.width_change_cost_weight,
            config.width_change_cost_weight);
        private_nh_.param(
            "search/boundary_spacing_cost_weight",
            config.boundary_spacing_cost_weight,
            config.boundary_spacing_cost_weight);
        private_nh_.param(
            "search/expected_boundary_spacing",
            config.expected_boundary_spacing,
            config.expected_boundary_spacing);
        private_nh_.param(
            "search/max_boundary_step",
            config.max_boundary_step,
            config.max_boundary_step);
        private_nh_.param(
            "search/turn_change_cost_weight",
            config.turn_change_cost_weight,
            config.turn_change_cost_weight);
        private_nh_.param(
            "search/turn_reversal_cost_weight",
            config.turn_reversal_cost_weight,
            config.turn_reversal_cost_weight);
        private_nh_.param(
            "search/cross_track_alignment_cost_weight",
            config.cross_track_alignment_cost_weight,
            config.cross_track_alignment_cost_weight);
        private_nh_.param(
            "search/shared_cone_transition_penalty",
            config.shared_cone_transition_penalty,
            config.shared_cone_transition_penalty);
        private_nh_.param(
            "search/disconnected_edge_penalty",
            config.disconnected_edge_penalty,
            config.disconnected_edge_penalty);
        private_nh_.param(
            "search/preferred_path_length",
            config.preferred_path_length,
            config.preferred_path_length);
        private_nh_.param(
            "search/short_path_cost_weight",
            config.short_path_cost_weight,
            config.short_path_cost_weight);
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
        private_nh_.param(
            "search/mission_wrong_turn_penalty",
            config.mission_wrong_turn_penalty,
            config.mission_wrong_turn_penalty);
        private_nh_.param(
            "search/mission_straight_penalty",
            config.mission_straight_penalty,
            config.mission_straight_penalty);
        private_nh_.param(
            "search/mission_turn_deadband",
            config.mission_turn_deadband,
            config.mission_turn_deadband);
        return config;
    }

    void missionTurnHintCallback(const std_msgs::Int8::ConstPtr& message)
    {
        mission_turn_hint_ = std::max(
            -1, std::min(1, static_cast<int>(message->data)));
        last_mission_turn_hint_time_ = ros::Time::now();
    }

    int currentMissionTurnHint() const
    {
        if (!use_mission_turn_hint_ ||
            last_mission_turn_hint_time_.isZero() ||
            (ros::Time::now() - last_mission_turn_hint_time_).toSec() >
                std::max(0.05, mission_turn_hint_timeout_))
        {
            return 0;
        }
        return mission_turn_hint_;
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
            cone.semantic_class = observation.semantic_class;

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

    std::uint8_t rememberedSemantic(const RememberedCone& cone) const
    {
        std::size_t best_class = SEMANTIC_UNKNOWN;
        double best_vote = 0.0;
        for (std::size_t semantic = SEMANTIC_BLUE;
             semantic <= SEMANTIC_BIG_ORANGE;
             ++semantic)
        {
            if (cone.semantic_votes[semantic] > best_vote)
            {
                best_vote = cone.semantic_votes[semantic];
                best_class = semantic;
            }
        }
        return best_vote > 1.0e-6 ?
            static_cast<std::uint8_t>(best_class) : SEMANTIC_UNKNOWN;
    }

    void updateAndApplyConeMemory(
        const std::vector<ConePoint>& observations,
        const OdomState& odom,
        const ros::Time& message_stamp,
        std::vector<ConePoint>& output)
    {
        if (!cone_memory_enabled_ || !odom.valid)
        {
            return;
        }

        const ros::Time stamp = message_stamp.isZero() ?
            ros::Time::now() : message_stamp;
        const double cosine = std::cos(odom.yaw);
        const double sine = std::sin(odom.yaw);
        const double merge_distance = std::max(
            0.05, cone_memory_merge_distance_);

        std::lock_guard<std::mutex> lock(history_mutex_);
        for (const ConePoint& observation : observations)
        {
            Point2D world;
            world.x = odom.x + cosine * observation.position.x -
                sine * observation.position.y;
            world.y = odom.y + sine * observation.position.x +
                cosine * observation.position.y;

            std::size_t nearest_index = remembered_cones_.size();
            double nearest_distance = merge_distance;
            for (std::size_t index = 0U;
                 index < remembered_cones_.size();
                 ++index)
            {
                const double distance = std::hypot(
                    remembered_cones_[index].world_position.x - world.x,
                    remembered_cones_[index].world_position.y - world.y);
                if (distance < nearest_distance)
                {
                    nearest_distance = distance;
                    nearest_index = index;
                }
            }

            if (nearest_index == remembered_cones_.size())
            {
                RememberedCone remembered;
                remembered.id = next_remembered_cone_id_++;
                remembered.world_position = world;
                remembered.confidence = observation.confidence;
                remembered.hits = 1;
                remembered.last_seen = stamp;
                if (observation.semantic_class <= SEMANTIC_BIG_ORANGE)
                {
                    remembered.semantic_votes[observation.semantic_class] =
                        std::max(0.05, observation.confidence);
                }
                remembered_cones_.push_back(remembered);
                continue;
            }

            RememberedCone& remembered = remembered_cones_[nearest_index];
            const double weight = cone_memory_position_update_weight_;
            remembered.world_position.x =
                (1.0 - weight) * remembered.world_position.x +
                weight * world.x;
            remembered.world_position.y =
                (1.0 - weight) * remembered.world_position.y +
                weight * world.y;
            remembered.confidence = std::max(
                remembered.confidence * 0.98, observation.confidence);
            remembered.hits = std::min(10000, remembered.hits + 1);
            remembered.last_seen = stamp;
            for (double& vote : remembered.semantic_votes)
            {
                vote *= cone_memory_semantic_vote_decay_;
            }
            if (observation.semantic_class <= SEMANTIC_BIG_ORANGE)
            {
                remembered.semantic_votes[observation.semantic_class] +=
                    std::max(0.05, observation.confidence);
            }
        }

        remembered_cones_.erase(
            std::remove_if(
                remembered_cones_.begin(),
                remembered_cones_.end(),
                [&](const RememberedCone& cone)
                {
                    const double age = (stamp - cone.last_seen).toSec();
                    const double radius = std::hypot(
                        cone.world_position.x - odom.x,
                        cone.world_position.y - odom.y);
                    return age < -0.05 ||
                        age > std::max(0.1, cone_memory_max_age_) ||
                        radius > std::max(2.0, cone_memory_max_radius_);
                }),
            remembered_cones_.end());

        if (remembered_cones_.size() > static_cast<std::size_t>(
                std::max(2, cone_memory_max_cones_)))
        {
            std::sort(
                remembered_cones_.begin(),
                remembered_cones_.end(),
                [](const RememberedCone& first, const RememberedCone& second)
                {
                    return first.last_seen > second.last_seen;
                });
            remembered_cones_.resize(static_cast<std::size_t>(
                std::max(2, cone_memory_max_cones_)));
        }

        output.clear();
        output.reserve(remembered_cones_.size());
        for (const RememberedCone& remembered : remembered_cones_)
        {
            const double dx = remembered.world_position.x - odom.x;
            const double dy = remembered.world_position.y - odom.y;
            ConePoint cone;
            cone.id = remembered.id;
            cone.position.x = cosine * dx + sine * dy;
            cone.position.y = -sine * dx + cosine * dy;
            if (cone.position.x < roi_min_x_ ||
                cone.position.x > roi_max_x_ ||
                std::abs(cone.position.y) > roi_max_abs_y_)
            {
                continue;
            }

            const double age = std::max(
                0.0, (stamp - remembered.last_seen).toSec());
            const double decay_time = std::max(
                0.5, 0.75 * cone_memory_max_age_);
            cone.confidence = std::max(0.0, std::min(
                1.0,
                remembered.confidence * std::exp(-age / decay_time)));
            if (cone.confidence < cone_memory_min_confidence_)
            {
                continue;
            }
            cone.confirmed = remembered.hits >= 2;
            cone.semantic_class = rememberedSemantic(remembered);
            output.push_back(cone);
        }
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
            remembered_cones_.clear();
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

    bool stabilizeWithReference(
        std::vector<PathPoint>& path,
        const std::vector<Point2D>& reference_path,
        const std::vector<ConePoint>& cones,
        double confidence,
        std::string& reason)
    {
        if (!temporal_enabled_ || path.size() < 2U ||
            reference_path.size() < 2U)
        {
            return true;
        }

        std::vector<Point2D> candidate;
        candidate.reserve(path.size());
        for (const PathPoint& point : path)
        {
            candidate.push_back(Point2D{point.x, point.y});
        }
        const std::vector<double> candidate_length =
            cumulativeLengths(candidate);
        const std::vector<double> reference_length =
            cumulativeLengths(reference_path);
        const double comparison_length = std::min(
            std::max(0.0, temporal_compare_distance_),
            std::min(candidate_length.back(), reference_length.back()));
        if (comparison_length <
            std::max(0.2, temporal_min_compare_length_))
        {
            return true;
        }

        double deviation_sum = 0.0;
        double near_maximum = 0.0;
        int samples = 0;
        const double sample_spacing = 0.25;
        for (double progress = sample_spacing;
             progress <= comparison_length + 1.0e-6;
             progress += sample_spacing)
        {
            Point2D current;
            Point2D previous;
            if (!samplePolyline(
                    candidate, candidate_length, progress, current) ||
                !samplePolyline(
                    reference_path, reference_length, progress, previous))
            {
                continue;
            }
            const double deviation = std::hypot(
                current.x - previous.x, current.y - previous.y);
            deviation_sum += deviation;
            ++samples;
            if (progress <= std::max(0.5, temporal_near_distance_))
            {
                near_maximum = std::max(near_maximum, deviation);
            }
        }
        if (samples <= 0)
        {
            return true;
        }

        const double mean_deviation =
            deviation_sum / static_cast<double>(samples);
        double maximum_curvature = 0.0;
        for (const PathPoint& point : path)
        {
            maximum_curvature = std::max(
                maximum_curvature, std::abs(point.curvature));
        }
        const double curvature_threshold = std::max(
            0.05, temporal_high_curvature_threshold_);
        const double sharp_turn_ratio = std::max(0.0, std::min(
            1.0,
            (maximum_curvature - curvature_threshold) /
                curvature_threshold));
        const double candidate_near_curvature = meanNearPolylineCurvature(
            candidate, temporal_corner_exit_detection_distance_);
        const double reference_near_curvature = meanNearPolylineCurvature(
            reference_path, temporal_corner_exit_detection_distance_);
        const bool near_path_straight =
            std::abs(candidate_near_curvature) < 0.60 * curvature_threshold;
        const bool near_turn_reversed =
            std::abs(candidate_near_curvature) > 0.35 * curvature_threshold &&
            candidate_near_curvature * reference_near_curvature < 0.0;
        const bool curvature_releasing =
            std::abs(reference_near_curvature) > 0.35 * curvature_threshold &&
            std::abs(candidate_near_curvature) <
                0.75 * std::abs(reference_near_curvature);
        const bool leaving_turn =
            (std::abs(reference_near_curvature) > curvature_threshold &&
             (near_path_straight || near_turn_reversed)) ||
            curvature_releasing;
        const double mean_deviation_limit =
            std::max(0.1, temporal_max_mean_deviation_) *
            (1.0 + 0.8 * sharp_turn_ratio);
        const double near_deviation_limit =
            std::max(0.1, temporal_max_near_deviation_) *
            (1.0 + 0.8 * sharp_turn_ratio);
        if (mean_deviation > mean_deviation_limit ||
            near_maximum > near_deviation_limit)
        {
            reason = "temporal path jump rejected (mean=" +
                std::to_string(mean_deviation) + ", near=" +
                std::to_string(near_maximum) + ")";
            ROS_WARN_STREAM_THROTTLE(1.0, reason);
            return false;
        }

        // Blend points by travelled distance, rather than array index.  This
        // remains stable when the spline produces a different point count.
        std::vector<Point2D> blended = candidate;
        for (std::size_t index = 0U; index < blended.size(); ++index)
        {
            const double progress = candidate_length[index];
            if (progress > reference_length.back())
            {
                break;
            }
            Point2D previous;
            if (!samplePolyline(
                    reference_path, reference_length, progress, previous))
            {
                continue;
            }
            double new_weight = temporal_new_path_weight_ +
                sharp_turn_ratio *
                    (temporal_high_curvature_new_path_weight_ -
                     temporal_new_path_weight_);
            if (leaving_turn)
            {
                // When the new observations form a straight exit but history
                // is still curved, release the old steering direction early.
                new_weight = std::max(
                    new_weight,
                    temporal_corner_exit_new_path_weight_);
            }
            const double far_ratio = std::max(0.0, std::min(
                1.0,
                (progress - temporal_far_blend_start_distance_) /
                    std::max(
                        0.1,
                        temporal_far_blend_full_distance_ -
                            temporal_far_blend_start_distance_)));
            new_weight += far_ratio *
                (temporal_far_new_path_weight_ - new_weight);
            if (progress > comparison_length)
            {
                new_weight += (1.0 - new_weight) * std::min(
                    1.0, (progress - comparison_length) / 2.0);
            }
            blended[index].x =
                (1.0 - new_weight) * previous.x +
                new_weight * candidate[index].x;
            blended[index].y =
                (1.0 - new_weight) * previous.y +
                new_weight * candidate[index].y;
        }

        std::vector<PathPoint> stabilized =
            path_smoother_->smooth(blended, confidence);
        std::string validation_reason;
        if (!path_validator_->validate(
                stabilized, cones, validation_reason))
        {
            reason = std::string("temporally stabilized path invalid: ") +
                validation_reason;
            return false;
        }
        path.swap(stabilized);
        return true;
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
        visualization_msgs::Marker boundary_edge_marker = edge_marker;
        boundary_edge_marker.ns = "semantic_boundary_edges";
        boundary_edge_marker.id = 3;
        boundary_edge_marker.scale.x = 0.045;
        boundary_edge_marker.color.r = 1.0F;
        boundary_edge_marker.color.g = 0.20F;
        boundary_edge_marker.color.b = 0.85F;
        boundary_edge_marker.color.a = 0.90F;
        for (const CandidateEdge& edge : edges)
        {
            if (edge.first_cone_index >= cones.size() ||
                edge.second_cone_index >= cones.size())
            {
                continue;
            }
            visualization_msgs::Marker& target_marker =
                edge.virtual_from_boundary ?
                boundary_edge_marker : edge_marker;
            target_marker.points.push_back(
                toRosPoint(cones[edge.first_cone_index].position, 0.04));
            target_marker.points.push_back(
                toRosPoint(cones[edge.second_cone_index].position, 0.04));
        }
        markers.markers.push_back(edge_marker);
        markers.markers.push_back(boundary_edge_marker);

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
            remembered_cones_.clear();
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
        bool used_boundary_recovery = false;
        bool prefer_reference_hold = false;
        OdomState current_odom;
        const bool have_current_odom =
            use_odometry_ &&
            getCurrentOdom(message->header.stamp, current_odom);
        const std::vector<Point2D> reference_path = have_current_odom ?
            buildReferencePath(current_odom, message->header.stamp) :
            std::vector<Point2D>();
        const double history_age = previousPathAge(message->header.stamp);

        if (!prepareCones(
                *message, cones, output_header, failure_reason))
        {
            planning_input_valid = false;
        }
        else
        {
            planning_input_valid = true;
            if (have_current_odom)
            {
                updateAndApplyConeMemory(
                    cones, current_odom, output_header.stamp, cones);
            }
            if (cones.size() < static_cast<std::size_t>(std::max(2, min_input_cones_)))
            {
                failure_reason = "not enough valid cones";
            }
            else
            {
                edges = edge_generator_->generate(cones, reference_path);
                if (prefer_real_cross_edges_)
                {
                    std::vector<CandidateEdge> real_cross_edges;
                    real_cross_edges.reserve(edges.size());
                    for (const CandidateEdge& edge : edges)
                    {
                        if (!edge.virtual_from_boundary &&
                            edge.has_left_boundary && edge.has_right_boundary)
                        {
                            real_cross_edges.push_back(edge);
                        }
                    }

                    const SearchResult real_result = path_searcher_->search(
                        real_cross_edges,
                        reference_path,
                        currentMissionTurnHint());
                    search_result = real_result;
                    if (!real_result.success)
                    {
                        const std::vector<double> reference_lengths =
                            cumulativeLengths(reference_path);
                        const double reference_length =
                            reference_lengths.empty() ?
                            0.0 : reference_lengths.back();
                        const double reference_curvature = std::abs(
                            meanNearPolylineCurvature(reference_path, 5.0));
                        prefer_reference_hold =
                            history_age >= 0.0 &&
                            history_age <= fallback_max_age_ &&
                            reference_length >= min_reference_hold_length_ &&
                            reference_curvature >=
                                reference_hold_min_curvature_;
                        if (!prefer_reference_hold)
                        {
                            const SearchResult recovery_result =
                                path_searcher_->search(
                                edges,
                                reference_path,
                                currentMissionTurnHint());
                            if (recovery_result.success)
                            {
                                search_result = recovery_result;
                                used_boundary_recovery = true;
                            }
                        }
                    }
                }
                else
                {
                    search_result = path_searcher_->search(
                        edges, reference_path, currentMissionTurnHint());
                }
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
                    else if (!stabilizeWithReference(
                            path,
                            reference_path,
                            cones,
                            search_result.confidence,
                            failure_reason))
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

        if (path.empty() && !prefer_reference_hold &&
            planning_input_valid && reference_path.size() >= 2U)
        {
            const std::vector<double> reference_lengths =
                cumulativeLengths(reference_path);
            const double reference_length = reference_lengths.empty() ?
                0.0 : reference_lengths.back();
            const double reference_curvature = std::abs(
                meanNearPolylineCurvature(reference_path, 5.0));
            prefer_reference_hold =
                history_age >= 0.0 &&
                history_age <= fallback_max_age_ &&
                reference_length >= min_reference_hold_length_ &&
                reference_curvature >= reference_hold_min_curvature_;
        }

        if (path.empty())
        {
            ++consecutive_failures_;
            if (!prefer_reference_hold && planning_input_valid &&
                have_current_odom &&
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
        else if (used_boundary_recovery)
        {
            status = "DEGRADED_BOUNDARY_RECOVERY";
        }
        else
        {
            status = "OK";
        }
        status += "; cones=" + std::to_string(cones.size()) +
            "; edges=" + std::to_string(edges.size()) +
            "; memory=" + std::to_string(remembered_cones_.size()) +
            "; turn_hint=" + std::to_string(currentMissionTurnHint()) +
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
    ros::Subscriber mission_turn_hint_subscriber_;
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
    std::vector<RememberedCone> remembered_cones_;
    ros::Time previous_path_stamp_;
    ros::Time last_mission_turn_hint_time_;
    double previous_path_confidence_ = 0.0;
    ros::WallTime last_input_wall_time_;
    std::string expected_frame_;
    bool publish_debug_ = true;
    bool require_confirmed_ = false;
    bool history_enabled_ = true;
    bool use_odometry_ = true;
    bool fallback_enabled_ = true;
    bool input_timed_out_ = false;
    bool use_mission_turn_hint_ = false;
    bool temporal_enabled_ = false;
    bool cone_memory_enabled_ = false;
    bool prefer_real_cross_edges_ = true;
    int min_input_cones_ = 4;
    int fallback_max_frames_ = 2;
    int single_side_max_frames_ = 12;
    int consecutive_failures_ = 0;
    int mission_turn_hint_ = 0;
    int cone_memory_max_cones_ = 240;
    std::uint32_t next_remembered_cone_id_ = 0x80000000U;
    double roi_min_x_ = 0.3;
    double roi_max_x_ = 10.0;
    double roi_max_abs_y_ = 5.0;
    double min_confidence_ = 0.35;
    double min_cone_separation_ = 0.08;
    double unconfirmed_confidence_scale_ = 0.75;
    double history_max_age_ = 0.6;
    double fallback_max_age_ = 0.35;
    double fallback_confidence_decay_ = 0.65;
    double temporal_new_path_weight_ = 0.30;
    double temporal_compare_distance_ = 5.0;
    double temporal_min_compare_length_ = 1.0;
    double temporal_max_mean_deviation_ = 0.60;
    double temporal_max_near_deviation_ = 0.85;
    double temporal_near_distance_ = 3.0;
    double temporal_high_curvature_threshold_ = 0.35;
    double temporal_high_curvature_new_path_weight_ = 0.38;
    double temporal_corner_exit_new_path_weight_ = 0.70;
    double temporal_corner_exit_detection_distance_ = 2.0;
    double temporal_far_new_path_weight_ = 0.85;
    double temporal_far_blend_start_distance_ = 1.0;
    double temporal_far_blend_full_distance_ = 4.0;
    double cone_memory_merge_distance_ = 0.45;
    double cone_memory_max_age_ = 5.0;
    double cone_memory_max_radius_ = 16.0;
    double cone_memory_min_confidence_ = 0.22;
    double cone_memory_position_update_weight_ = 0.35;
    double cone_memory_semantic_vote_decay_ = 0.96;
    double single_side_max_age_ = 1.2;
    double odometry_max_age_ = 0.25;
    double odometry_buffer_duration_ = 2.0;
    double odometry_reset_translation_ = 2.0;
    double odometry_reset_yaw_ = 1.05;
    double tf_lookup_timeout_ = 0.03;
    double input_timeout_ = 0.35;
    double watchdog_rate_ = 10.0;
    double mission_turn_hint_timeout_ = 0.50;
    double min_reference_hold_length_ = 2.0;
    double reference_hold_min_curvature_ = 0.08;
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
