#pragma once

#include <vector>

#include "local_path_planner/planner_types.hpp"

namespace local_path_planner
{

struct PathSearcherConfig
{
    double start_min_distance = 0.5;
    double start_max_distance = 4.0;
    double min_start_forward_x = 0.0;
    bool allow_reference_lateral_start = false;
    double max_start_heading = 0.96;
    double min_step_distance = 0.4;
    double max_step_distance = 3.2;
    double max_gap_step_distance = 6.5;
    double expected_step_distance = 2.0;
    double max_turn_angle = 0.96;
    double max_gap_turn_angle = 1.75;
    double max_path_length = 12.0;
    double max_reference_distance = 2.0;
    bool hard_reference_gate = true;
    bool require_reference_for_gap_bridge = false;
    double max_reference_progress_step = 9.0;
    double reference_backtrack_tolerance = 0.5;
    int min_midpoints = 3;
    int max_midpoints = 12;
    int max_gap_bridges = 1;
    int beam_width = 24;
    double edge_cost_weight = 1.0;
    double heading_cost_weight = 1.5;
    double spacing_cost_weight = 0.5;
    double history_cost_weight = 1.2;
    double reused_cone_penalty = 0.4;
    double width_change_cost_weight = 0.8;
    double boundary_spacing_cost_weight = 0.8;
    double expected_boundary_spacing = 3.0;
    double max_boundary_step = 7.0;
    double turn_change_cost_weight = 1.5;
    double turn_reversal_cost_weight = 4.0;
    double cross_track_alignment_cost_weight = 1.5;
    double shared_cone_transition_penalty = 0.15;
    double disconnected_edge_penalty = 0.25;
    double preferred_path_length = 8.0;
    double short_path_cost_weight = 1.5;
    double progress_reward_weight = 0.25;
    double gap_bridge_penalty = 2.0;
    double gap_confidence_scale = 0.70;
    double mission_wrong_turn_penalty = 8.0;
    double mission_straight_penalty = 2.0;
    double mission_turn_deadband = 0.06;
};

class PathSearcher
{
public:
    explicit PathSearcher(const PathSearcherConfig& config = PathSearcherConfig());

    SearchResult search(
        const std::vector<CandidateEdge>& edges,
        const std::vector<Point2D>& reference_path = std::vector<Point2D>(),
        int desired_turn = 0) const;

private:
    PathSearcherConfig config_;
};

}  // namespace local_path_planner
