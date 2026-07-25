#pragma once

#include <vector>

#include "local_path_planner/planner_types.hpp"

namespace local_path_planner
{

struct PathSearcherConfig
{
    double start_min_distance = 0.5;
    double start_max_distance = 4.0;
    double max_start_heading = 0.96;
    double min_step_distance = 0.4;
    double max_step_distance = 3.2;
    double max_gap_step_distance = 6.5;
    double expected_step_distance = 2.0;
    double max_turn_angle = 0.96;
    double max_gap_turn_angle = 1.75;
    double max_path_length = 12.0;
    double max_reference_distance = 2.0;
    int min_midpoints = 3;
    int max_midpoints = 12;
    int max_gap_bridges = 1;
    int beam_width = 24;
    double edge_cost_weight = 1.0;
    double heading_cost_weight = 1.5;
    double spacing_cost_weight = 0.5;
    double history_cost_weight = 1.2;
    double reused_cone_penalty = 0.4;
    double progress_reward_weight = 0.25;
    double gap_bridge_penalty = 2.0;
    double gap_confidence_scale = 0.70;
};

class PathSearcher
{
public:
    explicit PathSearcher(const PathSearcherConfig& config = PathSearcherConfig());

    SearchResult search(
        const std::vector<CandidateEdge>& edges,
        const std::vector<Point2D>& reference_path = std::vector<Point2D>()) const;

private:
    PathSearcherConfig config_;
};

}  // namespace local_path_planner
